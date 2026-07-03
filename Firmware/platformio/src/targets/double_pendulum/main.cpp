// =====================================================================
//  Pendubot firmware — ESP32 + 2x ODrive over CAN (TWAI)
//  Features: SwingUpXin controller, SysID Phase 3 (Fourier), safety limits
//
//  File layout:
//    1. Includes & forward declarations
//    2. CONFIGURATION      — pins, IDs, gearing, safety limits
//    3. TUNABLES           — gains, trajectories, log rates
//    4. TYPES              — structs, enums, controller interface
//    5. GLOBALS            — grouped by subsystem
//    6. UTILITIES          — serial helpers, unit conversions
//    7. CAN LAYER          — init, pump, health, ODrive callbacks
//    8. SCHEDULER          — state assembly + controller dispatch
//    9. CONTROLLERS        — SwingUpXin
//   10. SYSID              — Fourier trajectory
//   11. COMMANDS           — serial command handler
//   12. setup() / loop()
// =====================================================================

#include <Arduino.h>
#include "driver/twai.h"
#include "ODriveCAN.h"
#include "ODriveESP32TWAI.hpp"

/* ==================================================================
 * 1. FORWARD DECLARATIONS
 * ================================================================== */
void processSerialCommands();
void handleCommand(char* cmd);
void canPump();
void serialDebug(const char* msg);
void serialStatus(const char* msg);
void onCanMessage(const CanMsg& msg);
void handleSysIdCommand(const char* cmd);
void sysidTick();

/* ==================================================================
 * 2. CONFIGURATION — hardware, gearing, safety, timing
 * ================================================================== */

// ---- Serial ----
#define SERIAL_BAUD      921600
#define CMD_BUFFER_SIZE  128

// ---- CAN (TWAI) ----
#define CAN_TX_PIN   40
#define CAN_RX_PIN   41
#define CAN_BITRATE  125000

// ---- ODrive node IDs ----
#define ODRV0_NODE_ID  0
#define ODRV1_NODE_ID  1

// ---- Node 0 mechanical ----
#define NODE0_GEAR_RATIO   8.0f    // motor turns per 1 output turn
#define NODE0_OUTPUT_LIMIT 1.0f    // ± turns at the OUTPUT shaft (user-facing)
#define NODE0_POS_LIMIT    (NODE0_OUTPUT_LIMIT * NODE0_GEAR_RATIO)  // ±8 motor turns
#define MOTOR0_KT          0.496f  // N·m/A at motor — REPLACE with your ODrive torque_constant

// ---- Node 1 mechanical ----
#define NODE1_GEAR_RATIO   1.0f    // change to 8.0f if node 1 is also geared

// ---- Scheduler timing ----
static constexpr unsigned long STATE_TICK_MS     = 5;    // 200 Hz state assembly
static constexpr unsigned long FEEDBACK_STALE_MS = 50;   // feedback older than this = invalid
static constexpr unsigned long CTRL_WATCHDOG_MS  = 100;  // no valid cmd in this window → IDLE

// ---- Misc constants ----
static constexpr float TWO_PI_F = 6.28318530718f;
static constexpr unsigned long IQ_REQUEST_INTERVAL_MS = 50UL;  // Iq poll at 20 Hz

/* ==================================================================
 * 3. TUNABLES — gains, trajectories, logging
 * ================================================================== */

// ---- Swing-Up (Xin) ----
// Phys1
// ---- Physical parameters (measure these) ----
static constexpr float SU_M1   = 0.2733f;   // link 1 mass [kg]
static constexpr float SU_M2   = 0.023f;    // link 2 mass [kg]
static constexpr float SU_L1   = 0.125f;    // link 1 length [m]
static constexpr float SU_L2   = 0.070f;    // link 2 length [m]
static constexpr float SU_D1   = 0.0625f;   // link 1 COM distance [m]
static constexpr float SU_D2   = 0.035f;    // link 2 COM distance [m]
static constexpr float SU_I1ZZ = 1.0e-3f;   // link 1 inertia about COM [kg·m²]
static constexpr float SU_I2ZZ = 1.0e-4f;   // link 2 inertia about COM [kg·m²]
static constexpr float SU_G    = 9.81f;

// ---- Xin a-form (mass matrix / gravity coefficients) ----
// a1 = I1 + m1·d1² + m2·l1² ; a2 = I2 + m2·d2² ; a3 = m2·l1·d2
static constexpr float SU_A1 = SU_I1ZZ + SU_M1*SU_D1*SU_D1 + SU_M2*SU_L1*SU_L1;
static constexpr float SU_A2 = SU_I2ZZ + SU_M2*SU_D2*SU_D2;
static constexpr float SU_A3 = SU_M2*SU_L1*SU_D2;
// b1 = m1·d1 + m2·l1 ; b2 = m2·d2   (gravity terms, ×g applied in controller)
static constexpr float SU_B1 = SU_M1*SU_D1 + SU_M2*SU_L1;
static constexpr float SU_B2 = SU_M2*SU_D2;

// ---- Xin θ-form (used by F(q,qd), D, and Er) ----
static constexpr float SU_TH1 = SU_A1;
static constexpr float SU_TH2 = SU_A2;
static constexpr float SU_TH3 = SU_A3;
static constexpr float SU_TH4 = SU_B1;   // Er = (θ4+θ5)·g
static constexpr float SU_TH5 = SU_B2;

// Live-tunable gains and saturation (set via serial: SU_KP, SU_KD, SU_KV, SU_TAU_MAX)
static float SU_kp      = 1.0f;
static float SU_kd      = 1e-3f;
static float SU_kv      = 0.05f;
static float SU_tau_max = 0.5f;

// ---- SysID safety envelope ----
static constexpr float SYSID_N0_OUT_LIMIT = 0.85f;
static constexpr float SYSID_N0_MOT_LIMIT = SYSID_N0_OUT_LIMIT * NODE0_GEAR_RATIO;
static constexpr float SYSID_ZERO_TOL     = 0.02f;
static constexpr unsigned long SYSID_ZERO_TIMEOUT_MS = 5000UL;
static constexpr float SYSID_N0_VEL_LIMIT = 10.0f;

// ---- SysID Phase 3: Fourier trajectory (inertial identification) ----
static constexpr int   FOURIER_NH         = 5;
static constexpr float FOURIER_WF         = 0.5f;
static constexpr float FOURIER_AMP_OUT    = 0.70f;
static constexpr float FOURIER_DURATION_S = 125.0f;
static const float FA[FOURIER_NH] = { 0.30f,  0.15f,  0.08f,  0.04f,  0.02f };
static const float FB[FOURIER_NH] = { 0.00f,  0.10f, -0.06f,  0.03f, -0.01f };

// ---- Logging ----
#define CTRL_PREFIX   "[CTRL] "
#define CTRL_RATE_HZ  200
#define CTRL_RATE_MS  (1000 / CTRL_RATE_HZ)

#define SYSID_PREFIX  "[SYSID] "
#define SYSID_RATE_HZ 200
#define SYSID_RATE_MS (1000 / SYSID_RATE_HZ)

/* ==================================================================
 * 4. TYPES
 * ================================================================== */

// ---- Plant state (assembled at 200 Hz) ----
struct PlantState {
    float q1   = 0.0f;      // node 0 output rad
    float qd1  = 0.0f;      // node 0 output rad/s
    float q2   = 0.0f;      // node 1 output rad
    float qd2  = 0.0f;      // node 1 output rad/s
    float tau1_meas = 0.0f; // node 0 measured output torque (N·m)
    uint32_t t_ms = 0;
    bool valid = false;
};

// ---- Controller interface ----
enum class CmdMode : uint8_t { NONE, TORQUE, VELOCITY, POSITION };

struct ControllerOutput {
    CmdMode mode = CmdMode::NONE;
    float   value = 0.0f;   // SI units at OUTPUT shaft
    bool    valid = false;
};

class Controller {
public:
    virtual ~Controller() = default;
    virtual const char* name() const = 0;
    virtual CmdMode requiredMode() const = 0;
    virtual void enter(const PlantState&) {}
    virtual void exit() {}
    virtual ControllerOutput update(const PlantState&, float dt) = 0;
    virtual bool wantsHandoff() const { return false; }
};

// ---- SysID phases ----
enum SysIdPhase : uint8_t {
    SID_IDLE    = 0,
    SID_PHASE3  = 3,
    SID_ZEROING = 4,
};

// ---- Per-ODrive telemetry ----
struct ODriveUserData {
    Heartbeat_msg_t heartbeat;
    bool got_heartbeat = false;
    Get_Encoder_Estimates_msg_t feedback;
    bool got_feedback = false;
};

// ---- CAN statistics ----
struct CANStats {
    uint32_t totalMessages = 0;
    uint32_t heartbeatCount = 0;
    uint32_t feedbackCount = 0;
    uint32_t unknownCount = 0;
    uint32_t errorCount = 0;

    // Rate tracking
    uint32_t lastHeartbeatCount = 0;
    uint32_t lastFeedbackCount = 0;
    unsigned long lastRateCheck = 0;

    // Health tracking
    unsigned long lastHealthCheck = 0;
    unsigned long lastStatsReport = 0;
};

/* ==================================================================
 * 5. GLOBALS — grouped by subsystem
 * ================================================================== */

// ---- CAN / ODrive objects ----
bool twaiStarted = false;
ESP32TWAIIntf canIntf;
ODriveCAN odrv0(wrap_can_intf(canIntf), ODRV0_NODE_ID);
ODriveCAN odrv1(wrap_can_intf(canIntf), ODRV1_NODE_ID);
ODriveUserData odrv0_data;
ODriveUserData odrv1_data;
CANStats canStats;

// ---- Node 0 safety limit ----
float node0_initial_pos = 0.0f;
float node1_initial_pos = 0.0f;
bool node0_pos_initialized = false;
bool node1_pos_initialized = false;
bool node0_limit_tripped = false;
unsigned long node0_lastTripWarn = 0;

// ---- Mirroring ----
bool mirroringEnabled = false;

// ---- Scheduler ----
static Controller*    g_active           = nullptr;
static PlantState     g_state;
static unsigned long  g_lastStateTick    = 0;
static unsigned long  g_lastValidCommand = 0;
static bool           g_watchdogTripped  = false;

// ---- Controller logging ----
static bool          g_ctrl_log_enabled = false;
static unsigned long g_ctrl_lastLog     = 0;
static uint32_t      g_ctrl_sampleCount = 0;

// ---- SysID state machine ----
static SysIdPhase    sidNextPhase   = SID_IDLE;
static SysIdPhase    sidPhase       = SID_IDLE;
static unsigned long sidPhaseStart  = 0;
static unsigned long sidLastSample  = 0;
static float         sidCmdMotor    = 0.0f;
static bool          sidClipped     = false;
static uint32_t      sidSampleCount = 0;
static unsigned long sidZeroStart   = 0;

// ---- Iq measurement (node 0) ----
static float odrv0_iq_measured = 0.0f;
static unsigned long lastIqRequest = 0;

/* ==================================================================
 * 6. UTILITIES — serial output + unit conversions
 * ================================================================== */

// ---- Serial output helpers ----
void serialDebug(const char* msg) {
    unsigned long now = millis();
    Serial.print("[DEBUG ");
    Serial.print(now);
    Serial.print(" ms] ");
    Serial.println(msg);
}

void serialStatus(const char* msg) {
    Serial.print("[STATUS] ");
    Serial.println(msg);
}

void serialFeedback(const char* msg) {
    Serial.print("[FEEDBACK] ");
    Serial.println(msg);
}

void serialStats(const char* msg) {
    Serial.print("[STATS] ");
    Serial.println(msg);
}

// ---- Unit conversions (output ↔ motor, SI ↔ ODrive) ----
// Node 0: motor turns ↔ output turns
inline float motorToOutput(float motorPos)  { return motorPos / NODE0_GEAR_RATIO; }
inline float outputToMotor(float outputPos) { return outputPos * NODE0_GEAR_RATIO; }
static inline float outToMot(float out)     { return out * NODE0_GEAR_RATIO; }

// Node 0: motor turns ↔ output rad
inline float n0_motorTurns_to_outputRad(float t)   { return (t / NODE0_GEAR_RATIO) * TWO_PI_F; }
inline float n0_motorTps_to_outputRadps(float tps) { return (tps / NODE0_GEAR_RATIO) * TWO_PI_F; }
inline float n0_outputRad_to_motorTurns(float r)   { return (r / TWO_PI_F) * NODE0_GEAR_RATIO; }
inline float n0_outputRadps_to_motorTps(float rps) { return (rps / TWO_PI_F) * NODE0_GEAR_RATIO; }

// Node 1: motor turns ↔ output rad
inline float n1_motorTurns_to_outputRad(float t)   { return (t / NODE1_GEAR_RATIO) * TWO_PI_F; }
inline float n1_motorTps_to_outputRadps(float tps) { return (tps / NODE1_GEAR_RATIO) * TWO_PI_F; }

// Torque: output N·m ↔ motor N·m (ODrive setTorque expects motor N·m)
inline float n0_outputNm_to_motorNm(float tau_out) { return tau_out / NODE0_GEAR_RATIO; }
// Iq (A) at motor → output N·m
inline float n0_iq_to_outputNm(float iq) { return iq * MOTOR0_KT * NODE0_GEAR_RATIO; }

/* ==================================================================
 * 7. CAN LAYER — ODrive callbacks, health, pump, init
 * ================================================================== */

// ---- ODrive callbacks ----
void onHeartbeat(Heartbeat_msg_t& msg, void* user_data) {
    ODriveUserData* data = static_cast<ODriveUserData*>(user_data);
    data->heartbeat = msg;
    data->got_heartbeat = true;

    // Determine which node this is
    const char* nodeName = (user_data == &odrv0_data) ? "Node0" : "Node1";

    char buf[128];
    snprintf(buf, sizeof(buf), "%s Heartbeat: AxisErr=%lu State=%lu Result=%lu TrajDone=%d",
             nodeName, msg.Axis_Error, msg.Axis_State, 
             msg.Procedure_Result, msg.Trajectory_Done_Flag);
    serialDebug(buf);
}

void onCanMessage(const CanMsg& msg) {
    onReceive(msg, odrv0);
    onReceive(msg, odrv1);
}

void onFeedback(Get_Encoder_Estimates_msg_t& msg, void* user_data) {
    ODriveUserData* data = static_cast<ODriveUserData*>(user_data);
    data->feedback = msg;
    data->got_feedback = true;

    // Node 0 position safety limit
        if (user_data == &odrv0_data) {
            float pos = msg.Pos_Estimate;

            // Trip detection
            if (!node0_limit_tripped &&
                (pos >= NODE0_POS_LIMIT || pos <= -NODE0_POS_LIMIT)) {
                odrv0.setState(ODriveAxisState::AXIS_STATE_IDLE);
                mirroringEnabled = false;
                node0_limit_tripped = true;
                node0_lastTripWarn = millis();   // start reminder timer fresh

            char buf[200];
            snprintf(buf, sizeof(buf),
                    "🛑 NODE 0 LIMIT TRIPPED 🛑 output=%.3f turns (motor=%.3f) exceeded ±%.1f output - MOTOR IDLE",
                    motorToOutput(pos), pos, NODE0_OUTPUT_LIMIT);
            serialDebug(buf);
            Serial.println("================================================");
            Serial.println("⚠️  NODE 0 SAFETY LIMIT TRIPPED");
            Serial.println("    Output shaft exceeded ±1.0 turns (8:1 ratio)");
            Serial.println("    Motor set to IDLE for safety");
            Serial.println("    Mirroring auto-disabled");
            Serial.println("    Move motor back into range, then send RESET_LIMIT");
            Serial.println("    Send LIMIT_STATUS to query state");
            Serial.println("================================================");
            serialStatus("N0_LIMIT_TRIP");
            }

            // Periodic reminder while tripped (every 3s)
            if (node0_limit_tripped && (millis() - node0_lastTripWarn > 3000)) {
                char buf[200];
                snprintf(buf, sizeof(buf),
                        "⚠️  REMINDER: Node 0 tripped (output=%.3f turns, motor=%.3f). Send RESET_LIMIT after recovery.",
                        motorToOutput(pos), pos);
                serialDebug(buf);
                node0_lastTripWarn = millis();
            }
        }
    // Node 0 mirrors Node 1's position (only if enabled and not tripped)
    if (user_data == &odrv1_data && mirroringEnabled && !node0_limit_tripped) {
        static unsigned long lastMirror = 0;
        if (millis() - lastMirror >= 10) {  // Mirror at 100Hz
            float mirrorPos = msg.Pos_Estimate;
            // Clamp to safety range so we never command past the limit
            if (mirrorPos >  NODE0_POS_LIMIT) mirrorPos =  NODE0_POS_LIMIT;
            if (mirrorPos < -NODE0_POS_LIMIT) mirrorPos = -NODE0_POS_LIMIT;
            odrv0.setPosition(mirrorPos, 0.0f);
            lastMirror = millis();
        }
    }

    static unsigned long lastPublish = 0;
    if (millis() - lastPublish < 100) return;
    lastPublish = millis();

    char buf[256];
    snprintf(buf, sizeof(buf),
            "Node0: out=%.3f turns (motor=%.3f, vel=%.3f, iq=%.4fA) | Node1: pos=%.3f, vel=%.3f",
            motorToOutput(odrv0_data.feedback.Pos_Estimate),
            odrv0_data.feedback.Pos_Estimate,
            odrv0_data.feedback.Vel_Estimate,
            odrv0_iq_measured,
            odrv1_data.feedback.Pos_Estimate,
            odrv1_data.feedback.Vel_Estimate);
    serialFeedback(buf);
}

// ---- CAN health monitoring ----
void checkCANHealth() {
    unsigned long now = millis();
    if (now - canStats.lastHealthCheck < 2000) return;
    canStats.lastHealthCheck = now;

    twai_status_info_t status;
    if (twai_get_status_info(&status) != ESP_OK) {
        serialDebug("ERROR: Failed to get TWAI status");
        return;
    }

    if (status.state == TWAI_STATE_STOPPED) {
        serialDebug("Restarting CAN...");
        twai_start();
    }

    char buf[200];
    snprintf(buf, sizeof(buf), 
             "CAN: State=%d TxErr=%lu RxErr=%lu BusErr=%lu ArbitLost=%lu RxQ=%lu TxQ=%lu",
             status.state, status.tx_error_counter, status.rx_error_counter,
             status.bus_error_count, status.arb_lost_count, 
             status.msgs_to_rx, status.msgs_to_tx);
    serialDebug(buf);

    // Check for problems
    if (status.state == TWAI_STATE_BUS_OFF) {
        serialStatus("ERROR: CAN BUS-OFF state!");
        serialDebug("Attempting bus recovery...");
        twai_initiate_recovery();
    } else if (status.state == TWAI_STATE_RECOVERING) {
        serialStatus("WARNING: CAN recovering from error");
    }

    if (status.tx_error_counter >= 96 || status.rx_error_counter >= 96) {
        serialStatus("WARNING: High CAN error counters");
    }

    if (status.bus_error_count > 0) {
        char buf2[64];
        snprintf(buf2, sizeof(buf2), "WARNING: %lu bus errors detected", status.bus_error_count);
        serialDebug(buf2);
    }
}

void testCANTransmission() {
    twai_message_t tx_msg = {
        .identifier = 0x123,
        .data_length_code = 8,
        .data = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08}
    };

    esp_err_t result = twai_transmit(&tx_msg, pdMS_TO_TICKS(1000));
    char buf[128];

    if (result == ESP_OK) {
        snprintf(buf, sizeof(buf), "✅ TX Success - CAN message transmitted");
    } else if (result == ESP_ERR_TIMEOUT) {
        snprintf(buf, sizeof(buf), "❌ TX Timeout - No ACK from bus");
    } else {
        snprintf(buf, sizeof(buf), "❌ TX Failed - Error: 0x%x", result);
    }
    serialDebug(buf);

    twai_status_info_t status;
    twai_get_status_info(&status);
    snprintf(buf, sizeof(buf), "TX: State=%d TxErr=%lu RxErr=%lu", 
             status.state, status.tx_error_counter, status.rx_error_counter);
    serialDebug(buf);
}

// ---- Message rate validation ----
void validateMessageRates() {
    unsigned long now = millis();
    if (now - canStats.lastRateCheck < 1000) return;
    canStats.lastRateCheck = now;

    uint32_t hbRate = canStats.heartbeatCount - canStats.lastHeartbeatCount;
    uint32_t fbRate = canStats.feedbackCount - canStats.lastFeedbackCount;

    char buf[128];
    snprintf(buf, sizeof(buf), 
             "Msg Rates: Heartbeat=%lu/s (expect~20) Feedback=%lu/s (expect~200)",
             hbRate, fbRate);
    serialDebug(buf);

    // Alert if rates are abnormal
    if (canStats.totalMessages > 0) {
        if (hbRate < 10 || hbRate > 30) {
            serialStatus("WARNING: Heartbeat rate abnormal");
        }
        if (fbRate < 100 || fbRate > 300) {
            serialStatus("WARNING: Feedback rate abnormal");
        }
    }

    canStats.lastHeartbeatCount = canStats.heartbeatCount;
    canStats.lastFeedbackCount = canStats.feedbackCount;
}

// ---- Statistics report ----
void publishStats() {
    unsigned long now = millis();
    if (now - canStats.lastStatsReport < 5000) return;
    canStats.lastStatsReport = now;

    char buf[160];
    snprintf(buf, sizeof(buf), 
             "Total=%lu HB=%lu FB=%lu Unknown=%lu Errors=%lu Uptime=%lus",
             canStats.totalMessages, canStats.heartbeatCount, canStats.feedbackCount,
             canStats.unknownCount, canStats.errorCount, now / 1000);
    serialStats(buf);
}

// ---- CAN message pump ----
void canPump() {
    if (!twaiStarted) return;

    // Request Iq at 20Hz — non-blocking pattern using interval timer
    // getCurrents() blocks up to 10ms so we call it infrequently
    unsigned long now_iq = millis();
    if (now_iq - lastIqRequest >= IQ_REQUEST_INTERVAL_MS) {
        lastIqRequest = now_iq;
        Get_Iq_msg_t iq_msg;
        if (odrv0.getCurrents(iq_msg, 2)) {
            odrv0_iq_measured = iq_msg.Iq_Measured;
        }
    }

    twai_message_t msg;
    while (twai_receive(&msg, pdMS_TO_TICKS(5)) == ESP_OK) {
        canStats.totalMessages++;

        // Decode message ID
        uint8_t node_id = (msg.identifier >> 5) & 0x3F;
        uint8_t cmd_id = msg.identifier & 0x1F;

        // Track specific message types
        if (msg.identifier == ((ODRV0_NODE_ID << 5) | 0x01)) {
            canStats.heartbeatCount++;
        } else if (msg.identifier == ((ODRV1_NODE_ID << 5) | 0x01)) {
            canStats.heartbeatCount++;
        } else if (msg.identifier == ((ODRV0_NODE_ID << 5) | 0x09)) {
            canStats.feedbackCount++;
        } else if (msg.identifier == ((ODRV1_NODE_ID << 5) | 0x09)) {
            canStats.feedbackCount++;
        } else {
            canStats.unknownCount++;
        }

        // Detailed logging for first 50 messages or unknown messages
        if (canStats.totalMessages <= 50 || canStats.unknownCount <= 10) {
            char buf[160];
            snprintf(buf, sizeof(buf), 
                     "CAN[%lu]: ID=0x%03X (N=%d,C=0x%02X) Len=%d Data=%02X %02X %02X %02X %02X %02X %02X %02X",
                     canStats.totalMessages, msg.identifier, node_id, cmd_id, msg.data_length_code,
                     msg.data[0], msg.data[1], msg.data[2], msg.data[3],
                     msg.data[4], msg.data[5], msg.data[6], msg.data[7]);
            serialDebug(buf);
        }

        // Process the message through ODrive library
        CanMsg can_msg = {.id = msg.identifier, .len = msg.data_length_code};
        memcpy(can_msg.buffer, msg.data, msg.data_length_code);
        onReceive(can_msg, odrv0);
        onReceive(can_msg, odrv1);
    }

    uint32_t alerts_triggered;
    if (twai_read_alerts(&alerts_triggered, 0) == ESP_OK && alerts_triggered) {
        if (alerts_triggered & TWAI_ALERT_TX_FAILED) {
            serialDebug("ALERT: TX FAILED (no ACK)");
        }
        if (alerts_triggered & TWAI_ALERT_BUS_ERROR) {
            serialDebug("ALERT: BUS ERROR (physical issue)");
        }
        if (alerts_triggered & TWAI_ALERT_ARB_LOST) {
            serialDebug("ALERT: Arbitration lost");
        }
        if (alerts_triggered & TWAI_ALERT_ERR_PASS) {
            serialDebug("ALERT: Error passive");
        }
        if (alerts_triggered & TWAI_ALERT_BUS_OFF) {
            serialDebug("ALERT: BUS OFF");
        }
    }

    // Periodic monitoring
    checkCANHealth();
    validateMessageRates();
    publishStats();
}

// ---- CAN init ----
bool canInit() {
    serialDebug("Initializing CAN...");

    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        (gpio_num_t)CAN_TX_PIN,
        (gpio_num_t)CAN_RX_PIN,
        TWAI_MODE_NORMAL
    );
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_125KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    esp_err_t err;

    // Install driver
    err = twai_driver_install(&g_config, &t_config, &f_config);
    if (err != ESP_OK) {
        char buf[64];
        snprintf(buf, sizeof(buf), "ERROR: CAN driver install failed: 0x%x", err);
        serialDebug(buf);
        return false;
    }
    serialDebug("CAN driver installed");

    uint32_t alerts = 
        TWAI_ALERT_TX_FAILED |
        TWAI_ALERT_ERR_PASS |
        TWAI_ALERT_BUS_ERROR |
        TWAI_ALERT_ARB_LOST |
        TWAI_ALERT_BUS_OFF |
        TWAI_ALERT_RECOVERY_IN_PROGRESS;
    twai_reconfigure_alerts(alerts, NULL);
    serialDebug("TWAI alerts enabled");

    // Start driver
    err = twai_start();
    if (err != ESP_OK) {
        char buf[64];
        snprintf(buf, sizeof(buf), "ERROR: CAN start failed: 0x%x", err);
        serialDebug(buf);
        twai_driver_uninstall();
        return false;
    }
    serialDebug("CAN start() called");

    delay(100);

    // Verify it's actually running
    twai_status_info_t status;
    if (twai_get_status_info(&status) == ESP_OK) {
        char buf[128];
        snprintf(buf, sizeof(buf), "CAN State after start: %d (1=RUNNING)", status.state);
        serialDebug(buf);

        if (status.state == TWAI_STATE_STOPPED) {
            serialDebug("ERROR: CAN stuck in STOPPED state");
            twai_stop();
            twai_driver_uninstall();
            return false;
        }
    }

    twaiStarted = true;
    serialDebug("✅ CAN initialized and operational");
    return true;
}

// ---- Heartbeat wait (blocking, at boot) ----
void waitForHeartbeat() {
    serialDebug("Waiting for ODrive heartbeat...");
    const unsigned long heartbeatTimeout = 10000;
    unsigned long attemptCount = 0;
    unsigned long lastTxTest = 0;

    while (true) {
        attemptCount++;
        unsigned long start = millis();
        odrv0_data.got_heartbeat = false;

        // Test transmission during wait
        if (millis() - lastTxTest > 2000) {
            testCANTransmission();
            lastTxTest = millis();
        }

        while (!odrv0_data.got_heartbeat && millis() - start < heartbeatTimeout) {
            canPump();
            processSerialCommands();  // Process commands while waiting

            if (millis() - lastTxTest > 2000) {
                testCANTransmission();
                lastTxTest = millis();
            }
            delay(10);
        }

        if (odrv0_data.got_heartbeat) {
            serialStatus("ODRIVE_CONNECTED");
            serialDebug("ODrive heartbeat received!");

            char buf[128];
            snprintf(buf, sizeof(buf), 
                     "Connection after %lu attempts, %lu total msgs",
                     attemptCount, canStats.totalMessages);
            serialDebug(buf);
            break;
        } else {
            char buf[128];
            snprintf(buf, sizeof(buf), 
                     "No heartbeat (attempt %lu) - Total msgs: %lu",
                     attemptCount, canStats.totalMessages);
            serialDebug(buf);

            if (canStats.totalMessages > 0) {
                serialDebug("HINT: Getting CAN messages but no heartbeat - check node_id!");
            }
        }
    }
}

// ---- Serial command reader ----
void processSerialCommands() {
    static char cmdBuffer[CMD_BUFFER_SIZE];
    static int cmdIndex = 0;

    while (Serial.available() > 0) {
        char c = Serial.read();

        // Handle newline - process command
        if (c == '\n' || c == '\r') {
            if (cmdIndex > 0) {
                cmdBuffer[cmdIndex] = '\0';  // Null terminate
                handleCommand(cmdBuffer);
                cmdIndex = 0;  // Reset buffer
            }
        }
        // Add to buffer
        else if (cmdIndex < CMD_BUFFER_SIZE - 1) {
            cmdBuffer[cmdIndex++] = c;
        }
        // Buffer overflow protection
        else {
            serialDebug("ERROR: Command buffer overflow");
            cmdIndex = 0;
        }
    }
}

/* ==================================================================
 * 8. SCHEDULER — controller dispatch + 200 Hz state assembly
 * ================================================================== */

static void scheduler_apply_odrive_mode(CmdMode m) {
    switch (m) {
        case CmdMode::TORQUE:
            odrv0.setControllerMode(ODriveControlMode::CONTROL_MODE_TORQUE_CONTROL,
                                    ODriveInputMode::INPUT_MODE_PASSTHROUGH);
            break;
        case CmdMode::VELOCITY:
            odrv0.setControllerMode(ODriveControlMode::CONTROL_MODE_VELOCITY_CONTROL,
                                    ODriveInputMode::INPUT_MODE_PASSTHROUGH);
            break;
        case CmdMode::POSITION:
            odrv0.setControllerMode(ODriveControlMode::CONTROL_MODE_POSITION_CONTROL,
                                    ODriveInputMode::INPUT_MODE_PASSTHROUGH);
            break;
        default: break;
    }
}

void scheduler_switch(Controller* next) {
    if (g_active == next) return;
    char buf[128];
    snprintf(buf, sizeof(buf), "Scheduler: %s → %s",
             g_active ? g_active->name() : "none",
             next ? next->name() : "none");
    serialDebug(buf);

    if (g_active) g_active->exit();
    g_active = next;
    if (g_active) {
        scheduler_apply_odrive_mode(g_active->requiredMode());
        delay(20);                       // give ODrive time to switch modes
        g_active->enter(g_state);
    }
    g_lastValidCommand = millis();       // reset watchdog on switch
    g_watchdogTripped  = false;
}

void scheduler_tick(float dt) {
    if (!g_active) return;
    if (!g_state.valid) return;
    if (node0_limit_tripped) return;     // safety guard
    if (sidPhase != SID_IDLE) return;    // SysID owns the motor

    // Watchdog: if no valid command for too long, IDLE for safety
    if (millis() - g_lastValidCommand > CTRL_WATCHDOG_MS) {
        if (!g_watchdogTripped) {
            g_watchdogTripped = true;
            odrv0.setState(ODriveAxisState::AXIS_STATE_IDLE);
            serialDebug("⚠️  Controller watchdog: no command in 100ms → Node 0 IDLE");
            serialStatus("CTRL_WATCHDOG");
        }
        return;
    }

    ControllerOutput out = g_active->update(g_state, dt);
    if (!out.valid || out.mode == CmdMode::NONE) return;

    switch (out.mode) {
        case CmdMode::TORQUE: {
            float tau_motor = n0_outputNm_to_motorNm(out.value);
            odrv0.setTorque(tau_motor);
            break;
        }
        case CmdMode::VELOCITY: {
            float vel_motor_tps = n0_outputRadps_to_motorTps(out.value);
            odrv0.setVelocity(vel_motor_tps);
            break;
        }
        case CmdMode::POSITION: {
            float pos_motor_turns = n0_outputRad_to_motorTurns(out.value);
            odrv0.setPosition(pos_motor_turns);
            break;
        }
        default: break;
    }
    g_lastValidCommand = millis();
}

// ---- State assembly (200 Hz) ----
void assembleState() {
    unsigned long now = millis();
    if (now - g_lastStateTick < STATE_TICK_MS) return;
    float dt = (now - g_lastStateTick) * 0.001f;
    g_lastStateTick = now;

    // Freshness check — both encoders must have reported
    bool fresh = odrv0_data.got_feedback && odrv1_data.got_feedback;

    g_state.q1        = n0_motorTurns_to_outputRad(odrv0_data.feedback.Pos_Estimate);
    g_state.qd1       = n0_motorTps_to_outputRadps(odrv0_data.feedback.Vel_Estimate);
    g_state.q2        = n1_motorTurns_to_outputRad(odrv1_data.feedback.Pos_Estimate);
    g_state.qd2       = n1_motorTps_to_outputRadps(odrv1_data.feedback.Vel_Estimate);
    g_state.tau1_meas = n0_iq_to_outputNm(odrv0_iq_measured);
    g_state.t_ms      = now;
    g_state.valid     = fresh;

    scheduler_tick(dt);
}

/* ==================================================================
 * 9. CONTROLLERS — SwingUpXin (energy-based swing-up, Xin's method)
 * ================================================================== */

class SwingUpXin : public Controller {
public:
    const char* name() const override { return "swingup_xin"; }
    CmdMode requiredMode() const override { return CmdMode::TORQUE; }

    // Last-computed diagnostics, exposed for logging
    float last_q1_xin   = 0.0f;
    float last_qd1_xin  = 0.0f;
    float last_q2_xin   = 0.0f;
    float last_qd2_xin  = 0.0f;
    float last_E_tilde  = 0.0f;
    float last_tau_xin  = 0.0f;
    float last_tau_real = 0.0f;
    bool  last_saturated = false;

    // ---- Startup kick state (persistent across update() calls) ----
    bool          _kick_active     = false;
    unsigned long _kick_start_ms   = 0;
    static constexpr unsigned long KICK_DURATION_MS = 550;   
    static constexpr float         KICK_TORQUE_NM   = 1.0f;  // output N·m
    static constexpr float         KICK_VEL_THRESH  = 6.0f;  // rad/s
    bool _kick_done = false; 
    void enter(const PlantState&) override {
        _kick_done = false;
        g_ctrl_sampleCount = 0;
        g_ctrl_lastLog     = 0;
        _kick_active   = false;
        _kick_start_ms = millis() + 2000;  // arm kick 2s from now
        Serial.println(CTRL_PREFIX
            "HEADER:t_ms,mode,q1,qd1,q2,qd2,E_tilde,tau_out,tau_motor,iq,n0_out");
        serialDebug("SwingUpXin: ENTER — 2s pre-log, then kick");
    }

    void resetDiagnostics() {
        last_q1_xin    = 0.0f;
        last_qd1_xin   = 0.0f;
        last_q2_xin    = 0.0f;
        last_qd2_xin   = 0.0f;
        last_E_tilde   = 0.0f;
        last_tau_xin   = 0.0f;
        last_tau_real  = 0.0f;
        last_saturated = false;
    }

    void exit() override {
        odrv0.setState(ODriveAxisState::AXIS_STATE_IDLE);
        resetDiagnostics();
        _kick_active = false;
        serialDebug("SwingUpXin: EXIT — Node 0 → IDLE, diagnostics cleared");
        serialStatus("CTRL_DONE");
    }

    ControllerOutput update(const PlantState& s, float /*dt*/) override {
        ControllerOutput out;
        const float PI_F = 3.14159265359f;
        const float g = 9.81f;
 

        // ---- Frame map: real → Xin ----
        float q1_xin  = - 0.5f*PI_F - s.q1;
        float qd1_xin = -s.qd1;
        float q2_xin  =  s.q1 - s.q2;
        float qd2_xin =  s.qd1 - s.qd2;

        // Wrap to [0, 2π]
        q1_xin = fmodf(q1_xin, 2.0f*PI_F);
        if (q1_xin < 0) q1_xin += 2.0f*PI_F;
        q2_xin = fmodf(q2_xin, 2.0f*PI_F);
        if (q2_xin < 0) q2_xin += 2.0f*PI_F;

        // ---- Pre-log window: log state, output zero torque ----
        if (!_kick_active && millis() < _kick_start_ms) {
            last_q1_xin = q1_xin; last_qd1_xin = qd1_xin;
            last_q2_xin = q2_xin; last_qd2_xin = qd2_xin;
            last_E_tilde = 0.0f; last_tau_real = 0.0f; last_saturated = false;

            if (g_ctrl_log_enabled) {
                unsigned long now = millis();
                if (now - g_ctrl_lastLog >= CTRL_RATE_MS) {
                    g_ctrl_lastLog = now;
                    g_ctrl_sampleCount++;
                    char buf[224];
                    snprintf(buf, sizeof(buf),
                        CTRL_PREFIX "%lu,1,%.4f,%.4f,%.4f,%.4f,%.5f,%.4f,%.4f,%.4f,%.4f",
                        now, q1_xin, qd1_xin, q2_xin, qd2_xin,
                        0.0f, 0.0f, 0.0f, odrv0_iq_measured, s.q1/(2.0f*PI_F));
                    Serial.println(buf);
                }
            }
            out.mode = CmdMode::TORQUE; out.value = 0.0f; out.valid = true;
            return out;
        }

        // Arm kick after pre-log delay
        if (!_kick_active && !_kick_done && millis() >= _kick_start_ms) {
            _kick_active   = true;
            _kick_start_ms = millis();
            serialDebug("SwingUpXin: pre-log done, kick armed");
        }

        // ---- Startup kick: only fires while at rest, for KICK_DURATION_MS ----
        if (_kick_active) {
            unsigned long elapsed = millis() - _kick_start_ms;
            if (elapsed < KICK_DURATION_MS && fabsf(qd1_xin) < KICK_VEL_THRESH) {
                float tau_real = -KICK_TORQUE_NM;

                last_q1_xin    = q1_xin;
                last_qd1_xin   = qd1_xin;
                last_q2_xin    = q2_xin;
                last_qd2_xin   = qd2_xin;
                last_E_tilde   = 0.0f;
                last_tau_xin   = KICK_TORQUE_NM;
                last_tau_real  = tau_real;
                last_saturated = false;

                if (g_ctrl_log_enabled) {
                    unsigned long now = millis();
                    if (now - g_ctrl_lastLog >= CTRL_RATE_MS) {
                        g_ctrl_lastLog = now;
                        g_ctrl_sampleCount++;
                        const float tau_motor    = n0_outputNm_to_motorNm(tau_real);
                        const float n0_out_turns = s.q1 / (2.0f * PI_F);
                        char buf[224];
                        snprintf(buf, sizeof(buf),
                            CTRL_PREFIX "%lu,1,%.4f,%.4f,%.4f,%.4f,%.5f,%.4f,%.4f,%.4f,%.4f",
                            now,
                            q1_xin, qd1_xin, q2_xin, qd2_xin,
                            0.0f,          // E_tilde not computed during kick
                            tau_real, tau_motor,
                            odrv0_iq_measured, n0_out_turns);
                        Serial.println(buf);
                    }
                }

                out.mode  = CmdMode::TORQUE;
                out.value = tau_real;
                out.valid = true;
                return out;
            }
            _kick_active = false;
            _kick_done   = true;
            serialDebug("SwingUpXin: kick complete, energy law engaged");
        }

        // ---- Dynamics terms ----
        const float c1  = cosf(q1_xin);
        const float s2  = sinf(q2_xin);
        const float c2  = cosf(q2_xin);
        const float c12 = cosf(q1_xin + q2_xin);

        const float M11 = SU_A1 + SU_A2 + 2.0f*SU_A3*c2;
        const float M12 = SU_A2 + SU_A3*c2;
        const float M22 = SU_A2;

        // Energy — NOTE: g_acc is in V and Er to match Xin's paper and the sim
        const float T = 0.5f*(M11*qd1_xin*qd1_xin
                              + 2.0f*M12*qd1_xin*qd2_xin
                              + M22*qd2_xin*qd2_xin);
        const float V = (SU_B1*sinf(q1_xin) + SU_B2*sinf(q1_xin + q2_xin)) * g;
        const float E = T + V;
        const float Er = (SU_TH4 + SU_TH5) * g;
        const float E_tilde = E - Er;

        // F(q,qd) per Xin Eq.(15)
        const float F_qqd =
              SU_TH2 * SU_TH3 * (qd1_xin + qd2_xin) * (qd1_xin + qd2_xin) * s2
            + SU_TH3 * SU_TH3 *  qd1_xin * qd1_xin * c2 * s2
            - SU_TH2 * SU_TH4 * c1 * g
            + SU_TH3 * SU_TH5 * g * c2 * c12;

        // D = θ1·θ2 − θ3²·cos²(q2)
        const float D = SU_TH1*SU_TH2 - SU_TH3*SU_TH3 * c2 * c2;

        // Control law (Xin Eq.14)
        const float num = -SU_kd * F_qqd  -  D * (qd1_xin + SU_kp * (q1_xin - 0.5f * PI_F));
        float       den =  D * SU_kv * E_tilde  +  SU_kd * SU_TH2;

        // Sign-preserving denominator floor (matches the sim)
        const float DEN_FLOOR = 1e-10f;
        if (fabsf(den) < DEN_FLOOR) {
            den = (den >= 0.0f ? DEN_FLOOR : -DEN_FLOOR);
        }

        float tau_xin  = num / den;
        float tau_real = -tau_xin;

        // Saturation
        bool saturated = false;
        if (tau_real >  SU_tau_max) { tau_real =  SU_tau_max; saturated = true; }
        if (tau_real < -SU_tau_max) { tau_real = -SU_tau_max; saturated = true; }

        // Diagnostics
        last_q1_xin    = q1_xin;
        last_qd1_xin   = qd1_xin;
        last_q2_xin    = q2_xin;
        last_qd2_xin   = qd2_xin;
        last_E_tilde   = E_tilde;
        last_tau_xin   = tau_xin;
        last_tau_real  = tau_real;
        last_saturated = saturated;

        // Periodic log
        if (g_ctrl_log_enabled) {
            unsigned long now = millis();
            if (now - g_ctrl_lastLog >= CTRL_RATE_MS) {
                g_ctrl_lastLog = now;
                g_ctrl_sampleCount++;
                const float tau_motor = n0_outputNm_to_motorNm(tau_real);
                const float n0_out_turns = s.q1 / (2.0f * PI_F);
                char buf[224];
                snprintf(buf, sizeof(buf),
                    CTRL_PREFIX "%lu,1,%.4f,%.4f,%.4f,%.4f,%.5f,%.4f,%.4f,%.4f,%.4f",
                    now,
                    q1_xin, qd1_xin, q2_xin, qd2_xin,
                    E_tilde, tau_real, tau_motor,
                    odrv0_iq_measured, n0_out_turns);
                Serial.println(buf);
            }
        }

        out.mode  = CmdMode::TORQUE;
        out.value = tau_real;
        out.valid = true;
        return out;
    }
};

SwingUpXin g_swingup;

/* ==================================================================
 * 10. SYSID — Phase 3 Fourier trajectory (inertial identification)
 * ================================================================== */

// ----------------------------------------------------------------
// Scan full Fourier trajectory to find peak output position
// ----------------------------------------------------------------
static float computeFourierPeak() {
    float peak = 0.0f;
    const float dt_scan = 0.02f;   // 20 ms resolution
    for (float t = 0.0f; t < FOURIER_DURATION_S; t += dt_scan) {
        float pos = 0.0f;
        for (int k = 1; k <= FOURIER_NH; k++) {
            float kwf = (float)k * FOURIER_WF;
            pos += (FA[k-1] / kwf) * sinf(kwf * t)
                 - (FB[k-1] / kwf) * cosf(kwf * t);
        }
        pos *= FOURIER_AMP_OUT;
        if (fabsf(pos) > peak) peak = fabsf(pos);
    }
    return peak;
}

// ----------------------------------------------------------------
// Stop / command / log helpers
// ----------------------------------------------------------------
static void sidStop(const char* reason) {
    odrv0.setState(ODriveAxisState::AXIS_STATE_IDLE);
    sidPhase    = SID_IDLE;
    sidCmdMotor = 0.0f;
    sidClipped  = false;
    char buf[128];
    snprintf(buf, sizeof(buf), "SYSID STOPPED: %s  samples=%lu", reason, sidSampleCount);
    serialDebug(buf);
    serialStatus("SYSID_DONE");
}

// Clamp every command; set sidClipped if clamping occurred.
// sidClipped is written into the CSV so MATLAB can discard those windows.
static bool sidSetPos(float outTurns) {
    if (node0_limit_tripped) return false;

    float clamped = outTurns;
    bool  clipping = false;

    if      (clamped >  SYSID_N0_OUT_LIMIT) { clamped =  SYSID_N0_OUT_LIMIT; clipping = true; }
    else if (clamped < -SYSID_N0_OUT_LIMIT) { clamped = -SYSID_N0_OUT_LIMIT; clipping = true; }

    if (clipping) {
        sidClipped = true;
        char buf[96];
        snprintf(buf, sizeof(buf),
                 "SYSID WARN: cmd=%.4f clipped to ±%.4f output turns — sample flagged",
                 outTurns, SYSID_N0_OUT_LIMIT);
        serialDebug(buf);
    }

    sidCmdMotor = outToMot(clamped);
    odrv0.setPosition(sidCmdMotor);
    return !clipping;
}

// Log one CSV row — column 8 'clipped' flags data quality issues
static void sidLogSample(uint8_t phase) {
    float n0_out = motorToOutput(odrv0_data.feedback.Pos_Estimate);
    float n0_vel = odrv0_data.feedback.Vel_Estimate;
    float n1_pos = odrv1_data.feedback.Pos_Estimate;
    float n1_vel = odrv1_data.feedback.Vel_Estimate;

    char buf[192];
    snprintf(buf, sizeof(buf),
            SYSID_PREFIX "%lu,%u,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%d",
            millis(), phase,
            n0_out, n0_vel,
            n1_pos, n1_vel,
            sidCmdMotor / NODE0_GEAR_RATIO,
            odrv0_iq_measured,           
            sidClipped ? 1 : 0);
    Serial.println(buf);
    sidSampleCount++;
    sidClipped = false;   // reset per-tick
}

// ----------------------------------------------------------------
// Zeroing state tick
// ----------------------------------------------------------------
// Commands zero and polls encoder until within SYSID_ZERO_TOL.
// Transitions to sidNextPhase on success; aborts on timeout.
static void sidTickZeroing() {
    unsigned long now = millis();

    odrv0.setPosition(0.0f);
    sidCmdMotor = 0.0f;

    float pos = motorToOutput(odrv0_data.feedback.Pos_Estimate);

    if (fabsf(pos) <= SYSID_ZERO_TOL) {
        char buf[80];
        snprintf(buf, sizeof(buf),
                 "SYSID: Zero reached (%.4f turns) — transitioning to phase %d",
                 pos, (int)sidNextPhase);
        serialDebug(buf);

        sidPhaseStart = millis();
        sidLastSample = 0;
        sidPhase      = sidNextPhase;

        // Emit CSV header only now — data starts immediately after
        Serial.println(SYSID_PREFIX
            "HEADER:t_ms,phase,n0_pos_out,n0_vel_motor,n1_pos,n1_vel,cmd_out,iq_measured,clipped");
        return;
    }

    if (now - sidZeroStart > SYSID_ZERO_TIMEOUT_MS) {
        char buf[128];
        snprintf(buf, sizeof(buf),
                 "SYSID ABORT: Zero not reached in %lu ms (pos=%.4f turns) — check motor/encoders",
                 SYSID_ZERO_TIMEOUT_MS, pos);
        serialDebug(buf);
        serialStatus("SYSID_ZERO_TIMEOUT");
        sidStop("zero timeout");
    }
}

// ----------------------------------------------------------------
// Phase 3 tick — Fourier trajectory (inertial)
// ----------------------------------------------------------------
static void sidTickPhase3() {
    unsigned long now = millis();
    float         t_s = (now - sidPhaseStart) / 1000.0f;

    if (t_s >= FOURIER_DURATION_S) {
        sidStop("Phase 3 complete");
        return;
    }

    float pos_out = 0.0f;
    for (int k = 1; k <= FOURIER_NH; k++) {
        float kwf  = (float)k * FOURIER_WF;
        pos_out   += (FA[k-1] / kwf) * sinf(kwf * t_s)
                   - (FB[k-1] / kwf) * cosf(kwf * t_s);
    }
    pos_out *= FOURIER_AMP_OUT;

    // Peak verified at startup (FIX 1), zero confirmed before start (FIX 2),
    // clamp+flag still active as last-resort guard (FIX 3)
    sidSetPos(pos_out);

    if (now - sidLastSample >= SYSID_RATE_MS) {
        sidLastSample = now;
        sidLogSample(3);
    }
}

// ----------------------------------------------------------------
// Public API — call from loop()
// ----------------------------------------------------------------
void sysidTick() {
    switch (sidPhase) {
        case SID_ZEROING: sidTickZeroing(); break;
        case SID_PHASE3:  sidTickPhase3();  break;
        default: break;
    }
}

// ----------------------------------------------------------------
// Public API — call from handleCommand() for "SYSID*" commands
// ----------------------------------------------------------------
void handleSysIdCommand(const char* cmd) {

    // ---- Status ----
    if (strcmp(cmd, "SYSID_STATUS") == 0) {
        const char* names[] = {"IDLE","PHASE1","PHASE2","PHASE3","ZEROING"};
        float pos = motorToOutput(odrv0_data.feedback.Pos_Estimate);
        char buf[160];
        snprintf(buf, sizeof(buf),
                 "SysId state=%s samples=%lu limitTripped=%d n0_pos=%.4f turns",
                 names[sidPhase], sidSampleCount, node0_limit_tripped, pos);
        serialDebug(buf);
        serialStatus("SYSID_STATUS_OK");
        return;
    }

    // ---- Stop ----
    if (strcmp(cmd, "SYSID_STOP") == 0) {
        sidStop("user abort");
        return;
    }

    // ---- Common safety gate ----
    if (node0_limit_tripped) {
        serialDebug("SYSID: Node 0 limit tripped — send RESET_LIMIT first");
        serialStatus("N0_LIMIT_BLOCKED");
        return;
    }
    if (sidPhase != SID_IDLE) {
        serialDebug("SYSID: Already running — send SYSID_STOP first");
        serialStatus("SYSID_BUSY");
        return;
    }

    // ================================================================
    // PHASE 3 — FIX 1: Fourier peak scan | FIX 2: zero first
    // ================================================================
    if (strcmp(cmd, "SYSID_PHASE3") == 0) {

        // FIX 1: scan full trajectory for peak excursion
        serialDebug("SYSID Phase 3: scanning Fourier trajectory peak...");
        float peak = computeFourierPeak();

        char buf[128];
        snprintf(buf, sizeof(buf),
                 "SYSID Phase 3: Fourier peak=%.4f  limit=%.4f",
                 peak, SYSID_N0_OUT_LIMIT);
        serialDebug(buf);

        if (peak > SYSID_N0_OUT_LIMIT) {
            snprintf(buf, sizeof(buf),
                     "SYSID ABORT: peak=%.4f > limit=%.4f "
                     "— reduce FOURIER_AMP_OUT or re-optimise FA/FB coefficients",
                     peak, SYSID_N0_OUT_LIMIT);
            serialDebug(buf);
            serialStatus("SYSID_LIMIT_VIOLATION");
            return;
        }

        sidSampleCount = 0;
        sidLastSample  = 0;

        if (odrv0_data.heartbeat.Axis_State !=
            (uint32_t)ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL) {
            serialDebug("SYSID ABORT: Arm Node 0 first — send LOOP then POS_MODE");
            serialStatus("SYSID_NOT_ARMED");
            return;
        }
        delay(200);

        // FIX 2: zero first
        sidNextPhase = SID_PHASE3;
        sidPhase     = SID_ZEROING;
        sidZeroStart = millis();

        serialDebug("SYSID Phase 3: zeroing before Fourier trajectory...");
        serialStatus("SYSID_ZEROING");
        return;
    }

    serialDebug("Unknown SYSID command. Options: SYSID_PHASE3  SYSID_STOP  SYSID_STATUS");
}

/* ==================================================================
 * 11. COMMANDS — serial command handler
 * ================================================================== */

void handleCommand(char* cmd) {
    char buf[256];
    snprintf(buf, sizeof(buf), "📥 RX Command: '%s'", cmd);
    serialDebug(buf);
    
    // ========== NODE 0 COMMANDS ==========
    if (strcmp(cmd, "LOOP") == 0) {
        if (node0_limit_tripped) {
            serialDebug("⚠️  Node 0 limit is TRIPPED - send CLEAR or RESET_LIMIT first");
            serialStatus("N0_LIMIT_BLOCKED");
            return;
        }
        serialDebug("Node 0: Executing LOOP command");
        odrv0.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
        serialStatus("N0_LOOP_ON");
    }
    else if (strcmp(cmd, "IDLE") == 0) {
        serialDebug("Node 0: Executing IDLE command");
        odrv0.setState(ODriveAxisState::AXIS_STATE_IDLE);
        serialStatus("N0_IDLE_ON");
    }

    else if (strncmp(cmd, "POS ", 4) == 0) {
        if (node0_limit_tripped) {
            float p = atof(cmd + 4);
            snprintf(buf, sizeof(buf),
                    "⚠️  Node 0 limit TRIPPED - POS %.3f (output) IGNORED. Send RESET_LIMIT to clear.", p);
            serialDebug(buf);
            serialStatus("N0_LIMIT_BLOCKED");
            return;
        }
        float outputPos = atof(cmd + 4);
        // Reject commands clearly outside the safety range up front
        if (outputPos > NODE0_OUTPUT_LIMIT || outputPos < -NODE0_OUTPUT_LIMIT) {
            snprintf(buf, sizeof(buf),
                    "⚠️  Node 0: POS %.3f (output) out of range [±%.1f] - IGNORED",
                    outputPos, NODE0_OUTPUT_LIMIT);
            serialDebug(buf);
            serialStatus("N0_POS_OUT_OF_RANGE");
            return;
        }
        float motorPos = outputToMotor(outputPos);
        snprintf(buf, sizeof(buf),
                "Node 0: POS output=%.3f turns -> motor=%.3f turns",
                outputPos, motorPos);
        serialDebug(buf);
        odrv0.setPosition(motorPos);
        serialStatus("N0_POS_OK");
    }
    else if (strncmp(cmd, "VEL ", 4) == 0) {
        if (node0_limit_tripped) {
            serialDebug("⚠️  Node 0 limit TRIPPED - VEL command IGNORED");
            serialStatus("N0_LIMIT_BLOCKED");
            return;
        }
        float vel = atof(cmd + 4);
        snprintf(buf, sizeof(buf), "Node 0: Executing VEL: %.3f", vel);
        serialDebug(buf);
        odrv0.setVelocity(vel);
        serialStatus("N0_VEL_OK");
    }
    else if (strcmp(cmd, "CLEAR") == 0) {
        serialDebug("Node 0: Executing CLEAR command");
        odrv0.clearErrors();
        if (node0_limit_tripped) {
            serialDebug("ℹ️  Note: safety limit still tripped - send RESET_LIMIT to re-arm");
        }
        serialStatus("N0_CLEARED");
    }
    else if (strcmp(cmd, "POS_MODE") == 0) {
        serialDebug("Node 0: Setting Position Control Mode");
        odrv0.setControllerMode(ODriveControlMode::CONTROL_MODE_POSITION_CONTROL,
                                ODriveInputMode::INPUT_MODE_PASSTHROUGH);
        serialStatus("N0_POS_MODE_SET");
    }
    else if (strcmp(cmd, "TORQUE_MODE") == 0) {
        serialDebug("Node 0: Setting Torque Control Mode");
        odrv0.setControllerMode(ODriveControlMode::CONTROL_MODE_TORQUE_CONTROL,
                                ODriveInputMode::INPUT_MODE_PASSTHROUGH);
        serialStatus("N0_TORQUE_MODE_SET");
    }

    else if (strncmp(cmd, "TORQUE ", 7) == 0) {
        if (node0_limit_tripped) {
            serialDebug("⚠️  Node 0 limit TRIPPED - TORQUE command IGNORED");
            serialStatus("N0_LIMIT_BLOCKED");
            return;
        }
        float tau_out = atof(cmd + 7);
        if (fabsf(tau_out) > SU_tau_max) {
            snprintf(buf, sizeof(buf),
                    "⚠️  TORQUE %.4f exceeds SU_tau_max=%.4f Nm (output) - IGNORED",
                    tau_out, SU_tau_max);
            serialDebug(buf);
            serialStatus("N0_TORQUE_OUT_OF_RANGE");
            return;
        }
        float tau_motor = n0_outputNm_to_motorNm(tau_out);
        snprintf(buf, sizeof(buf),
                "Node 0: TORQUE output=%.4f Nm -> motor=%.4f Nm",
                tau_out, tau_motor);
        serialDebug(buf);
        odrv0.setTorque(tau_motor);
        serialStatus("N0_TORQUE_OK");
    }

    // ========== NODE 1 COMMANDS ==========
    else if (strcmp(cmd, "N1_LOOP") == 0) {
        serialDebug("Node 1: Executing LOOP command");
        odrv1.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
        serialStatus("N1_LOOP_ON");
    }
    else if (strcmp(cmd, "N1_IDLE") == 0) {
        serialDebug("Node 1: Executing IDLE command");
        odrv1.setState(ODriveAxisState::AXIS_STATE_IDLE);
        serialStatus("N1_IDLE_ON");
    }
    else if (strncmp(cmd, "N1_POS ", 7) == 0) {
        float pos = atof(cmd + 7);
        snprintf(buf, sizeof(buf), "Node 1: Executing POS: %.3f", pos);
        serialDebug(buf);
        odrv1.setPosition(pos);
        serialStatus("N1_POS_OK");
    }
    else if (strncmp(cmd, "N1_VEL ", 7) == 0) {
        float vel = atof(cmd + 7);
        snprintf(buf, sizeof(buf), "Node 1: Executing VEL: %.3f", vel);
        serialDebug(buf);
        odrv1.setVelocity(vel);
        serialStatus("N1_VEL_OK");
    }
    else if (strcmp(cmd, "N1_CLEAR") == 0) {
        serialDebug("Node 1: Executing CLEAR command");
        odrv1.clearErrors();
        serialStatus("N1_CLEARED");
    }
    else if (strcmp(cmd, "N1_POS_MODE") == 0) {
        serialDebug("Node 1: Setting Position Control Mode");
        odrv1.setControllerMode(ODriveControlMode::CONTROL_MODE_POSITION_CONTROL,
                                ODriveInputMode::INPUT_MODE_PASSTHROUGH);
        serialStatus("N1_POS_MODE_SET");
    }
    
    // ========== BOTH NODES ==========
    else if (strcmp(cmd, "BOTH_LOOP") == 0) {
        if (node0_limit_tripped) {
            serialDebug("⚠️  Cannot run BOTH_LOOP - Node 0 limit tripped. Send RESET_LIMIT first.");
            serialStatus("N0_LIMIT_BLOCKED");
            return;
        }
        serialDebug("Both nodes: Executing LOOP command");
        odrv0.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
        odrv1.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
        serialStatus("BOTH_LOOP_ON");
    }
    else if (strcmp(cmd, "BOTH_IDLE") == 0) {
        serialDebug("Both nodes: Executing IDLE command");
        odrv0.setState(ODriveAxisState::AXIS_STATE_IDLE);
        odrv1.setState(ODriveAxisState::AXIS_STATE_IDLE);
        serialStatus("BOTH_IDLE_ON");
    }
    
    // ========== INDIVIDUAL NODE PRESETS ==========
    else if (strcmp(cmd, "N0_ONLY") == 0) {
        if (node0_limit_tripped) {
            serialDebug("⚠️  Cannot run N0_ONLY - Node 0 limit tripped. Send RESET_LIMIT first.");
            serialStatus("N0_LIMIT_BLOCKED");
            return;
        }
        serialDebug("Preset: Node 0 active, Node 1 idle");
        odrv0.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
        odrv1.setState(ODriveAxisState::AXIS_STATE_IDLE);
        serialStatus("N0_ONLY_ACTIVE");
    }
    else if (strcmp(cmd, "N1_ONLY") == 0) {
        serialDebug("Preset: Node 1 active, Node 0 idle");
        odrv1.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
        odrv0.setState(ODriveAxisState::AXIS_STATE_IDLE);
        serialStatus("N1_ONLY_ACTIVE");
    }

    // ========== SAFETY LIMIT CONTROL ==========
    else if (strcmp(cmd, "RESET_LIMIT") == 0) {
        float pos = odrv0_data.feedback.Pos_Estimate;
        if (pos >= NODE0_POS_LIMIT || pos <= -NODE0_POS_LIMIT) {
            snprintf(buf, sizeof(buf),
                    "❌ Cannot reset - Node 0 still out of range (output=%.3f turns, motor=%.3f). Move it back first.",
                    motorToOutput(pos), pos);
            serialDebug(buf);
            serialStatus("N0_LIMIT_BLOCKED");
            return;
        }
        if (node0_limit_tripped) {
            node0_limit_tripped = false;
            serialDebug("✅ Node 0 safety limit RESET - guard re-armed");
            serialStatus("N0_LIMIT_RESET");
        } else {
            serialDebug("ℹ️  Node 0 limit not tripped - nothing to reset");
            serialStatus("N0_LIMIT_OK");
        }
    }
    else if (strcmp(cmd, "LIMIT_STATUS") == 0) {
        float pos = odrv0_data.feedback.Pos_Estimate;
        if (node0_limit_tripped) {
            snprintf(buf, sizeof(buf),
                    "⚠️  Node 0 TRIPPED  output=%.3f turns (motor=%.3f, range=±%.1f output)",
                    motorToOutput(pos), pos, NODE0_OUTPUT_LIMIT);
            serialStatus("N0_LIMIT_TRIPPED");
        } else {
            snprintf(buf, sizeof(buf),
                    "✅ Node 0 OK  output=%.3f turns (motor=%.3f, range=±%.1f output)",
                    motorToOutput(pos), pos, NODE0_OUTPUT_LIMIT);
            serialStatus("N0_LIMIT_OK");
        }
        serialDebug(buf);
    }
    
    // ========== MIRRORING CONTROL ==========  
    else if (strcmp(cmd, "MIRROR_ON") == 0) {
        if (node0_limit_tripped) {
            serialDebug("⚠️  Cannot enable mirroring - Node 0 limit tripped. Send RESET_LIMIT first.");
            serialStatus("N0_LIMIT_BLOCKED");
            return;
        }
        mirroringEnabled = true;
        serialDebug("Mirroring enabled: Node 0 will follow Node 1");
        serialStatus("MIRROR_ENABLED");
    }
    else if (strcmp(cmd, "MIRROR_OFF") == 0) {
        mirroringEnabled = false;
        serialDebug("Mirroring disabled: Independent control");
        serialStatus("MIRROR_DISABLED");
    }

    // ========== CONTROLLER SCHEDULER ==========
    else if (strcmp(cmd, "CTRL_OFF") == 0) {
        scheduler_switch(nullptr);
        serialDebug("Controller scheduler: cleared");
        serialStatus("CTRL_OFF");
    }
    else if (strcmp(cmd, "CTRL_LOG_ON") == 0) {
        g_ctrl_log_enabled = true;
        serialDebug("CTRL logging enabled");
        serialStatus("CTRL_LOG_ON");
    }
    else if (strcmp(cmd, "CTRL_LOG_OFF") == 0) {
        g_ctrl_log_enabled = false;
        serialDebug("CTRL logging disabled");
        serialStatus("CTRL_LOG_OFF");
    }
    else if (strcmp(cmd, "CTRL_STATUS") == 0) {
        snprintf(buf, sizeof(buf),
                "Active controller: %s | state.valid=%d | q1=%.3f rad qd1=%.3f rad/s | q2=%.3f rad qd2=%.3f rad/s | tau1_meas=%.4f Nm",
                g_active ? g_active->name() : "none",
                g_state.valid,
                g_state.q1, g_state.qd1,
                g_state.q2, g_state.qd2,
                g_state.tau1_meas);
        serialDebug(buf);
        serialStatus("CTRL_STATUS_OK");
    }

    // ========== SWING-UP (Xin) ==========
    else if (strcmp(cmd, "SWINGUP") == 0) {
        if (node0_limit_tripped) {
            serialDebug("⚠️  Cannot start SwingUp — Node 0 limit tripped. Send RESET_LIMIT.");
            serialStatus("N0_LIMIT_BLOCKED");
            return;
        }
        if (sidPhase != SID_IDLE) {
            serialDebug("⚠️  Cannot start SwingUp — SysID is running. Send SYSID_STOP.");
            serialStatus("SYSID_BUSY");
            return;
        }

        odrv0.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
        g_ctrl_log_enabled = true;

        // Emit header NOW so Python opens the file immediately
        Serial.println(CTRL_PREFIX
            "HEADER:t_ms,mode,q1,qd1,q2,qd2,E_tilde,tau_out,tau_motor,iq,n0_out");

        serialDebug("SwingUpXin: logging started — waiting before engage...");
        serialStatus("SWINGUP_ON");

        // Non-blocking pre-log: keep pumping CAN and serial
        unsigned long waitStart = millis();
        while (millis() - waitStart < 1000) {
            canPump();
            processSerialCommands();
            assembleState();
            delay(5);
        }

        scheduler_switch(&g_swingup);
        serialDebug("SwingUpXin engaged — send CTRL_OFF to stop");
    }
    else if (strcmp(cmd, "SU_STATUS") == 0) {
        const bool stale = (g_active != &g_swingup);
        snprintf(buf, sizeof(buf),
            "SwingUp%s gains: kp=%.5f kd=%.5f kv=%.5f  tau_max=%.3f Nm  | "
            "last: q1_xin=%.3f qd1_xin=%.3f q2_xin=%.3f qd2_xin=%.3f E~=%.4f tau=%.3f sat=%d",
            stale ? " [STALE - not active]" : "",
            SU_kp, SU_kd, SU_kv, SU_tau_max,
            g_swingup.last_q1_xin, g_swingup.last_qd1_xin,
            g_swingup.last_q2_xin, g_swingup.last_qd2_xin,
            g_swingup.last_E_tilde, g_swingup.last_tau_real,
            (int)g_swingup.last_saturated);
        serialDebug(buf);
        serialStatus("SU_STATUS_OK");
    }
    else if (strncmp(cmd, "SU_KP ", 6) == 0) {
        SU_kp = atof(cmd + 6);
        snprintf(buf, sizeof(buf), "SwingUp kp = %.6f", SU_kp);
        serialDebug(buf);
        serialStatus("SU_KP_SET");
    }
    else if (strncmp(cmd, "SU_KD ", 6) == 0) {
        SU_kd = atof(cmd + 6);
        snprintf(buf, sizeof(buf), "SwingUp kd = %.6f", SU_kd);
        serialDebug(buf);
        serialStatus("SU_KD_SET");
    }
    else if (strncmp(cmd, "SU_KV ", 6) == 0) {
        SU_kv = atof(cmd + 6);
        snprintf(buf, sizeof(buf), "SwingUp kv = %.6f", SU_kv);
        serialDebug(buf);
        serialStatus("SU_KV_SET");
    }
    else if (strncmp(cmd, "SU_TAU_MAX ", 11) == 0) {
        SU_tau_max = atof(cmd + 11);
        if (SU_tau_max < 0.0f) SU_tau_max = 0.0f;
        snprintf(buf, sizeof(buf), "SwingUp tau_max = %.4f Nm (output)", SU_tau_max);
        serialDebug(buf);
        serialStatus("SU_TAU_MAX_SET");
    }

    // ========== PARAMS DUMP (for Python JSON sidecar) ==========
    else if (strcmp(cmd, "CTRL_PARAMS") == 0) {
        char pbuf[160];
        Serial.println("[PARAMS] BEGIN");

        // --- Meta ---
        snprintf(pbuf, sizeof(pbuf), "[PARAMS] meta.firmware_build=%s %s", __DATE__, __TIME__);
        Serial.println(pbuf);
        snprintf(pbuf, sizeof(pbuf), "[PARAMS] meta.t_ms=%lu", millis());
        Serial.println(pbuf);
        snprintf(pbuf, sizeof(pbuf), "[PARAMS] meta.active_controller=%s",
                 g_active ? g_active->name() : "none");
        Serial.println(pbuf);

        // --- Node 0 mechanical ---
        snprintf(pbuf, sizeof(pbuf), "[PARAMS] node0.gear_ratio=%.6f", NODE0_GEAR_RATIO);
        Serial.println(pbuf);
        snprintf(pbuf, sizeof(pbuf), "[PARAMS] node0.output_limit_turns=%.6f", NODE0_OUTPUT_LIMIT);
        Serial.println(pbuf);
        snprintf(pbuf, sizeof(pbuf), "[PARAMS] node0.pos_limit_motor_turns=%.6f", NODE0_POS_LIMIT);
        Serial.println(pbuf);
        snprintf(pbuf, sizeof(pbuf), "[PARAMS] node0.motor_kt=%.6f", MOTOR0_KT);
        Serial.println(pbuf);

        // --- Node 1 mechanical ---
        snprintf(pbuf, sizeof(pbuf), "[PARAMS] node1.gear_ratio=%.6f", NODE1_GEAR_RATIO);
        Serial.println(pbuf);

        // --- Physical (SwingUp model) ---
        snprintf(pbuf, sizeof(pbuf), "[PARAMS] physical.m1=%.6f", SU_M1); Serial.println(pbuf);
        snprintf(pbuf, sizeof(pbuf), "[PARAMS] physical.m2=%.6f", SU_M2); Serial.println(pbuf);
        snprintf(pbuf, sizeof(pbuf), "[PARAMS] physical.l1=%.6f", SU_L1); Serial.println(pbuf);
        snprintf(pbuf, sizeof(pbuf), "[PARAMS] physical.l2=%.6f", SU_L2); Serial.println(pbuf);
        snprintf(pbuf, sizeof(pbuf), "[PARAMS] physical.d1=%.6f", SU_D1); Serial.println(pbuf);
        snprintf(pbuf, sizeof(pbuf), "[PARAMS] physical.d2=%.6f", SU_D2); Serial.println(pbuf);
        snprintf(pbuf, sizeof(pbuf), "[PARAMS] physical.I1zz=%.6e", SU_I1ZZ); Serial.println(pbuf);
        snprintf(pbuf, sizeof(pbuf), "[PARAMS] physical.I2zz=%.6e", SU_I2ZZ); Serial.println(pbuf);
        snprintf(pbuf, sizeof(pbuf), "[PARAMS] physical.g=%.6f", SU_G); Serial.println(pbuf);

        // --- Xin a/b form ---
        snprintf(pbuf, sizeof(pbuf), "[PARAMS] xin.a1=%.6e", SU_A1); Serial.println(pbuf);
        snprintf(pbuf, sizeof(pbuf), "[PARAMS] xin.a2=%.6e", SU_A2); Serial.println(pbuf);
        snprintf(pbuf, sizeof(pbuf), "[PARAMS] xin.a3=%.6e", SU_A3); Serial.println(pbuf);
        snprintf(pbuf, sizeof(pbuf), "[PARAMS] xin.b1=%.6e", SU_B1); Serial.println(pbuf);
        snprintf(pbuf, sizeof(pbuf), "[PARAMS] xin.b2=%.6e", SU_B2); Serial.println(pbuf);

        // --- Xin θ form ---
        snprintf(pbuf, sizeof(pbuf), "[PARAMS] xin.theta1=%.6e", SU_TH1); Serial.println(pbuf);
        snprintf(pbuf, sizeof(pbuf), "[PARAMS] xin.theta2=%.6e", SU_TH2); Serial.println(pbuf);
        snprintf(pbuf, sizeof(pbuf), "[PARAMS] xin.theta3=%.6e", SU_TH3); Serial.println(pbuf);
        snprintf(pbuf, sizeof(pbuf), "[PARAMS] xin.theta4=%.6e", SU_TH4); Serial.println(pbuf);
        snprintf(pbuf, sizeof(pbuf), "[PARAMS] xin.theta5=%.6e", SU_TH5); Serial.println(pbuf);

        // --- SwingUp gains (live, tunable) ---
        snprintf(pbuf, sizeof(pbuf), "[PARAMS] swingup.kp=%.6f", SU_kp); Serial.println(pbuf);
        snprintf(pbuf, sizeof(pbuf), "[PARAMS] swingup.kd=%.6f", SU_kd); Serial.println(pbuf);
        snprintf(pbuf, sizeof(pbuf), "[PARAMS] swingup.kv=%.6f", SU_kv); Serial.println(pbuf);
        snprintf(pbuf, sizeof(pbuf), "[PARAMS] swingup.tau_max_Nm=%.6f", SU_tau_max); Serial.println(pbuf);

        // --- Scheduler ---
        snprintf(pbuf, sizeof(pbuf), "[PARAMS] scheduler.state_rate_hz=%lu",
                 (unsigned long)(1000UL / STATE_TICK_MS));
        Serial.println(pbuf);
        snprintf(pbuf, sizeof(pbuf), "[PARAMS] scheduler.watchdog_ms=%lu",
                 (unsigned long)CTRL_WATCHDOG_MS);
        Serial.println(pbuf);
        snprintf(pbuf, sizeof(pbuf), "[PARAMS] scheduler.feedback_stale_ms=%lu",
                 (unsigned long)FEEDBACK_STALE_MS);
        Serial.println(pbuf);
        snprintf(pbuf, sizeof(pbuf), "[PARAMS] scheduler.ctrl_log_rate_hz=%d", CTRL_RATE_HZ);
        Serial.println(pbuf);

        Serial.println("[PARAMS] END");
        serialStatus("CTRL_PARAMS_OK");
    }
        
    // ========== HELP ==========
    else if (strcmp(cmd, "HELP") == 0) {
        Serial.println("\n=== Available Commands ===");
        Serial.println("Node 0:  LOOP, IDLE, POS <value>, VEL <value>, TORQUE <value>, CLEAR, POS_MODE, TORQUE_MODE");
        Serial.println("Node 1:  N1_LOOP, N1_IDLE, N1_POS <value>, N1_VEL <value>, N1_CLEAR, N1_POS_MODE");
        Serial.println("Both:    BOTH_LOOP, BOTH_IDLE");
        Serial.println("Presets: N0_ONLY, N1_ONLY");
        Serial.println("Mirror:  MIRROR_ON, MIRROR_OFF");
        Serial.println("Safety:  RESET_LIMIT, LIMIT_STATUS");
        Serial.println("Ctrl:    CTRL_OFF, CTRL_STATUS, CTRL_LOG_ON, CTRL_LOG_OFF");
        Serial.println("SwingUp: SWINGUP, SU_STATUS, SU_KP <v>, SU_KD <v>, SU_KV <v>, SU_TAU_MAX <v>");
        Serial.println("Params:  CTRL_PARAMS  (dump all firmware params as [PARAMS] block)");
        Serial.println("");
        Serial.println("Note: Node 0 has an 8:1 gearbox.");
        Serial.println("      POS commands and limits are in OUTPUT shaft turns.");
        Serial.println("      Output range: ±1.0 turns (= ±8 motor turns).");
        Serial.println("      Exceeding range trips an IDLE safety stop.");
        Serial.println("SysId:   SYSID_PHASE3, SYSID_STOP, SYSID_STATUS"); 
        Serial.println("=========================\n");
    }
    else if (strncmp(cmd, "SYSID", 5) == 0) { handleSysIdCommand(cmd); }  
    else {
        snprintf(buf, sizeof(buf), "❌ Unknown command: %s (type HELP for commands)", cmd);
        serialDebug(buf);
        serialStatus("UNKNOWN_CMD");
    }
}

/* ==================================================================
 * 12. SETUP / LOOP
 * ================================================================== */

void setup() {
    Serial.begin(SERIAL_BAUD);
    delay(1000);
    
    Serial.println("\n\n=== ESP32 ODrive CAN Controller (USB Mode) ===");
    
    // Check chip info
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    
    char chipInfo[256];
    snprintf(chipInfo, sizeof(chipInfo), "ESP32 Model: %s, Cores: %d, Revision: %d, Features: WiFi%s%s",
             CONFIG_IDF_TARGET,
             chip_info.cores,
             chip_info.revision,
             (chip_info.features & CHIP_FEATURE_BT) ? "+BT" : "",
             (chip_info.features & CHIP_FEATURE_BLE) ? "+BLE" : "");
    Serial.println(chipInfo);
    serialDebug(chipInfo);
    
    Serial.println("\nType HELP for available commands\n");
    
    // Register ODrive callbacks
    odrv0.onFeedback(onFeedback, &odrv0_data);
    odrv0.onStatus(onHeartbeat, &odrv0_data);
    odrv1.onFeedback(onFeedback, &odrv1_data);
    odrv1.onStatus(onHeartbeat, &odrv1_data);
    serialDebug("ODrive callbacks registered");
    
    // Initialize CAN bus
    if (!canInit()) {
        serialStatus("CAN_INIT_FAIL");
        serialDebug("FATAL: CAN initialization failed - halting");
        while (true) {
            delay(1000);
        }
    }
    
    // Verify CAN is running
    delay(100);
    twai_status_info_t status;
    if (twai_get_status_info(&status) == ESP_OK) {
        char statusBuf[128];
        snprintf(statusBuf, sizeof(statusBuf), "POST-INIT CAN State: %d (1=RUNNING)", status.state);
        serialDebug(statusBuf);
        
        if (status.state != TWAI_STATE_RUNNING) {
            serialDebug("WARNING: CAN not in RUNNING state after init!");
        }
    }
    
    serialStatus("CAN_READY");
    
    // Wait for ODrive to appear on the bus
    waitForHeartbeat();
    
    serialDebug("=== Setup Complete - Entering Main Loop ===");
    Serial.println("\nReady for commands!");
}

void loop() {
    processSerialCommands();
    canPump();
    assembleState();  
    sysidTick();  
    delay(1);
}