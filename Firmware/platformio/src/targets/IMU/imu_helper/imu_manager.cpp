#include "imu_manager.h"

IMUManager::IMUManager() : sampleCount(0) {
    memset(&accum, 0, sizeof(accum));
    memset(&latest, 0, sizeof(latest));
}

bool IMUManager::begin() {
    Serial.println("\n=== IMU Initialization ===");

    // --- HARD RESET ---
    pinMode(IMU_RST, OUTPUT);
    digitalWrite(IMU_RST, LOW);
    delay(50);
    digitalWrite(IMU_RST, HIGH);
    delay(300);

    // --- I2C ---
    Wire.begin(IMU_SDA, IMU_SCL, 100000);
    delay(100);

    bool ok = imu.begin(0x4B, Wire, -1, -1);
    Serial.print("begin() returned: ");
    Serial.println(ok ? "TRUE" : "FALSE");
    if (!ok) return false;

    // Enable accelerometer (50 Hz)
    if (!imu.enableAccelerometer(20000)) {
        Serial.println("ACC ENABLE FAILED");
        return false;
    }

    Serial.println("IMU setup complete");
    return true;
}

bool IMUManager::update() {
    if (!imu.getSensorEvent()) return false;

    accum.accX += imu.getAccelX();
    accum.accY += imu.getAccelY();
    accum.accZ += imu.getAccelZ();

    accum.gyrX += imu.getGyroX();
    accum.gyrY += imu.getGyroY();
    accum.gyrZ += imu.getGyroZ();

    accum.linX += imu.getLinAccelX();
    accum.linY += imu.getLinAccelY();
    accum.linZ += imu.getLinAccelZ();

    accum.qi += imu.getQuatI();
    accum.qj += imu.getQuatJ();
    accum.qk += imu.getQuatK();
    accum.qr += imu.getQuatReal();

    accum.quatAcc = imu.getQuatAccuracy();

    sampleCount++;

    if (sampleCount >= IMU_AVG_SAMPLES) {
        float inv = 1.0f / sampleCount;
        latest.accX = accum.accX * inv;
        latest.accY = accum.accY * inv;
        latest.accZ = accum.accZ * inv;

        latest.gyrX = accum.gyrX * inv;
        latest.gyrY = accum.gyrY * inv;
        latest.gyrZ = accum.gyrZ * inv;

        latest.linX = accum.linX * inv;
        latest.linY = accum.linY * inv;
        latest.linZ = accum.linZ * inv;

        latest.qi = accum.qi * inv;
        latest.qj = accum.qj * inv;
        latest.qk = accum.qk * inv;
        latest.qr = accum.qr * inv;

        latest.quatAcc = accum.quatAcc;

        // reset accumulators
        memset(&accum, 0, sizeof(accum));
        sampleCount = 0;

        return true; // data ready
    }

    return false; // not enough samples yet
}

IMUData IMUManager::getData() const {
    return latest;
}
