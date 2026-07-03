#!/usr/bin/env python3
"""
ESP32 ODrive Controller - Plot After Use
Logs data during operation, plots when you quit.

Matches firmware feedback format:
  Node0: out=0.063 turns (motor=0.500, vel=1.230, iq=0.0123A) | Node1: pos=0.500, vel=1.230

Node 0 has an 8:1 gearbox - positions and limits are shown in OUTPUT shaft turns.

Captures two CSV streams from firmware (each into its own directory):
  - SYSID -> odrive_logs/SYS_ID/sysid_phase3_<ts>.csv   (9 fields)
  - CTRL  -> odrive_logs/CTRL/ctrl_swingup_<ts>.csv      (11 fields, SwingUpXin mode=1)

The firmware also emits a [PARAMS] block on the CTRL_PARAMS command; it is
parsed into _params_latest for optional inspection.
"""

import serial
import serial.tools.list_ports
import time
import threading
import argparse
import matplotlib.pyplot as plt
import re
import os
import json
from datetime import datetime


class CsvStream:
    """A single firmware->CSV stream: buffered file + row list + open/close.

    Replaces the per-stream open/close method pairs with one reusable object
    so the read loop and disconnect path stay flat.
    """

    def __init__(self, name, log_dir, header, emoji="📂"):
        self.name = name
        self.log_dir = log_dir
        self.header = header
        self.emoji = emoji
        self.file = None
        self.path = None
        self.rows = []
        os.makedirs(self.log_dir, exist_ok=True)

    def open(self, base):
        """Open a fresh CSV named <base>.csv, writing the header row."""
        if self.file:
            self.close()
        self.path = os.path.join(self.log_dir, base + ".csv")
        self.file = open(self.path, "w", buffering=1)
        self.file.write(self.header + "\n")
        self.rows.clear()
        print(f"\n{self.emoji} {self.name} CSV : {self.path}\n")

    def write(self, line):
        self.file.write(line + "\n")
        self.rows.append(line)

    def close(self):
        if not self.file:
            return
        self.file.close()
        self.file = None
        print(f"\n✅ {self.name} saved {len(self.rows)} samples → {self.path}")
        self.rows.clear()

    @property
    def is_open(self):
        return self.file is not None


class ODriveLogger:
    # ---- CTRL: SwingUpXin only (firmware emits mode=1, 11 fields) ----
    CTRL_HEADER = "t_ms,mode,q1,qd1,q2,qd2,E_tilde,tau_out,tau_motor,iq,n0_out"
    CTRL_NFIELDS = 11

    # ---- SYSID: phase 3 Fourier trajectory (9 fields) ----
    SYSID_HEADER = "t_ms,phase,n0_pos_out,n0_vel_motor,n1_pos,n1_vel,cmd_out,iq_measured,clipped"
    SYSID_NFIELDS = 9

    # ---- Firmware-backed macros (every command exists in handleCommand) ----
    MACRO_GAP_S = 0.25
    MACROS = {
        "ARM":          ["LOOP"],
        "STOP":         ["CTRL_OFF", "IDLE"],
        "LOG":          ["CTRL_LOG_ON"],
        "LOGOFF":       ["CTRL_LOG_OFF"],
        "STATUS":       ["CTRL_STATUS", "LIMIT_STATUS"],
        "RESET":        ["CLEAR", "RESET_LIMIT"],
        "SWINGUP_GO":   ["CLEAR", "LOOP", "CTRL_LOG_ON", "SWINGUP"],
        "SWINGUP_STOP": ["CTRL_OFF", "IDLE"],
        "SYSID_GO":     ["CLEAR", "LOOP", "POS_MODE", "SYSID_PHASE3"],
        "SYSID_STOP":   ["SYSID_STOP"],
    }

    # ---- SysID sidecar params (phase 3 only, matches firmware constants) ----
    SYSID_PARAMS = {
        "node0": {"gear_ratio": 8.0, "output_limit_turns": 1.0,
                  "sysid_output_limit": 0.85, "role": "actuated"},
        "node1": {"gear_ratio": 1.0, "output_limit_turns": None, "role": "passive"},
        "physical": {"g": 9.81, "l1": 0.125, "l2": 0.070,
                     "m1": 0.2733, "m2": 0.023},
        "sysid": {
            "sample_rate_hz": 200,
            "phase3": {"description": "Fourier trajectory — inertial params",
                       "n_harmonics": 5, "wf_rad_s": 0.5,
                       "amp_out_turns": 0.70, "duration_s": 125.0,
                       "fourier_a": [0.30, 0.15, 0.08, 0.04, 0.02],
                       "fourier_b": [0.00, 0.10, -0.06, 0.03, -0.01]},
            "reference": "Swevers et al. (1997) IEEE Trans. Robotics and Automation",
        },
    }

    def __init__(self, port=None, baudrate=921600, filters=None):
        self.baudrate = baudrate
        self.port = port
        self.ser = None
        self.running = False
        self.read_thread = None

        self.filters = ({'DEBUG', 'STATUS', 'FEEDBACK', 'STATS', 'OTHER'}
                        if filters is None else {f.upper() for f in filters})

        # Live feedback buffers (for end-of-session plot)
        self.time_data = []
        self.n0_pos_motor = []
        self.n0_pos_output = []
        self.n0_vel = []
        self.n0_iq = []
        self.n1_pos = []
        self.n1_vel = []
        self.start_time = None

        # Node 0 mechanical config (must match firmware)
        self.NODE0_GEAR_RATIO = 8.0
        self.NODE0_OUTPUT_LIMIT = 1.0

        # Streams
        self.ctrl = CsvStream("CTRL", os.path.join("odrive_logs", "CTRL"),
                              self.CTRL_HEADER)
        self.sysid = CsvStream("SysId", os.path.join("odrive_logs", "SYS_ID"),
                               self.SYSID_HEADER, emoji="📂")
        self.sysid_phase = 0  # tracks which phase file is currently open

        # [PARAMS] block capture
        self._params_buffer = None
        self._params_latest = None
        self._params_lock = threading.Lock()

    # =================================================================
    # Connection
    # =================================================================
    def list_ports(self):
        ports = serial.tools.list_ports.comports()
        print("\nAvailable Serial Ports:")
        for i, p in enumerate(ports):
            print(f"  {i}: {p.device} - {p.description}")
        return [p.device for p in ports]

    def connect(self, port=None):
        if port:
            self.port = port
        if not self.port:
            ports = self.list_ports()
            if not ports:
                print("No serial ports found!")
                return False
            if len(ports) == 1:
                self.port = ports[0]
                print(f"\nAuto-selected: {self.port}")
            else:
                self.port = ports[int(input("\nSelect port number: "))]

        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1.0)
            time.sleep(2)
            self.ser.reset_input_buffer()
            print(f"✅ Connected to {self.port} at {self.baudrate} baud")
            print(f"📊 Active filters: {', '.join(sorted(self.filters))}")
            print(f"⚙️  Node 0: 8:1 gearbox, output limit ±{self.NODE0_OUTPUT_LIMIT} turns")
            print("📈 Data logging: ENABLED (will plot when you quit)")
            print("📂 CSV streams captured to ./odrive_logs/{SYS_ID,CTRL}/\n")

            self.running = True
            self.read_thread = threading.Thread(target=self._read_loop, daemon=True)
            self.read_thread.start()
            return True
        except serial.SerialException as e:
            print(f"❌ Failed to connect: {e}")
            return False

    # =================================================================
    # Stream handlers
    # =================================================================
    def _handle_ctrl_line(self, content):
        """One [CTRL] payload line. Firmware only emits SwingUp mode=1."""
        if content.startswith("HEADER:"):
            self.ctrl.open(f"ctrl_swingup_{datetime.now():%Y%m%d_%H%M%S}")
            return

        parts = content.split(",")
        if len(parts) != self.CTRL_NFIELDS:
            print(f"\n[WARN] CTRL fields={len(parts)} expected={self.CTRL_NFIELDS}: {content}")
            return
        if not self.ctrl.is_open:
            self.ctrl.open(f"ctrl_swingup_{datetime.now():%Y%m%d_%H%M%S}")

        self.ctrl.write(content)
        try:
            t_s = int(parts[0]) / 1000.0
            # t_ms,mode,q1,qd1,q2,qd2,E_tilde,tau_out,tau_motor,iq,n0_out
            print(f"\r  [CTRL/swingup] t={t_s:7.1f}s "
                  f"q1={parts[2]:>7} qd1={parts[3]:>7} "
                  f"q2={parts[4]:>7} qd2={parts[5]:>7} "
                  f"E~={parts[6]:>7} tau={parts[7]:>7}   ",
                  end="", flush=True)
        except (ValueError, IndexError):
            pass

    def _handle_sysid_line(self, content):
        """One [SYSID] payload line. Firmware inlines its own HEADER."""
        if content.startswith("HEADER:"):
            return  # file opened on first data row (need the phase number)

        parts = content.split(",")
        if len(parts) != self.SYSID_NFIELDS:
            return
        try:
            phase = int(parts[1])
        except (ValueError, IndexError):
            return

        if phase != self.sysid_phase or not self.sysid.is_open:
            self._sysid_open(phase)

        self.sysid.write(content)
        try:
            t_s = int(parts[0]) / 1000.0
            print(f"\r  [SYSID] t={t_s:7.1f}s  ph={parts[1]}"
                  f"  n0={parts[2]}  n1={parts[4]}  clip={parts[8]}   ",
                  end="", flush=True)
        except (ValueError, IndexError):
            pass

    def _handle_params_line(self, payload):
        """Collect a [PARAMS] BEGIN..END block into _params_latest."""
        with self._params_lock:
            if payload == "BEGIN":
                self._params_buffer = []
            elif payload == "END":
                if self._params_buffer is not None:
                    self._params_latest = self._parse_params_block(self._params_buffer)
                    self._params_buffer = None
            elif self._params_buffer is not None:
                self._params_buffer.append(payload)

    def _parse_feedback(self, line):
        """Parse a [FEEDBACK] line into a dict, or None."""
        if '[FEEDBACK]' not in line:
            return None
        n0 = re.search(
            r'Node0:\s*out=([-\d.]+)\s*turns\s*\(motor=([-\d.]+),\s*vel=([-\d.]+),\s*iq=([-\d.]+)A\)',
            line)
        n1 = re.search(r'Node1:\s*pos=([-\d.]+),\s*vel=([-\d.]+)', line)
        if not (n0 and n1):
            return None
        return {
            'n0_pos_output': float(n0.group(1)),
            'n0_pos_motor':  float(n0.group(2)),
            'n0_vel':        float(n0.group(3)),
            'n0_iq':         float(n0.group(4)),
            'n1_pos':        float(n1.group(1)),
            'n1_vel':        float(n1.group(2)),
        }

    # =================================================================
    # Read loop — table-driven prefix routing (was a long if/continue chain)
    # =================================================================
    def _get_message_type(self, line):
        for tag, kind in (('[STATUS]', 'STATUS'), ('[FEEDBACK]', 'FEEDBACK'),
                          ('[DEBUG', 'DEBUG'), ('[STATS]', 'STATS'),
                          ('[SYSID]', 'STREAM'), ('[CTRL]', 'STREAM')):
            if tag in line:
                return kind
        return 'OTHER'

    def _read_loop(self):
        # prefix -> handler(payload after the prefix)
        routes = (
            ("[SYSID] ",  self._handle_sysid_line),
            ("[CTRL] ",   self._handle_ctrl_line),
            ("[PARAMS] ", self._handle_params_line),
        )
        while self.running:
            try:
                if not self.ser:
                    continue
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if not line:
                    continue

                # Close streams on the firmware's completion status lines
                if "[STATUS] SYSID_DONE" in line:
                    self.sysid.close()
                if "[STATUS] CTRL_DONE" in line:
                    self.ctrl.close()

                # Prefix routing
                routed = False
                for prefix, handler in routes:
                    if line.startswith(prefix):
                        handler(line[len(prefix):])
                        routed = True
                        break
                if routed:
                    continue

                # Live feedback for the plot
                data = self._parse_feedback(line)
                if data:
                    if self.start_time is None:
                        self.start_time = time.time()
                    self.time_data.append(time.time() - self.start_time)
                    self.n0_pos_motor.append(data['n0_pos_motor'])
                    self.n0_pos_output.append(data['n0_pos_output'])
                    self.n0_vel.append(data['n0_vel'])
                    self.n0_iq.append(data['n0_iq'])
                    self.n1_pos.append(data['n1_pos'])
                    self.n1_vel.append(data['n1_vel'])

                # Terminal print (filter-controlled)
                self._print_line(line)

            except Exception as e:
                if self.running:
                    print(f"Read error: {e}")
                    time.sleep(0.1)

    def _print_line(self, line):
        msg_type = self._get_message_type(line)
        if msg_type not in self.filters:
            return
        if 'TRIP' in line or '🛑' in line:
            print(f"\033[91m\033[1m{line}\033[0m")
        elif 'ERROR' in line or '❌' in line:
            print(f"\033[91m{line}\033[0m")
        elif 'WARNING' in line or '⚠️' in line:
            print(f"\033[93m{line}\033[0m")
        elif msg_type == 'STATUS':
            print(f"\033[92m{line}\033[0m")
        elif msg_type == 'FEEDBACK':
            print(f"\033[94m{line}\033[0m")
        elif msg_type == 'STATS':
            print(f"\033[96m{line}\033[0m")
        else:
            print(line)

    # =================================================================
    # SysID file open (needs phase-numbered name + JSON sidecar)
    # =================================================================
    def _sysid_open(self, phase):
        base = f"sysid_phase{phase}_{datetime.now():%Y%m%d_%H%M%S}"
        self.sysid_phase = phase
        self.sysid.open(base)
        json_path = self.sysid.path.replace(".csv", "_params.json")
        with open(json_path, "w") as jf:
            json.dump(self.SYSID_PARAMS, jf, indent=2)
        print(f"📋 SysId JSON : {json_path}\n")

    # =================================================================
    # [PARAMS] block parser
    # =================================================================
    @staticmethod
    def _parse_params_block(lines):
        """Convert ['node0.gear_ratio=8.0', 'xin.theta1=1.2e-3', ...] into a
        nested dict. Handles scalars, CSV-list values, and indexed leaves
        like 'K[2]' / 'P[0][1]'."""
        result = {}
        idx_re = re.compile(r"^([A-Za-z_][A-Za-z0-9_]*)((?:\[\d+\])+)$")

        def coerce(v):
            for cast in (int, float):
                try:
                    return cast(v)
                except ValueError:
                    pass
            if "," in v:
                return [coerce(x.strip()) for x in v.split(",")]
            return v

        for raw in lines:
            if "=" not in raw:
                continue
            key, _, val = raw.partition("=")
            val = coerce(val.strip())
            parts = key.strip().split(".")

            m = idx_re.match(parts[-1])
            if m:
                base = m.group(1)
                idxs = [int(x) for x in re.findall(r"\d+", m.group(2))]
                d = result
                for k in parts[:-1]:
                    d = d.setdefault(k, {})
                cur = d.setdefault(base, [])
                for depth, i in enumerate(idxs):
                    is_last = depth == len(idxs) - 1
                    while len(cur) <= i:
                        cur.append(None if is_last else [])
                    if is_last:
                        cur[i] = val
                    else:
                        if cur[i] is None:
                            cur[i] = []
                        cur = cur[i]
            else:
                d = result
                for k in parts[:-1]:
                    d = d.setdefault(k, {})
                d[parts[-1]] = val
        return result

    # =================================================================
    # Sending
    # =================================================================
    def send_command(self, cmd):
        if not self.ser:
            print("❌ Not connected!")
            return False
        try:
            self.ser.write(f"{cmd}\n".encode('utf-8'))
            return True
        except Exception as e:
            print(f"❌ Failed to send command: {e}")
            return False

    # =================================================================
    # Plot
    # =================================================================
    def plot_data(self):
        if not self.time_data:
            print("\n📊 No data to plot (no [FEEDBACK] lines were received)")
            return
        print(f"\n📊 Plotting {len(self.time_data)} data points...")

        fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 9))
        fig.suptitle('ODrive Session Summary  -  Node 0 (Blue, 8:1 gearbox) | Node 1 (Red)',
                     fontsize=14, fontweight='bold')

        ax1.plot(self.time_data, self.n0_pos_output, 'b-',
                 label='Node 0 (output turns)', linewidth=2)
        ax1.plot(self.time_data, self.n1_pos, 'r-',
                 label='Node 1 (motor turns)', linewidth=2)
        ax1.axhline(y=self.NODE0_OUTPUT_LIMIT, color='b', linestyle='--',
                    alpha=0.4, label=f'N0 limit ±{self.NODE0_OUTPUT_LIMIT}')
        ax1.axhline(y=-self.NODE0_OUTPUT_LIMIT, color='b', linestyle='--', alpha=0.4)
        ax1.axhline(y=0, color='k', linestyle='-', alpha=0.2)
        ax1.set(ylabel='Position (turns)', xlabel='Time (s)')
        ax1.legend(loc='upper right'); ax1.grid(True, alpha=0.3)
        ax1.set_title('Position', fontweight='bold')

        ax2.plot(self.time_data, self.n0_vel, 'b-', label='Node 0 (motor)', linewidth=2)
        ax2.plot(self.time_data, self.n1_vel, 'r-', label='Node 1 (motor)', linewidth=2)
        ax2.set(ylabel='Velocity (rev/s)', xlabel='Time (s)')
        ax2.legend(loc='upper right'); ax2.grid(True, alpha=0.3)
        ax2.set_title('Velocity', fontweight='bold')

        if self.n0_iq:
            ax3.plot(self.time_data, self.n0_iq, 'g-', label='Node 0 Iq (A)', linewidth=1.5)
            ax3.axhline(y=0, color='k', linestyle='-', alpha=0.2)
            ax3.set(ylabel='Current (A)', xlabel='Time (s)')
            ax3.legend(loc='upper right'); ax3.grid(True, alpha=0.3)
            ax3.set_title('Motor Current (Iq measured)', fontweight='bold')

        plt.tight_layout()

        n0_out = self.n0_pos_output
        print("\n=== Session summary ===")
        print(f"  Duration:       {self.time_data[-1]:.1f} s")
        print(f"  Samples:        {len(self.time_data)}")
        print(f"  Node 0 output:  min={min(n0_out):+.3f}  max={max(n0_out):+.3f}  turns")
        print(f"  Node 0 motor:   min={min(self.n0_pos_motor):+.3f}  max={max(self.n0_pos_motor):+.3f}  turns")
        print(f"  Node 1:         min={min(self.n1_pos):+.3f}  max={max(self.n1_pos):+.3f}  turns")
        if self.n0_iq:
            print(f"  Node 0 Iq:      min={min(self.n0_iq):+.4f}  max={max(self.n0_iq):+.4f}  A")
        if any(abs(p) >= self.NODE0_OUTPUT_LIMIT for p in n0_out):
            print(f"  ⚠️  Node 0 reached/exceeded ±{self.NODE0_OUTPUT_LIMIT} output turns")
        print()

        if input("Save plot? (y/n): ").strip().lower() == 'y':
            fname = input("Filename (default: odrive_plot.png): ").strip() or "odrive_plot.png"
            plt.savefig(fname, dpi=300, bbox_inches='tight')
            print(f"✅ Saved to {fname}")
        print("Close plot window to exit...")
        plt.show()

    # =================================================================
    # Disconnect
    # =================================================================
    def disconnect(self):
        self.running = False
        if self.read_thread:
            self.read_thread.join(timeout=1)
        self.sysid.close()
        self.ctrl.close()
        if self.ser:
            self.ser.close()
        print("\nDisconnected")


# =================================================================
# Main
# =================================================================
def main():
    parser = argparse.ArgumentParser(description='ESP32 ODrive Controller - Plot After Use')
    parser.add_argument('--port', '-p', help='Serial port (auto-detect if not specified)')
    parser.add_argument('--baud', '-b', type=int, default=921600, help='Baud rate (default: 921600)')
    parser.add_argument('--only', '-o', nargs='+',
                        choices=['debug', 'status', 'feedback', 'stats', 'other'],
                        help='Show only these message types')
    parser.add_argument('--hide', '-x', nargs='+',
                        choices=['debug', 'status', 'feedback', 'stats', 'other'],
                        help='Hide these message types')
    args = parser.parse_args()

    if args.only:
        filters = {f.upper() for f in args.only}
    elif args.hide:
        filters = {'DEBUG', 'STATUS', 'FEEDBACK', 'STATS', 'OTHER'} - {f.upper() for f in args.hide}
    else:
        filters = None

    logger = ODriveLogger(port=args.port, baudrate=args.baud, filters=filters)
    if not logger.connect():
        return 1

    print("Commands:")
    print("  Node 0:  LOOP, IDLE, POS <output_turns>, VEL <val>, TORQUE <val>, CLEAR, POS_MODE, TORQUE_MODE")
    print("  Node 1:  N1_LOOP, N1_IDLE, N1_POS <val>, N1_VEL <val>, N1_CLEAR, N1_POS_MODE")
    print("  Both:    BOTH_LOOP, BOTH_IDLE")
    print("  Presets: N0_ONLY, N1_ONLY")
    print("  Mirror:  MIRROR_ON, MIRROR_OFF")
    print("  Safety:  RESET_LIMIT, LIMIT_STATUS")
    print("  Ctrl:    CTRL_OFF, CTRL_STATUS, CTRL_LOG_ON, CTRL_LOG_OFF, CTRL_PARAMS")
    print("  SwingUp: SWINGUP, SU_STATUS, SU_KP <v>, SU_KD <v>, SU_KV <v>, SU_TAU_MAX <v>")
    print("  SysId:   SYSID_PHASE3, SYSID_STOP, SYSID_STATUS")
    print("           Workflow: LOOP → POS_MODE → SYSID_PHASE3")
    print("  Note:    POS values for Node 0 are OUTPUT shaft turns (±1.0 max)")
    print("  Macros:  type 'MACROS' to list, e.g. ARM / STOP / STATUS / RESET")
    print("           SWINGUP_GO / SWINGUP_STOP / SYSID_GO / SYSID_STOP")
    print("Type 'quit' to exit and see plots\n")

    try:
        while True:
            cmd = input("> ").strip()
            if cmd.lower() in ('quit', 'exit', 'q'):
                break
            if not cmd:
                continue

            up = cmd.upper()
            if up == "MACROS":
                print("  Available macros:")
                for k, v in logger.MACROS.items():
                    print(f"    {k:14s} -> {' ; '.join(v)}")
            elif up in logger.MACROS:
                seq = logger.MACROS[up]
                print(f"  [macro {up}] -> {' ; '.join(seq)}")
                for i, c in enumerate(seq):
                    logger.send_command(c)
                    if i < len(seq) - 1:
                        time.sleep(logger.MACRO_GAP_S)
            else:
                logger.send_command(up)
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        logger.disconnect()
        logger.plot_data()

    return 0


if __name__ == "__main__":
    main()