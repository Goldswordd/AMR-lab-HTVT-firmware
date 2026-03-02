#!/usr/bin/env python3
"""
AMR Robot Control — Interactive serial terminal + command interface.

Usage:
    python3 robot_control.py [PORT] [BAUD]
    python3 robot_control.py /dev/ttyACM0 115200

Commands (type in terminal):
    fwd <speed>           — Go straight (speed in ticks/s, e.g. 200)
    rot <angle> [speed]   — Rotate angle degrees (e.g. 90, -90)
    vel <linear> <angular>— cmd_vel (linear ticks/s, angular deg/s)
    stop                  — Stop all motors
    query                 — Request odometry once
    pid <Kp> <Ki> <Kd>   — Tune heading PID live
    spd <Kp> <Ki> <Kd>   — Tune speed PID live (both wheels)
    bias <left> <right>   — Set per-wheel FF gains (e.g. bias 1.5 1.6)
    stream [on|off]       — Toggle continuous odometry display
    help                  — Show this help
    quit / exit           — Exit

Keyboard shortcuts (when stream is on):
    w — forward 200      s — stop
    a — rotate left 15°  d — rotate right 15°
    q — quit
"""

import sys
import threading
import time
import os
from serial import Serial, SerialException
from serial.tools.list_ports import comports as list_serial_ports

# ─────────────────── Configuration ───────────────────
DEFAULT_PORT = "/dev/ttyACM0"
DEFAULT_BAUD = 115200
DEFAULT_SPEED = 400      # ticks/s for keyboard control (reduced for safer testing)
DEFAULT_ROT_SPEED = 150  # ticks/s for rotation
DEFAULT_ROT_ANGLE = 90   # degrees for a/d keys


class RobotController:
    def __init__(self, port, baud):
        self.port = port
        self.baud = baud
        self.ser = None
        self.running = False
        self.stream_odom = False
        self.rx_thread = None

    def connect(self):
        """Open serial connection."""
        try:
            self.ser = Serial(self.port, self.baud, timeout=0.1)
            time.sleep(0.5)  # Wait for connection to settle
            # Flush any pending data
            self.ser.flushInput()
            print(f"✓ Connected to {self.port} @ {self.baud} baud")
            return True
        except SerialException as e:
            print(f"✗ Failed to open {self.port}: {e}")
            print("  Available ports:")
            try:
                for p in list_serial_ports():
                    print(f"    {p.device} — {p.description}")
            except Exception:
                os.system("ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null")
            return False

    def send(self, cmd):
        """Send a command string (adds \\n)."""
        if self.ser and self.ser.is_open:
            full_cmd = cmd.strip() + "\n"
            self.ser.write(full_cmd.encode())
            print(f"  → {cmd.strip()}")

    def rx_loop(self):
        """Background thread: read and display serial output."""
        while self.running:
            try:
                if self.ser and self.ser.in_waiting:
                    line = self.ser.readline().decode("utf-8", errors="replace").strip()
                    if line:
                        if line.startswith("$ODO"):
                            if self.stream_odom:
                                self._print_odom(line)
                        elif line.startswith("$ESTOP"):
                            print(f"\n  ⚠ EMERGENCY STOP: {line}")
                        else:
                            print(f"  ← {line}")
                else:
                    time.sleep(0.01)
            except Exception:
                if self.running:
                    time.sleep(0.05)

    def _print_odom(self, line):
        """Pretty-print odometry: $ODO,heading,enc_l,enc_r,spd_l,spd_r"""
        try:
            parts = line.split(",")
            if len(parts) >= 6:
                heading = float(parts[1])
                enc_l = int(parts[2])
                enc_r = int(parts[3])
                spd_l = float(parts[4])
                spd_r = float(parts[5])
                # Overwrite same line
                print(f"\r  HDG: {heading:+7.2f}°  "
                      f"ENC: L={enc_l:+5d} R={enc_r:+5d}  "
                      f"SPD: L={spd_l:+7.1f} R={spd_r:+7.1f}  ", end="", flush=True)
            else:
                print(f"  ← {line}")
        except (ValueError, IndexError):
            print(f"  ← {line}")

    def start(self):
        """Start RX background thread."""
        self.running = True
        self.rx_thread = threading.Thread(target=self.rx_loop, daemon=True)
        self.rx_thread.start()

    def stop(self):
        """Stop everything."""
        self.running = False
        if self.ser and self.ser.is_open:
            self.send("$STP")
            time.sleep(0.1)
            self.ser.close()
        print("\n✓ Disconnected")

    def handle_command(self, user_input):
        """Parse and execute user command."""
        parts = user_input.strip().split()
        if not parts:
            return True

        cmd = parts[0].lower()

        if cmd in ("quit", "exit", "q"):
            return False

        elif cmd == "help":
            print(__doc__)

        elif cmd == "fwd":
            speed = int(parts[1]) if len(parts) > 1 else DEFAULT_SPEED
            self.send(f"$FWD,{speed}")

        elif cmd == "rot":
            angle = float(parts[1]) if len(parts) > 1 else 90
            speed = int(parts[2]) if len(parts) > 2 else DEFAULT_ROT_SPEED
            self.send(f"$ROT,{angle},{speed}")

        elif cmd == "vel":
            if len(parts) >= 3:
                self.send(f"$CMD,{parts[1]},{parts[2]}")
            else:
                print("  Usage: vel <linear> <angular>")

        elif cmd in ("stop", "s"):
            self.send("$STP")

        elif cmd == "query":
            self.send("$QRY")

        elif cmd == "pid":
            if len(parts) >= 4:
                self.send(f"$PID,{parts[1]},{parts[2]},{parts[3]}")
            else:
                print("  Usage: pid <Kp> <Ki> <Kd>")

        elif cmd == "spd":
            if len(parts) >= 4:
                self.send(f"$SPD,{parts[1]},{parts[2]},{parts[3]}")
            else:
                print("  Usage: spd <Kp> <Ki> <Kd>")

        elif cmd == "bias":
            if len(parts) >= 3:
                self.send(f"$BIAS,{parts[1]},{parts[2]}")
            else:
                print("  Usage: bias <left_ff> <right_ff>  (e.g. bias 1.5 1.6)")

        elif cmd == "stream":
            if len(parts) > 1 and parts[1] == "off":
                self.stream_odom = False
                print("  Odometry stream OFF")
            else:
                self.stream_odom = True
                print("  Odometry stream ON (Ctrl+C to stop)")

        elif cmd == "tst":
            # Raw motor test: bypasses PID
            left_pwm = int(parts[1]) if len(parts) > 1 else 300
            right_pwm = int(parts[2]) if len(parts) > 2 else left_pwm
            self.send(f"$TST,{left_pwm},{right_pwm}")

        elif cmd == "dbg":
            self.send("$DBG")

        elif cmd == "raw":
            # Send raw command
            raw = " ".join(parts[1:])
            self.send(raw)

        # Keyboard shortcuts
        elif cmd == "w":
            self.send(f"$FWD,{DEFAULT_SPEED}")
        elif cmd == "a":
            self.send(f"$ROT,{DEFAULT_ROT_ANGLE},{DEFAULT_ROT_SPEED}")
        elif cmd == "d":
            self.send(f"$ROT,{-DEFAULT_ROT_ANGLE},{DEFAULT_ROT_SPEED}")

        else:
            print(f"  Unknown command: '{cmd}'. Type 'help' for usage.")

        return True


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_PORT
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else DEFAULT_BAUD

    print("╔══════════════════════════════════════════╗")
    print("║       AMR Robot Control Terminal         ║")
    print("╚══════════════════════════════════════════╝")
    print()

    ctrl = RobotController(port, baud)

    if not ctrl.connect():
        sys.exit(1)

    ctrl.start()

    # Wait a moment for calibration messages
    print("\n  Waiting for robot startup messages...\n")
    time.sleep(3)

    print("\n  Type 'help' for commands. 'quit' to exit.\n")

    try:
        while True:
            try:
                user_input = input("AMR> ")
                if not ctrl.handle_command(user_input):
                    break
            except KeyboardInterrupt:
                print("\n  (Ctrl+C) Sending STOP...")
                ctrl.send("$STP")
                ctrl.stream_odom = False
                print()
    except EOFError:
        pass
    finally:
        ctrl.stop()


if __name__ == "__main__":
    main()
