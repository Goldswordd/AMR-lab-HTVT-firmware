#!/usr/bin/env python3
"""
AMR Robot — Gyro Calibration Monitor & Drift Test

This script monitors the MPU6050 heading output to verify:
1. Calibration completes (robot must be stationary ~2s after power-on)
2. Heading drift rate when stationary (should be < 1°/min)
3. Heading stability during and after movement

Usage:
    python3 gyro_test.py [PORT] [BAUD]
"""

import sys
import serial
import time
import signal

DEFAULT_PORT = "/dev/ttyACM0"
DEFAULT_BAUD = 115200


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_PORT
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else DEFAULT_BAUD

    print("=" * 50)
    print("  MPU6050 Gyro Calibration & Drift Test")
    print("=" * 50)

    try:
        ser = serial.Serial(port, baud, timeout=1.0)
    except serial.SerialException as e:
        print(f"Error: {e}")
        sys.exit(1)

    print(f"Connected to {port}")
    print("Waiting for robot startup...\n")

    # Phase 1: Watch for calibration messages
    start_time = time.time()
    calibrated = False
    heading_samples = []

    def signal_handler(sig, frame):
        print("\n\nInterrupted. Sending STOP...")
        ser.write(b"$STP\n")
        ser.close()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)

    while True:
        try:
            line = ser.readline().decode("utf-8", errors="replace").strip()
        except Exception:
            continue

        if not line:
            continue

        elapsed = time.time() - start_time

        # Print all messages during startup
        if not calibrated:
            print(f"  [{elapsed:5.1f}s] {line}")
            if "Ready" in line or "done" in line.lower():
                calibrated = True
                print("\n✓ Calibration complete!")
                print("\n--- DRIFT TEST (keep robot STILL) ---")
                print("Recording heading for 30 seconds...\n")
                start_time = time.time()  # Reset timer
                # Request odometry stream
                ser.write(b"$QRY\n")
            continue

        # Phase 2: Drift test — collect heading samples
        if line.startswith("$ODO"):
            try:
                parts = line.split(",")
                heading = float(parts[1])
                heading_samples.append((elapsed, heading))

                print(f"\r  t={elapsed:5.1f}s  heading={heading:+8.3f}°  "
                      f"samples={len(heading_samples)}", end="", flush=True)

                # Query again
                time.sleep(0.1)
                ser.write(b"$QRY\n")

                # After 30 seconds, report
                if elapsed >= 30.0 and len(heading_samples) >= 10:
                    print("\n\n" + "=" * 50)
                    print("  DRIFT TEST RESULTS")
                    print("=" * 50)

                    h_start = heading_samples[0][1]
                    h_end = heading_samples[-1][1]
                    h_min = min(h for _, h in heading_samples)
                    h_max = max(h for _, h in heading_samples)
                    total_drift = h_end - h_start
                    drift_per_min = total_drift / (elapsed / 60.0)

                    print(f"  Duration:     {elapsed:.1f} seconds")
                    print(f"  Samples:      {len(heading_samples)}")
                    print(f"  Start heading: {h_start:+.3f}°")
                    print(f"  End heading:   {h_end:+.3f}°")
                    print(f"  Min/Max:       {h_min:+.3f}° / {h_max:+.3f}°")
                    print(f"  Total drift:   {total_drift:+.3f}°")
                    print(f"  Drift rate:    {drift_per_min:+.3f}°/min")
                    print()

                    if abs(drift_per_min) < 1.0:
                        print("  ✓ EXCELLENT — drift < 1°/min")
                    elif abs(drift_per_min) < 3.0:
                        print("  △ ACCEPTABLE — drift < 3°/min")
                    else:
                        print("  ✗ HIGH DRIFT — check calibration, vibration, or wiring")
                        print("    Tips: ensure robot is perfectly still during calibration")
                        print("    Try increasing GYRO_DEADZONE_DPS in mpu6050.c")

                    print("\nPress Ctrl+C to exit, or the test will continue monitoring...")
                    print()
                    heading_samples = []
                    start_time = time.time()

            except (ValueError, IndexError):
                pass
        else:
            print(f"\n  ← {line}")


if __name__ == "__main__":
    main()
