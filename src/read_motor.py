"""
read_motor.py — continuously read and display motor position.

Usage:
    python src/read_motor.py
    python src/read_motor.py --port /dev/ttyCH341USB0 --id 1 --hz 10
"""

import argparse
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from robstride import RobstrideMotor

PORT     = "/dev/ttyCH341USB0"
MOTOR_ID = 1
LOOP_HZ  = 10


def main():
    ap = argparse.ArgumentParser(description="Read and display motor position")
    ap.add_argument("--port", default=PORT)
    ap.add_argument("--id",   type=int,   default=MOTOR_ID)
    ap.add_argument("--hz",   type=float, default=LOOP_HZ)
    args = ap.parse_args()

    period = 1.0 / args.hz

    print(f"Reading motor {args.id} on {args.port}  @ {args.hz:.0f} Hz")
    print("Press Ctrl+C to stop.\n")
    print(f"{'t (s)':>8}  {'pos (rad)':>10}  {'pos (deg)':>10}  "
          f"{'vel (rad/s)':>12}  {'torque (Nm)':>12}  {'temp (°C)':>9}")
    print("-" * 72)

    with RobstrideMotor(port=args.port, motor_id=args.id) as m:
        t0 = time.monotonic()
        try:
            while True:
                loop_start = time.monotonic()

                state = m.read_feedback(timeout=period * 0.8)
                t = loop_start - t0

                if state:
                    print(f"{t:8.3f}  {state.angle_rad:+10.4f}  "
                          f"{math.degrees(state.angle_rad):+10.3f}  "
                          f"{state.vel_rads:+12.4f}  "
                          f"{state.torque_nm:+12.4f}  "
                          f"{state.temp_c:9.1f}")
                else:
                    print(f"{t:8.3f}  (no feedback — check motor power and ID)")

                elapsed = time.monotonic() - loop_start
                if elapsed < period:
                    time.sleep(period - elapsed)

        except KeyboardInterrupt:
            print("\nDone.")


if __name__ == "__main__":
    main()
