"""
zero_motor.py — set the motor's current position as the zero reference.

Usage:
    python src/zero_motor.py
    python src/zero_motor.py --port /dev/ttyCH341USB0 --id 1
"""

import argparse
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from robstride import RobstrideMotor

PORT     = "/dev/ttyCH341USB0"
MOTOR_ID = 1


def main():
    ap = argparse.ArgumentParser(description="Set motor current position as zero")
    ap.add_argument("--port", default=PORT)
    ap.add_argument("--id",   type=int, default=MOTOR_ID)
    args = ap.parse_args()

    print(f"Zeroing motor {args.id} on {args.port} ...")

    with RobstrideMotor(port=args.port, motor_id=args.id) as m:
        # read position before zeroing
        before = m.read_feedback()
        if before:
            print(f"  Before: {before.angle_rad:+.4f} rad  ({math.degrees(before.angle_rad):+.2f}°)")
        else:
            print("  Before: (no feedback — check motor power and ID)")

        state = m.zero()

        if state:
            print(f"  After:  {state.angle_rad:+.4f} rad  ({math.degrees(state.angle_rad):+.2f}°)")
            print("Done.")
        else:
            print("  No feedback after zero command — check motor connection.")


if __name__ == "__main__":
    main()
