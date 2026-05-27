"""
goto_zero.py — creep motor to position zero at a configurable rate.

Usage:
    python src/goto_zero.py
    python src/goto_zero.py --port /dev/ttyUSB0 --id 1 --rate 1.0
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
RATE_RPS = 1.0    # revolutions per second
DT       = 0.01   # 100 Hz control loop
KP       = 30.0
KD       = 2.0


def main():
    ap = argparse.ArgumentParser(description="Creep motor to zero at a controlled rate")
    ap.add_argument("--port", default=PORT)
    ap.add_argument("--id",   type=int,   default=MOTOR_ID)
    ap.add_argument("--rate", type=float, default=RATE_RPS, help="speed in rev/s (default 1.0)")
    ap.add_argument("--kp",   type=float, default=KP)
    ap.add_argument("--kd",   type=float, default=KD)
    args = ap.parse_args()

    rate_rads = args.rate * 2 * math.pi

    with RobstrideMotor(port=args.port, motor_id=args.id) as m:
        state = m.read_feedback(timeout=0.3)
        if not state:
            print("No feedback — check motor power and ID")
            return

        setpoint = state.angle_rad
        print(f"Starting at {setpoint:+.4f} rad  ({math.degrees(setpoint):+.2f} deg)")
        print(f"Creeping to zero at {args.rate:.2f} rev/s ...")
        print()
        print(f"{'t(s)':>6}  {'setpoint':>10}  {'actual':>10}  {'error':>8}")
        print("-" * 42)

        t0 = time.monotonic()
        try:
            while True:
                loop_start = time.monotonic()
                t = loop_start - t0

                error = -setpoint
                if abs(error) < 0.001:
                    setpoint = 0.0
                else:
                    step = rate_rads * DT * (1 if error > 0 else -1)
                    setpoint += step if abs(step) < abs(error) else error

                state = m.set_angle(setpoint, kp=args.kp, kd=args.kd)

                if state and int(t * 10) % 5 == 0:
                    print(f"{t:6.2f}  {setpoint:+10.4f}  {state.angle_rad:+10.4f}  {-state.angle_rad:+8.4f}")

                if abs(setpoint) < 0.001:
                    if state:
                        print(f"\nReached zero: actual={state.angle_rad:+.4f} rad  ({math.degrees(state.angle_rad):+.2f} deg)")
                    break

                elapsed = time.monotonic() - loop_start
                if elapsed < DT:
                    time.sleep(DT - elapsed)

        except KeyboardInterrupt:
            print("\nStopped.")


if __name__ == "__main__":
    main()
