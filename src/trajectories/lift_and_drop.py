"""
lift_and_drop.py — gravity response: hold position, then release

Phase 1  LIFT  (duration: --hold seconds)
  Motor holds the target angle with full stiffness (--kp, --kd).
  Wait for the arm to settle before the drop.

Phase 2  DROP  (duration: --drop seconds)
  kp zeroed, only light damping (--drop-kd) applied.
  Motor falls freely under gravity and oscillates like a pendulum.
  Records the free-fall / ring-down response.

Useful for estimating:
  • gravitational torque vs. angle
  • effective inertia (from oscillation period)
  • natural damping

Usage:
    python lift_and_drop.py                          # lift to 0.8 rad, hold 3 s, drop 6 s
    python lift_and_drop.py --lift 1.0 --hold 5 --drop 8
    python lift_and_drop.py --lift 0.5 --drop-kd 0.5
"""

import argparse
import csv
import math
import os
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from robstride import RobstrideMotor

PORT     = "/dev/ttyCH341USB0"
MOTOR_ID = 1
CTRL_HZ  = 100
DEFAULT_KP      = 30.0
DEFAULT_KD      =  2.0
DEFAULT_DROP_KD =  1.0   # light damping during free fall


def run(args) -> None:
    period = 1.0 / CTRL_HZ

    ts  = time.strftime("%Y%m%d_%H%M%S")
    out = args.output or f"lift_and_drop_{ts}.csv"
    out = os.path.abspath(out)

    total_duration = args.hold + args.drop

    print(f"\n{'='*60}")
    print(f"  Trajectory  : lift_and_drop")
    print(f"  Lift angle  : {args.lift:.3f} rad  ({math.degrees(args.lift):.1f}°)")
    print(f"  Hold phase  : {args.hold:.1f} s  (kp={args.kp}, kd={args.kd})")
    print(f"  Drop phase  : {args.drop:.1f} s  (kp=0, kd={args.drop_kd})")
    print(f"  Total       : {total_duration:.1f} s")
    print(f"  Motor ID    : {args.id}  on  {args.port}")
    print(f"  Output      : {out}")
    print(f"{'='*60}")
    print("Press Ctrl+C to abort (data saved).\n")
    print(f"{'t (s)':>7}  {'phase':>5}  {'cmd (°)':>8}  "
          f"{'act (°)':>8}  {'vel(r/s)':>9}  {'τ (Nm)':>8}")
    print("-" * 60)

    rows = []

    with RobstrideMotor(port=args.port, motor_id=args.id) as m:
        # move to lift position before starting the clock
        print(f"Moving to lift position {math.degrees(args.lift):.1f}° ...")
        m.enable()
        time.sleep(0.1)
        t_ramp_start = time.monotonic()
        while time.monotonic() - t_ramp_start < 1.5:
            m.set_angle(args.lift, kp=args.kp, kd=args.kd)
            time.sleep(period)
        print("Holding — press Enter or wait for auto-drop ...\n")

        t0       = time.monotonic()
        n_missed = 0

        try:
            while True:
                loop_start = time.monotonic()
                t = loop_start - t0

                if t >= total_duration:
                    break

                if t < args.hold:
                    # Phase 1: stiff hold
                    phase  = "HOLD"
                    cmd    = args.lift
                    kp_now = args.kp
                    kd_now = args.kd
                else:
                    # Phase 2: gravity free-fall (kp=0)
                    phase  = "DROP"
                    cmd    = args.lift   # irrelevant when kp=0
                    kp_now = 0.0
                    kd_now = args.drop_kd

                state = m.set_angle(cmd, kp=kp_now, kd=kd_now)

                if state:
                    rows.append({
                        "t_s":      round(t, 6),
                        "phase":    phase,
                        "cmd_rad":  round(cmd, 6),
                        "act_rad":  round(state.angle_rad, 6),
                        "err_rad":  round(cmd - state.angle_rad, 6),
                        "vel_rads": round(state.vel_rads, 6),
                        "torque_nm":round(state.torque_nm, 6),
                        "temp_c":   round(state.temp_c, 2),
                    })
                    if len(rows) % 10 == 1:
                        print(f"{t:7.3f}  {phase:>5}  "
                              f"{math.degrees(cmd):+8.2f}  "
                              f"{math.degrees(state.angle_rad):+8.2f}  "
                              f"{state.vel_rads:+9.3f}  "
                              f"{state.torque_nm:+8.3f}")
                else:
                    n_missed += 1

                elapsed = time.monotonic() - loop_start
                if elapsed < period:
                    time.sleep(period - elapsed)

        except KeyboardInterrupt:
            print("\n[interrupted]")

    _save_csv(rows, out, n_missed)


def _save_csv(rows, path, n_missed):
    if not rows:
        print("No data recorded.")
        return
    fields = ["t_s", "phase", "cmd_rad", "act_rad", "err_rad",
              "vel_rads", "torque_nm", "temp_c"]
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        w.writerows(rows)
    print(f"\nSaved {len(rows)} rows → {path}")
    print(f"Missed frames: {n_missed}")


def main():
    ap = argparse.ArgumentParser(
        description="lift_and_drop: hold position then release to gravity"
    )
    ap.add_argument("--port",     default=PORT)
    ap.add_argument("--id",       type=int,   default=MOTOR_ID)
    ap.add_argument("--lift",     type=float, default=0.8,
                    help="Lift angle in radians (default: 0.8)")
    ap.add_argument("--hold",     type=float, default=3.0,
                    help="Hold duration in seconds (default: 3.0)")
    ap.add_argument("--drop",     type=float, default=6.0,
                    help="Free-fall recording duration in seconds (default: 6.0)")
    ap.add_argument("--kp",       type=float, default=DEFAULT_KP,
                    help="Position gain during hold (default: 30)")
    ap.add_argument("--kd",       type=float, default=DEFAULT_KD,
                    help="Damping gain during hold (default: 2)")
    ap.add_argument("--drop-kd",  type=float, default=DEFAULT_DROP_KD,
                    dest="drop_kd",
                    help="Damping gain during free fall (default: 1.0)")
    ap.add_argument("--output",   default=None)
    run(ap.parse_args())


if __name__ == "__main__":
    main()
