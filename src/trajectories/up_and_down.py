"""
up_and_down.py — slow controlled triangular motion

Motor traces a repeating trapezoidal position profile:

  0 → +A  (ramp up)   →  hold  →  +A → 0  (ramp down)
  0 → -A  (ramp down) →  hold  →  -A → 0  (ramp up)
  (repeat)

Designed for:
  • verifying tracking accuracy at low / constant velocity
  • measuring steady-state torque vs. position (gravity / friction mapping)
  • visual and tactile inspection with a human in the loop

Uses conservative defaults (lower kp, moderate kd) for smooth, forgiving motion.

Default parameters:
  amp=0.8 rad (±46°), ramp=3 s, hold=1 s → one full cycle ≈ 16 s

Usage:
    python up_and_down.py
    python up_and_down.py --amp 0.6 --ramp 4 --hold 2 --cycles 3
    python up_and_down.py --kp 20 --kd 1.5
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
DEFAULT_KP = 20.0   # softer than default — slow motion doesn't need high stiffness
DEFAULT_KD =  2.0


def _trapezoid(t: float, amp: float, ramp: float, hold: float) -> float:
    """
    Trapezoidal position profile (one full cycle = 4·ramp + 4·hold):

      segment 0: 0 → +A  over ramp s
      segment 1: hold at +A for hold s
      segment 2: +A → 0  over ramp s
      segment 3: hold at 0 for hold s
      segment 4: 0 → -A  over ramp s
      segment 5: hold at -A for hold s
      segment 6: -A → 0  over ramp s
      segment 7: hold at 0 for hold s
    """
    seg = ramp + hold
    cycle = 4 * seg
    t = t % cycle                  # wrap into one cycle

    if t < seg:
        # 0 → +A
        r = min(t / ramp, 1.0)
        return amp * r
    t -= seg

    if t < seg:
        # +A → 0
        r = min(t / ramp, 1.0)
        return amp * (1.0 - r)
    t -= seg

    if t < seg:
        # 0 → -A
        r = min(t / ramp, 1.0)
        return -amp * r
    t -= seg

    # -A → 0
    r = min(t / ramp, 1.0)
    return -amp * (1.0 - r)


def run(args) -> None:
    period = 1.0 / CTRL_HZ

    seg        = args.ramp + args.hold
    cycle_time = 4 * seg
    duration   = args.cycles * cycle_time

    ts  = time.strftime("%Y%m%d_%H%M%S")
    out = args.output or f"up_and_down_{ts}.csv"
    out = os.path.abspath(out)

    print(f"\n{'='*60}")
    print(f"  Trajectory  : up_and_down  (trapezoidal)")
    print(f"  Amplitude   : ±{args.amp:.3f} rad  (±{math.degrees(args.amp):.1f}°)")
    print(f"  Ramp time   : {args.ramp:.1f} s  (avg vel: {args.amp/args.ramp:.3f} rad/s)")
    print(f"  Hold time   : {args.hold:.1f} s  per extreme")
    print(f"  Cycles      : {args.cycles}  ({cycle_time:.1f} s / cycle)")
    print(f"  Total       : {duration:.1f} s")
    print(f"  kp / kd     : {args.kp} / {args.kd}")
    print(f"  Motor ID    : {args.id}  on  {args.port}")
    print(f"  Output      : {out}")
    print(f"{'='*60}")
    print("Press Ctrl+C to stop early (data saved).\n")
    print(f"{'t (s)':>7}  {'cmd (°)':>8}  {'act (°)':>8}  "
          f"{'err (°)':>8}  {'vel(r/s)':>9}  {'τ (Nm)':>8}")
    print("-" * 60)

    rows = []

    with RobstrideMotor(port=args.port, motor_id=args.id) as m:
        m.enable()
        time.sleep(0.15)

        t0       = time.monotonic()
        n_missed = 0

        try:
            while True:
                loop_start = time.monotonic()
                t = loop_start - t0

                if t >= duration:
                    break

                cmd   = _trapezoid(t, args.amp, args.ramp, args.hold)
                state = m.set_angle(cmd, kp=args.kp, kd=args.kd)

                if state:
                    err = cmd - state.angle_rad
                    rows.append({
                        "t_s":      round(t, 6),
                        "cmd_rad":  round(cmd, 6),
                        "act_rad":  round(state.angle_rad, 6),
                        "err_rad":  round(err, 6),
                        "vel_rads": round(state.vel_rads, 6),
                        "torque_nm":round(state.torque_nm, 6),
                        "temp_c":   round(state.temp_c, 2),
                    })
                    if len(rows) % 10 == 1:
                        print(f"{t:7.3f}  {math.degrees(cmd):+8.2f}  "
                              f"{math.degrees(state.angle_rad):+8.2f}  "
                              f"{math.degrees(err):+8.2f}  "
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
    fields = ["t_s", "cmd_rad", "act_rad", "err_rad", "vel_rads", "torque_nm", "temp_c"]
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        w.writerows(rows)
    print(f"\nSaved {len(rows)} rows → {path}")
    print(f"Missed frames: {n_missed}")


def main():
    ap = argparse.ArgumentParser(
        description="up_and_down: slow trapezoidal triangular motion"
    )
    ap.add_argument("--port",     default=PORT)
    ap.add_argument("--id",       type=int,   default=MOTOR_ID)
    ap.add_argument("--amp",      type=float, default=0.8,
                    help="Amplitude in radians (default: 0.8)")
    ap.add_argument("--ramp",     type=float, default=3.0,
                    help="Time (s) to ramp from 0 to ±amp (default: 3.0)")
    ap.add_argument("--hold",     type=float, default=1.0,
                    help="Hold time (s) at each extreme and zero (default: 1.0)")
    ap.add_argument("--cycles",   type=int,   default=3,
                    help="Number of full ±amp cycles (default: 3)")
    ap.add_argument("--kp",       type=float, default=DEFAULT_KP)
    ap.add_argument("--kd",       type=float, default=DEFAULT_KD)
    ap.add_argument("--output",   default=None)
    run(ap.parse_args())


if __name__ == "__main__":
    main()
