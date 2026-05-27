"""
sin_sin.py — trajectory: A · sin(B · sin(2π·f·t))

The inner sinusoid drives the phase of the outer sine, producing a signal
that:
  • dwells near the amplitude extremes  (velocity ≈ 0 at peaks)
  • sweeps quickly through zero crossing (higher instantaneous velocity)

This is gentler than a square wave but still exercises both travel directions
and creates non-constant velocity — useful for probing stick-slip, backlash,
and gravity asymmetry.

Output range: ±A·sin(B)  (≈ ±0.84 A for the default B=1.0)

Default parameters:
  A=1.0 rad, B=1.0 rad, f=0.2 Hz → period 5 s, peak ≈ ±0.84 rad (±48°)

Usage:
    python sin_sin.py
    python sin_sin.py --amp 1.0 --inner 1.2 --freq 0.15 --duration 60
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
DEFAULT_KP = 30.0
DEFAULT_KD =  2.0


def trajectory(t: float, amp: float, inner: float, freq: float) -> float:
    """pos(t) = A · sin(B · sin(2π·f·t))"""
    return amp * math.sin(inner * math.sin(2.0 * math.pi * freq * t))


def run(args) -> None:
    period = 1.0 / CTRL_HZ

    ts  = time.strftime("%Y%m%d_%H%M%S")
    out = args.output or f"sin_sin_{ts}.csv"
    out = os.path.abspath(out)

    peak_rad = args.amp * math.sin(args.inner)

    print(f"\n{'='*60}")
    print(f"  Trajectory  : sin_sin  A·sin(B·sin(2π·f·t))")
    print(f"  A (outer)   : {args.amp:.3f} rad")
    print(f"  B (inner)   : {args.inner:.3f} rad")
    print(f"  f           : {args.freq:.3f} Hz  (period {1/args.freq:.1f} s)")
    print(f"  Peak pos    : ±{peak_rad:.3f} rad  (±{math.degrees(peak_rad):.1f}°)")
    print(f"  Duration    : {args.duration:.1f} s")
    print(f"  kp / kd     : {args.kp} / {args.kd}")
    print(f"  Motor ID    : {args.id}  on  {args.port}")
    print(f"  Output      : {out}")
    print(f"{'='*60}")
    print("Press Ctrl+C to stop early (data saved up to that point).\n")
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

                if t >= args.duration:
                    break

                cmd = trajectory(t, args.amp, args.inner, args.freq)
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
        description="sin_sin trajectory: A·sin(B·sin(2π·f·t)) — nested sinusoid"
    )
    ap.add_argument("--port",     default=PORT)
    ap.add_argument("--id",       type=int,   default=MOTOR_ID)
    ap.add_argument("--amp",      type=float, default=1.0,
                    help="Outer amplitude A in radians (default: 1.0)")
    ap.add_argument("--inner",    type=float, default=1.0,
                    help="Inner amplitude B in radians (default: 1.0)")
    ap.add_argument("--freq",     type=float, default=0.2,
                    help="Outer frequency f in Hz (default: 0.2)")
    ap.add_argument("--duration", type=float, default=30.0)
    ap.add_argument("--kp",       type=float, default=DEFAULT_KP)
    ap.add_argument("--kd",       type=float, default=DEFAULT_KD)
    ap.add_argument("--output",   default=None)
    run(ap.parse_args())


if __name__ == "__main__":
    main()
