"""
sin_time_square.py — trajectory: A · sin(k · t²)

Squaring time creates a linear frequency sweep (chirp): instantaneous
frequency rises as f(t) = k·t / π Hz.  A single run covers low-frequency
quasi-static behaviour through mid-frequency structural modes.

Default parameters:
  k=0.3, A=1.0 rad, duration=30 s → sweeps 0 → ~2.9 Hz

Usage:
    python sin_time_square.py
    python sin_time_square.py --duration 40 --k 0.2 --amp 0.8
    python sin_time_square.py --port /dev/ttyCH341USB0 --id 1
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


def trajectory(t: float, amp: float, k: float) -> float:
    """pos(t) = A · sin(k · t²)"""
    return amp * math.sin(k * t * t)


def run(args) -> None:
    period = 1.0 / CTRL_HZ

    ts  = time.strftime("%Y%m%d_%H%M%S")
    out = args.output or f"sin_time_square_{ts}.csv"
    out = os.path.abspath(out)

    print(f"\n{'='*60}")
    print(f"  Trajectory  : sin_time_square  A·sin(k·t²)")
    print(f"  A           : {args.amp:.3f} rad  ({math.degrees(args.amp):.1f}°)")
    print(f"  k           : {args.k}")
    print(f"  Duration    : {args.duration:.1f} s")
    print(f"  Freq range  : 0 → {args.k * args.duration / math.pi:.2f} Hz")
    print(f"  kp / kd     : {args.kp} / {args.kd}")
    print(f"  Motor ID    : {args.id}  on  {args.port}")
    print(f"  Output      : {out}")
    print(f"{'='*60}")
    print("Press Ctrl+C to stop early (data saved up to that point).\n")
    print(f"{'t (s)':>7}  {'f(t) Hz':>8}  {'cmd (°)':>8}  "
          f"{'act (°)':>8}  {'err (°)':>8}  {'τ (Nm)':>8}")
    print("-" * 60)

    rows = []

    with RobstrideMotor(port=args.port, motor_id=args.id) as m:
        # enable and let motor settle briefly
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

                cmd = trajectory(t, args.amp, args.k)
                freq_now = args.k * t / math.pi   # instantaneous freq in Hz

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
                        print(f"{t:7.3f}  {freq_now:8.3f}  "
                              f"{math.degrees(cmd):+8.2f}  "
                              f"{math.degrees(state.angle_rad):+8.2f}  "
                              f"{math.degrees(err):+8.2f}  "
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
        description="sin_time_square trajectory: A·sin(k·t²) — linear frequency sweep"
    )
    ap.add_argument("--port",     default=PORT)
    ap.add_argument("--id",       type=int,   default=MOTOR_ID)
    ap.add_argument("--amp",      type=float, default=1.0,
                    help="Amplitude in radians (default: 1.0)")
    ap.add_argument("--k",        type=float, default=0.3,
                    help="Sweep rate constant k (default: 0.3)")
    ap.add_argument("--duration", type=float, default=15.0)
    ap.add_argument("--kp",       type=float, default=DEFAULT_KP)
    ap.add_argument("--kd",       type=float, default=DEFAULT_KD)
    ap.add_argument("--output",   default=None)
    run(ap.parse_args())


if __name__ == "__main__":
    main()
