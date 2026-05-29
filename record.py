"""Run a BAM trajectory on an RS02 motor and record to data_raw/.

Usage:
    python record.py sin_time_square
    python record.py sin_sin --id 1 --kp 30 --kd 2
    python record.py lift_and_drop --id 1
    python record.py up_and_down --id 1 --duration 12
    python record.py nothing --id 1
    python record.py sin_time_square --port /dev/ttyCH341USB1 --id 2 --amp 0.8 --k 0.15

Output:
    data_raw/<traj_name>/<YYYYMMDD_HHMMSS>_id<N>_kp<kp>_kd<kd>.csv

Columns:
    t_s, cmd_rad, act_rad, err_rad, vel_rads, torque_nm, temp_c
"""
import argparse
import csv
import math
import os
import sys
import time
from datetime import datetime
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent / "src"))

from rs02_motor import RS02Motor, DEFAULT_PORT, DEFAULT_BAUD
from trajectories import traj_list

DEFAULT_KP = 30.0
DEFAULT_KD = 2.0
DEFAULT_HZ = 100.0


def _print_header() -> None:
    print(f"{'t (s)':>7}  {'cmd (rad)':>10}  {'act (rad)':>10}  {'err (rad)':>10}  "
          f"{'vel (r/s)':>10}  {'torq (Nm)':>10}  {'temp (°C)':>9}")
    print("-" * 82)


def run(
    traj_name: str,
    port: str,
    motor_id: int,
    kp: float,
    kd: float,
    hz: float,
    duration: float | None,
    out_dir: str,
    traj_kwargs: dict,
) -> None:
    traj = traj_list[traj_name](**traj_kwargs)
    total = duration if duration is not None else traj.duration
    dt = 1.0 / hz

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_path = Path(out_dir) / traj_name / f"{stamp}_id{motor_id}_kp{kp:.0f}_kd{kd:.0f}.csv"
    out_path.parent.mkdir(parents=True, exist_ok=True)

    print(f"\nTrajectory : {traj_name}  ({total:.1f} s)")
    print(f"Motor      : ID {motor_id} on {port}")
    print(f"Gains      : kp={kp}  kd={kd}  hz={hz:.0f}")
    print(f"Output     : {out_path}")
    print()

    rows: list[dict] = []

    with RS02Motor(port=port, motor_id=motor_id, baud=DEFAULT_BAUD) as m:
        print("Initializing motor (MIT mode + enable)...")
        fb = m.init_mit()
        if fb is None:
            print("ERROR: no feedback — check motor power and CAN ID")
            return
        print(f"Ready: {fb}\n")

        _print_header()

        start = time.monotonic()
        torque_was_on = True
        missed = 0
        print_interval = max(1, int(hz))  # print ~1 Hz

        try:
            while True:
                loop_start = time.monotonic()
                t = loop_start - start
                if t >= total:
                    break

                cmd_rad, torque_enable = traj(t)

                if torque_enable:
                    if not torque_was_on:
                        m.enable()
                        torque_was_on = True
                    fb = m.mit_control(pos=cmd_rad, vel=0.0, kp=kp, kd=kd, torque=0.0)
                else:
                    if torque_was_on:
                        m.disable()
                        torque_was_on = False
                    fb = m.read()

                if fb is not None:
                    err = cmd_rad - fb.pos
                    rows.append({
                        "t_s":      f"{t:.4f}",
                        "cmd_rad":  f"{cmd_rad:.6f}",
                        "act_rad":  f"{fb.pos:.6f}",
                        "err_rad":  f"{err:.6f}",
                        "vel_rads": f"{fb.vel:.6f}",
                        "torque_nm": f"{fb.torque:.6f}",
                        "temp_c":   f"{fb.temp:.1f}",
                    })
                    if len(rows) % print_interval == 1:
                        print(f"{t:7.2f}  {cmd_rad:+10.4f}  {fb.pos:+10.4f}  {err:+10.4f}  "
                              f"{fb.vel:+10.4f}  {fb.torque:+10.4f}  {fb.temp:9.1f}")
                else:
                    missed += 1

                elapsed = time.monotonic() - loop_start
                if elapsed < dt:
                    time.sleep(dt - elapsed)

        except KeyboardInterrupt:
            print("\n[interrupted — saving data]")

        print("\nReturning to zero...")
        if not torque_was_on:
            m.enable()
        m.goto(pos=0.0, rate=0.5, kp=kp, kd=kd)
        m.disable()

    if rows:
        cols = ["t_s", "cmd_rad", "act_rad", "err_rad", "vel_rads", "torque_nm", "temp_c"]
        with open(out_path, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=cols)
            w.writeheader()
            w.writerows(rows)
        print(f"\nSaved {len(rows)} rows  ({missed} missed)  →  {out_path}")
    else:
        print("No data recorded.")


def main() -> None:
    ap = argparse.ArgumentParser(
        description="Run a BAM trajectory and record motor data to CSV",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    ap.add_argument("trajectory", choices=list(traj_list), help="trajectory to run")
    ap.add_argument("--port", default=os.environ.get("RS02_PORT", DEFAULT_PORT))
    ap.add_argument("--id", type=int, default=1, dest="motor_id", metavar="MOTOR_ID")
    ap.add_argument("--kp", type=float, default=DEFAULT_KP, help="position gain")
    ap.add_argument("--kd", type=float, default=DEFAULT_KD, help="damping gain")
    ap.add_argument("--hz", type=float, default=DEFAULT_HZ, help="control loop rate")
    ap.add_argument("--duration", type=float, default=None, help="override trajectory duration (s)")
    ap.add_argument("--out", default="data_raw", metavar="DIR", help="output root directory")
    # sin_time_square options
    ap.add_argument("--amp", type=float, default=1.0, help="[sin_time_square] amplitude in rad")
    ap.add_argument("--k", type=float, default=0.1, help="[sin_time_square] sweep rate constant")
    args = ap.parse_args()

    traj_kwargs: dict = {}
    if args.trajectory == "sin_time_square":
        traj_kwargs = {"amplitude": args.amp, "k": args.k}

    run(
        traj_name=args.trajectory,
        port=args.port,
        motor_id=args.motor_id,
        kp=args.kp,
        kd=args.kd,
        hz=args.hz,
        duration=args.duration,
        out_dir=args.out,
        traj_kwargs=traj_kwargs,
    )


if __name__ == "__main__":
    main()
