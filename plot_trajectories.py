"""Plot cmd vs actual from data_raw/ CSV recordings.

Usage:
    python plot_trajectories.py                          # all recordings
    python plot_trajectories.py --traj sin_time_square  # one trajectory type
    python plot_trajectories.py --save my_plot.png
    python plot_trajectories.py data_raw/sin_sin/*.csv  # specific files
"""
import argparse
import glob
import math
import sys
from pathlib import Path

import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec


def plot_files(csv_files: list[str], save: str | None = None) -> None:
    if not csv_files:
        print("No CSV files found.")
        return

    n = len(csv_files)
    fig = plt.figure(figsize=(13, 5 * n))
    gs = gridspec.GridSpec(n, 1, figure=fig, hspace=0.55)

    for i, path in enumerate(csv_files):
        df = pd.read_csv(path)
        ax = fig.add_subplot(gs[i])

        ax.plot(df["t_s"], df["cmd_rad"], label="cmd_rad", linewidth=1.5, color="steelblue")
        ax.plot(df["t_s"], df["act_rad"], label="act_rad", linewidth=1.5,
                linestyle="--", color="darkorange")

        if "err_rad" in df.columns:
            ax2 = ax.twinx()
            ax2.plot(df["t_s"], df["err_rad"], label="err_rad",
                     color="crimson", alpha=0.55, linewidth=0.9)
            ax2.set_ylabel("error (rad)", color="crimson", fontsize=9)
            ax2.tick_params(axis="y", labelcolor="crimson", labelsize=8)
            ax2.axhline(0, color="crimson", linewidth=0.4, linestyle=":")

        if "vel_rads" in df.columns:
            rms_vel = (df["vel_rads"] ** 2).mean() ** 0.5
            vel_info = f"  vel_rms={rms_vel:.3f} r/s"
        else:
            vel_info = ""

        n_rows = len(df)
        dt_mean = df["t_s"].diff().mean() if n_rows > 1 else float("nan")
        hz_est = 1.0 / dt_mean if dt_mean > 0 else float("nan")
        info = f"  n={n_rows}  ~{hz_est:.0f} Hz{vel_info}"

        ax.set_xlabel("Time (s)", fontsize=9)
        ax.set_ylabel("Angle (rad)", fontsize=9)
        ax.set_title(f"{Path(path).stem}{info}", fontsize=10)
        ax.legend(loc="upper left", fontsize=8)
        ax.grid(True, alpha=0.3)
        ax.tick_params(labelsize=8)

    out = save or "trajectories.png"
    plt.savefig(out, dpi=150, bbox_inches="tight")
    print(f"Saved → {out}")
    plt.show()


def main() -> None:
    ap = argparse.ArgumentParser(description="Plot BAM trajectory recordings")
    ap.add_argument("files", nargs="*", help="specific CSV files to plot")
    ap.add_argument("--traj", metavar="NAME",
                    help="filter to one trajectory type (e.g. sin_time_square)")
    ap.add_argument("--save", metavar="FILE", help="output image filename")
    args = ap.parse_args()

    if args.files:
        files = sorted(args.files)
    elif args.traj:
        files = sorted(glob.glob(f"data_raw/{args.traj}/*.csv"))
    else:
        files = sorted(glob.glob("data_raw/**/*.csv", recursive=True))

    if not files:
        print("No CSV files found. Run record.py first.")
        sys.exit(0)

    print(f"Plotting {len(files)} file(s):")
    for f in files:
        print(f"  {f}")
    print()

    plot_files(files, args.save)


if __name__ == "__main__":
    main()
