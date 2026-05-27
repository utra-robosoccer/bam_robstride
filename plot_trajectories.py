import glob
import pandas as pd
import matplotlib.pyplot as plt

csv_files = sorted(glob.glob("sin_time_square_*.csv"))

fig, axes = plt.subplots(len(csv_files), 1, figsize=(12, 4 * len(csv_files)), sharex=False)
if len(csv_files) == 1:
    axes = [axes]

for ax, path in zip(axes, csv_files):
    df = pd.read_csv(path)
    ax.plot(df["t_s"], df["cmd_rad"], label="cmd_rad", linewidth=1.5)
    ax.plot(df["t_s"], df["act_rad"], label="act_rad", linewidth=1.5, linestyle="--")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Angle (rad)")
    ax.set_title(path)
    ax.legend()
    ax.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig("trajectories.png", dpi=150)
print("Saved trajectories.png")
plt.show()
