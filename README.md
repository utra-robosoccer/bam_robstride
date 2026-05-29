# BAM Pipeline — Robstride Motor Identification

BAM (Better Actuator Model) data collection pipeline for Robstride RS02 actuators.
Communicates over CH341 USB-CAN using the raw MIT-mode framing from
[soccer-firmware/tools/robostride_usb_can](../soccer-firmware/tools/robostride_usb_can).
Runs the standard Rhoban BAM excitation trajectories and records commanded vs. actual
position/velocity/torque for offline friction model parameter fitting.

---

## Prerequisites

**CH341 Linux kernel driver** — install before plugging in the adapter:

```bash
git clone https://github.com/WCHSoftGroup/ch341ser_linux
cd ch341ser_linux
make && sudo make install
```

After installation the adapter appears as `/dev/ttyCH341USB0`.

**Python environment** — activate the included venv:

```bash
source .bam_env/bin/activate
```

---

## Workspace structure

```
bam_robstride/
├── src/
│   ├── rs02_can.py              # CAN frame encode/decode (29-bit extended, CH341 AT framing)
│   ├── rs02_motor.py            # RS02Motor class (MIT mode, enable, goto, sine, …)
│   ├── cli.py                   # Direct motor control CLI (read, goto, sine, zero, set-id, …)
│   └── trajectories/
│       ├── __init__.py          # traj_list dict
│       ├── base.py              # Trajectory ABC + cubic Hermite spline interpolation
│       ├── sin_time_square.py   # A·sin(k·t²)  — primary BAM chirp excitation
│       ├── sin_sin.py           # sin(t)·π/2 + sin(5t)·0.5·sin(2t)
│       ├── lift_and_drop.py     # Cubic lift to -π/2, then torque release
│       ├── up_and_down.py       # Cubic lift to π/2, partial descent
│       └── nothing.py           # Zero torque — baseline / backdrive test
├── record.py                    # Run any trajectory and record to CSV
├── plot_trajectories.py         # Plot cmd vs actual from data_raw/
├── data_raw/                    # Recorded CSVs, organised by trajectory type
│   ├── sin_time_square/
│   ├── sin_sin/
│   ├── lift_and_drop/
│   ├── up_and_down/
│   └── nothing/
└── .bam_env/                    # Python virtual environment
```

---

## Quick start

### 1. Check the motor

```bash
python src/cli.py read 1
python src/cli.py init 1
```

### 2. Zero the motor

```bash
python src/cli.py zero 1 --rate 0.3 --set-mech-zero
```

### 3. Run an identification trajectory

```bash
# Primary BAM chirp (30 s, sweeps 0 → ~0.95 Hz)
python record.py sin_time_square

# Multi-frequency sine (30 s)
python record.py sin_sin

# Lift and drop — captures free-fall inertia/friction
python record.py lift_and_drop

# Controlled up-and-down — gravity loading both ways
python record.py up_and_down

# Zero-torque baseline
python record.py nothing
```

All flags:

```bash
python record.py sin_time_square \
    --port /dev/ttyCH341USB0 \
    --id 1 \
    --kp 30 --kd 2 \
    --hz 100 \
    --duration 30 \
    --amp 1.0 --k 0.1
```

### 4. Plot recordings

```bash
python plot_trajectories.py                          # all recordings
python plot_trajectories.py --traj sin_time_square  # one type only
python plot_trajectories.py --save my_run.png
```

---

## Trajectory reference

| Name | Formula | Duration | Notes |
|---|---|---|---|
| `sin_time_square` | A·sin(k·t²) | 30 s | Chirp — freq grows linearly with time |
| `sin_sin` | sin(t)·π/2 + sin(5t)·0.5·sin(2t) | 30 s | Multi-frequency, non-periodic |
| `lift_and_drop` | Cubic to −π/2, then torque off | 6 s | Free-fall from t=2 s |
| `up_and_down` | Cubic: 0→π/2→0.8·π/2 | 6 s | Always torqued |
| `nothing` | 0, torque off | 6 s | Passive / backdrive baseline |

---

## Data format

Each recording is a CSV saved to `data_raw/<traj_name>/<timestamp>_id<N>_kp<kp>_kd<kd>.csv`.

| Column | Unit | Description |
|---|---|---|
| `t_s` | s | Elapsed time |
| `cmd_rad` | rad | Commanded position |
| `act_rad` | rad | Measured position |
| `err_rad` | rad | cmd − act |
| `vel_rads` | rad/s | Measured velocity |
| `torque_nm` | Nm | Measured torque |
| `temp_c` | °C | Motor temperature |

---

## Direct motor control (cli.py)

```bash
python src/cli.py read 1                          # single feedback read
python src/cli.py read-loop 1 --hz 50             # continuous poll
python src/cli.py goto 1 1.57 0.3                 # ramp to position
python src/cli.py sine 1 --amp 0.2 --freq 0.25   # sine sweep
python src/cli.py zero 1 --rate 0.3               # slow ramp to zero
python src/cli.py set-zero 1                      # set mech zero at current pos
python src/cli.py set-id 1 2                      # change CAN ID
python src/cli.py off 1                           # disable output
```

Port defaults to `/dev/ttyCH341USB0` or the `RS02_PORT` environment variable.

---

## CAN protocol

Communication is CH341 serial at **921600 baud**. Frames use the Robstride RS02
extended-ID format:

| Direction | CAN ID | Payload |
|---|---|---|
| TX | bits[28:24]=comm_type, [23:8]=torque_ff, [7:0]=motor_id | 8 B: pos[2] vel[2] kp[2] kd[2] |
| RX | bits[28:24]=COMM_FEEDBACK, [15:14]=mode, [13:8]=faults, [7:0]=motor_id | 8 B: pos[2] vel[2] torq[2] temp[2] |

RS02 physical limits: ±12.57 rad, ±44 rad/s, ±17 Nm, Kp 0–500, Kd 0–5.
