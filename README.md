# BAM Pipeline — Robstride Motor Identification

Backlash-Aware Model (BAM) identification pipeline for Robstride RS00 actuators, following the [Rhoban methodology](https://rhoban.com). Communicates via raw CAN-over-USB using the CH341 adapter and MIT-mode control frames. Runs motion routines (step, ramp, sine, chirp) and records commanded vs. actual position for offline friction and backlash parameter estimation.

---

## What it does

- Sends MIT-mode position commands (Kp/Kd impedance control) at up to 100 Hz
- Streams motor feedback: angle, velocity, torque, temperature
- Executes predefined excitation trajectories including a chirp (`A·sin(k·t²)`) for broadband system identification
- Records all channels to CSV for post-processing and BAM parameter fitting
- Provides standalone utilities for zeroing and continuous position monitoring

---

## Prerequisites

### Hardware

- **USB CAN debugger** — CH341-based USB-to-CAN adapter (e.g. "USB CAN Analyzer")
- Robstride RS00 actuator with CAN bus wired to the adapter

### Software

**CH341 Linux kernel driver** — install before plugging in the adapter:

```bash
git clone https://github.com/WCHSoftGroup/ch341ser_linux
cd ch341ser_linux
make
sudo make install
```

After installation the device appears as `/dev/ttyCH341USB0`.

**Python environment** — activate the included venv:

```bash
source .bam_env/bin/activate
```

---

## Workspace structure

```
bam_ws/
├── src/
│   ├── robstride/
│   │   ├── __init__.py          # Public API (RobstrideMotor, MotorState)
│   │   ├── motor.py             # High-level motor class (open/enable/set_angle/zero)
│   │   └── protocol.py          # CAN frame encode/decode, RS00 physical limits
│   ├── trajectories/
│   │   ├── sin_time_square.py   # Chirp A·sin(k·t²) — primary BAM excitation
│   │   ├── sin_sin.py           # Sinusoidal trajectory
│   │   ├── lift_and_drop.py     # Lift-and-drop routine
│   │   └── up_and_down.py       # Reciprocal up/down motion
│   ├── examples/
│   │   ├── motor_command.py     # Interactive MIT-mode position commands
│   │   ├── motor_record.py      # Record step/ramp/sine/chirp to CSV
│   │   ├── motor_read.py        # Raw protocol state reader
│   │   └── supervisor.py        # Multi-motor supervision demo
│   ├── read_motor.py            # Continuous position/velocity/torque monitor
│   └── zero_motor.py            # Set current position as zero reference
├── params/
│   └── rs00/                    # Motor parameter files
├── data_raw/                    # Output directory for recorded CSVs
├── plot_trajectories.py         # Plot cmd vs actual from CSV files
└── .bam_env/                    # Python virtual environment
```

---

## Basic usage

### 1. Zero the motor

```bash
python src/zero_motor.py --port /dev/ttyCH341USB0 --id 1
```

### 2. Monitor live feedback

```bash
python src/read_motor.py --port /dev/ttyCH341USB0 --id 1 --hz 100
```

### 3. Run the chirp identification trajectory

```bash
python src/trajectories/sin_time_square.py --port /dev/ttyCH341USB0 --id 1
```

Output CSV is written to `data_raw/` with columns: `t_s, cmd_rad, act_rad, err_rad, vel_rads, torque_nm, temp_c`.

### 4. Run other excitation routines

```bash
python src/examples/motor_record.py --routine sine  --freq 0.5 --duration 30 --kp 30 --kd 2
python src/examples/motor_record.py --routine chirp --duration 30 --kp 30 --kd 2
python src/examples/motor_record.py --routine step  --duration 20 --kp 50 --kd 3
```

### 5. Plot results

```bash
python plot_trajectories.py
```

Reads all `sin_time_square_*.csv` files in the working directory and saves `trajectories.png`.

---

## CAN protocol notes

Communication is CH341 serial at **921600 baud**. Frames use the Robstride v2 extended-ID format:

| Direction | CAN ID bits | Payload |
|-----------|-------------|---------|
| TX | `[28:24]` comm_type, `[23:8]` host/torque, `[7:0]` motor_id | 8-byte control data |
| RX | `[28:24]` comm_type, `[23:16]` mode/fault, `[15:8]` motor_id, `[7:0]` host_id | 8-byte feedback |

RS00 physical limits: ±4π rad, ±33 rad/s, ±14 Nm, Kp 0–500, Kd 0–5.
