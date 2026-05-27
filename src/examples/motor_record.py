"""
motor_record.py — run a motion routine and record commanded vs actual position.

Designed for the BAM-style experiment: motor under a lever-arm torque load,
measuring the error between commanded and actual position at each timestep.

Routines (select with --routine):
  step      ±1 rad square wave, hold each side for --hold seconds
  ramp      linear sweep from -1 to +1 rad and back
  sine      sinusoidal ±1 rad at --freq Hz
  chirp     frequency sweep from --freq_lo to --freq_hi Hz over the run

Output: CSV with columns
  t_s, cmd_rad, act_rad, err_rad, vel_rads, torque_nm, temp_c

Usage examples:
  python motor_record.py --routine sine   --freq 0.5  --duration 30
  python motor_record.py --routine step   --hold 2.0  --duration 20
  python motor_record.py --routine ramp   --duration 20
  python motor_record.py --routine chirp  --freq_lo 0.1 --freq_hi 2.0 --duration 40
  python motor_record.py --routine sine   --kp 15 --kd 1  # softer gains
"""

import argparse
import csv
import math
import os
import struct
import time

import serial

# ── defaults ──────────────────────────────────────────────────────────────────
PORT     = "/dev/ttyCH341USB0"
BAUD     = 921600
HOST_ID  = 0xFD
MOTOR_ID = 1

CTRL_HZ    = 100        # control loop rate (Hz)
DEFAULT_KP = 30.0       # position gain  (N·m/rad)
DEFAULT_KD =  2.0       # velocity gain  (N·m·s/rad)
AMPLITUDE  =  1.0       # motion amplitude (rad)  — keep at 1.0 for BAM

# ── RS00 physical limits ──────────────────────────────────────────────────────
MIN_ANGLE_RAD = -4.0 * math.pi
MAX_ANGLE_RAD =  4.0 * math.pi
MIN_VEL_RADS  = -33.0
MAX_VEL_RADS  =  33.0
MIN_TORQUE_NM = -14.0
MAX_TORQUE_NM =  14.0
MIN_KP        =   0.0
MAX_KP        = 500.0
MIN_KD        =   0.0
MAX_KD        =   5.0

COMM_CONTROL  = 1
COMM_FEEDBACK = 2
COMM_ENABLE   = 3
COMM_STOP     = 4


# ── motion routines ───────────────────────────────────────────────────────────

def routine_step(t, amp=AMPLITUDE, hold=2.0):
    """
    Square wave: hold +amp for `hold` seconds, then -amp for `hold` seconds.
    The abrupt step reveals motor lag, overshoot, and settling time.
    """
    phase = t % (2 * hold)
    return amp if phase < hold else -amp


def routine_ramp(t, amp=AMPLITUDE, period=4.0):
    """
    Triangle wave (constant velocity): linear ramp ±amp with period `period`.
    Reveals steady-state tracking error under constant velocity.
    """
    phase = t % period
    if phase < period / 2:
        return amp * (2 * phase / (period / 2) - 1)
    else:
        return amp * (1 - 2 * (phase - period / 2) / (period / 2))


def routine_sine(t, amp=AMPLITUDE, freq=0.5):
    """
    Sinusoidal motion at fixed frequency.
    Reveals frequency-domain tracking error (gain and phase).
    """
    return amp * math.sin(2 * math.pi * freq * t)


def routine_chirp(t, amp=AMPLITUDE, freq_lo=0.1, freq_hi=2.0, duration=40.0):
    """
    Linear frequency sweep (chirp) from freq_lo to freq_hi over `duration` s.
    Best for full system-identification: one experiment covers all frequencies.
    Instantaneous frequency: f(t) = freq_lo + (freq_hi - freq_lo) * t / duration
    Integrated phase:       φ(t) = 2π * [freq_lo*t + (freq_hi-freq_lo)*t²/(2*duration)]
    """
    phi = 2 * math.pi * (freq_lo * t + (freq_hi - freq_lo) * t * t / (2 * duration))
    return amp * math.sin(phi)


def get_routine(name, args):
    """Return a callable cmd(t) → position_rad for the selected routine."""
    if name == "step":
        return lambda t: routine_step(t, AMPLITUDE, args.hold)
    if name == "ramp":
        return lambda t: routine_ramp(t, AMPLITUDE, 2 * args.hold)
    if name == "sine":
        return lambda t: routine_sine(t, AMPLITUDE, args.freq)
    if name == "chirp":
        return lambda t: routine_chirp(t, AMPLITUDE, args.freq_lo, args.freq_hi, args.duration)
    raise ValueError(f"Unknown routine: {name}")


# ── protocol helpers ──────────────────────────────────────────────────────────

def normalize(value, in_min, in_max, out_min, out_max):
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def encode_frame(can_id, data: bytes) -> bytes:
    addr = (can_id << 3) | 0x4
    return b"AT" + struct.pack(">I", addr) + bytes([len(data)]) + data + b"\r\n"


def encode_control(motor_id, pos_rad, vel_rads=0.0,
                   kp=DEFAULT_KP, kd=DEFAULT_KD, torque_nm=0.0) -> bytes:
    """
    Build a MIT-mode Control frame (comm_type=1).
    Torque feedforward is packed into bits [23:8] of the CAN ID.
    data[0:2]=angle, [2:4]=vel, [4:6]=kp, [6:8]=kd  (all u16 big-endian).
    """
    angle_raw  = int(clamp(normalize(pos_rad,   MIN_ANGLE_RAD, MAX_ANGLE_RAD, 0, 65535), 0, 65535))
    vel_raw    = int(clamp(normalize(vel_rads,  MIN_VEL_RADS,  MAX_VEL_RADS,  0, 65535), 0, 65535))
    kp_raw     = int(clamp(normalize(kp,        MIN_KP,        MAX_KP,        0, 65535), 0, 65535))
    kd_raw     = int(clamp(normalize(kd,        MIN_KD,        MAX_KD,        0, 65535), 0, 65535))
    torque_raw = int(clamp(normalize(torque_nm, MIN_TORQUE_NM, MAX_TORQUE_NM, 0, 65535), 0, 65535))

    can_id = (motor_id & 0x7F) | ((torque_raw & 0xFFFF) << 8) | (COMM_CONTROL << 24)
    data   = struct.pack(">HHHH", angle_raw, vel_raw, kp_raw, kd_raw)
    return encode_frame(can_id, data)


def decode_one_frame(buf: bytes):
    for i in range(len(buf) - 7):
        if buf[i:i+2] != b"AT":
            continue
        if i + 8 > len(buf):
            break
        data_len = buf[i + 6]
        total    = 7 + data_len + 2
        if i + total > len(buf):
            break
        if buf[i + total - 2 : i + total] != b"\r\n":
            continue
        raw_id    = struct.unpack(">I", buf[i+2:i+6])[0]
        can_id    = (raw_id >> 3) & 0x1FFFFFFF
        comm_type = (can_id >> 24) & 0x1F
        motor_id  = (can_id >> 8)  & 0xFF
        data      = buf[i+7 : i+7+data_len]
        return comm_type, motor_id, data, i + total
    return None


def decode_feedback(data: bytes):
    if len(data) < 8:
        return None
    angle_raw  = struct.unpack(">H", data[0:2])[0]
    vel_raw    = struct.unpack(">H", data[2:4])[0]
    torque_raw = struct.unpack(">H", data[4:6])[0]
    temp_raw   = struct.unpack(">H", data[6:8])[0]
    angle_rad  = normalize(angle_raw,  0, 65535, MIN_ANGLE_RAD, MAX_ANGLE_RAD)
    vel_rads   = normalize(vel_raw,    0, 65535, MIN_VEL_RADS,  MAX_VEL_RADS)
    torque_nm  = normalize(torque_raw, 0, 65535, MIN_TORQUE_NM, MAX_TORQUE_NM)
    temp_c     = temp_raw / 10.0
    return angle_rad, vel_rads, torque_nm, temp_c


# ── main recording loop ───────────────────────────────────────────────────────

def run(args):
    period   = 1.0 / CTRL_HZ
    cmd_func = get_routine(args.routine, args)

    enable_pkt = encode_frame(
        (args.id & 0x7F) | ((HOST_ID & 0xFFFF) << 8) | (COMM_ENABLE << 24),
        bytes(8)
    )
    stop_pkt = encode_frame(
        (args.id & 0x7F) | ((HOST_ID & 0xFFFF) << 8) | (COMM_STOP << 24),
        bytes(8)
    )

    # prepare output CSV path
    ts_str   = time.strftime("%Y%m%d_%H%M%S")
    out_path = args.output or f"bam_{args.routine}_{ts_str}.csv"
    out_path = os.path.abspath(out_path)

    print(f"\n{'='*60}")
    print(f"  Routine   : {args.routine}")
    print(f"  Duration  : {args.duration:.1f} s")
    print(f"  Rate      : {CTRL_HZ} Hz  (period {period*1000:.1f} ms)")
    print(f"  kp / kd   : {args.kp} / {args.kd}")
    print(f"  Amplitude : ±{AMPLITUDE:.2f} rad  (±{math.degrees(AMPLITUDE):.1f}°)")
    print(f"  Motor ID  : {args.id}  on {args.port}")
    print(f"  Output    : {out_path}")
    print(f"{'='*60}")
    print("Press Ctrl+C to abort early (data up to that point is saved).\n")

    rows = []  # accumulate in memory, write at end

    with serial.Serial(args.port, BAUD, timeout=0.02) as ser:
        ser.reset_input_buffer()
        buf = b""

        # enable motor
        ser.write(enable_pkt)
        ser.flush()
        time.sleep(0.15)
        ser.reset_input_buffer()

        t0       = time.monotonic()
        n_missed = 0

        print(f"{'t (s)':>7}  {'cmd (°)':>8}  {'act (°)':>8}  "
              f"{'err (°)':>8}  {'vel(r/s)':>9}  {'τ (Nm)':>8}")
        print("-" * 60)

        try:
            while True:
                loop_start = time.monotonic()
                t = loop_start - t0

                if t >= args.duration:
                    break

                cmd_pos = cmd_func(t)

                # send control command
                ctrl_pkt = encode_control(
                    args.id, cmd_pos,
                    vel_rads=0.0,          # velocity feedforward = 0
                    kp=args.kp, kd=args.kd,
                    torque_nm=0.0,         # torque feedforward = 0
                )
                ser.write(ctrl_pkt)
                ser.flush()

                # collect feedback
                got_fb   = False
                deadline = loop_start + period * 0.85
                while time.monotonic() < deadline:
                    chunk = ser.read(256)
                    if chunk:
                        buf += chunk
                    result = decode_one_frame(buf)
                    if result:
                        ct, mid, data, end = result
                        buf = buf[end:]
                        if ct == COMM_FEEDBACK and mid == args.id:
                            fb = decode_feedback(data)
                            if fb:
                                act_pos, vel, torque, temp = fb
                                err = cmd_pos - act_pos
                                rows.append({
                                    "t_s":      round(t, 6),
                                    "cmd_rad":  round(cmd_pos, 6),
                                    "act_rad":  round(act_pos, 6),
                                    "err_rad":  round(err, 6),
                                    "vel_rads": round(vel, 6),
                                    "torque_nm":round(torque, 6),
                                    "temp_c":   round(temp, 2),
                                })
                                # print every ~10 rows to avoid overwhelming the terminal
                                if len(rows) % 10 == 1:
                                    print(f"{t:7.3f}  {math.degrees(cmd_pos):+8.2f}  "
                                          f"{math.degrees(act_pos):+8.2f}  "
                                          f"{math.degrees(err):+8.2f}  "
                                          f"{vel:+9.3f}  {torque:+8.3f}")
                                got_fb = True
                                break

                if not got_fb:
                    n_missed += 1

                # sleep to maintain loop rate
                elapsed = time.monotonic() - loop_start
                if elapsed < period:
                    time.sleep(period - elapsed)

        except KeyboardInterrupt:
            print("\n[interrupted by user]")

        finally:
            print("\nStopping motor...")
            ser.write(stop_pkt)
            ser.flush()
            time.sleep(0.1)

    # ── save CSV ──────────────────────────────────────────────────────────────
    if rows:
        fieldnames = ["t_s", "cmd_rad", "act_rad", "err_rad", "vel_rads",
                      "torque_nm", "temp_c"]
        with open(out_path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(rows)
        print(f"\nSaved {len(rows)} rows → {out_path}")
        print(f"Missed feedback frames: {n_missed}")
    else:
        print("No data recorded.")


# ── entry point ───────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(
        description="Run a motion routine on RS00 and record cmd vs actual position."
    )
    ap.add_argument("--port",     default=PORT)
    ap.add_argument("--id",       type=int,   default=MOTOR_ID)
    ap.add_argument("--routine",  default="sine",
                    choices=["step", "ramp", "sine", "chirp"],
                    help="Motion routine to run (default: sine)")
    ap.add_argument("--duration", type=float, default=30.0,
                    help="Total recording time in seconds (default: 30)")
    ap.add_argument("--kp",       type=float, default=DEFAULT_KP,
                    help=f"Position gain (default: {DEFAULT_KP})")
    ap.add_argument("--kd",       type=float, default=DEFAULT_KD,
                    help=f"Velocity gain (default: {DEFAULT_KD})")
    # step / ramp
    ap.add_argument("--hold",     type=float, default=2.0,
                    help="Seconds per step/ramp half-cycle (default: 2.0)")
    # sine
    ap.add_argument("--freq",     type=float, default=0.5,
                    help="Sine frequency in Hz (default: 0.5)")
    # chirp
    ap.add_argument("--freq_lo",  type=float, default=0.1,
                    help="Chirp start frequency in Hz (default: 0.1)")
    ap.add_argument("--freq_hi",  type=float, default=2.0,
                    help="Chirp end frequency in Hz (default: 2.0)")
    # output
    ap.add_argument("--output",   default=None,
                    help="Output CSV path (default: bam_<routine>_<timestamp>.csv)")
    args = ap.parse_args()

    run(args)


if __name__ == "__main__":
    main()
