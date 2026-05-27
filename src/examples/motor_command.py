"""
motor_command.py — send MIT-mode position commands to an RS00 motor.

Control frame layout (comm_type=1, Robstride v2 / kscale encoding):
  CAN ID  [7:0]   motor_id
  CAN ID  [23:8]  torque_feedforward (u16, mapped from ±14 Nm → 0–65535)
  CAN ID  [28:24] 0x01 (Control)
  data[0:2]  target_angle  (u16 BE, mapped from ±4π rad  → 0–65535)
  data[2:4]  target_vel    (u16 BE, mapped from ±33 rad/s → 0–65535)
  data[4:6]  kp            (u16 BE, mapped from 0–500     → 0–65535)
  data[6:8]  kd            (u16 BE, mapped from 0–5       → 0–65535)

Usage:
    python motor_command.py                          # interactive prompt
    python motor_command.py --pos 1.0               # hold +1 rad
    python motor_command.py --pos -1.0 --kp 20      # hold -1 rad, lower stiffness
    python motor_command.py --pos 0 --kp 0 --kd 1   # zero-torque damped
"""

import argparse
import math
import struct
import time

import serial

# ── defaults ──────────────────────────────────────────────────────────────────
PORT     = "/dev/ttyCH341USB0"
BAUD     = 921600
HOST_ID  = 0xFD
MOTOR_ID = 1

CTRL_HZ  = 100          # control loop rate (Hz)
DEFAULT_KP = 30.0       # position gain  (N·m/rad),  RS00 range 0–500
DEFAULT_KD =  2.0       # velocity gain  (N·m·s/rad), RS00 range 0–5

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

# ── CAN comm types ────────────────────────────────────────────────────────────
COMM_CONTROL  = 1
COMM_FEEDBACK = 2
COMM_ENABLE   = 3
COMM_STOP     = 4


# ── helpers ───────────────────────────────────────────────────────────────────

def normalize(value, in_min, in_max, out_min, out_max):
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


def build_can_id_enable(host_id, motor_id):
    return (motor_id & 0x7F) | ((host_id & 0xFFFF) << 8) | (COMM_ENABLE << 24)


def build_can_id_stop(host_id, motor_id):
    return (motor_id & 0x7F) | ((host_id & 0xFFFF) << 8) | (COMM_STOP << 24)


def build_can_id_control(motor_id, torque_raw_u16):
    """Torque feedforward is packed into bits [23:8] of the control CAN ID."""
    return (motor_id & 0x7F) | ((torque_raw_u16 & 0xFFFF) << 8) | (COMM_CONTROL << 24)


def encode_frame(can_id, data: bytes) -> bytes:
    addr = (can_id << 3) | 0x4
    return b"AT" + struct.pack(">I", addr) + bytes([len(data)]) + data + b"\r\n"


def encode_control(motor_id, pos_rad, vel_rads=0.0, kp=DEFAULT_KP,
                   kd=DEFAULT_KD, torque_nm=0.0) -> bytes:
    """Build a complete MIT-mode control frame."""
    angle_raw  = int(clamp(normalize(pos_rad,    MIN_ANGLE_RAD, MAX_ANGLE_RAD, 0, 65535), 0, 65535))
    vel_raw    = int(clamp(normalize(vel_rads,   MIN_VEL_RADS,  MAX_VEL_RADS,  0, 65535), 0, 65535))
    kp_raw     = int(clamp(normalize(kp,         MIN_KP,        MAX_KP,        0, 65535), 0, 65535))
    kd_raw     = int(clamp(normalize(kd,         MIN_KD,        MAX_KD,        0, 65535), 0, 65535))
    torque_raw = int(clamp(normalize(torque_nm,  MIN_TORQUE_NM, MAX_TORQUE_NM, 0, 65535), 0, 65535))

    can_id = build_can_id_control(motor_id, torque_raw)
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


# ── control loop ──────────────────────────────────────────────────────────────

def run(port, motor_id, target_pos_rad, kp, kd, torque_ff=0.0):
    period = 1.0 / CTRL_HZ

    enable_pkt  = encode_frame(build_can_id_enable(HOST_ID, motor_id), bytes(8))
    stop_pkt    = encode_frame(build_can_id_stop(HOST_ID, motor_id),   bytes(8))

    print(f"\nPort={port}  motor_id={motor_id}  target={math.degrees(target_pos_rad):+.2f}°  "
          f"kp={kp}  kd={kd}  ff={torque_ff:.2f} Nm  @ {CTRL_HZ} Hz")
    print("Press Ctrl+C to stop.\n")
    print(f"{'t (s)':>7}  {'cmd (rad)':>10}  {'act (rad)':>10}  "
          f"{'err (rad)':>10}  {'vel (r/s)':>10}  {'torque (Nm)':>12}")
    print("-" * 70)

    with serial.Serial(port, BAUD, timeout=0.02) as ser:
        ser.reset_input_buffer()
        buf = b""

        # enable the motor first
        ser.write(enable_pkt)
        ser.flush()
        time.sleep(0.1)
        ser.reset_input_buffer()

        t0 = time.monotonic()
        try:
            while True:
                loop_start = time.monotonic()
                t = loop_start - t0

                ctrl_pkt = encode_control(motor_id, target_pos_rad,
                                          vel_rads=0.0, kp=kp, kd=kd,
                                          torque_nm=torque_ff)
                ser.write(ctrl_pkt)
                ser.flush()

                # read feedback
                deadline = loop_start + period * 0.8
                while time.monotonic() < deadline:
                    chunk = ser.read(256)
                    if chunk:
                        buf += chunk
                    result = decode_one_frame(buf)
                    if result:
                        ct, mid, data, end = result
                        buf = buf[end:]
                        if ct == COMM_FEEDBACK and mid == motor_id:
                            fb = decode_feedback(data)
                            if fb:
                                act_pos, vel, torque, _ = fb
                                err = target_pos_rad - act_pos
                                print(f"{t:7.3f}  {target_pos_rad:+10.4f}  {act_pos:+10.4f}  "
                                      f"{err:+10.4f}  {vel:+10.4f}  {torque:+12.4f}")
                            break

                elapsed = time.monotonic() - loop_start
                if elapsed < period:
                    time.sleep(period - elapsed)

        except KeyboardInterrupt:
            print("\nStopping...")
            ser.write(stop_pkt)
            ser.flush()
            time.sleep(0.1)


# ── entry point ───────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port",   default=PORT)
    ap.add_argument("--id",     type=int,   default=MOTOR_ID)
    ap.add_argument("--pos",    type=float, default=None,
                    help="Target position in radians (default: interactive prompt)")
    ap.add_argument("--kp",     type=float, default=DEFAULT_KP)
    ap.add_argument("--kd",     type=float, default=DEFAULT_KD)
    ap.add_argument("--torque", type=float, default=0.0,
                    help="Torque feedforward in Nm")
    args = ap.parse_args()

    if args.pos is None:
        val = input("Target position in radians (e.g. 1.0, -1.0, 0): ").strip()
        target = float(val)
    else:
        target = args.pos

    # safety clamp to ±1 rad for bench testing
    target = clamp(target, -1.0, 1.0)
    print(f"Clamped to ±1 rad safety limit → {target:+.4f} rad")

    run(args.port, args.id, target, args.kp, args.kd, args.torque)


if __name__ == "__main__":
    main()
