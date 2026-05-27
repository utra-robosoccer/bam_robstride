"""
motor_read.py — continuously read and print RS00 position/velocity/torque.

Sends Enable (comm_type=3) every loop iteration; the motor replies with a
Feedback frame (comm_type=2) each time.  No kscale library needed.

Usage:
    python motor_read.py [--port /dev/ttyCH341USB0] [--id 1] [--hz 10]
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
LOOP_HZ  = 10          # how often to poll the motor

# ── RS00 physical limits ──────────────────────────────────────────────────────
MIN_ANGLE_RAD = -4.0 * math.pi
MAX_ANGLE_RAD =  4.0 * math.pi
MIN_VEL_RADS  = -33.0
MAX_VEL_RADS  =  33.0
MIN_TORQUE_NM = -14.0
MAX_TORQUE_NM =  14.0

# ── CAN comm types ────────────────────────────────────────────────────────────
COMM_FEEDBACK = 2
COMM_ENABLE   = 3
COMM_STOP     = 4


# ── helpers ───────────────────────────────────────────────────────────────────

def normalize(value, in_min, in_max, out_min, out_max):
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


def build_can_id(comm_type, host_id, motor_id):
    """Pack a 29-bit extended CAN ID per Robstride v2 host→motor format."""
    return (motor_id & 0x7F) | ((host_id & 0xFFFF) << 8) | ((comm_type & 0x1F) << 24)


def encode_frame(can_id, data: bytes) -> bytes:
    """Wrap a CAN frame in the CH341 USB-CAN AT-frame format."""
    addr = (can_id << 3) | 0x4
    return b"AT" + struct.pack(">I", addr) + bytes([len(data)]) + data + b"\r\n"


def decode_one_frame(buf: bytes):
    """
    Scan *buf* for the first complete AT frame.
    Returns (comm_type, motor_id, data_bytes, end_pos) or None.

    Robstride v2 response CAN ID layout (29 bits):
      [28:24] comm_type
      [23:16] mode / fault flags
      [15:8]  motor_id  ← correct field (NOT bits [6:0])
      [7:0]   host_id
    """
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
    """
    Decode the 8-byte feedback payload.
    Returns (angle_rad, vel_rads, torque_nm, temp_c) or None.
    """
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


# ── main ──────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=PORT)
    ap.add_argument("--id",   type=int, default=MOTOR_ID)
    ap.add_argument("--hz",   type=float, default=LOOP_HZ)
    args = ap.parse_args()

    period     = 1.0 / args.hz
    enable_pkt = encode_frame(build_can_id(COMM_ENABLE, HOST_ID, args.id), bytes(8))
    stop_pkt   = encode_frame(build_can_id(COMM_STOP,   HOST_ID, args.id), bytes(8))

    print(f"Reading RS00  motor_id={args.id}  port={args.port}  {args.hz:.0f} Hz")
    print("Press Ctrl+C to stop.\n")
    print(f"{'t (s)':>8}  {'pos (rad)':>10}  {'pos (deg)':>10}  "
          f"{'vel (rad/s)':>12}  {'torque (Nm)':>12}  {'temp (°C)':>9}")
    print("-" * 72)

    with serial.Serial(args.port, BAUD, timeout=0.05) as ser:
        ser.reset_input_buffer()
        buf = b""
        t0  = time.monotonic()

        try:
            while True:
                loop_start = time.monotonic()

                ser.write(enable_pkt)
                ser.flush()

                # collect bytes until we see a feedback frame or time out
                deadline = loop_start + period * 0.8
                got_fb   = False
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
                                angle_rad, vel_rads, torque_nm, temp_c = fb
                                t = loop_start - t0
                                print(f"{t:8.3f}  {angle_rad:+10.4f}  "
                                      f"{math.degrees(angle_rad):+10.3f}  "
                                      f"{vel_rads:+12.4f}  {torque_nm:+12.4f}  "
                                      f"{temp_c:9.1f}")
                                got_fb = True
                                break

                if not got_fb:
                    print("  (no feedback — check motor power and ID)")

                # sleep remainder of period
                elapsed = time.monotonic() - loop_start
                if elapsed < period:
                    time.sleep(period - elapsed)

        except KeyboardInterrupt:
            print("\nStopping motor...")
            ser.write(stop_pkt)
            ser.flush()
            time.sleep(0.1)
            print("Done.")


if __name__ == "__main__":
    main()
