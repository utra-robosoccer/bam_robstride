"""
Low-level Robstride CAN-over-CH341 protocol helpers.

Frame wire format (CH341 AT-frame):
  b"AT" + struct.pack(">I", (can_id << 3) | 0x4) + bytes([len]) + data + b"\\r\\n"

29-bit extended CAN ID layout:
  TX (host→motor):  [28:24]=comm_type  [23:8]=host_id (or torque_ff)  [7:0]=motor_id
  RX (motor→host):  [28:24]=comm_type  [23:16]=mode/fault  [15:8]=motor_id  [7:0]=host_id
"""

import math
import struct

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

# ── defaults ──────────────────────────────────────────────────────────────────
BAUD    = 921600
HOST_ID = 0xFD

# ── CAN comm types ────────────────────────────────────────────────────────────
COMM_OBTAIN_ID = 0
COMM_CONTROL   = 1
COMM_FEEDBACK  = 2
COMM_ENABLE    = 3
COMM_STOP      = 4
COMM_SET_ZERO  = 6


# ── frame helpers ─────────────────────────────────────────────────────────────

def normalize(value, in_min, in_max, out_min, out_max):
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


def build_can_id(comm_type: int, host_id: int, motor_id: int) -> int:
    """Pack a 29-bit extended CAN ID for host→motor frames."""
    return (motor_id & 0x7F) | ((host_id & 0xFFFF) << 8) | ((comm_type & 0x1F) << 24)


def encode_frame(can_id: int, data: bytes) -> bytes:
    """Wrap a CAN frame in the CH341 USB-CAN AT-frame format."""
    addr = (can_id << 3) | 0x4
    return b"AT" + struct.pack(">I", addr) + bytes([len(data)]) + data + b"\r\n"


def decode_one_frame(buf: bytes):
    """
    Scan *buf* for the first complete AT frame.

    Returns (comm_type, motor_id, data, end_pos) or None.
    Motor ID is taken from bits [15:8] of the response CAN ID (Robstride v2 layout).
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


def encode_control(motor_id: int, pos_rad: float, vel_rads: float = 0.0,
                   kp: float = 30.0, kd: float = 2.0,
                   torque_nm: float = 0.0) -> bytes:
    """
    Build a complete MIT-mode Control frame (comm_type=1).

    Torque feedforward is packed into bits [23:8] of the CAN ID.
    data = [angle_u16, vel_u16, kp_u16, kd_u16]  (all big-endian).
    """
    angle_raw  = int(clamp(normalize(pos_rad,   MIN_ANGLE_RAD, MAX_ANGLE_RAD, 0, 65535), 0, 65535))
    vel_raw    = int(clamp(normalize(vel_rads,  MIN_VEL_RADS,  MAX_VEL_RADS,  0, 65535), 0, 65535))
    kp_raw     = int(clamp(normalize(kp,        MIN_KP,        MAX_KP,        0, 65535), 0, 65535))
    kd_raw     = int(clamp(normalize(kd,        MIN_KD,        MAX_KD,        0, 65535), 0, 65535))
    torque_raw = int(clamp(normalize(torque_nm, MIN_TORQUE_NM, MAX_TORQUE_NM, 0, 65535), 0, 65535))
    can_id = (motor_id & 0x7F) | ((torque_raw & 0xFFFF) << 8) | (COMM_CONTROL << 24)
    data   = struct.pack(">HHHH", angle_raw, vel_raw, kp_raw, kd_raw)
    return encode_frame(can_id, data)
