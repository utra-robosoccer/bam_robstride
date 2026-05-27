"""
Direct motor position reader using raw CH341 serial protocol.
Bypasses the kscale library entirely. Works with Robstride 00 at motor ID 1.
"""

import math
import serial
import struct
import time

PORT     = "/dev/ttyCH341USB0"
BAUD     = 921600
HOST_ID  = 0xFD
MOTOR_ID = 1

# RS00 physical limits (from actuator/robstride/src/actuators/robstride00.rs)
MIN_ANGLE_RAD  = -4.0 * math.pi
MAX_ANGLE_RAD  =  4.0 * math.pi
MIN_VEL_RADS   = -33.0
MAX_VEL_RADS   =  33.0
MIN_TORQUE_NM  = -14.0
MAX_TORQUE_NM  =  14.0

COMM_ENABLE   = 3
COMM_FEEDBACK = 2
COMM_STOP     = 4


def build_can_id(comm_type, host_id, motor_id):
    return (motor_id & 0x7F) | ((host_id & 0xFFFF) << 8) | ((comm_type & 0x1F) << 24)


def encode_frame(can_id, data):
    addr = (can_id << 3) | 0x4
    return b"AT" + struct.pack(">I", addr) + bytes([len(data)]) + data + b"\r\n"


def decode_one_frame(buf):
    """Find and decode the first complete AT frame in buf.
    Returns (comm_type, motor_id, data, end_pos) or None.

    Robstride v2 CAN extended ID layout (29 bits):
      [28:24] = message type
      [23:16] = mode/fault info (in feedback) or reserved
      [15:8]  = motor_id
      [7:0]   = host_id
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
        if buf[i + total - 2:i + total] != b"\r\n":
            continue
        raw_id    = struct.unpack(">I", buf[i+2:i+6])[0]
        can_id    = (raw_id >> 3) & 0x1FFFFFFF
        comm_type = (can_id >> 24) & 0x1F
        motor_id  = (can_id >> 8) & 0xFF   # actual motor ID per Robstride v2
        data      = buf[i+7:i+7+data_len]
        return comm_type, motor_id, data, i + total
    return None


def normalize(value, in_min, in_max, out_min, out_max):
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


def decode_feedback(data):
    """Decode 8-byte feedback payload. Returns (angle_deg, velocity_rads, torque_nm, temp_c)."""
    if len(data) < 8:
        return None
    angle_raw    = struct.unpack(">H", data[0:2])[0]
    velocity_raw = struct.unpack(">H", data[2:4])[0]
    torque_raw   = struct.unpack(">H", data[4:6])[0]
    temp_raw     = struct.unpack(">H", data[6:8])[0]

    # Step 1: raw u16 → internal normalized float (-100..100)
    angle_norm    = normalize(angle_raw,    0, 65535, -100.0, 100.0)
    velocity_norm = normalize(velocity_raw, 0, 65535, -100.0, 100.0)
    torque_norm   = normalize(torque_raw,   0, 65535, -100.0, 100.0)

    # Step 2: normalized → physical units (RS00 limits)
    angle_rad  = normalize(angle_norm,    -100.0, 100.0, MIN_ANGLE_RAD, MAX_ANGLE_RAD)
    vel_rads   = normalize(velocity_norm, -100.0, 100.0, MIN_VEL_RADS,  MAX_VEL_RADS)
    torque_nm  = normalize(torque_norm,   -100.0, 100.0, MIN_TORQUE_NM, MAX_TORQUE_NM)
    temp_c     = temp_raw / 10.0

    return math.degrees(angle_rad), vel_rads, torque_nm, temp_c


enable_pkt = encode_frame(build_can_id(COMM_ENABLE, HOST_ID, MOTOR_ID), bytes(8))
stop_pkt   = encode_frame(build_can_id(COMM_STOP,   HOST_ID, MOTOR_ID), bytes(8))

print(f"Reading position from RS00 motor ID={MOTOR_ID} on {PORT}")
print(f"Press Ctrl+C to quit.\n")

with serial.Serial(PORT, BAUD, timeout=0.1) as ser:
    ser.reset_input_buffer()
    buf = b""

    try:
        while True:
            # Send Enable — motor responds with a Feedback frame
            ser.write(enable_pkt)
            ser.flush()

            # Read for up to 150 ms
            deadline = time.monotonic() + 0.15
            while time.monotonic() < deadline:
                chunk = ser.read(256)
                if chunk:
                    buf += chunk
                result = decode_one_frame(buf)
                if result:
                    comm_type, mid, data, end_pos = result
                    buf = buf[end_pos:]
                    if comm_type == COMM_FEEDBACK:
                        decoded = decode_feedback(data)
                        if decoded:
                            angle_deg, vel_rads, torque_nm, temp_c = decoded
                            print(f"motor={mid}  pos={angle_deg:+8.3f}°  vel={vel_rads:+7.3f} rad/s  "
                                  f"torque={torque_nm:+6.3f} Nm  temp={temp_c:.1f}°C")
                        break
            else:
                print("(no feedback received — check motor power and ID)")

            time.sleep(0.2)

    except KeyboardInterrupt:
        print("\nSending Stop before exit...")
        ser.write(stop_pkt)
        ser.flush()
        time.sleep(0.1)
        print("Done.")
