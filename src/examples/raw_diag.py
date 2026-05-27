"""
Raw CH341 diagnostic: directly probe the motor without the kscale library.
Sends ObtainID and Enable to motor ID 1 (and a few others) and decodes any response.
"""

import serial
import struct
import time

PORT = "/dev/ttyCH341USB0"
BAUD = 921600
HOST_ID = 0xFD  # standard host ID used by kscale library

# CommunicationType values
OBTAIN_ID = 0
CONTROL   = 1
FEEDBACK  = 2
ENABLE    = 3
STOP      = 4


def build_can_id(comm_type: int, host_id: int, motor_id: int) -> int:
    """Reconstruct the CAN ID as the kscale library does."""
    return (motor_id & 0x7F) | ((host_id & 0xFFFF) << 8) | ((comm_type & 0x1F) << 24)


def encode_frame(can_id: int, data: bytes) -> bytes:
    """Encode a CAN frame in the AT-framed CH341 protocol."""
    addr = (can_id << 3) | 0x4
    addr_bytes = struct.pack(">I", addr)
    return b"AT" + addr_bytes + bytes([len(data)]) + data + b"\r\n"


def decode_frame(buf: bytes):
    """Decode a single AT-framed CAN packet. Returns (can_id, data) or None."""
    if len(buf) < 8:
        return None
    if buf[0:2] != b"AT":
        return None
    raw_id = struct.unpack(">I", buf[2:6])[0]
    can_id = (raw_id >> 3) & 0x1FFFFFFF
    data_len = buf[6]
    total = 7 + data_len + 2
    if len(buf) < total:
        return None
    if buf[total-2:total] != b"\r\n":
        return None
    data = buf[7:7+data_len]
    return can_id, data, total


def decode_can_id(can_id: int):
    """Split a CAN ID into its fields."""
    motor_id  = can_id & 0x7F
    host_id   = (can_id >> 8) & 0xFFFF
    comm_type = (can_id >> 24) & 0x1F
    return comm_type, host_id, motor_id


COMM_NAMES = {
    0: "ObtainID", 1: "Control", 2: "Feedback", 3: "Enable",
    4: "Stop", 6: "SetZero", 7: "SetID", 17: "Read", 18: "Write",
    19: "ParaStrInfo", 21: "Fault",
}


with serial.Serial(PORT, BAUD, timeout=0.5) as ser:
    print(f"Opened {PORT} at {BAUD} baud\n")
    ser.reset_input_buffer()

    # Listen first for any spontaneous messages (motor might already be broadcasting)
    print("=== Listening 1s for spontaneous messages ===")
    t0 = time.time()
    raw = b""
    while time.time() - t0 < 1.0:
        chunk = ser.read(256)
        if chunk:
            raw += chunk
    if raw:
        print(f"  Got {len(raw)} bytes spontaneously: {raw.hex()}")
        # Try to decode
        pos = 0
        while pos < len(raw) - 7:
            if raw[pos:pos+2] == b"AT":
                result = decode_frame(raw[pos:])
                if result:
                    can_id, data, msg_len = result
                    ct, hid, mid = decode_can_id(can_id)
                    print(f"  [{COMM_NAMES.get(ct, ct)}] motor={mid} host={hex(hid)} data={data.hex()}")
                    pos += msg_len
                    continue
            pos += 1
    else:
        print("  No spontaneous messages (expected)")

    # Try sending ObtainID and Enable to IDs 1 through 5
    for motor_id in [1, 2, 3, 4, 5]:
        for cmd_name, comm_type, extra_data in [
            ("ObtainID", OBTAIN_ID, bytes(8)),
            ("Enable",   ENABLE,    bytes(8)),
        ]:
            can_id = build_can_id(comm_type, HOST_ID, motor_id)
            pkt = encode_frame(can_id, extra_data)
            print(f"\n>>> Sending {cmd_name} to motor {motor_id}  CAN_ID=0x{can_id:08X}  bytes={pkt.hex()}")
            ser.write(pkt)
            ser.flush()
            time.sleep(0.05)

            raw = b""
            t0 = time.time()
            while time.time() - t0 < 0.15:
                chunk = ser.read(256)
                if chunk:
                    raw += chunk

            if not raw:
                print(f"  No response")
                continue

            print(f"  Raw response ({len(raw)} bytes): {raw.hex()}")
            pos = 0
            while pos < len(raw) - 7:
                if raw[pos:pos+2] == b"AT":
                    result = decode_frame(raw[pos:])
                    if result:
                        can_id_r, data, msg_len = result
                        ct, hid, mid = decode_can_id(can_id_r)
                        print(f"  DECODED [{COMM_NAMES.get(ct, ct)}] motor={mid} host={hex(hid)} data={data.hex()}")
                        pos += msg_len
                        continue
                pos += 1

    print("\n=== Done ===")
