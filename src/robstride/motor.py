"""
RobstrideMotor — high-level interface for a single Robstride RS00 motor.

Typical usage:

    # context manager (auto stop + close on exit)
    with RobstrideMotor(port="/dev/ttyCH341USB0", motor_id=1) as m:
        state = m.read_feedback()
        print(f"angle: {state.angle_rad:.3f} rad")
        m.set_angle(1.0)

    # explicit open/close
    m = RobstrideMotor()
    m.open()
    m.zero()
    angle = m.read_angle()
    m.stop()
    m.close()
"""

import time
from typing import NamedTuple, Optional

import serial

from .protocol import (
    BAUD, HOST_ID,
    COMM_ENABLE, COMM_STOP, COMM_FEEDBACK, COMM_SET_ZERO,
    build_can_id, encode_frame, encode_control,
    decode_one_frame, decode_feedback,
)


class MotorState(NamedTuple):
    angle_rad: float   # position in radians
    vel_rads:  float   # velocity in rad/s
    torque_nm: float   # torque in N·m
    temp_c:    float   # temperature in °C


class RobstrideMotor:
    """
    Single-motor interface over the CH341 USB-CAN adapter.

    Parameters
    ----------
    port     : serial port, e.g. "/dev/ttyCH341USB0"
    motor_id : Robstride CAN motor ID (default 1)
    baud     : serial baud rate (default 921600)
    host_id  : host CAN ID (default 0xFD)
    """

    def __init__(self,
                 port:     str = "/dev/ttyCH341USB0",
                 motor_id: int = 1,
                 baud:     int = BAUD,
                 host_id:  int = HOST_ID):
        self.port     = port
        self.motor_id = motor_id
        self.baud     = baud
        self.host_id  = host_id
        self._ser: Optional[serial.Serial] = None
        self._buf = b""

    # ── lifecycle ─────────────────────────────────────────────────────────────

    def open(self) -> None:
        """Open the serial port."""
        self._ser = serial.Serial(self.port, self.baud, timeout=0.02)
        self._ser.reset_input_buffer()
        self._buf = b""

    def close(self) -> None:
        """Close the serial port."""
        if self._ser and self._ser.is_open:
            self._ser.close()
        self._ser = None

    def __enter__(self) -> "RobstrideMotor":
        self.open()
        return self

    def __exit__(self, *_) -> None:
        self.stop()
        self.close()

    # ── low-level send / receive ───────────────────────────────────────────────

    def _send(self, pkt: bytes) -> None:
        self._ser.write(pkt)
        self._ser.flush()

    def _wait_feedback(self, timeout: float = 0.15) -> Optional[MotorState]:
        """Drain the serial buffer until a Feedback frame arrives or timeout."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            chunk = self._ser.read(256)
            if chunk:
                self._buf += chunk
            result = decode_one_frame(self._buf)
            if result:
                ct, mid, data, end = result
                self._buf = self._buf[end:]
                if ct == COMM_FEEDBACK and mid == self.motor_id:
                    fb = decode_feedback(data)
                    if fb:
                        return MotorState(*fb)
        return None

    # ── motor commands ────────────────────────────────────────────────────────

    def enable(self) -> None:
        """Send an Enable frame (comm_type=3). Motor will reply with feedback."""
        pkt = encode_frame(build_can_id(COMM_ENABLE, self.host_id, self.motor_id), bytes(8))
        self._send(pkt)

    def stop(self) -> None:
        """Send a Stop frame (comm_type=4). Motor de-energises."""
        pkt = encode_frame(build_can_id(COMM_STOP, self.host_id, self.motor_id), bytes(8))
        self._send(pkt)

    # ── read ──────────────────────────────────────────────────────────────────

    def read_feedback(self, timeout: float = 0.15) -> Optional[MotorState]:
        """
        Send Enable and return the full motor state from the feedback frame.

        Returns None if no response arrives within *timeout* seconds.
        """
        self.enable()
        return self._wait_feedback(timeout)

    def read_angle(self, timeout: float = 0.15) -> Optional[float]:
        """
        Return the current angle in radians, or None on timeout.

        Convenience wrapper around read_feedback().
        """
        state = self.read_feedback(timeout)
        return state.angle_rad if state else None

    # ── write ─────────────────────────────────────────────────────────────────

    def set_angle(self,
                  pos_rad:   float,
                  kp:        float = 30.0,
                  kd:        float = 2.0,
                  vel_ff:    float = 0.0,
                  torque_ff: float = 0.0,
                  timeout:   float = 0.15) -> Optional[MotorState]:
        """
        Send a single MIT-mode position command and return the resulting feedback.

        Parameters
        ----------
        pos_rad   : target angle in radians (RS00 range: ±4π rad)
        kp        : position stiffness gain, N·m/rad  (0–500, default 30)
        kd        : damping gain, N·m·s/rad           (0–5,   default 2)
        vel_ff    : velocity feedforward, rad/s        (default 0)
        torque_ff : torque feedforward, N·m            (default 0)
        timeout   : seconds to wait for feedback       (default 0.15)

        Returns the MotorState from the motor's reply, or None on timeout.
        """
        pkt = encode_control(self.motor_id, pos_rad, vel_ff, kp, kd, torque_ff)
        self._send(pkt)
        return self._wait_feedback(timeout)

    def zero(self, timeout: float = 0.15) -> Optional[MotorState]:
        """
        Set the current position as the motor's zero reference (comm_type=6).

        The motor must be in STOP mode to accept a SetZero command — it silently
        ignores the command while enabled/running.  This method handles that:
          1. Send Stop
          2. Wait for motor to de-energise
          3. Send SetZero
          4. Wait for the motor to commit the new zero (~150 ms)
          5. Re-enable and read back to confirm (angle_rad should be ~0.0)

        Returns the post-zero MotorState, or None if no feedback arrives.
        """
        self.stop()
        time.sleep(0.1)
        pkt = encode_frame(build_can_id(COMM_SET_ZERO, self.host_id, self.motor_id), bytes([0x01]) + bytes(7))
        self._send(pkt)
        time.sleep(0.15)        # motor writes new zero to non-volatile memory
        return self.read_feedback(timeout)
