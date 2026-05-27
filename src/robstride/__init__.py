"""
robstride — raw CH341/CAN motor control for Robstride RS00.

Public API
----------
    from robstride import RobstrideMotor, MotorState

    with RobstrideMotor(port="/dev/ttyCH341USB0", motor_id=1) as m:
        state = m.read_feedback()      # MotorState(angle_rad, vel_rads, torque_nm, temp_c)
        angle = m.read_angle()         # float (radians)
        state = m.set_angle(1.0)       # move to +1 rad, return feedback
        state = m.zero()               # set current position as zero
"""

from .motor import MotorState, RobstrideMotor

__all__ = ["RobstrideMotor", "MotorState"]
