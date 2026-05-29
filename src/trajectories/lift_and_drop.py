"""Lift to -π/2 over 2 s, hold briefly, then release torque for free drop.

torque_enable goes False at t=2 s — the motor becomes passive so the arm
falls freely under gravity. Captures inertia/friction from the drop response.
"""
import math
from .base import Trajectory, cubic_interpolate

_KEYFRAMES = [[0.0, 0.0, 0.0], [2.0, -math.pi / 2.0, 0.0]]


class LiftAndDrop(Trajectory):
    duration = 6.0

    def __call__(self, t: float) -> tuple[float, bool]:
        return cubic_interpolate(_KEYFRAMES, t), t < 2.0
