"""Controlled lift to π/2, then partial descent to 0.8·π/2.

Uses cubic Hermite splines through three keyframes. Torque stays on
throughout — exercises gravity loading in both travel directions.
"""
import math
from .base import Trajectory, cubic_interpolate

_KEYFRAMES = [
    [0.0, 0.0, 0.0],
    [3.0, math.pi / 2.0, 0.0],
    [6.0, 0.8 * math.pi / 2.0, 0.0],
]


class UpAndDown(Trajectory):
    duration = 6.0

    def __call__(self, t: float) -> tuple[float, bool]:
        return cubic_interpolate(_KEYFRAMES, t), True
