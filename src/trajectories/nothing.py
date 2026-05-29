"""Zero-torque baseline: motor is passive for the full duration.

Records free-spinning / gravity-only dynamics. Useful for measuring
back-EMF friction and cogging at near-zero commanded torque.
"""
from .base import Trajectory


class Nothing(Trajectory):
    duration = 6.0

    def __call__(self, t: float) -> tuple[float, bool]:
        return 0.0, False
