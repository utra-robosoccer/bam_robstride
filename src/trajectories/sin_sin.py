"""Two sinusoids summed: sin(t)·π/2 + sin(5t)·0.5·sin(2t).

Produces a rich, non-periodic signal with multiple frequency components
for friction model identification across velocity regimes.
"""
import math
from .base import Trajectory


class SinSin(Trajectory):
    duration = 30.0

    def __call__(self, t: float) -> tuple[float, bool]:
        angle = math.sin(t) * math.pi / 2.0 + math.sin(5.0 * t) * 0.5 * math.sin(2.0 * t)
        return angle, True
