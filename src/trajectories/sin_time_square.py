"""Chirp trajectory: A·sin(k·t²) — progressively increasing frequency.

Primary BAM excitation. Instantaneous frequency rises as f(t) = k·t/π Hz,
sweeping from quasi-static through mid-frequency in a single recording.

Default: k=0.1, A=1.0 rad, 30 s → sweeps 0 → ~0.95 Hz
"""
import math
from .base import Trajectory


class SinTimeSquare(Trajectory):
    duration = 30.0

    def __init__(self, amplitude: float = 1.0, k: float = 0.1):
        self.amplitude = amplitude
        self.k = k

    def __call__(self, t: float) -> tuple[float, bool]:
        return self.amplitude * math.sin(self.k * t * t), True
