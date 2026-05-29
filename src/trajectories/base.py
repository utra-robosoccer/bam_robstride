"""Base trajectory class and cubic Hermite spline interpolation."""
from abc import ABC, abstractmethod


def cubic_hermite(t0: float, p0: float, v0: float, t1: float, p1: float, v1: float, t: float) -> float:
    """Cubic Hermite spline evaluation between two keyframes."""
    if t <= t0:
        return p0
    if t >= t1:
        return p1
    dt = t1 - t0
    s = (t - t0) / dt
    s2, s3 = s * s, s * s * s
    return ((2*s3 - 3*s2 + 1) * p0
            + (s3 - 2*s2 + s) * dt * v0
            + (-2*s3 + 3*s2) * p1
            + (s3 - s2) * dt * v1)


def cubic_interpolate(keyframes: list, t: float) -> float:
    """Interpolate through keyframes = [[t, pos, vel], ...] using cubic Hermite splines."""
    if t <= keyframes[0][0]:
        return keyframes[0][1]
    if t >= keyframes[-1][0]:
        return keyframes[-1][1]
    for i in range(len(keyframes) - 1):
        k0, k1 = keyframes[i], keyframes[i + 1]
        if k0[0] <= t <= k1[0]:
            return cubic_hermite(k0[0], k0[1], k0[2], k1[0], k1[1], k1[2], t)
    return keyframes[-1][1]


class Trajectory(ABC):
    """Base class for BAM excitation trajectories.

    Subclasses implement __call__(t) -> (cmd_rad, torque_enable).
    """
    duration: float = 6.0

    @abstractmethod
    def __call__(self, t: float) -> tuple[float, bool]:
        """Return (cmd_rad, torque_enable) at time t seconds."""
