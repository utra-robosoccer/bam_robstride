from .sin_time_square import SinTimeSquare
from .sin_sin import SinSin
from .lift_and_drop import LiftAndDrop
from .up_and_down import UpAndDown
from .nothing import Nothing

traj_list: dict = {
    "sin_time_square": SinTimeSquare,
    "sin_sin": SinSin,
    "lift_and_drop": LiftAndDrop,
    "up_and_down": UpAndDown,
    "nothing": Nothing,
}
