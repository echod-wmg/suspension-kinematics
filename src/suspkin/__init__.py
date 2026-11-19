"""suspkin - 3-D kinematics of double wishbone and MacPherson suspensions."""

from .analysis import roll_sweep, steer_sweep, summary, travel_sweep
from .io import load_model
from .legacy import ExcelDoubleWishbone
from .metrics import all_metrics
from .models import CornerState, DoubleWishbone, MacPherson, Wheel, build

__all__ = [
    "CornerState",
    "DoubleWishbone",
    "ExcelDoubleWishbone",
    "MacPherson",
    "Wheel",
    "all_metrics",
    "build",
    "load_model",
    "roll_sweep",
    "steer_sweep",
    "summary",
    "travel_sweep",
]

__version__ = "1.0.0"
