"""Sweeps: wheel travel (bump/rebound), steering and body roll."""

from __future__ import annotations

from typing import Iterable, Optional

import numpy as np
import pandas as pd

from .metrics import all_metrics
from .models import Corner


def _ordered_from_zero(values: Iterable[float]):
    """Visit sweep values outward from zero so each solve starts close by."""
    values = np.asarray(list(values), dtype=float)
    order = np.argsort(np.abs(values), kind="stable")
    return values, order


def travel_sweep(model: Corner, travel: Optional[Iterable[float]] = None, rack: float = 0.0) -> pd.DataFrame:
    """Bump / rebound sweep. Default range: -100 mm to +100 mm in 2 mm steps."""
    if travel is None:
        travel = np.arange(-100.0, 100.0 + 1e-9, 2.0)
    values, order = _ordered_from_zero(travel)
    rows = [None] * len(values)
    model.solve(0.0, rack)
    for i in order:
        rows[i] = all_metrics(model.solve(values[i], rack))
    return pd.DataFrame(rows)


def steer_sweep(model: Corner, rack: Optional[Iterable[float]] = None, travel: float = 0.0) -> pd.DataFrame:
    """Steering sweep driven by rack displacement.

    ``rack`` is the inner tie-rod displacement in mm, + = towards +y (left).
    ``steer_angle_deg`` is the road-wheel angle, + = wheel points left.
    For a tie rod behind the axle a +y rack movement steers the left wheel
    to the right.
    """
    if rack is None:
        rack = np.arange(-40.0, 40.0 + 1e-9, 2.0)
    values, order = _ordered_from_zero(rack)
    rows = [None] * len(values)
    for i in order:
        rows[i] = all_metrics(model.solve(travel, values[i]))
    df = pd.DataFrame(rows)
    df["steer_angle_deg"] = -(df["toe_deg"] - df["toe_deg"].iloc[int(np.argmin(np.abs(values)))])
    return df


def roll_sweep(
    model: Corner, roll_deg: Optional[Iterable[float]] = None, half_track: Optional[float] = None
) -> pd.DataFrame:
    """Body roll for a symmetric axle.

    Rolling the body by ``phi`` (positive = body rolls to the right, i.e. the
    modelled left corner compresses) about a roll axis on the ground is
    approximated as wheel travel ``+half_track * tan(phi)`` on this corner.
    Camber relative to the ground is camber-to-body minus the roll angle.
    """
    if roll_deg is None:
        roll_deg = np.arange(-5.0, 5.0 + 1e-9, 0.5)
    if half_track is None:
        from .metrics import contact_patch

        half_track = float(contact_patch(model.design)[1])
    rows = []
    for phi in roll_deg:
        travel = half_track * np.tan(np.radians(phi))
        m = all_metrics(model.solve(travel))
        m["roll_deg"] = float(phi)
        m["camber_to_ground_deg"] = m["camber_deg"] - float(phi)
        rows.append(m)
    return pd.DataFrame(rows)


def summary(model: Corner) -> dict:
    """Static (design position) metrics plus a few gradients around it."""
    s = all_metrics(model.design)
    up, down = all_metrics(model.solve(10.0)), all_metrics(model.solve(-10.0))
    s["camber_gain_deg_per_10mm"] = (up["camber_deg"] - down["camber_deg"]) / 2
    s["bump_steer_deg_per_10mm"] = (up["toe_deg"] - down["toe_deg"]) / 2
    s["rch_migration_mm_per_10mm"] = (up["roll_centre_height_mm"] - down["roll_centre_height_mm"]) / 2
    return s
