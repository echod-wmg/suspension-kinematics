"""Suspension corner models: double wishbone and MacPherson strut.

A model is defined by *hardpoints* (3-D coordinates in the design position)
plus wheel data. ``solve(travel, rack)`` assembles the linkage for a given
vertical wheel travel and steering-rack displacement and returns a
:class:`CornerState` from which all kinematic metrics are derived (see
:mod:`suspkin.metrics`).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Optional

import numpy as np

from . import solver
from .geometry import unit, vec

#: hardpoints that belong to the body (chassis) side
DW_BODY = (
    "lower_front_pivot",
    "lower_rear_pivot",
    "upper_front_pivot",
    "upper_rear_pivot",
    "tie_rod_inner",
)
#: hardpoints that move with the knuckle
DW_KNUCKLE = ("lower_ball_joint", "upper_ball_joint", "tie_rod_outer", "wheel_centre")

MP_BODY = ("lower_front_pivot", "lower_rear_pivot", "strut_top_mount", "tie_rod_inner")
MP_KNUCKLE = ("lower_ball_joint", "strut_lower", "tie_rod_outer", "wheel_centre")


@dataclass
class Wheel:
    """Wheel / tyre data.

    ``radius`` is the loaded rolling radius used to place the contact patch.
    Static camber and toe define the spindle direction in the design
    position (camber negative = top of wheel inboard, toe positive = toe-in).
    """

    radius: float
    width: float = 200.0
    static_camber_deg: float = 0.0
    static_toe_deg: float = 0.0

    def spindle_direction(self) -> np.ndarray:
        """Unit vector along the axle, pointing outboard (+y)."""
        c = np.radians(self.static_camber_deg)
        t = np.radians(self.static_toe_deg)
        # toe-in: front of the wheel turns inboard -> spindle tip moves forward
        # positive camber: top outboard -> spindle tip moves down
        return unit(np.array([np.sin(t) * np.cos(c), np.cos(t) * np.cos(c), -np.sin(c)]))


@dataclass
class CornerState:
    """Solved position of every hardpoint for one travel / steer input."""

    travel: float
    rack: float
    points: Dict[str, np.ndarray]
    spindle: np.ndarray
    steering_axis: tuple  # (point, unit direction pointing up)
    model: "Corner" = field(repr=False)

    def __getitem__(self, name: str) -> np.ndarray:
        return self.points[name]


class Corner:
    """Base class holding hardpoints and the generic solve loop."""

    kind: str = "corner"
    body_names: tuple = ()
    knuckle_names: tuple = ()

    def __init__(self, hardpoints: Dict[str, object], wheel: Wheel, travel_reference: str = "wheel_centre"):
        missing = [n for n in (*self.body_names, *self.knuckle_names) if n not in hardpoints]
        if missing:
            raise ValueError(f"{self.kind}: missing hardpoints {missing}")
        self.hardpoints = {n: vec(hardpoints[n]) for n in (*self.body_names, *self.knuckle_names)}
        self.wheel = wheel
        if travel_reference not in self.knuckle_names:
            raise ValueError(f"travel_reference must be one of {self.knuckle_names}")
        self.travel_reference = travel_reference
        self._spindle0 = wheel.spindle_direction()
        # the knuckle also carries virtual points (names starting with "_"),
        # e.g. a spindle tip so the axle direction rotates with the knuckle
        self._knuckle0 = {n: self.hardpoints[n] for n in self.knuckle_names}
        self._knuckle0["_spindle_tip"] = self.hardpoints["wheel_centre"] + 100.0 * self._spindle0
        self._knuckle0.update(self._virtual_points())
        self._ref = self.hardpoints["wheel_centre"]
        self._lengths = self._link_lengths()
        self._q_cache: Dict[tuple, np.ndarray] = {}
        self.design = self.solve(0.0)

    # -- to be provided by subclasses -------------------------------------
    def _virtual_points(self) -> Dict[str, np.ndarray]:
        return {}

    def _link_lengths(self) -> Dict[str, float]:
        raise NotImplementedError

    def _constraints(self):
        raise NotImplementedError

    def _steering_axis(self, pts: Dict[str, np.ndarray]):
        raise NotImplementedError

    # -- solving -------------------------------------------------------------
    def body_points(self, rack: float = 0.0) -> Dict[str, np.ndarray]:
        pts = {n: self.hardpoints[n].copy() for n in self.body_names}
        pts["tie_rod_inner"] = pts["tie_rod_inner"] + np.array([0.0, rack, 0.0])
        return pts

    def solve(self, travel: float = 0.0, rack: float = 0.0, q0: Optional[np.ndarray] = None) -> CornerState:
        """Assemble the corner for ``travel`` (mm, +bump) and ``rack`` (mm, +left)."""
        body = self.body_points(rack)
        target = self._knuckle0[self.travel_reference][2] + travel
        residuals = self._constraints() + [solver.vertical_position(self.travel_reference, target)]
        if q0 is None:
            q0 = self._nearest_cached(travel, rack)
        q = solver.solve_pose(self._knuckle0, self._ref, body, residuals, q0)
        self._q_cache[(round(travel, 6), round(rack, 6))] = q
        pose = solver.KnucklePose(q)
        moved = {n: pose.apply(p, self._ref) for n, p in self._knuckle0.items()}
        spindle = unit(moved["_spindle_tip"] - moved["wheel_centre"])
        pts = {**body, **{n: p for n, p in moved.items() if not n.startswith("_")}}
        return CornerState(travel, rack, pts, spindle, self._steering_axis(pts), self)

    def _nearest_cached(self, travel: float, rack: float) -> np.ndarray:
        if not self._q_cache:
            return np.zeros(6)
        key = min(self._q_cache, key=lambda k: abs(k[0] - travel) + abs(k[1] - rack))
        return self._q_cache[key]

    def to_dict(self) -> dict:
        return {
            "type": self.kind,
            "travel_reference": self.travel_reference,
            "hardpoints": {n: [round(float(c), 4) for c in p] for n, p in self.hardpoints.items()},
            "wheel": {
                "radius": self.wheel.radius,
                "width": self.wheel.width,
                "static_camber_deg": self.wheel.static_camber_deg,
                "static_toe_deg": self.wheel.static_toe_deg,
            },
        }


class DoubleWishbone(Corner):
    """Short-long-arm / double wishbone corner.

    Required hardpoints: ``lower_front_pivot``, ``lower_rear_pivot``,
    ``lower_ball_joint``, ``upper_front_pivot``, ``upper_rear_pivot``,
    ``upper_ball_joint``, ``tie_rod_inner``, ``tie_rod_outer``,
    ``wheel_centre``.
    """

    kind = "double_wishbone"
    body_names = DW_BODY
    knuckle_names = DW_KNUCKLE

    def _link_lengths(self):
        h = self.hardpoints
        d = lambda a, b: float(np.linalg.norm(h[a] - h[b]))  # noqa: E731
        return {
            "lf": d("lower_ball_joint", "lower_front_pivot"),
            "lr": d("lower_ball_joint", "lower_rear_pivot"),
            "uf": d("upper_ball_joint", "upper_front_pivot"),
            "ur": d("upper_ball_joint", "upper_rear_pivot"),
            "tr": d("tie_rod_outer", "tie_rod_inner"),
        }

    def _constraints(self):
        L = self._lengths
        return [
            solver.distance("lower_ball_joint", "lower_front_pivot", L["lf"]),
            solver.distance("lower_ball_joint", "lower_rear_pivot", L["lr"]),
            solver.distance("upper_ball_joint", "upper_front_pivot", L["uf"]),
            solver.distance("upper_ball_joint", "upper_rear_pivot", L["ur"]),
            solver.distance("tie_rod_outer", "tie_rod_inner", L["tr"]),
        ]

    def _steering_axis(self, pts):
        return pts["lower_ball_joint"], unit(pts["upper_ball_joint"] - pts["lower_ball_joint"])


class MacPherson(Corner):
    """MacPherson strut corner.

    Required hardpoints: ``lower_front_pivot``, ``lower_rear_pivot``,
    ``lower_ball_joint``, ``strut_top_mount`` (body), ``strut_lower`` (a
    point on the strut axis fixed to the knuckle), ``tie_rod_inner``,
    ``tie_rod_outer``, ``wheel_centre``.
    """

    kind = "macpherson"
    body_names = MP_BODY
    knuckle_names = MP_KNUCKLE

    def _link_lengths(self):
        h = self.hardpoints
        d = lambda a, b: float(np.linalg.norm(h[a] - h[b]))  # noqa: E731
        return {
            "lf": d("lower_ball_joint", "lower_front_pivot"),
            "lr": d("lower_ball_joint", "lower_rear_pivot"),
            "tr": d("tie_rod_outer", "tie_rod_inner"),
        }

    def _virtual_points(self):
        # second knuckle point on the strut axis, so "top mount lies on the
        # strut axis" can be written as a knuckle-fixed line constraint
        h = self.hardpoints
        axis = unit(h["strut_top_mount"] - h["strut_lower"])
        return {"_strut_axis_tip": h["strut_lower"] + 100.0 * axis}

    def _constraints(self):
        L = self._lengths
        return [
            solver.distance("lower_ball_joint", "lower_front_pivot", L["lf"]),
            solver.distance("lower_ball_joint", "lower_rear_pivot", L["lr"]),
            solver.distance("tie_rod_outer", "tie_rod_inner", L["tr"]),
            solver.point_on_knuckle_line("strut_lower", "_strut_axis_tip", "strut_top_mount"),
        ]

    def _steering_axis(self, pts):
        # kingpin axis of a strut: lower ball joint -> top mount
        return pts["lower_ball_joint"], unit(pts["strut_top_mount"] - pts["lower_ball_joint"])

    def strut_length(self, state: CornerState) -> float:
        """Distance strut_lower -> top mount (damper length change = travel ratio)."""
        return float(np.linalg.norm(state["strut_top_mount"] - state["strut_lower"]))


def build(config: dict) -> Corner:
    """Create a model from a config dictionary (see ``examples/*.json``)."""
    kind = config.get("type")
    if kind == "excel_double_wishbone":
        from .legacy import ExcelDoubleWishbone

        return ExcelDoubleWishbone(**config["parameters"]).to_model(config.get("travel_reference", "wheel_centre"))
    wheel = Wheel(**config["wheel"])
    ref = config.get("travel_reference", "wheel_centre")
    if kind == "double_wishbone":
        return DoubleWishbone(config["hardpoints"], wheel, ref)
    if kind in ("macpherson", "mcpherson"):
        return MacPherson(config["hardpoints"], wheel, ref)
    raise ValueError(f"unknown suspension type {kind!r}")
