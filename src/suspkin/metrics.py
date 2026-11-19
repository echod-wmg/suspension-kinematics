"""Kinematic metrics derived from a solved :class:`~suspkin.models.CornerState`.

Sign conventions (left corner, y outboard):

* camber  - negative when the top of the wheel leans inboard
* toe     - positive for toe-in (front of the wheel pointing inboard)
* caster  - positive when the top of the steering axis is rearward
* KPI     - positive when the top of the steering axis is inboard
* scrub radius - positive when the steering axis meets the ground inboard
  of the contact patch
* mechanical trail - positive when the steering axis meets the ground ahead
  of the contact patch
* half-track change (HTC) - contact patch lateral movement, + = outboard
* wheel recession (WR) - wheel-centre longitudinal movement, + = forward
* roll centre height - front-view kinematic roll centre above ground,
  assuming a left/right symmetric axle
"""

from __future__ import annotations

from typing import Dict, Optional

import numpy as np

from .geometry import line_intersection_2d, plane_trace_at_x, unit
from .models import CornerState, DoubleWishbone, MacPherson


def camber(state: CornerState) -> float:
    s = state.spindle
    return float(np.degrees(np.arctan2(-s[2], np.hypot(s[0], s[1]))))


def toe(state: CornerState) -> float:
    s = state.spindle
    return float(np.degrees(np.arctan2(s[0], s[1])))


def caster(state: CornerState) -> float:
    _, k = state.steering_axis
    return float(np.degrees(np.arctan2(-k[0], k[2])))


def kpi(state: CornerState) -> float:
    _, k = state.steering_axis
    return float(np.degrees(np.arctan2(-k[1], k[2])))


def contact_patch(state: CornerState) -> np.ndarray:
    """Lowest point of the (rigid, thin) wheel in the wheel plane."""
    s = state.spindle
    down = np.array([0.0, 0.0, -1.0])
    d = unit(down - np.dot(down, s) * s)
    return state["wheel_centre"] + state.model.wheel.radius * d


def steering_axis_ground_point(state: CornerState) -> Optional[np.ndarray]:
    p, k = state.steering_axis
    z_ground = contact_patch(state)[2]
    if abs(k[2]) < 1e-9:
        return None
    return p + (z_ground - p[2]) / k[2] * k


def scrub_radius(state: CornerState) -> float:
    g = steering_axis_ground_point(state)
    return float(contact_patch(state)[1] - g[1])


def mechanical_trail(state: CornerState) -> float:
    g = steering_axis_ground_point(state)
    return float(g[0] - contact_patch(state)[0])


def front_view_instant_centre(state: CornerState) -> Optional[np.ndarray]:
    """Front-view instant centre (y, z) of the knuckle relative to the body.

    Double wishbone: intersection of the traces of the two arm planes on the
    transverse plane through the wheel centre. MacPherson: lower-arm plane
    trace intersected with the line through the top mount perpendicular to
    the strut axis. Returns ``None`` if the lines are parallel.
    """
    x0 = state["wheel_centre"][0]
    model = state.model
    traces = [_arm_trace(state, "lower", x0)]
    if isinstance(model, DoubleWishbone):
        traces.append(_arm_trace(state, "upper", x0))
    elif isinstance(model, MacPherson):
        strut = unit(state["strut_top_mount"] - state["strut_lower"])
        traces.append(plane_trace_at_x(state["strut_top_mount"], strut, x0))
    else:  # pragma: no cover
        raise TypeError(type(model))
    if any(t is None for t in traces):
        return None
    (p1, d1), (p2, d2) = traces
    return line_intersection_2d(p1, d1, p2, d2)


def _arm_trace(state: CornerState, arm: str, x0: float):
    f = state[f"{arm}_front_pivot"]
    r = state[f"{arm}_rear_pivot"]
    b = state[f"{arm}_ball_joint"]
    n = np.cross(r - f, b - f)
    return plane_trace_at_x(f, n, x0)


def roll_centre_height(state: CornerState) -> float:
    """Kinematic roll centre height above ground (symmetric axle)."""
    cp = contact_patch(state)
    cp2 = np.array([cp[1], cp[2]])
    ic = front_view_instant_centre(state)
    if ic is None:
        # parallel arms: the force line is parallel to the arms
        x0 = state["wheel_centre"][0]
        _, d = _arm_trace(state, "lower", x0)
    else:
        d = ic - cp2
    if abs(d[0]) < 1e-12:
        return float("nan")
    z_centre = cp2[1] + (0.0 - cp2[0]) * d[1] / d[0]
    return float(z_centre - cp2[1])


def all_metrics(state: CornerState, reference: Optional[CornerState] = None) -> Dict[str, float]:
    """Every scalar metric for one state; changes are relative to ``reference``."""
    ref = reference if reference is not None else state.model.design
    cp, cp0 = contact_patch(state), contact_patch(ref)
    ic = front_view_instant_centre(state)
    out = {
        "travel_mm": state.travel,
        "rack_mm": state.rack,
        "camber_deg": camber(state),
        "toe_deg": toe(state),
        "caster_deg": caster(state),
        "kpi_deg": kpi(state),
        "scrub_radius_mm": scrub_radius(state),
        "mechanical_trail_mm": mechanical_trail(state),
        "half_track_mm": float(cp[1]),
        "half_track_change_mm": float(cp[1] - cp0[1]),
        "wheel_recession_mm": float(state["wheel_centre"][0] - ref["wheel_centre"][0]),
        "wheel_centre_z_mm": float(state["wheel_centre"][2]),
        "roll_centre_height_mm": roll_centre_height(state),
        "ic_y_mm": float(ic[0]) if ic is not None else float("nan"),
        "ic_z_mm": float(ic[1]) if ic is not None else float("nan"),
    }
    out["camber_change_deg"] = out["camber_deg"] - camber(ref)
    out["bump_steer_deg"] = out["toe_deg"] - toe(ref)
    if isinstance(state.model, MacPherson):
        out["strut_length_mm"] = state.model.strut_length(state)
    return out
