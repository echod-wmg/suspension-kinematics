"""Generic rigid-knuckle constraint solver.

Every independent front suspension modelled here is a single rigid body (the
knuckle / upright, with the wheel bolted to it) connected to the body by
links. The knuckle has six degrees of freedom, parameterised as

``q = [rx, ry, rz, tx, ty, tz]``

i.e. a rotation vector (radians) about a reference point plus a translation
(mm), both relative to the design position. Each link contributes one or
more scalar residuals, and the "driver" (prescribed wheel travel) closes the
system. The set of equations is solved with Gauss-Newton iterations using a
finite-difference Jacobian, which is plenty fast for a 6x6 problem and
avoids hand-deriving Jacobians for every linkage type.

This replaces the 2020 spreadsheet approach (which chained trigonometric
approximations, e.g. it assumed the knuckle stays vertical when locating the
upper ball joint) with an exact loop closure.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable, Dict, List

import numpy as np

from .geometry import rotation_matrix

Points = Dict[str, np.ndarray]
Residual = Callable[[Points, Points], np.ndarray]


class SolverError(RuntimeError):
    """Raised when the linkage cannot be assembled (e.g. travel out of range)."""


@dataclass
class KnucklePose:
    q: np.ndarray = field(default_factory=lambda: np.zeros(6))

    def rotation(self) -> np.ndarray:
        return rotation_matrix(self.q[:3])

    def apply(self, p: np.ndarray, ref: np.ndarray) -> np.ndarray:
        return self.rotation() @ (p - ref) + ref + self.q[3:]


def solve_pose(
    knuckle_points: Points,
    reference: np.ndarray,
    body_points: Points,
    residuals: List[Residual],
    q0: np.ndarray,
    tol: float = 1e-9,
    max_iter: int = 60,
) -> np.ndarray:
    """Find the knuckle pose ``q`` that zeros every residual.

    ``residuals`` are callables ``f(knuckle_now, body_points) -> array`` where
    ``knuckle_now`` holds the knuckle points transformed by the trial pose.
    """
    names = list(knuckle_points)
    base = np.array([knuckle_points[n] for n in names])

    def transform(q):
        r = rotation_matrix(q[:3])
        moved = (base - reference) @ r.T + reference + q[3:]
        return dict(zip(names, moved))

    def f(q):
        moved = transform(q)
        return np.concatenate([np.atleast_1d(res(moved, body_points)) for res in residuals])

    q = np.array(q0, dtype=float)
    steps = np.array([1e-7] * 3 + [1e-4] * 3)
    r = f(q)
    for _ in range(max_iter):
        if np.max(np.abs(r)) < tol:
            return q
        jac = np.empty((r.size, 6))
        for i in range(6):
            dq = np.zeros(6)
            dq[i] = steps[i]
            jac[:, i] = (f(q + dq) - f(q - dq)) / (2 * steps[i])
        delta, *_ = np.linalg.lstsq(jac, -r, rcond=None)
        # simple backtracking keeps the iteration on the assembled branch
        alpha = 1.0
        r_norm = np.linalg.norm(r)
        while alpha > 1e-4:
            q_try = q + alpha * delta
            r_try = f(q_try)
            if np.all(np.isfinite(r_try)) and np.linalg.norm(r_try) < r_norm:
                break
            alpha *= 0.5
        q, r = q_try, r_try
    if np.max(np.abs(r)) < 1e-6:
        return q
    raise SolverError(
        f"linkage did not assemble (max residual {np.max(np.abs(r)):.3g} mm); "
        "the requested travel/steer is probably outside the mechanism's range"
    )


# --- residual builders ---------------------------------------------------


def distance(knuckle_point: str, body_point: str, length: float) -> Residual:
    """Fixed-length link (ball joint to body pivot, tie rod, ...)."""

    def res(k: Points, b: Points) -> np.ndarray:
        return np.array([np.linalg.norm(k[knuckle_point] - b[body_point]) - length])

    return res


def point_on_knuckle_line(line_a: str, line_b: str, body_point: str) -> Residual:
    """Body point must lie on the line through two knuckle points.

    Models a MacPherson strut: the damper rod slides through the top mount,
    so the top mount stays on the (knuckle-fixed) strut axis.
    Returns two independent residuals.
    """

    def res(k: Points, b: Points) -> np.ndarray:
        a = k[line_a]
        d = k[line_b] - a
        d = d / np.linalg.norm(d)
        w = b[body_point] - a
        perp = w - np.dot(w, d) * d
        # express the perpendicular error in a basis orthogonal to d
        e1 = np.cross(d, [1.0, 0.0, 0.0])
        if np.linalg.norm(e1) < 0.1:
            e1 = np.cross(d, [0.0, 1.0, 0.0])
        e1 /= np.linalg.norm(e1)
        e2 = np.cross(d, e1)
        return np.array([np.dot(perp, e1), np.dot(perp, e2)])

    return res


def vertical_position(knuckle_point: str, target_z: float) -> Residual:
    """Driver: prescribe the z coordinate of a knuckle point (wheel travel)."""

    def res(k: Points, b: Points) -> np.ndarray:
        return np.array([k[knuckle_point][2] - target_z])

    return res
