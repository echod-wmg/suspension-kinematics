"""Small vector / rotation helpers used throughout the package.

Coordinate system (ISO 8855 style, one *left* corner of the vehicle):

* ``x`` - forward
* ``y`` - to the left, i.e. **outboard** for the modelled corner
* ``z`` - up, ground plane at ``z = 0`` in the design position

All lengths are in millimetres, all user-facing angles in degrees.
"""

from __future__ import annotations

import numpy as np

Vector = np.ndarray


def vec(values) -> Vector:
    """Return ``values`` as a float64 numpy array of shape (3,)."""
    v = np.asarray(values, dtype=float)
    if v.shape != (3,):
        raise ValueError(f"expected a 3-vector, got shape {v.shape}")
    return v


def unit(v: Vector) -> Vector:
    n = np.linalg.norm(v)
    if n == 0.0:
        raise ValueError("cannot normalise a zero-length vector")
    return v / n


def rotation_matrix(rotvec: Vector) -> Vector:
    """Rotation matrix from a rotation vector (axis * angle, radians).

    This is the Euler-Rodrigues formula
    ``R = I + sin(t) K + (1 - cos(t)) K^2`` with ``K`` the cross-product
    matrix of the unit axis.
    """
    theta = float(np.linalg.norm(rotvec))
    if theta < 1e-14:
        # first-order expansion keeps the Jacobian smooth around zero
        k = np.asarray(rotvec, dtype=float)
        return np.eye(3) + _skew(k)
    k = np.asarray(rotvec, dtype=float) / theta
    kx = _skew(k)
    return np.eye(3) + np.sin(theta) * kx + (1.0 - np.cos(theta)) * (kx @ kx)


def rotate_about_axis(point: Vector, axis_point: Vector, axis_dir: Vector, angle_rad: float) -> Vector:
    """Rotate ``point`` about the line through ``axis_point`` along ``axis_dir``."""
    r = rotation_matrix(unit(axis_dir) * angle_rad)
    return axis_point + r @ (point - axis_point)


def _skew(k: Vector) -> Vector:
    return np.array([[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]], [-k[1], k[0], 0.0]])


def line_intersection_2d(p1, d1, p2, d2):
    """Intersect two 2-D lines ``p1 + s*d1`` and ``p2 + t*d2``.

    Returns ``None`` when the lines are (numerically) parallel.
    """
    a = np.array([[d1[0], -d2[0]], [d1[1], -d2[1]]], dtype=float)
    det = np.linalg.det(a)
    if abs(det) < 1e-12 * max(1.0, np.linalg.norm(d1) * np.linalg.norm(d2)):
        return None
    s, _ = np.linalg.solve(a, np.asarray(p2, float) - np.asarray(p1, float))
    return np.asarray(p1, float) + s * np.asarray(d1, float)


def plane_trace_at_x(point: Vector, normal: Vector, x0: float):
    """Trace of a plane on the transverse plane ``x = x0``.

    Returns ``(p, d)`` - a point and direction of the resulting line in
    (y, z) coordinates, or ``None`` if the plane is parallel to ``x = x0``.
    """
    ny, nz = normal[1], normal[2]
    if abs(ny) < 1e-12 and abs(nz) < 1e-12:
        return None
    rhs = -normal[0] * (x0 - point[0])
    # n_y*(y - py) + n_z*(z - pz) = rhs
    if abs(nz) >= abs(ny):
        p = np.array([point[1], point[2] + rhs / nz])
    else:
        p = np.array([point[1] + rhs / ny, point[2]])
    d = np.array([nz, -ny])
    return p, d
