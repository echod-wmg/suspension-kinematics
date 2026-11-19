"""Matplotlib plots: K&C-style sweep curves and a three-view geometry sketch."""

from __future__ import annotations

from typing import Dict, Optional, Sequence

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from .metrics import contact_patch
from .models import CornerState, DoubleWishbone

TRAVEL_PANELS = (
    ("camber_deg", "Camber [deg]"),
    ("bump_steer_deg", "Bump steer (toe change) [deg]"),
    ("half_track_change_mm", "Half-track change [mm]"),
    ("wheel_recession_mm", "Wheel recession [mm]"),
    ("caster_deg", "Caster [deg]"),
    ("roll_centre_height_mm", "Roll centre height [mm]"),
)


def plot_travel_sweep(
    sweeps: Dict[str, pd.DataFrame],
    panels: Sequence = TRAVEL_PANELS,
    title: Optional[str] = None,
):
    """Plot one or more travel sweeps (dict label -> DataFrame) side by side.

    Wheel travel is on the vertical axis, as is customary for K&C plots.
    """
    n = len(panels)
    cols = 3 if n > 4 else n
    rows = int(np.ceil(n / cols))
    fig, axes = plt.subplots(rows, cols, figsize=(4.2 * cols, 3.6 * rows), squeeze=False)
    for ax, (col, label) in zip(axes.flat, panels):
        for name, df in sweeps.items():
            ax.plot(df[col], df["travel_mm"], label=name, linewidth=1.8)
        ax.set_xlabel(label)
        ax.set_ylabel("Wheel travel [mm]")
        ax.axhline(0, color="0.6", linewidth=0.6)
        ax.grid(True, alpha=0.3)
    for ax in list(axes.flat)[n:]:
        ax.set_visible(False)
    if len(sweeps) > 1:
        axes.flat[0].legend(fontsize=8)
    if title:
        fig.suptitle(title)
    fig.tight_layout()
    return fig


def _links(state: CornerState):
    if isinstance(state.model, DoubleWishbone):
        return [
            ("lower_front_pivot", "lower_ball_joint", "tab:blue"),
            ("lower_rear_pivot", "lower_ball_joint", "tab:blue"),
            ("upper_front_pivot", "upper_ball_joint", "tab:red"),
            ("upper_rear_pivot", "upper_ball_joint", "tab:red"),
            ("lower_ball_joint", "upper_ball_joint", "goldenrod"),
            ("tie_rod_inner", "tie_rod_outer", "tab:green"),
        ]
    return [
        ("lower_front_pivot", "lower_ball_joint", "tab:blue"),
        ("lower_rear_pivot", "lower_ball_joint", "tab:blue"),
        ("lower_ball_joint", "strut_lower", "goldenrod"),
        ("strut_lower", "strut_top_mount", "tab:red"),
        ("tie_rod_inner", "tie_rod_outer", "tab:green"),
    ]


def _wheel_outline(state: CornerState, n: int = 60):
    """Points on the outer rim circle of the wheel (for drawing)."""
    s, wc = state.spindle, state["wheel_centre"]
    r = state.model.wheel.radius
    a = np.cross(s, [0.0, 0.0, 1.0])
    a /= np.linalg.norm(a)
    b = np.cross(s, a)
    t = np.linspace(0, 2 * np.pi, n)
    return wc + r * (np.outer(np.cos(t), a) + np.outer(np.sin(t), b))


def plot_geometry(states: Sequence[CornerState], labels: Optional[Sequence[str]] = None, title: Optional[str] = None):
    """Front (y-z), side (x-z) and top (x-y) views of one or more states."""
    views = (("Front view", 1, 2, "y [mm]"), ("Side view", 0, 2, "x [mm]"), ("Top view", 0, 1, "x [mm]"))
    fig, axes = plt.subplots(1, 3, figsize=(15, 5))
    styles = ["-", "--", ":", "-."]
    for k, state in enumerate(states):
        ls = styles[k % len(styles)]
        wheel = _wheel_outline(state)
        cp = contact_patch(state)
        for ax, (name, i, j, xl) in zip(axes, views):
            for a, b, colour in _links(state):
                pa, pb = state[a], state[b]
                ax.plot([pa[i], pb[i]], [pa[j], pb[j]], ls, color=colour, linewidth=2)
            ax.plot(wheel[:, i], wheel[:, j], ls, color="0.35", linewidth=1)
            ax.plot(cp[i], cp[j], "kv", markersize=5)
            ax.set_title(name)
            ax.set_xlabel(xl)
            ax.set_ylabel("z [mm]" if j == 2 else "y [mm]")
            ax.set_aspect("equal", adjustable="datalim")
            ax.grid(True, alpha=0.3)
    for ax in axes[:1]:
        ax.axhline(0, color="saddlebrown", linewidth=1)
    if labels:
        from matplotlib.lines import Line2D

        handles = [Line2D([], [], color="k", linestyle=styles[k % 4]) for k in range(len(labels))]
        axes[0].legend(handles, labels, fontsize=8, loc="upper left")
    if title:
        fig.suptitle(title)
    fig.tight_layout()
    return fig
