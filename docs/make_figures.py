"""Regenerate the figures used in the README:  python docs/make_figures.py"""

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import numpy as np  # noqa: E402

from suspkin import load_model, travel_sweep  # noqa: E402
from suspkin.plotting import plot_geometry, plot_travel_sweep  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
IMG = ROOT / "docs" / "img"
IMG.mkdir(parents=True, exist_ok=True)

dw = load_model(ROOT / "examples" / "double_wishbone.json")
mp = load_model(ROOT / "examples" / "macpherson.json")
travel = np.arange(-90.0, 90.0 + 1e-9, 2.0)
fig = plot_travel_sweep(
    {"Double wishbone": travel_sweep(dw, travel), "MacPherson": travel_sweep(mp, travel)},
    title="Wheel travel sweep - example corners",
)
fig.savefig(IMG / "travel_sweep_comparison.png", dpi=120)

for name, model in (("double_wishbone", dw), ("macpherson", mp)):
    fig = plot_geometry(
        [model.solve(0.0), model.solve(80.0), model.solve(-80.0)],
        labels=["design", "bump +80 mm", "rebound -80 mm"],
        title=name.replace("_", " ").title(),
    )
    fig.savefig(IMG / f"geometry_{name}.png", dpi=110)
print("figures written to", IMG)
