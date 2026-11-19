"""Interactive explorer for suspkin.

Run from the repository root:

    pip install -e ".[app]"
    streamlit run app/streamlit_app.py

Pick an example, edit hardpoints and wheel data in the sidebar, and move the
travel and steer sliders. Every change re-solves the corner.
"""

from __future__ import annotations

import copy
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402
import streamlit as st  # noqa: E402

from suspkin import all_metrics, build, roll_sweep, steer_sweep, travel_sweep  # noqa: E402
from suspkin.io import load_config  # noqa: E402
from suspkin.plotting import plot_geometry, plot_travel_sweep  # noqa: E402

EXAMPLES = Path(__file__).resolve().parents[1] / "examples"

METRIC_LABELS = {
    "camber_deg": "Camber [deg]",
    "toe_deg": "Toe [deg]",
    "caster_deg": "Caster [deg]",
    "kpi_deg": "KPI [deg]",
    "scrub_radius_mm": "Scrub radius [mm]",
    "mechanical_trail_mm": "Mechanical trail [mm]",
    "half_track_change_mm": "Half-track change [mm]",
    "wheel_recession_mm": "Wheel recession [mm]",
    "roll_centre_height_mm": "Roll centre height [mm]",
    "bump_steer_deg": "Bump steer [deg]",
    "camber_change_deg": "Camber change [deg]",
}

st.set_page_config(page_title="suspkin explorer", layout="wide")


# --------------------------------------------------------------------------- model state
def _base_config(name: str) -> dict:
    """Load an example and convert it to plain hardpoints (legacy Excel models included)."""
    return build(load_config(EXAMPLES / name)).to_dict()


def _reset(name: str) -> None:
    st.session_state.config = _base_config(name)
    st.session_state.loaded = name
    st.session_state.rev = st.session_state.get("rev", 0) + 1  # forces fresh widget keys


@st.cache_resource(max_entries=8)
def _model(config_json: str):
    return build(json.loads(config_json))


@st.cache_data(max_entries=16)
def _sweeps(config_json: str, t_lo: float, t_hi: float, r_lim: float, rack: float):
    m = _model(config_json)
    tr = travel_sweep(m, np.arange(t_lo, t_hi + 1e-9, 2.0), rack=rack)
    st_ = steer_sweep(m, np.arange(-r_lim, r_lim + 1e-9, 2.0))
    rl = roll_sweep(m)
    return tr, st_, rl


# --------------------------------------------------------------------------- sidebar
examples = sorted(p.name for p in EXAMPLES.glob("*.json"))
st.sidebar.title("suspkin explorer")
choice = st.sidebar.selectbox("Example", examples, index=examples.index("double_wishbone.json"))
if st.session_state.get("loaded") != choice:
    _reset(choice)
if st.sidebar.button("Reset to example"):
    _reset(choice)

cfg = copy.deepcopy(st.session_state.config)
rev = st.session_state.rev

st.sidebar.caption(f"Type: **{cfg['type']}**. Units mm, x forward, y outboard, z up, ground at z = 0.")

with st.sidebar.expander("Wheel", expanded=False):
    w = cfg["wheel"]
    w["radius"] = st.number_input("Loaded radius [mm]", value=float(w["radius"]), step=5.0, key=f"w_r_{rev}")
    w["width"] = st.number_input("Width [mm]", value=float(w["width"]), step=5.0, key=f"w_w_{rev}")
    w["static_camber_deg"] = st.number_input(
        "Static camber [deg]", value=float(w["static_camber_deg"]), step=0.1, key=f"w_c_{rev}"
    )
    w["static_toe_deg"] = st.number_input(
        "Static toe [deg]", value=float(w["static_toe_deg"]), step=0.05, key=f"w_t_{rev}"
    )

st.sidebar.markdown("**Hardpoints** (x, y, z)")
step = st.sidebar.select_slider("Edit step [mm]", options=[1.0, 2.0, 5.0, 10.0], value=5.0)
for name, p in cfg["hardpoints"].items():
    with st.sidebar.expander(name.replace("_", " ")):
        c = st.columns(3)
        cfg["hardpoints"][name] = [
            c[i].number_input(ax, value=float(p[i]), step=step, key=f"hp_{name}_{i}_{rev}", format="%.1f")
            for i, ax in enumerate("xyz")
        ]

st.session_state.config = cfg
config_json = json.dumps(cfg, sort_keys=True)

# --------------------------------------------------------------------------- main
st.title("Suspension kinematics explorer")

c1, c2, c3 = st.columns(3)
travel = c1.slider("Wheel travel [mm] (+ bump)", -120.0, 120.0, 0.0, 1.0)
rack = c2.slider("Rack travel [mm] (+ left)", -50.0, 50.0, 0.0, 0.5)
t_lo, t_hi = c3.slider("Travel sweep range [mm]", -150.0, 150.0, (-100.0, 100.0), 10.0)

try:
    model = _model(config_json)
    design = model.design
    state = model.solve(travel, rack)
    now = all_metrics(state, reference=design)
    ref = all_metrics(design)
except Exception as exc:  # solver failure or impossible geometry
    st.error(f"The corner could not be assembled with these inputs: {exc}")
    st.stop()

rows = [
    {"Quantity": label, "Design": ref.get(k, np.nan), "Current": now.get(k, np.nan)}
    for k, label in METRIC_LABELS.items()
    if k in now
]
table = pd.DataFrame(rows)
table["Change"] = table["Current"] - table["Design"]

left, right = st.columns([1, 2])
with left:
    st.subheader("Metrics")
    st.dataframe(table.style.format({"Design": "{:.3f}", "Current": "{:.3f}", "Change": "{:+.3f}"}), hide_index=True)
with right:
    st.subheader("Geometry")
    labels = ["design", f"travel {travel:+.0f} mm, rack {rack:+.1f} mm"]
    fig = plot_geometry([design, state], labels=labels)
    st.pyplot(fig)
    plt.close(fig)

tab_t, tab_s, tab_r, tab_e = st.tabs(["Bump / rebound", "Steer", "Roll", "Export"])
try:
    tr, sw, rl = _sweeps(config_json, t_lo, t_hi, 40.0, rack)
except Exception as exc:
    st.warning(f"Sweep failed over this range: {exc}. Try a smaller travel range.")
    st.stop()

with tab_t:
    fig = plot_travel_sweep({"current": tr})
    for ax in fig.axes:
        ax.axhline(travel, color="tab:red", linewidth=0.8, linestyle="--")
    st.pyplot(fig)
    plt.close(fig)

with tab_s:
    fig, axes = plt.subplots(1, 3, figsize=(13, 3.8))
    for ax, (col, lab) in zip(
        axes,
        [
            ("steer_angle_deg", "Road-wheel angle [deg]"),
            ("camber_deg", "Camber [deg]"),
            ("scrub_radius_mm", "Scrub radius [mm]"),
        ],
    ):
        ax.plot(sw["rack_mm"], sw[col], linewidth=1.8)
        ax.axvline(rack, color="tab:red", linewidth=0.8, linestyle="--")
        ax.set_xlabel("Rack travel [mm]")
        ax.set_ylabel(lab)
        ax.grid(True, alpha=0.3)
    fig.tight_layout()
    st.pyplot(fig)
    plt.close(fig)

with tab_r:
    fig, axes = plt.subplots(1, 2, figsize=(10, 3.8))
    axes[0].plot(rl["roll_deg"], rl["camber_to_ground_deg"], linewidth=1.8)
    axes[0].set_ylabel("Camber to ground [deg]")
    axes[1].plot(rl["roll_deg"], rl["roll_centre_height_mm"], linewidth=1.8)
    axes[1].set_ylabel("Roll centre height [mm]")
    for ax in axes:
        ax.set_xlabel("Body roll [deg]")
        ax.grid(True, alpha=0.3)
    fig.tight_layout()
    st.pyplot(fig)
    plt.close(fig)

with tab_e:
    st.download_button("Model JSON", json.dumps(cfg, indent=2), file_name="model.json", mime="application/json")
    st.download_button("Travel sweep CSV", tr.to_csv(index=False), file_name="travel_sweep.csv", mime="text/csv")
    st.download_button("Steer sweep CSV", sw.to_csv(index=False), file_name="steer_sweep.csv", mime="text/csv")
    st.download_button("Roll sweep CSV", rl.to_csv(index=False), file_name="roll_sweep.csv", mime="text/csv")
    st.caption("Save the JSON into examples/ to use it with the command line tool.")
