"""Regression against the 2020 ADAMS validation and the 2020 Excel tool."""

from pathlib import Path

import numpy as np
import pandas as pd
import pytest

from suspkin import ExcelDoubleWishbone
from suspkin.io import load_config
from suspkin.metrics import all_metrics

ROOT = Path(__file__).resolve().parents[1]
TABLE = pd.read_csv(ROOT / "validation" / "adams_2020_table1.csv", comment="#")
PARAMS = load_config(ROOT / "examples" / "adams_validation_2020.json")["parameters"]


def test_matches_adams_camber(validation_model):
    for d, expected in zip(TABLE["adams_displacement_mm"], TABLE["adams_camber_deg"]):
        got = all_metrics(validation_model.solve(d))["camber_change_deg"]
        # ADAMS values are tabulated to 0.01 deg
        assert got == pytest.approx(expected, abs=0.02), d


def test_excel_formula_reproduces_2020_spreadsheet():
    rows = TABLE.dropna(subset=["excel_displacement_mm"])
    excel = ExcelDoubleWishbone(**PARAMS)
    got = excel.excel_camber(rows["excel_displacement_mm"].to_numpy())
    assert np.allclose(got, rows["excel_camber_deg"], atol=0.05)


def test_static_values_match_report_fig35(validation_model):
    m = all_metrics(validation_model.design)
    assert 2 * m["half_track_mm"] == pytest.approx(1398.0, abs=0.1)
    assert m["roll_centre_height_mm"] == pytest.approx(141.64, abs=0.01)
    h = validation_model.hardpoints
    upper = np.degrees(np.arcsin((h["upper_ball_joint"][2] - h["upper_front_pivot"][2]) / 325.0))
    lower = np.degrees(np.arcsin((h["lower_ball_joint"][2] - h["lower_front_pivot"][2]) / 500.0))
    assert upper == pytest.approx(6.93, abs=0.01)
    assert lower == pytest.approx(-3.53, abs=0.01)


def test_matches_independent_planar_solution(validation_model):
    """Zero caster -> the 3-D solver must equal a closed-form 4-bar solution."""
    excel = ExcelDoubleWishbone(**PARAMS)
    yl, zl = PARAMS["chassis_lower_width"] / 2, PARAMS["chassis_lower_height"]
    yu, zu = PARAMS["chassis_upper_width"] / 2, PARAMS["chassis_upper_height"]
    lu, ll, k = PARAMS["upper_arm_length"], PARAMS["lower_arm_length"], PARAMS["knuckle_length"]
    h0 = validation_model.hardpoints
    k_actual = np.linalg.norm(h0["upper_ball_joint"] - h0["lower_ball_joint"])  # 220.00001 mm
    for travel in (-120.0, -40.0, 35.0, 110.0):
        z_l = excel.wheel_radius - k / 2 + travel
        lbj = np.array([yl + np.sqrt(ll**2 - (zl - z_l) ** 2), z_l])
        up = np.array([yu, zu])
        dist = np.linalg.norm(lbj - up)
        a = (lu**2 - k_actual**2 + dist**2) / (2 * dist)
        h = np.sqrt(lu**2 - a**2)
        base = up + a * (lbj - up) / dist
        perp = np.array([-(lbj - up)[1], (lbj - up)[0]]) / dist
        ubj = max(base + h * perp, base - h * perp, key=lambda p: p[1])
        state = validation_model.solve(travel)
        assert state["upper_ball_joint"][1:] == pytest.approx(ubj, abs=1e-8)
