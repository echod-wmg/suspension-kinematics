from pathlib import Path

import numpy as np

from suspkin import roll_sweep, steer_sweep, summary
from suspkin.cli import main

EXAMPLES = Path(__file__).resolve().parents[1] / "examples"


def test_steer_sweep_sign(example_model):
    df = steer_sweep(example_model, np.arange(-20, 21, 5.0))
    # rack to the left (+y) pushes the tie rod (behind the axle) outboard on the
    # left corner -> front of the wheel turns inboard -> the wheel steers right
    assert df.loc[df.rack_mm == 20, "steer_angle_deg"].item() < 0
    assert df.loc[df.rack_mm == -20, "steer_angle_deg"].item() > 0
    assert np.all(np.diff(df["steer_angle_deg"]) < 0)


def test_roll_sweep_columns(validation_model):
    df = roll_sweep(validation_model, [-2.0, 0.0, 2.0])
    assert {"roll_deg", "camber_to_ground_deg"} <= set(df.columns)
    assert df.loc[df.roll_deg == 0, "camber_to_ground_deg"].item() == df.loc[df.roll_deg == 0, "camber_deg"].item()


def test_summary_keys(validation_model):
    s = summary(validation_model)
    assert "camber_gain_deg_per_10mm" in s and "bump_steer_deg_per_10mm" in s
    # the generated tie rod gives near-zero bump steer for legacy models
    assert abs(s["bump_steer_deg_per_10mm"]) < 0.01


def test_cli_info_and_sweep(tmp_path, capsys):
    cfg = EXAMPLES / "macpherson.json"
    assert main(["info", str(cfg)]) == 0
    assert "camber_deg" in capsys.readouterr().out
    assert main(["sweep", str(cfg), "--travel", "-40", "40", "10", "--out", str(tmp_path)]) == 0
    for suffix in ("travel.csv", "steer.csv", "roll.csv", "travel.png", "geometry.png"):
        assert (tmp_path / f"macpherson_{suffix}").exists()


def test_cli_hardpoints_roundtrip(tmp_path, capsys):
    import json

    from suspkin import build

    assert main(["hardpoints", str(EXAMPLES / "excel_default_2020.json")]) == 0
    cfg = json.loads(capsys.readouterr().out)
    assert cfg["type"] == "double_wishbone"
    model = build(cfg)
    assert model.design["wheel_centre"][2] > 0
