import numpy as np
import pytest

from suspkin import DoubleWishbone, Wheel, travel_sweep
from suspkin.metrics import all_metrics
from suspkin.solver import SolverError


def _links(model):
    if isinstance(model, DoubleWishbone):
        return [
            ("lower_ball_joint", "lower_front_pivot"),
            ("lower_ball_joint", "lower_rear_pivot"),
            ("upper_ball_joint", "upper_front_pivot"),
            ("upper_ball_joint", "upper_rear_pivot"),
            ("tie_rod_outer", "tie_rod_inner"),
        ]
    return [
        ("lower_ball_joint", "lower_front_pivot"),
        ("lower_ball_joint", "lower_rear_pivot"),
        ("tie_rod_outer", "tie_rod_inner"),
    ]


@pytest.mark.parametrize("travel", [-80.0, -25.0, 0.0, 30.0, 90.0])
@pytest.mark.parametrize("rack", [-20.0, 0.0, 15.0])
def test_constraints_hold(example_model, travel, rack):
    state = example_model.solve(travel, rack)
    design = example_model.design
    for a, b in _links(example_model):
        if b == "tie_rod_inner":
            d0 = np.linalg.norm(design[a] - design[b])
        else:
            d0 = np.linalg.norm(example_model.hardpoints[a] - example_model.hardpoints[b])
        assert np.linalg.norm(state[a] - state[b]) == pytest.approx(d0, abs=1e-6)
    # travel is what was asked for
    ref = example_model.travel_reference
    assert state[ref][2] - design[ref][2] == pytest.approx(travel, abs=1e-6)
    # the knuckle is rigid: distances between knuckle points are preserved
    names = example_model.knuckle_names
    for i, a in enumerate(names):
        for b in names[i + 1 :]:
            d0 = np.linalg.norm(design[a] - design[b])
            assert np.linalg.norm(state[a] - state[b]) == pytest.approx(d0, abs=1e-6)


def test_macpherson_top_mount_on_strut_axis():
    from pathlib import Path

    from suspkin import load_model

    model = load_model(Path(__file__).resolve().parents[1] / "examples" / "macpherson.json")
    h = model.hardpoints
    axis_dir0 = (h["strut_top_mount"] - h["strut_lower"]) / np.linalg.norm(h["strut_top_mount"] - h["strut_lower"])
    for travel in (-70.0, 60.0):
        state = model.solve(travel)
        # rotate the design strut axis with the knuckle: use two knuckle points
        q = state["strut_lower"]
        # direction of the strut axis in the moved knuckle = R * axis_dir0;
        # recover R from three knuckle points
        a0 = np.array([h[n] for n in ("lower_ball_joint", "strut_lower", "tie_rod_outer")])
        a1 = np.array([state[n] for n in ("lower_ball_joint", "strut_lower", "tie_rod_outer")])
        c0, c1 = a0.mean(0), a1.mean(0)
        u, _, vt = np.linalg.svd((a0 - c0).T @ (a1 - c1))
        r = (u @ vt).T
        if np.linalg.det(r) < 0:
            vt[-1] *= -1
            r = (u @ vt).T
        d = r @ axis_dir0
        w = state["strut_top_mount"] - q
        assert np.linalg.norm(np.cross(w, d)) == pytest.approx(0.0, abs=1e-5)


def test_static_camber_and_toe_are_respected():
    hp = {
        "lower_front_pivot": [150, 330, 160],
        "lower_rear_pivot": [-200, 330, 165],
        "lower_ball_joint": [0, 735, 150],
        "upper_front_pivot": [100, 420, 470],
        "upper_rear_pivot": [-150, 420, 460],
        "upper_ball_joint": [-20, 690, 480],
        "tie_rod_inner": [-120, 360, 232.5],
        "tie_rod_outer": [-130, 715, 230],
        "wheel_centre": [0, 760, 315],
    }
    m = DoubleWishbone(hp, Wheel(radius=315, static_camber_deg=-1.25, static_toe_deg=0.3))
    s = all_metrics(m.design)
    assert s["camber_deg"] == pytest.approx(-1.25, abs=1e-9)
    assert s["toe_deg"] == pytest.approx(0.3, abs=1e-9)
    assert s["caster_deg"] == pytest.approx(np.degrees(np.arctan2(20, 330)), abs=1e-9)
    assert s["kpi_deg"] == pytest.approx(np.degrees(np.arctan2(45, 330)), abs=1e-9)


def test_short_long_arm_gains_negative_camber_in_bump(example_model):
    df = travel_sweep(example_model, np.arange(-60, 61, 10.0))
    bump = df.loc[df.travel_mm == 60, "camber_change_deg"].item()
    assert bump < 0


def test_sweep_is_independent_of_order(validation_model):
    a = travel_sweep(validation_model, [-100, -50, 0, 50, 100])
    b = travel_sweep(validation_model, [100, 50, 0, -50, -100]).iloc[::-1].reset_index(drop=True)
    assert np.allclose(a.to_numpy(), b.to_numpy(), atol=1e-8)


def test_out_of_range_travel_raises(validation_model):
    with pytest.raises(SolverError):
        validation_model.solve(900.0)


def test_missing_hardpoint_is_reported():
    with pytest.raises(ValueError, match="missing hardpoints"):
        DoubleWishbone({"wheel_centre": [0, 700, 300]}, Wheel(radius=300))
