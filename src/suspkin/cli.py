"""Command line interface: ``suspkin <command> CONFIG [options]``."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np


def _range(values):
    lo, hi, step = values
    return np.arange(lo, hi + step / 2, step)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(prog="suspkin", description="Suspension kinematics (double wishbone / MacPherson)")
    sub = parser.add_subparsers(dest="command", required=True)

    p_info = sub.add_parser("info", help="print static geometry metrics")
    p_info.add_argument("config", type=Path)

    p_sweep = sub.add_parser("sweep", help="bump/rebound, steer and roll sweeps -> CSV + PNG")
    p_sweep.add_argument("config", type=Path)
    p_sweep.add_argument("--travel", nargs=3, type=float, metavar=("MIN", "MAX", "STEP"), default=(-100, 100, 2))
    p_sweep.add_argument("--rack", nargs=3, type=float, metavar=("MIN", "MAX", "STEP"), default=(-40, 40, 2))
    p_sweep.add_argument("--roll", nargs=3, type=float, metavar=("MIN", "MAX", "STEP"), default=(-5, 5, 0.5))
    p_sweep.add_argument("--out", type=Path, default=Path("results"))
    p_sweep.add_argument("--no-plots", action="store_true")

    p_hp = sub.add_parser("hardpoints", help="export the 3-D hardpoints of a model (e.g. a legacy Excel model)")
    p_hp.add_argument("config", type=Path)

    args = parser.parse_args(argv)

    from . import analysis
    from .io import load_model

    model = load_model(args.config)

    if args.command == "info":
        for key, value in analysis.summary(model).items():
            print(f"{key:28s} {value:10.3f}")
        return 0

    if args.command == "hardpoints":
        json.dump(model.to_dict(), sys.stdout, indent=2)
        print()
        return 0

    out: Path = args.out
    out.mkdir(parents=True, exist_ok=True)
    stem = args.config.stem
    travel = analysis.travel_sweep(model, _range(args.travel))
    steer = analysis.steer_sweep(model, _range(args.rack))
    roll = analysis.roll_sweep(model, _range(args.roll))
    travel.to_csv(out / f"{stem}_travel.csv", index=False, float_format="%.5g")
    steer.to_csv(out / f"{stem}_steer.csv", index=False, float_format="%.5g")
    roll.to_csv(out / f"{stem}_roll.csv", index=False, float_format="%.5g")
    print(f"wrote {out}/{stem}_{{travel,steer,roll}}.csv")

    if not args.no_plots:
        import matplotlib

        matplotlib.use("Agg")
        from .plotting import plot_geometry, plot_travel_sweep

        fig = plot_travel_sweep({stem: travel}, title=f"{stem} - wheel travel sweep")
        fig.savefig(out / f"{stem}_travel.png", dpi=130)
        lo, hi = float(travel["travel_mm"].min()), float(travel["travel_mm"].max())
        fig = plot_geometry(
            [model.solve(0.0), model.solve(hi), model.solve(lo)],
            labels=["design", f"bump {hi:+.0f} mm", f"rebound {lo:+.0f} mm"],
            title=f"{stem} - geometry",
        )
        fig.savefig(out / f"{stem}_geometry.png", dpi=130)
        print(f"wrote {out}/{stem}_{{travel,geometry}}.png")
    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
