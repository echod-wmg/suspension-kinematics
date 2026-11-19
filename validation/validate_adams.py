"""Re-run the 2020 ADAMS validation case with the exact solver.

Usage:  python validation/validate_adams.py

Compares, against the MSC ADAMS camber curve reported in 2020:
  * the camber values the 2020 Excel tool reported (from the report table),
  * this repository's re-implementation of the Excel approximation,
  * the exact 3-D loop-closure solver in ``suspkin``.

Writes ``validation/adams_comparison.csv`` and ``docs/img/validation_adams.png``.
"""

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402

from suspkin import load_model  # noqa: E402
from suspkin.io import load_config  # noqa: E402
from suspkin.legacy import ExcelDoubleWishbone  # noqa: E402
from suspkin.metrics import all_metrics  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "examples" / "adams_validation_2020.json"


def run() -> pd.DataFrame:
    table = pd.read_csv(Path(__file__).with_name("adams_2020_table1.csv"), comment="#")
    # ADAMS repeats end points (motion reversal); keep unique travel values
    table = table.drop_duplicates("adams_displacement_mm").reset_index(drop=True)
    model = load_model(CONFIG)
    excel = ExcelDoubleWishbone(**load_config(CONFIG)["parameters"])
    d = table["adams_displacement_mm"].to_numpy()
    table["suspkin_camber_deg"] = [all_metrics(model.solve(x))["camber_change_deg"] for x in d]
    table["excel_formula_camber_deg"] = excel.excel_camber(d)
    table["suspkin_error_deg"] = table["suspkin_camber_deg"] - table["adams_camber_deg"]
    table["excel_formula_error_deg"] = table["excel_formula_camber_deg"] - table["adams_camber_deg"]
    return table


def plot(table: pd.DataFrame, path: Path) -> None:
    fig, (ax, ax2) = plt.subplots(1, 2, figsize=(11, 4.4))
    sweep = np.linspace(-165, 161, 200)
    model = load_model(CONFIG)
    excel = ExcelDoubleWishbone(**load_config(CONFIG)["parameters"])
    ax.plot(
        [all_metrics(model.solve(x))["camber_change_deg"] for x in sweep],
        sweep,
        color="tab:blue",
        label="suspkin (exact)",
    )
    ax.plot(excel.excel_camber(sweep), sweep, "--", color="tab:orange", label="2020 Excel formula")
    ax.plot(table["adams_camber_deg"], table["adams_displacement_mm"], "ko", ms=4, label="MSC ADAMS (2020)")
    ax.set_xlabel("Camber [deg]")
    ax.set_ylabel("Lower ball joint travel [mm]")
    ax.set_title("Camber vs wheel travel")
    ax.grid(alpha=0.3)
    ax.legend()
    ax2.axvline(0, color="0.5", lw=0.8)
    ax2.plot(
        table["suspkin_error_deg"],
        table["adams_displacement_mm"],
        "o-",
        color="tab:blue",
        ms=4,
        label="suspkin - ADAMS",
    )
    ax2.plot(
        table["excel_formula_error_deg"],
        table["adams_displacement_mm"],
        "s--",
        color="tab:orange",
        ms=4,
        label="Excel - ADAMS",
    )
    ax2.set_xlabel("Camber error vs ADAMS [deg]")
    ax2.set_title("Error")
    ax2.grid(alpha=0.3)
    ax2.legend()
    fig.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=130)


if __name__ == "__main__":
    t = run()
    t.to_csv(Path(__file__).with_name("adams_comparison.csv"), index=False, float_format="%.4f")
    plot(t, ROOT / "docs" / "img" / "validation_adams.png")
    print(t.round(3).to_string(index=False))
    print()
    print(f"max |error| suspkin vs ADAMS: {t['suspkin_error_deg'].abs().max():.3f} deg")
    print(f"max |error| Excel   vs ADAMS: {t['excel_formula_error_deg'].abs().max():.3f} deg")
