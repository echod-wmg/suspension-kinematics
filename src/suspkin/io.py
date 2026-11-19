"""Loading models from JSON files."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Union

from .models import Corner, build


def load_config(path: Union[str, Path]) -> dict:
    with open(path, encoding="utf-8") as fh:
        return json.load(fh)


def load_model(path: Union[str, Path]) -> Corner:
    """Build a suspension model from a JSON config (see ``examples/``)."""
    return build(load_config(path))


def save_model(model: Corner, path: Union[str, Path]) -> None:
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(model.to_dict(), fh, indent=2)
        fh.write("\n")
