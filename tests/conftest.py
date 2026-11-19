from pathlib import Path

import pytest

from suspkin import load_model

EXAMPLES = Path(__file__).resolve().parents[1] / "examples"


@pytest.fixture(params=sorted(p.name for p in EXAMPLES.glob("*.json")))
def example_model(request):
    return load_model(EXAMPLES / request.param)


@pytest.fixture
def validation_model():
    return load_model(EXAMPLES / "adams_validation_2020.json")
