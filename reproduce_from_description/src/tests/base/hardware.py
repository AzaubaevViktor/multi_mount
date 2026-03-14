from __future__ import annotations

import os

import pytest


def require_hardware() -> None:
    if os.getenv("REPRODUCE_HW") != "1":
        pytest.skip("Set REPRODUCE_HW=1 to enable hardware acceptance tests")
