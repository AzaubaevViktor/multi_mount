from __future__ import annotations

from tests.base.hardware import require_hardware


def test_tmc2209_motor_acceptance() -> None:
    require_hardware()
