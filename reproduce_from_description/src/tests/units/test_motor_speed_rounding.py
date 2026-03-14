from __future__ import annotations

import pytest

from sky.constants import SIDEREAL_RATE_HOURS_PER_SECOND
from sky.physics import DecPerSecond, HaPerSecond
from skywatcher.motor import SkyWatcherConfig, SkyWatcherMotor
from skywatcher.protocol import SkyWatcherCommand
from tmc2209.motor import TMC2209Config, TMC2209Motor


def test_skywatcher_speed_switches_to_high_speed_mode() -> None:
    motor = SkyWatcherMotor(
        SkyWatcherConfig(
            steps_per_revolution=100000,
            timer_frequency_hz=1000000,
            high_speed_ratio=8,
            high_speed_threshold_sps=256.0,
        )
    )
    calls: list[tuple[SkyWatcherCommand, str]] = []
    motor._query = lambda command, payload="": calls.append((command, payload)) or "000000"  # type: ignore[method-assign]

    motor.set_speed(800.0)

    assert calls[0][0] == SkyWatcherCommand.SET_STEP_PERIOD
    assert calls[1] == (SkyWatcherCommand.SET_MOTION_MODE, "06")


def test_skywatcher_speed_conversion_rejects_negative_values() -> None:
    motor = SkyWatcherMotor(SkyWatcherConfig(steps_per_revolution=100000, timer_frequency_hz=1000000, high_speed_ratio=4))

    with pytest.raises(ValueError):
        motor.convert_speed_to_steps_per_second(HaPerSecond(-SIDEREAL_RATE_HOURS_PER_SECOND))


def test_tmc2209_rounds_speed_command() -> None:
    motor = TMC2209Motor(TMC2209Config())
    calls: list[str] = []
    motor._command = lambda payload: calls.append(payload) or None  # type: ignore[method-assign]

    motor.set_speed(12.6)

    assert calls == ["speed 13"]


def test_tmc2209_speed_conversion_rejects_negative_values() -> None:
    motor = TMC2209Motor(TMC2209Config())

    with pytest.raises(ValueError):
        motor.convert_speed_to_steps_per_second(DecPerSecond(-0.1))
