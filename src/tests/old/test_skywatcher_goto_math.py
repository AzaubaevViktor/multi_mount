import time

import pytest

from sky.constants import STELLAR_SPEED
from sky.physics import Direction, Ha, Second
from skywatcher.skywatcher import (
    Direction as MotorDirection,
    SkyWatcherMount,
    SkyWatcherStatus,
    SlewMode,
    SpeedMode,
)
from skywatcher.skywatcher_lx200 import SkyWatcherLX200


class _DummySerial:
    terminator = b"\r"
    encoding = "ascii"


class _GotoDummyMount:
    MAX_SPEED = STELLAR_SPEED * 800

    def __init__(self) -> None:
        self.is_connected = True
        self._ha = Ha(0)
        self.stop_calls = 0
        self.slew_calls: list[Ha] = []
        self._status = SkyWatcherStatus(
            raw=0,
            running=False,
            initialized=True,
            slew_mode=SlewMode.SLEW,
            direction=MotorDirection.FORWARD,
            speed_mode=SpeedMode.LOWSPEED,
        )

    def get_status(self) -> SkyWatcherStatus:
        return self._status

    def get_telescope_ha(self) -> Ha:
        return self._ha

    def wait_till_stop(self, do_stop: bool = False, timeout_s: float | None = None, func=None) -> None:
        self.stop_calls += 1
        self._status = SkyWatcherStatus(
            raw=0,
            running=False,
            initialized=True,
            slew_mode=SlewMode.SLEW,
            direction=self._status.direction,
            speed_mode=self._status.speed_mode,
        )

    def start_tracking(self, rate) -> None:
        return None

    def connect(self) -> None:
        self.is_connected = True

    def disconnect(self) -> None:
        self.is_connected = False

    def move_ra(self, speed) -> bool:
        return True

    def get_slew_real_speed(self, delta_seconds: Ha):
        if abs(delta_seconds) > Ha(10 * 60):
            speed = STELLAR_SPEED * 800
        else:
            speed = STELLAR_SPEED * 128

        if delta_seconds < Ha(0):
            speed *= -1

        return speed

    def slew_delta(self, delta: Ha) -> bool:
        self.slew_calls.append(delta)
        self._status = SkyWatcherStatus(
            raw=0,
            running=True,
            initialized=True,
            slew_mode=SlewMode.GOTO,
            direction=MotorDirection.FORWARD if delta > Ha(0) else MotorDirection.BACKWARD,
            speed_mode=SpeedMode.LOWSPEED,
        )
        return True


class _TestSkyWatcherLX200(SkyWatcherLX200):
    _GOTO_CHECK_INTERVAL_S = 0.05
    _RATE_COMPENSATE_INTERVAL_S = Second(10)
    _TELEMETRY_INTERVAL_S = 10


def _wait_for_slew(axis: SkyWatcherLX200, mount: _GotoDummyMount, timeout_s: float = 1.0) -> None:
    start = time.monotonic()
    while not mount.slew_calls:
        if time.monotonic() - start > timeout_s:
            pytest.fail("GOTO did not issue slew_delta in time")
        time.sleep(0.01)


@pytest.mark.parametrize(
    ("delta_seconds", "expected_abs_seconds", "expected_rate_sign"),
    (
        pytest.param(1000.0, 1000.0, 1, id="positive_delta_keeps_positive_rate"),
        pytest.param(-1000.0, 1000.0, -1, id="negative_delta_keeps_negative_rate"),
        pytest.param(50000.0, 36400.0, -1, id="positive_wraps_to_shortest_negative"),
        pytest.param(-50000.0, 36400.0, 1, id="negative_wraps_to_shortest_positive"),
    ),
)
def test_wrap_delta_move_keeps_direction(delta_seconds: float, expected_abs_seconds: float, expected_rate_sign: int) -> None:
    mount = SkyWatcherMount(_DummySerial())

    wrapped_seconds, rate = mount._do_wrap_delta_move(Ha(delta_seconds))

    assert float(wrapped_seconds) == pytest.approx(expected_abs_seconds)
    assert (float(rate) > 0) == (expected_rate_sign > 0)


@pytest.mark.parametrize(
    ("coordinate_delta_seconds", "expected_motor_sign", "expected_direction_sign"),
    (
        pytest.param(400.0, -1, Direction.FORWARD, id="positive_coordinate_delta_uses_negative_motor_delta"),
        pytest.param(-400.0, 1, Direction.BACKWARD, id="negative_coordinate_delta_uses_positive_motor_delta"),
    ),
)
def test_lx200_goto_starts_short_motor_move_without_immediate_overshoot(
    coordinate_delta_seconds: float,
    expected_motor_sign: int,
    expected_direction_sign: Direction,
) -> None:
    mount = _GotoDummyMount()
    axis = _TestSkyWatcherLX200(mount)

    try:
        start = Ha(12 * 60 * 60)
        axis.sync_telescope_ra(start)
        target = Ha((float(start) + coordinate_delta_seconds) % (24 * 60 * 60))

        assert axis.slew_to_ra(target) is True
        _wait_for_slew(axis, mount)

        commanded_motor_delta = float(mount.slew_calls[0])
        assert commanded_motor_delta * expected_motor_sign > 0
        assert abs(commanded_motor_delta) == pytest.approx(abs(coordinate_delta_seconds), rel=0.02, abs=2.0)

        time.sleep(axis._GOTO_CHECK_INTERVAL_S * 3)

        assert axis._goto_to == target
        assert axis._goto_direction_sign == expected_direction_sign
        assert mount.stop_calls == 0
    finally:
        axis.stop()
