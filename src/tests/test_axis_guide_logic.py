import time

import pytest

from lx200.base import LX200DECHandler, LX200RAHandler
from lx200.splitter import LX200Splitter
from sky.physics import DecPerSecond, HaPerSecond, Second


def _wait_for_rate_updates(axis, count: int, timeout_s: float = 1.5) -> None:
    start = time.monotonic()
    while len(axis.applied_rates) < count:
        if time.monotonic() - start > timeout_s:
            pytest.fail(
                f"expected at least {count} guide updates, got {len(axis.applied_rates)}: {axis.applied_rates}"
            )
        time.sleep(0.01)


def _wait_for_halt(axis, count: int, timeout_s: float = 1.5) -> None:
    start = time.monotonic()
    while axis.halt_calls < count:
        if time.monotonic() - start > timeout_s:
            pytest.fail(f"expected at least {count} halt calls, got {axis.halt_calls}")
        time.sleep(0.01)


class _DummyRAHandler(LX200RAHandler):
    _TELEMETRY_INTERVAL_S = 0.05
    _RATE_COMPENSATE_INTERVAL_S = Second(0.05)

    def __init__(self) -> None:
        self.applied_rates: list[float] = []
        self.halt_calls = 0
        super().__init__()

    def _is_motor_connected(self) -> bool:
        return True

    def _get_motor_status(self):
        return "ok"

    def _get_motor_raw_position(self):
        return self._motor_position_raw

    def _set_tracking_speed(self, rate: HaPerSecond) -> None:
        self.applied_rates.append(float(rate))

    def _halt_motion(self) -> None:
        self.halt_calls += 1


class _DummyDECHandler(LX200DECHandler):
    _TELEMETRY_INTERVAL_S = 0.05
    _RATE_COMPENSATE_INTERVAL_S = Second(0.05)

    def __init__(self) -> None:
        self.applied_rates: list[float] = []
        self.halt_calls = 0
        super().__init__()

    def _is_motor_connected(self) -> bool:
        return True

    def _get_motor_status(self):
        return "ok"

    def _get_motor_raw_position(self):
        return self._motor_position_raw

    def _set_tracking_speed(self, rate: DecPerSecond) -> None:
        self.applied_rates.append(float(rate))

    def _halt_motion(self) -> None:
        self.halt_calls += 1


class _SlowQueueRAHandler(_DummyRAHandler):
    _RATE_COMPENSATE_INTERVAL_S = Second(2.0)


def test_ra_guide_applies_east_and_west_with_opposite_rates() -> None:
    axis = _DummyRAHandler()
    try:
        axis.guide_east(2000)
        axis.guide_west(2000)
        _wait_for_rate_updates(axis, count=2)
        default_rate = float(axis.DEFAULT_TRACKING_SPEED)
        assert axis.applied_rates[:2] == pytest.approx([default_rate * 1.5, default_rate * 0.5])
        assert float(axis._sky_track_speed) == pytest.approx(default_rate * 0.5)
    finally:
        axis.stop()


def test_dec_guide_applies_north_and_south_with_opposite_rates() -> None:
    axis = _DummyDECHandler()
    try:
        axis.guide_north(2000)
        axis.guide_south(2000)
        _wait_for_rate_updates(axis, count=2)
        half_range = float(axis.FORWARD_TRACKING_SPEED) * 0.5
        assert axis.applied_rates[:2] == pytest.approx([-half_range, half_range])
        assert float(axis._sky_track_speed) == pytest.approx(half_range)
    finally:
        axis.stop()


def test_set_tracking_rate_updates_current_and_sky_by_flag() -> None:
    axis = _DummyRAHandler()
    try:
        axis.set_tracking_speed(HaPerSecond(1.25), update_sky_speed=False)
        _wait_for_rate_updates(axis, count=1)
        assert axis.applied_rates[-1] == pytest.approx(1.25)
        assert float(axis._sky_track_speed) == pytest.approx(float(axis.DEFAULT_TRACKING_SPEED))

        axis.set_tracking_speed(HaPerSecond(1.4), update_sky_speed=True)
        _wait_for_rate_updates(axis, count=2)
        assert axis.applied_rates[-1] == pytest.approx(1.4)
        assert float(axis._sky_track_speed) == pytest.approx(1.4)
    finally:
        axis.stop()


def test_axis_commands_apply_immediately_without_waiting_compensate_interval() -> None:
    axis = _SlowQueueRAHandler()
    try:
        start = time.monotonic()
        axis.set_tracking_speed(HaPerSecond(1.3))
        _wait_for_rate_updates(axis, count=1, timeout_s=0.5)
        assert time.monotonic() - start < 0.5

        axis.set_tracking_speed(HaPerSecond(1.1), update_sky_speed=True)
        _wait_for_rate_updates(axis, count=2, timeout_s=0.5)

        halt_start = time.monotonic()
        assert axis.halt_east() is True
        _wait_for_halt(axis, count=1, timeout_s=0.5)
        _wait_for_rate_updates(axis, count=3, timeout_s=0.5)
        assert time.monotonic() - halt_start < 0.5
        assert axis.applied_rates[-1] == pytest.approx(1.1)
    finally:
        axis.stop()


def test_splitter_ra_guide_applies_without_dec_update() -> None:
    ra = _DummyRAHandler()
    dec = _DummyDECHandler()
    splitter = LX200Splitter(ra=ra, dec=dec)
    splitter._polar_compensator.start()

    try:
        splitter.guide_east(2000)
        _wait_for_rate_updates(ra, count=1)
        _wait_for_rate_updates(dec, count=1)

        assert ra.applied_rates[-1] == pytest.approx(float(ra.DEFAULT_TRACKING_SPEED) * 1.5)
        assert dec.applied_rates[-1] == pytest.approx(0.0)
    finally:
        splitter.stop()


def test_splitter_dec_guide_applies_without_ra_update() -> None:
    ra = _DummyRAHandler()
    dec = _DummyDECHandler()
    splitter = LX200Splitter(ra=ra, dec=dec)
    splitter._polar_compensator.start()

    try:
        splitter.guide_north(2000)
        _wait_for_rate_updates(ra, count=1)
        _wait_for_rate_updates(dec, count=1)

        assert ra.applied_rates[-1] == pytest.approx(float(ra.DEFAULT_TRACKING_SPEED))
        assert dec.applied_rates[-1] == pytest.approx(-float(dec.FORWARD_TRACKING_SPEED) * 0.5)
    finally:
        splitter.stop()
