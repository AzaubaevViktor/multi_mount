import time

import pytest

from lx200.base import LX200DECHandler, LX200RAHandler


def _wait_for_rate_updates(axis, count: int, timeout_s: float = 1.5) -> None:
    start = time.monotonic()
    while len(axis.applied_rates) < count:
        if time.monotonic() - start > timeout_s:
            pytest.fail(
                f"expected at least {count} guide updates, got {len(axis.applied_rates)}: {axis.applied_rates}"
            )
        time.sleep(0.01)


class _DummyRAHandler(LX200RAHandler):
    _TELEMETRY_INTERVAL_S = 0.05
    _RATE_COMPENSATE_INTERVAL_S = 0.05

    def __init__(self) -> None:
        self.applied_rates: list[float] = []
        super().__init__()

    def _is_motor_connected(self) -> bool:
        return True

    def _get_motor_status(self):
        return "ok"

    def _get_motor_raw_position(self) -> float:
        return self._motor_position_raw

    def _get_default_tracking_speed(self) -> float:
        return 1.0

    def _wrap_mount_position(self, mount_position: float) -> float:
        return mount_position

    def _set_tracking_rate(self, rate: float) -> None:
        self.applied_rates.append(rate)

    def _halt_motion(self) -> None:
        pass


class _DummyDECHandler(LX200DECHandler):
    _TELEMETRY_INTERVAL_S = 0.05
    _RATE_COMPENSATE_INTERVAL_S = 0.05

    def __init__(self) -> None:
        self.applied_rates: list[float] = []
        super().__init__()

    def _is_motor_connected(self) -> bool:
        return True

    def _get_motor_status(self):
        return "ok"

    def _get_motor_raw_position(self) -> float:
        return self._motor_position_raw

    def _get_default_tracking_speed(self) -> float:
        return 1.0

    def _wrap_mount_position(self, mount_position: float) -> float:
        return mount_position

    def _set_tracking_rate(self, rate: float) -> None:
        self.applied_rates.append(rate)

    def _halt_motion(self) -> None:
        pass


def test_ra_guide_applies_east_and_west_with_opposite_rates() -> None:
    axis = _DummyRAHandler()
    try:
        axis.guide_east(2000)
        axis.guide_west(2000)
        _wait_for_rate_updates(axis, count=2)
        assert axis.applied_rates[:2] == pytest.approx([1.5, 0.5])
    finally:
        axis.stop()


def test_dec_guide_applies_north_and_south_with_opposite_rates() -> None:
    axis = _DummyDECHandler()
    try:
        axis.guide_north(2000)
        axis.guide_south(2000)
        _wait_for_rate_updates(axis, count=2)
        assert axis.applied_rates[:2] == pytest.approx([-0.5, 0.5])
    finally:
        axis.stop()

