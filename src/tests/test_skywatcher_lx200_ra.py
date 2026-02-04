import time

import pytest

from lx200.protocols import LX200Ha
from skywatcher.skywatcher import (
    Direction,
    SkyWatcherMount,
    SkyWatcherStatus,
    SlewMode,
    SpeedMode,
)
from skywatcher.skywatcher_lx200 import DEGREES_PER_HOUR, SkyWatcherLX200


NO_TRACKING_SPEED = 0.0


class FakeClock:
    def __init__(self, start_s: float = 0.0) -> None:
        self._now_s = start_s

    def monotonic(self) -> float:
        return self._now_s

    def advance(self, delta_s: float) -> None:
        if delta_s < 0:
            raise ValueError("delta_s must be non-negative")
        self._now_s += delta_s


class FakeMount:
    STELLAR_SPEED = SkyWatcherMount.STELLAR_SPEED

    def __init__(self, clock: FakeClock, start_ra: LX200Ha) -> None:
        self._clock = clock
        self._tracking_speed = NO_TRACKING_SPEED
        self._reference_seconds = start_ra.to_seconds()
        self._reference_time = self._clock.monotonic()
        self._status = SkyWatcherStatus(
            raw=0,
            running=False,
            initialized=True,
            slew_mode=SlewMode.SLEW,
            direction=Direction.FORWARD,
            speed_mode=SpeedMode.LOWSPEED,
        )
        self.connect_called = False
        self.start_tracking_calls: list[float] = []
        self.slew_to_ra_calls: list[LX200Ha] = []

    def connect(self) -> None:
        self.connect_called = True
        self._status.initialized = True

    def start_tracking(self, trackspeed: float = STELLAR_SPEED) -> bool:
        self._freeze_reference()
        self._tracking_speed = trackspeed
        self._status.running = trackspeed != 0
        self._status.slew_mode = SlewMode.SLEW
        self._status.direction = Direction.FORWARD if trackspeed >= 0 else Direction.BACKWARD
        self.start_tracking_calls.append(trackspeed)
        return True

    def gracefully_stop_motor(self) -> None:
        self.start_tracking(NO_TRACKING_SPEED)

    def get_status(self) -> SkyWatcherStatus:
        return self._status

    def get_telescope_ra(self) -> LX200Ha:
        return LX200Ha.from_seconds(self._current_seconds())

    def set_telescope_ra(self, position: LX200Ha) -> bool:
        self._reference_seconds = position.to_seconds()
        self._reference_time = self._clock.monotonic()
        return True

    def slew_to_ra(self, position: LX200Ha) -> bool:
        self.set_telescope_ra(position)
        self._status.slew_mode = SlewMode.GOTO
        self._status.running = False
        self.slew_to_ra_calls.append(position)
        return True

    def _current_seconds(self) -> int:
        elapsed = self._clock.monotonic() - self._reference_time
        delta_seconds = elapsed * (self._tracking_speed / DEGREES_PER_HOUR)
        return int(round(self._reference_seconds + delta_seconds)) % LX200Ha.SECONDS_PER_CIRCLE

    def _freeze_reference(self) -> None:
        self._reference_seconds = self._current_seconds()
        self._reference_time = self._clock.monotonic()


@pytest.fixture
def clock(monkeypatch: pytest.MonkeyPatch) -> FakeClock:
    clock = FakeClock()
    monkeypatch.setattr(time, "monotonic", clock.monotonic)
    return clock


def _expected_ra_seconds(start_seconds: int, elapsed_s: float, tracking_speed: float) -> int:
    delta_seconds = elapsed_s * (tracking_speed / DEGREES_PER_HOUR)
    return int(round(start_seconds + delta_seconds)) % LX200Ha.SECONDS_PER_CIRCLE


def test_lx200_ra_stays_constant_when_mount_tracks(clock: FakeClock) -> None:
    start = LX200Ha.from_string("10:00:00")
    mount = FakeMount(clock, start)
    mount.start_tracking()

    lx200 = SkyWatcherLX200(mount)
    lx200.set_telescope_ra(start)

    clock.advance(2.0)
    assert lx200.get_telescope_ra().to_seconds() == start.to_seconds()

    clock.advance(2.0)
    assert lx200.get_telescope_ra().to_seconds() == start.to_seconds()


def test_lx200_ra_advances_when_mount_stationary(clock: FakeClock) -> None:
    start = LX200Ha.from_string("10:00:00")
    mount = FakeMount(clock, start)
    mount.start_tracking(NO_TRACKING_SPEED)

    lx200 = SkyWatcherLX200(mount)
    lx200.set_telescope_ra(start)

    elapsed_s = 10.0
    clock.advance(elapsed_s)

    ra = lx200.get_telescope_ra()
    expected = _expected_ra_seconds(start.to_seconds(), elapsed_s, mount.STELLAR_SPEED)
    assert ra.to_seconds() == expected


def test_lx200_ra_ignores_mount_rollover(clock: FakeClock) -> None:
    start = LX200Ha.from_string("23:59:59")
    mount = FakeMount(clock, start)
    mount.start_tracking()

    lx200 = SkyWatcherLX200(mount)
    lx200.set_telescope_ra(start)

    clock.advance(2.0)

    ra = lx200.get_telescope_ra()
    expected_mount = _expected_ra_seconds(start.to_seconds(), 2.0, mount.STELLAR_SPEED)

    assert mount.get_telescope_ra().to_seconds() == expected_mount
    assert ra.to_seconds() == start.to_seconds()


def test_lx200_connect_starts_tracking_and_stabilizes_ra(clock: FakeClock) -> None:
    start = LX200Ha.from_string("03:21:09")
    mount = FakeMount(clock, start)
    lx200 = SkyWatcherLX200(mount)

    lx200.connect()

    assert mount.connect_called is True
    assert mount.start_tracking_calls == [mount.STELLAR_SPEED]

    clock.advance(3.0)
    assert lx200.get_telescope_ra().to_seconds() == 0

    clock.advance(2.0)
    assert lx200.get_telescope_ra().to_seconds() == 0


def test_lx200_slew_to_ra_delegates_to_mount(clock: FakeClock) -> None:
    start = LX200Ha.from_string("01:00:00")
    mount = FakeMount(clock, start)
    lx200 = SkyWatcherLX200(mount)

    target = LX200Ha.from_string("19:30:00")

    assert lx200.slew_to_ra(target) is True
    assert mount.slew_to_ra_calls == [target]
    assert mount.get_telescope_ra().to_seconds() == target.to_seconds()
