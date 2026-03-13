import pytest
from sky.constants import STELLAR_SPEED
from sky.physics import HaPerSecond

from skywatcher.skywatcher import (
    Direction,
    SkyWatcherMotionStatus,
    SkyWatcherMount,
    SkyWatcherStatus,
    SlewMode,
    SpeedMode,
)


class _DummySerial:
    terminator = b"\r"
    encoding = "ascii"


def _build_mount() -> SkyWatcherMount:
    mount = SkyWatcherMount(_DummySerial())
    mount.ra_steps_360 = 1000
    mount.ra_steps_worm = 1000
    mount.ra_highspeed_ratio = 2
    return mount


def _status(running: bool) -> SkyWatcherStatus:
    return SkyWatcherStatus(
        raw=0,
        running=running,
        initialized=True,
        slew_mode=SlewMode.SLEW,
        direction=Direction.FORWARD,
        speed_mode=SpeedMode.LOWSPEED,
    )


def test_set_ra_rate_zero_stops_motor(monkeypatch: pytest.MonkeyPatch) -> None:
    mount = _build_mount()
    calls = {"stop": 0, "motion": 0, "speed": 0, "start": 0}

    def _mark_stop() -> None:
        calls["stop"] += 1

    def _mark_motion(*args, **kwargs) -> None:
        calls["motion"] += 1

    def _mark_speed(*args, **kwargs) -> None:
        calls["speed"] += 1

    def _mark_start() -> None:
        calls["start"] += 1

    monkeypatch.setattr(mount, "gracefully_stop_motor", _mark_stop)
    monkeypatch.setattr(mount, "_set_motion", _mark_motion)
    monkeypatch.setattr(mount, "_set_speed", _mark_speed)
    monkeypatch.setattr(mount, "_start_motor", _mark_start)

    assert mount.set_ra_speed(HaPerSecond(0.0)) is True
    assert calls["stop"] == 1
    assert calls["motion"] == 0
    assert calls["speed"] == 0
    assert calls["start"] == 0


def test_set_ra_rate_starts_motor_when_axis_not_running(monkeypatch: pytest.MonkeyPatch) -> None:
    mount = _build_mount()
    speed_periods: list[int] = []
    start_calls = 0

    def _fake_set_motion(
        target_status: SkyWatcherMotionStatus,
        current_status: SkyWatcherStatus | None = None,
    ) -> SkyWatcherStatus:
        assert isinstance(target_status, SkyWatcherMotionStatus)
        return _status(running=False)

    def _fake_set_speed(period: int) -> None:
        speed_periods.append(period)

    def _fake_start_motor() -> None:
        nonlocal start_calls
        start_calls += 1

    monkeypatch.setattr(mount, "_set_motion", _fake_set_motion)
    monkeypatch.setattr(mount, "_set_speed", _fake_set_speed)
    monkeypatch.setattr(mount, "_start_motor", _fake_start_motor)

    assert mount.set_ra_speed(STELLAR_SPEED) is True
    assert len(speed_periods) == 1
    assert speed_periods[0] > 0
    assert start_calls == 1


def test_set_ra_rate_does_not_start_motor_for_goto_mode(monkeypatch: pytest.MonkeyPatch) -> None:
    mount = _build_mount()
    start_calls = 0

    def _fake_set_motion(
        target_status: SkyWatcherMotionStatus,
        current_status: SkyWatcherStatus | None = None,
    ) -> SkyWatcherStatus:
        assert target_status.slew_mode == SlewMode.GOTO
        return _status(running=False)

    def _fake_start_motor() -> None:
        nonlocal start_calls
        start_calls += 1

    monkeypatch.setattr(mount, "_set_motion", _fake_set_motion)
    monkeypatch.setattr(mount, "_set_speed", lambda period: None)
    monkeypatch.setattr(mount, "_start_motor", _fake_start_motor)

    assert mount.set_ra_speed(STELLAR_SPEED, mode=SlewMode.GOTO) is True
    assert start_calls == 0


def test_set_motion_stops_running_axis_before_mode_change(monkeypatch: pytest.MonkeyPatch) -> None:
    mount = _build_mount()
    stop_calls = 0
    motion_command_calls = 0

    current_status = _status(running=True)
    target_status = SkyWatcherMotionStatus(
        slew_mode=SlewMode.GOTO,
        direction=Direction.BACKWARD,
        speed_mode=SpeedMode.LOWSPEED,
    )

    def _fake_wait_till_stop(
        timeout_s: float | None = None,
        do_stop: bool = False,
        func=None,
    ) -> None:
        nonlocal stop_calls
        assert timeout_s is None
        assert do_stop is True
        assert func is None
        stop_calls += 1

    def _fake_transact(*args, **kwargs) -> str:
        nonlocal motion_command_calls
        motion_command_calls += 1
        return ""

    monkeypatch.setattr(mount, "wait_till_stop", _fake_wait_till_stop)
    monkeypatch.setattr(mount, "_transact", _fake_transact)

    result_status = mount._set_motion(target_status, current_status=current_status)

    assert stop_calls == 1
    assert motion_command_calls == 1
    assert result_status.running is False
