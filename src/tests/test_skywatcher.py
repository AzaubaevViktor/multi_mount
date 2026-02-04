import logging
import time

import pytest

from lx200.protocols import LX200Ha
from serial_wrapper.wrapper import SerialLine
from skywatcher.skywatcher import SkyWatcherMount, SlewMode


@pytest.fixture
def mount() -> SkyWatcherMount:
    serial_line = SerialLine("/dev/tty.PL2303G-USBtoUART2110", 112500, 0.2, "skywatcher")
    mount = SkyWatcherMount(serial_line)
    mount.connect()

    try:
        yield mount
    finally:
        serial_line.close()


@pytest.mark.parametrize(
    "hours_value",
    [
        "00:00:00",
        "00:00:01",
        "00:10:00",
        "01:23:45",
        "03:30:15",
        "06:00:00",
        "09:15:30",
        "12:00:00",
        "15:45:30",
        "18:20:05",
        "21:10:50",
        "23:59:59",
    ],
)
def test_skywatcher_hours_roundtrip(mount: SkyWatcherMount, hours_value: str):
    expected = LX200Ha.from_string(hours_value)
    mount.set_telescope_ra(expected)
    time.sleep(0.2)
    actual = mount.get_telescope_ra()
    assert actual.to_seconds() == expected.to_seconds(), (actual, expected)


def _distance_seconds(a: int, b: int) -> int:
    delta = abs(a - b)
    return min(delta, 24 * 3600 - delta)


def _ensure_idle(mount: SkyWatcherMount, timeout_s: float) -> None:
    mount.wait_till_stop(timeout_s=timeout_s, do_stop=True)


def _measure_ra_shift(
    mount: SkyWatcherMount,
    start_motion,
    duration_s: float,
    timeout_s: float,
) -> tuple[int, int, float]:
    _ensure_idle(mount, timeout_s)
    mid_ra_seconds = 12 * 3600
    mount.set_telescope_ra(LX200Ha.from_seconds(mid_ra_seconds))
    time.sleep(0.2)

    start_seconds = mount.get_telescope_ra().to_seconds()
    assert start_motion() is True

    time.sleep(1)  # Wait while motor get to full speed

    start_time = time.monotonic()
    time.sleep(duration_s)
    end_seconds = mount.get_telescope_ra().to_seconds()
    elapsed = time.monotonic() - start_time

    mount.gracefully_stop_motor()
    mount.wait_till_stop(timeout_s=timeout_s)

    return start_seconds, end_seconds, elapsed


def _assert_speed_and_direction(
    start_seconds: int,
    end_seconds: int,
    elapsed: float,
    expected_rate: float,
    rate_sign: float,
) -> None:
    delta = end_seconds - start_seconds
    if rate_sign > 0:
        assert end_seconds > start_seconds, (
            f"Expected forward motion: start={start_seconds} end={end_seconds}"
        )
    else:
        assert end_seconds < start_seconds, (
            f"Expected backward motion: start={start_seconds} end={end_seconds}"
        )

    actual_speed = abs(delta) / elapsed
    assert actual_speed == pytest.approx(expected_rate, rel=0.35, abs=0.25), (
        f"Speed mismatch: actual={actual_speed:.3f} expected={expected_rate:.3f} "
        f"delta={delta} elapsed={elapsed:.2f}s"
    )


def test_slew_to_ra_moves_mount(mount: SkyWatcherMount):
    slew_delta_seconds = 240
    timeout_s = 15
    target_tolerance_seconds = 5

    mount.set_telescope_ra(LX200Ha.from_seconds(0))

    current = mount.get_telescope_ra()
    target_seconds = (current.to_seconds() + slew_delta_seconds) % (24 * 3600)
    target = LX200Ha.from_seconds(target_seconds)
    delta = LX200Ha.from_seconds(slew_delta_seconds)

    assert mount.slew_to_ra(delta) is True

    def get_position(mount: SkyWatcherMount):
        logging.info("pos: %s", mount.get_telescope_ra())

    mount.wait_till_stop(timeout_s, func=get_position)

    actual = mount.get_telescope_ra()
    distance = _distance_seconds(actual.to_seconds(), target.to_seconds())
    assert distance <= target_tolerance_seconds, (
        f"Mount did not reach target: distance={distance}s target={target} actual={actual}, status={mount.get_status()}"
    )


def test_move_ra_rejects_goto_in_progress(mount: SkyWatcherMount) -> None:
    delta = LX200Ha.from_seconds(1800)

    assert mount.slew_to_ra(delta) is True

    wait_for_goto_s = 2.0
    start_time = time.monotonic()
    while time.monotonic() - start_time < wait_for_goto_s:
        status = mount.get_status()
        if status.running and status.slew_mode == SlewMode.GOTO:
            break
        time.sleep(0.1)
    else:
        pytest.fail("Mount did not enter GOTO mode in time")

    assert mount.move_ra(1) is False
    mount.wait_till_stop(timeout_s=5, do_stop=True)


@pytest.mark.parametrize("rate", [100.0, 1.0, -1.0, -100.0])
def test_move_ra_speed_and_direction(mount: SkyWatcherMount, rate: float) -> None:
    duration_s = 4.0
    timeout_s = 5.0

    start_seconds, end_seconds, elapsed = _measure_ra_shift(
        mount,
        lambda: mount.move_ra(rate),
        duration_s,
        timeout_s,
    )

    _assert_speed_and_direction(start_seconds, end_seconds, elapsed, abs(rate), rate)


@pytest.mark.parametrize("trackspeed", [10.0, 1.0, -1.0, -10.0])
def test_start_tracking_speed_and_direction(mount: SkyWatcherMount, trackspeed: float) -> None:
    duration_s = 4.0
    timeout_s = 5.0

    speed_value = trackspeed * mount.STELLAR_SPEED
    expected_rate = abs(speed_value) / mount.STELLAR_SPEED

    start_seconds, end_seconds, elapsed = _measure_ra_shift(
        mount,
        lambda: mount.start_tracking(speed_value),
        duration_s,
        timeout_s,
    )

    _assert_speed_and_direction(start_seconds, end_seconds, elapsed, expected_rate, trackspeed)


def test_start_tracking_zero_stops_motor(mount: SkyWatcherMount) -> None:
    timeout_s = 5.0
    speed_value = mount.STELLAR_SPEED
    stop_speed = 0

    assert mount.start_tracking(speed_value) is True
    time.sleep(0.5)
    assert mount.get_status().running is True

    assert mount.start_tracking(stop_speed) is True
    mount.wait_till_stop(timeout_s=timeout_s)
    assert mount.get_status().running is False
