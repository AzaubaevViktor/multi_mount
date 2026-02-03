import time

import pytest

from lx200.protocols import LX200Hours
from serial_wrapper.wrapper import SerialLine
from skywatcher.skywatcher import SkyWatcherMount


@pytest.fixture
def mount() -> SkyWatcherMount:
    serial_line = SerialLine("/dev/tty.PL2303G-USBtoUART2120", 112500, 0.2, "skywatcher")
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
    expected = LX200Hours.from_string(hours_value)
    mount.set_telescope_ra(expected)
    time.sleep(0.2)
    actual = mount.get_telescope_ra()
    assert actual.to_seconds() == expected.to_seconds(), (actual, expected)


def _distance_seconds(a: int, b: int) -> int:
    delta = abs(a - b)
    return min(delta, 24 * 3600 - delta)


def test_slew_to_ra_moves_mount(mount: SkyWatcherMount):
    slew_delta_seconds = 120
    timeout_s = 15
    target_tolerance_seconds = 5

    mount.set_telescope_ra(LX200Hours.from_seconds(0))

    current = mount.get_telescope_ra()
    target_seconds = (current.to_seconds() + slew_delta_seconds) % (24 * 3600)
    target = LX200Hours.from_seconds(target_seconds)

    assert mount.slew_to_ra(target) is True

    start = time.monotonic()
    while True:
        if not mount.get_status().running:
            break
        if time.monotonic() - start > timeout_s:
            raise AssertionError(f"Slew did not finish within {timeout_s}s")
        print(mount.get_telescope_ra().to_seconds())
        time.sleep(0.2)

    actual = mount.get_telescope_ra()
    distance = _distance_seconds(actual.to_seconds(), target.to_seconds())
    assert distance <= target_tolerance_seconds, (
        f"Mount did not reach target: distance={distance}s target={target} actual={actual}"
    )
