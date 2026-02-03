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
