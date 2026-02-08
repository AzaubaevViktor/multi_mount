import pytest

from skywatcher.skywatcher import (
    Direction,
    SpeedMode,
    SlewMode,
    SkyWatcherStatus,
)

NO_STATUS_FLAGS = 0

NO_FLAGS_BYTES = bytes((0x00, 0x00, 0x00))
STATUS_COMMANDS = (
    (NO_FLAGS_BYTES, "20"),
    (bytes((0x07, 0x01, 0x01)), "31"),
    (bytes((0xA5, 0x5A, 0xC3)), "30"),
    (bytes((0xF8, 0xFE, 0x7E)), "20"),
)

@pytest.mark.parametrize(("payload", "expected_command"), STATUS_COMMANDS)
def test_status_from_bytes_to_command(payload: bytes, expected_command: str) -> None:
    status = SkyWatcherStatus.from_bytes(payload)
    assert status.to_command() == expected_command


def test_status_to_command_encodes_slew_highspeed_backward() -> None:
    status = SkyWatcherStatus(
        raw=NO_STATUS_FLAGS,
        running=True,
        initialized=True,
        slew_mode=SlewMode.SLEW,
        direction=Direction.BACKWARD,
        speed_mode=SpeedMode.HIGHSPEED,
    )

    assert status.to_command() == "31"


def test_status_to_command_encodes_goto_lowspeed_forward() -> None:
    status = SkyWatcherStatus(
        raw=NO_STATUS_FLAGS,
        running=False,
        initialized=False,
        slew_mode=SlewMode.GOTO,
        direction=Direction.FORWARD,
        speed_mode=SpeedMode.LOWSPEED,
    )

    assert status.to_command() == "20"
