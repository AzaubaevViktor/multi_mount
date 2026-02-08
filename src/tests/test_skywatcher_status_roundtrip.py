import pytest

from skywatcher.skywatcher import (
    Direction,
    SpeedMode,
    SlewMode,
    SkyWatcherStatus,
)

STATUS_FLAG_MASK = 0x010701
RAW_BASE = 0xA5B6C7
RAW_NON_FLAG_BITS = RAW_BASE & ~STATUS_FLAG_MASK
RAW_WITH_FLAGS = RAW_NON_FLAG_BITS | STATUS_FLAG_MASK
NO_STATUS_FLAGS = 0

NO_FLAGS_BYTES = bytes((0x00, 0x00, 0x00))
ROUNDTRIP_PAYLOADS = (
    NO_FLAGS_BYTES,
    bytes((0x07, 0x01, 0x01)),
    bytes((0xA5, 0x5A, 0xC3)),
    bytes((0xF8, 0xFE, 0x7E)),
)
# TODO: Rewrite to_bytes -> to_command

@pytest.mark.parametrize("payload", ROUNDTRIP_PAYLOADS)
def test_status_roundtrip_preserves_bytes(payload: bytes) -> None:
    status = SkyWatcherStatus.from_bytes(payload)
    assert status.to_bytes() == payload


def test_status_to_bytes_sets_flags_and_preserves_non_flag_bits() -> None:
    status = SkyWatcherStatus(
        raw=RAW_NON_FLAG_BITS,
        running=True,
        initialized=True,
        slew_mode=SlewMode.SLEW,
        direction=Direction.BACKWARD,
        speed_mode=SpeedMode.HIGHSPEED,
    )

    status.to_bytes()

    assert status.raw & STATUS_FLAG_MASK == STATUS_FLAG_MASK
    assert status.raw & ~STATUS_FLAG_MASK == RAW_NON_FLAG_BITS


def test_status_to_bytes_clears_flags_and_preserves_non_flag_bits() -> None:
    status = SkyWatcherStatus(
        raw=RAW_WITH_FLAGS,
        running=False,
        initialized=False,
        slew_mode=SlewMode.GOTO,
        direction=Direction.FORWARD,
        speed_mode=SpeedMode.LOWSPEED,
    )

    status.to_bytes()

    assert status.raw & STATUS_FLAG_MASK == NO_STATUS_FLAGS
    assert status.raw & ~STATUS_FLAG_MASK == RAW_NON_FLAG_BITS
