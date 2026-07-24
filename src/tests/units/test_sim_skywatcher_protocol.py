import pytest

from sim import Clock, FaultKind, SkyWatcherSim, decode_revu24, encode_revu24

_OFFSET = 0x800000


def _exchange(sim: SkyWatcherSim, command: bytes) -> bytes:
    sim.feed(command)
    return sim.drain()


@pytest.mark.parametrize("value", [0, 1, 0x1234, 0x123456, 0xFFFFFF])
def test_revu24_roundtrip(value: int) -> None:
    assert decode_revu24(encode_revu24(value)) == value


def test_revu24_two_char_response_is_low_byte() -> None:
    assert decode_revu24("10") == 0x10
    assert encode_revu24(0x123456) == "563412"


def test_revu24_rejects_out_of_range_and_bad_hex() -> None:
    with pytest.raises(ValueError):
        encode_revu24(-1)
    with pytest.raises(ValueError):
        encode_revu24(0x1000000)
    with pytest.raises(ValueError):
        decode_revu24("XYZXYZ")


def test_initialize_sets_status_bit() -> None:
    sim = SkyWatcherSim(Clock())

    assert _exchange(sim, b":f1\r") == b"=100\r"
    assert _exchange(sim, b":F1\r") == b"=\r"
    assert _exchange(sim, b":f1\r") == b"=101\r"


def test_board_config_inquiries() -> None:
    sim = SkyWatcherSim(Clock(), cpr=8_000_000, timer_freq=64935, highspeed_ratio=16, mount_code=0x0A)

    assert _exchange(sim, b":e1\r") == b"=03110A\r"
    assert _exchange(sim, b":a1\r") == b"=" + encode_revu24(8_000_000).encode() + b"\r"
    assert _exchange(sim, b":b1\r") == b"=" + encode_revu24(64935).encode() + b"\r"
    assert _exchange(sim, b":g1\r") == b"=10\r"


def test_position_query_and_set_use_offset() -> None:
    sim = SkyWatcherSim(Clock())

    assert _exchange(sim, b":j1\r") == b"=" + encode_revu24(_OFFSET).encode() + b"\r"

    raw = encode_revu24(_OFFSET + 1234).encode()
    assert _exchange(sim, b":E1" + raw + b"\r") == b"=\r"
    assert _exchange(sim, b":j1\r") == b"=" + raw + b"\r"


def test_tracking_run_moves_position_over_virtual_time() -> None:
    clock = Clock()
    sim = SkyWatcherSim(clock, cpr=8_000_000, timer_freq=64935)

    assert _exchange(sim, b":F1\r") == b"=\r"
    assert _exchange(sim, b":G110\r") == b"=\r"
    assert _exchange(sim, b":I1" + encode_revu24(699).encode() + b"\r") == b"=\r"
    assert _exchange(sim, b":J1\r") == b"=\r"
    assert _exchange(sim, b":f1\r") == b"=111\r"

    clock.advance(60)

    expected = round(64935 / 699 * 60)
    assert _exchange(sim, b":j1\r") == b"=" + encode_revu24(_OFFSET + expected).encode() + b"\r"

    assert _exchange(sim, b":K1\r") == b"=\r"
    assert _exchange(sim, b":f1\r") == b"=101\r"


def test_goto_reaches_target_and_returns_to_tracking_mode() -> None:
    clock = Clock()
    sim = SkyWatcherSim(clock, cpr=8_000_000, timer_freq=64935, highspeed_ratio=16)

    assert _exchange(sim, b":F1\r") == b"=\r"
    assert _exchange(sim, b":G100\r") == b"=\r"
    assert _exchange(sim, b":I1" + encode_revu24(13).encode() + b"\r") == b"=\r"
    assert _exchange(sim, b":H1" + encode_revu24(1000).encode() + b"\r") == b"=\r"
    assert _exchange(sim, b":M1" + encode_revu24(200).encode() + b"\r") == b"=\r"
    assert _exchange(sim, b":J1\r") == b"=\r"
    assert _exchange(sim, b":f1\r") == b"=411\r"

    clock.advance(0.1)

    assert _exchange(sim, b":j1\r") == b"=" + encode_revu24(_OFFSET + 1000).encode() + b"\r"
    # Auto-return to tracking mode after stop; the speed-mode bit is retained.
    assert _exchange(sim, b":f1\r") == b"=501\r"


def test_error_codes() -> None:
    sim = SkyWatcherSim(Clock())

    assert _exchange(sim, b":J1\r") == b"!4\r"
    assert _exchange(sim, b":X1\r") == b"!0\r"
    assert _exchange(sim, b":I1ZZZZZZ\r") == b"!3\r"
    assert _exchange(sim, b":G10\r") == b"!1\r"

    assert _exchange(sim, b":F1\r") == b"=\r"
    assert _exchange(sim, b":G110\r") == b"=\r"
    assert _exchange(sim, b":J1\r") == b"=\r"
    assert _exchange(sim, b":E1" + encode_revu24(_OFFSET).encode() + b"\r") == b"!2\r"
    assert _exchange(sim, b":G100\r") == b"!2\r"


def test_second_colon_resets_partial_command() -> None:
    sim = SkyWatcherSim(Clock())

    assert _exchange(sim, b":a1:f1\r") == b"=100\r"


def test_voltage_inquiry_uses_hash_terminator() -> None:
    sim = SkyWatcherSim(Clock(), voltage_v=12.6)

    assert _exchange(sim, b":fL#") == b"7E#"


def test_voltage_unsupported_board_stays_silent() -> None:
    sim = SkyWatcherSim(Clock(), voltage_supported=False)

    assert _exchange(sim, b":fL#") == b""
    assert _exchange(sim, b":f1\r") == b"=100\r"


def test_fault_empty_and_truncated_replies() -> None:
    sim = SkyWatcherSim(Clock())

    sim.faults.push(FaultKind.EMPTY)
    assert _exchange(sim, b":f1\r") == b""
    assert _exchange(sim, b":f1\r") == b"=100\r"

    sim.faults.push(FaultKind.TRUNCATE)
    assert _exchange(sim, b":f1\r") == b"=100"


def test_fault_targets_specific_command() -> None:
    sim = SkyWatcherSim(Clock())

    sim.faults.push(FaultKind.EMPTY, command="fL")
    assert _exchange(sim, b":f1\r") == b"=100\r"
    assert _exchange(sim, b":fL#") == b""
    assert _exchange(sim, b":fL#") == b"7E#"


def test_fault_dead_motor_and_revive() -> None:
    sim = SkyWatcherSim(Clock())

    sim.faults.make_dead()
    assert _exchange(sim, b":F1\r") == b""
    assert _exchange(sim, b":f1\r") == b""

    sim.faults.revive()
    assert _exchange(sim, b":f1\r") == b"=101\r"
