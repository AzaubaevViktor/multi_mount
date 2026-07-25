import pytest

from sim import Clock, FaultKind, SkyWatcherSim, decode_revu24, encode_revu24
from sky.constants import STELLAR_DAY
from skywatcher.motor import _Revu24

_OFFSET = 0x800000


def _exchange(sim: SkyWatcherSim, command: bytes) -> bytes:
    sim.feed(command)
    return sim.drain()


def test_revu24_matches_the_reference_data_format() -> None:
    """Vectors from the command set reference, §4.5 "Data format".

    24-bit `0x123456` travels as `"5" "6" "3" "4" "1" "2"`, 16-bit `0x1234` as
    `"3" "4" "1" "2"` (the low two thirds of the same field) and 8-bit `0x12`
    as `"1" "2"`.
    """
    assert encode_revu24(0x123456) == "563412"
    assert decode_revu24("563412") == 0x123456
    assert encode_revu24(0x001234)[:4] == "3412"
    assert decode_revu24("12") == 0x12


@pytest.mark.parametrize("value", [0, 1, 0x1234, 0x123456, 0xFFFFFF])
def test_revu24_agrees_with_the_production_driver_codec(value: int) -> None:
    """Cross-check against the codec the real driver uses on the wire.

    Encoding and decoding are each other's inverse in any self-consistent
    implementation, so the sim is compared to an independent one instead:
    what the sim emits must be what `SkyWatcherMotor` decodes, and vice versa.
    """
    assert encode_revu24(value) == _Revu24.from_int(value)
    assert decode_revu24(_Revu24.from_int(value)) == value
    assert _Revu24.from_mount(encode_revu24(value)) == value


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
    """`I` presets the T1 timer, so the axis steps `timer_freq / period` per second.

    The board is given a deliberately round configuration — 1000 timer ticks
    per second, period 2 — so the expected travel is arithmetic done by hand
    (500 steps/s for 60 s = 30000 steps), not a copy of the simulator's own
    expression.
    """
    clock = Clock()
    sim = SkyWatcherSim(clock, cpr=8_000_000, timer_freq=1000)

    assert _exchange(sim, b":F1\r") == b"=\r"
    assert _exchange(sim, b":G110\r") == b"=\r"
    assert _exchange(sim, b":I1" + encode_revu24(2).encode() + b"\r") == b"=\r"
    assert _exchange(sim, b":J1\r") == b"=\r"
    assert _exchange(sim, b":f1\r") == b"=111\r"

    clock.advance(60)

    assert _exchange(sim, b":j1\r") == b"=" + encode_revu24(_OFFSET + 30000).encode() + b"\r"

    assert _exchange(sim, b":K1\r") == b"=\r"
    assert _exchange(sim, b":f1\r") == b"=101\r"


def test_default_step_period_is_the_sidereal_tracking_preset() -> None:
    """A board that was never told a period tracks at the sidereal rate.

    The preset the mount powers up with must make the axis complete exactly one
    revolution (CPR steps) per stellar day, so an hour of tracking is
    `cpr / STELLAR_DAY * 3600` steps. Only `J` is sent — no `I` — and the
    tolerance is the integer rounding of the period (0.04% here), well inside
    the 0.273% by which a solar day would differ.
    """
    clock = Clock()
    sim = SkyWatcherSim(clock, cpr=8_000_000, timer_freq=64935)

    assert _exchange(sim, b":F1\r") == b"=\r"
    assert _exchange(sim, b":G110\r") == b"=\r"
    assert _exchange(sim, b":J1\r") == b"=\r"

    clock.advance(3600)

    answer = _exchange(sim, b":j1\r")
    steps = decode_revu24(answer[1:-1].decode("ascii")) - _OFFSET
    assert steps == pytest.approx(3600 * 8_000_000 / float(STELLAR_DAY), rel=0.001)


def test_stop_returns_the_channel_to_tracking_mode() -> None:
    """Reference §5.1 note *4: after `K` the channel is always in Tracking Mode.

    A GOTO interrupted by `K` is the case that distinguishes it: the mode bit
    has to flip back to tracking even though the run never reached its target.
    """
    clock = Clock()
    sim = SkyWatcherSim(clock, cpr=8_000_000, timer_freq=1000)

    assert _exchange(sim, b":F1\r") == b"=\r"
    assert _exchange(sim, b":G100\r") == b"=\r"
    assert _exchange(sim, b":I1" + encode_revu24(2).encode() + b"\r") == b"=\r"
    assert _exchange(sim, b":H1" + encode_revu24(100000).encode() + b"\r") == b"=\r"
    assert _exchange(sim, b":J1\r") == b"=\r"
    # Goto mode (bit0 clear), highspeed, running, initialized.
    assert _exchange(sim, b":f1\r") == b"=411\r"

    clock.advance(1)
    assert _exchange(sim, b":K1\r") == b"=\r"

    # Stopped mid-flight, yet the mode nibble is back to tracking (bit0 set).
    assert _exchange(sim, b":f1\r") == b"=501\r"
    position_after_stop = _exchange(sim, b":j1\r")
    clock.advance(10)
    assert _exchange(sim, b":j1\r") == position_after_stop


def test_position_answer_wraps_modulo_cpr() -> None:
    """`j` reports the axis angle, so it wraps once the axis passes CPR steps.

    The driver reads `(revu24 - offset) % steps_360` back, so a board that let
    the raw counter run past CPR would desync the two by a whole revolution.
    """
    clock = Clock()
    sim = SkyWatcherSim(clock, cpr=8_000_000, timer_freq=1000)

    assert _exchange(sim, b":F1\r") == b"=\r"
    assert _exchange(sim, b":E1" + encode_revu24(_OFFSET + 7_999_000).encode() + b"\r") == b"=\r"
    assert _exchange(sim, b":G110\r") == b"=\r"
    assert _exchange(sim, b":I1" + encode_revu24(2).encode() + b"\r") == b"=\r"
    assert _exchange(sim, b":J1\r") == b"=\r"

    # 500 steps/s for 6 s = 3000 steps: 7_999_000 + 3000 is 2000 past a full turn.
    clock.advance(6)

    assert _exchange(sim, b":j1\r") == b"=" + encode_revu24(_OFFSET + 2000).encode() + b"\r"


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
