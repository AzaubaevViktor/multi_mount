import pytest

from sim import Clock, FaultKind, SkyWatcherSim, decode_revu24, encode_revu24
from sky.constants import STELLAR_DAY
from skywatcher.codec import SkyWatcherCodec

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
    assert encode_revu24(value) == SkyWatcherCodec.encode_revu24(value)
    assert decode_revu24(SkyWatcherCodec.encode_revu24(value)) == value
    assert SkyWatcherCodec.decode_revu24(encode_revu24(value)) == value


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


def test_default_board_answers_exactly_what_the_live_ra_board_answers() -> None:
    """Protocol §2/§3: byte-for-byte the replies recorded from the controller.

    A simulator whose defaults describe some other board makes every test that
    does not spell out a configuration prove nothing about our hardware — and
    the numbers here are what the mount arithmetic is built on: CPR 12 492 146,
    timer 16 MHz, no high-speed multiplier, mount code 0x0A, firmware 0x0311.
    """
    sim = SkyWatcherSim(Clock())

    assert _exchange(sim, b":e1\r") == b"=03110A\r"
    assert _exchange(sim, b":a1\r") == b"=729DBE\r"
    assert _exchange(sim, b":b1\r") == b"=0024F4\r"
    assert _exchange(sim, b":g1\r") == b"=01\r"
    # `:i1` at power-up is the 1x tracking period the board reports in `:D1`.
    assert _exchange(sim, b":i1\r") == b"=17AF01\r"
    assert _exchange(sim, b":j1\r") == b"=000080\r"
    assert _exchange(sim, b":f1\r") == b"=100\r"


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
    # `K` only starts the brake ramp (§10.3); a second is more than enough for
    # 500 steps/s to run out.
    clock.advance(1)
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

    The Fast bit stays down throughout even though `G` asked for `0` (goto,
    highspeed): period 2 out of a 1x period of 11 is nowhere near 32x sidereal,
    and the board reports the speed it is stepping at, not the request (§10.2).
    """
    clock = Clock()
    sim = SkyWatcherSim(clock, cpr=8_000_000, timer_freq=1000)

    assert _exchange(sim, b":F1\r") == b"=\r"
    assert _exchange(sim, b":G100\r") == b"=\r"
    assert _exchange(sim, b":I1" + encode_revu24(2).encode() + b"\r") == b"=\r"
    assert _exchange(sim, b":H1" + encode_revu24(100000).encode() + b"\r") == b"=\r"
    assert _exchange(sim, b":J1\r") == b"=\r"
    # Goto mode (bit0 clear), running, initialized.
    assert _exchange(sim, b":f1\r") == b"=011\r"

    clock.advance(1)
    assert _exchange(sim, b":K1\r") == b"=\r"

    # Braking mid-flight, yet the mode nibble is back to tracking (bit0 set)
    # while the Running bit is still up: the channel changes mode at once, the
    # motion only winds down.
    assert _exchange(sim, b":f1\r") == b"=111\r"
    clock.advance(1)
    assert _exchange(sim, b":f1\r") == b"=101\r"
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
    # Goto mode, and the Fast bit is up because period 13 out of a 1x period of
    # 699 is past 32x sidereal — the board's own verdict, §10.2.
    assert _exchange(sim, b":f1\r") == b"=411\r"

    clock.advance(0.1)

    assert _exchange(sim, b":j1\r") == b"=" + encode_revu24(_OFFSET + 1000).encode() + b"\r"
    # Auto-return to tracking mode after stop, and the Fast bit goes down with
    # the axis: §10.3 saw `=101`/`=301` after every stop, never `=5xx`.
    assert _exchange(sim, b":f1\r") == b"=101\r"


def test_error_codes() -> None:
    sim = SkyWatcherSim(Clock())

    assert _exchange(sim, b":X1\r") == b"!0\r"
    assert _exchange(sim, b":I1ZZZZZZ\r") == b"!3\r"
    assert _exchange(sim, b":G10\r") == b"!1\r"

    assert _exchange(sim, b":F1\r") == b"=\r"
    assert _exchange(sim, b":G110\r") == b"=\r"
    assert _exchange(sim, b":J1\r") == b"=\r"
    # `!2 Motor not Stopped` is real, but only for `G` (§7): the board rejects a
    # motion-mode change on the move and nothing else.
    assert _exchange(sim, b":G100\r") == b"!2\r"


def test_start_without_initialize_is_accepted_there_is_no_error_4() -> None:
    """§7/§13 #9: `!4 Not Initialized` could not be provoked at all.

    `:J1` on a board reporting `=100` (flag clear) starts the axis and answers
    `=`. The flag is an indicator, not a guard, so a driver must not expect the
    board to stop it from running uninitialized — and must not treat a `!4` it
    will never see as its reboot detector (§12.2).
    """
    clock = Clock()
    sim = SkyWatcherSim(clock, cpr=8_000_000, timer_freq=1000)

    assert _exchange(sim, b":f1\r") == b"=100\r"
    assert _exchange(sim, b":G110\r") == b"=\r"
    assert _exchange(sim, b":I1" + encode_revu24(2).encode() + b"\r") == b"=\r"
    assert _exchange(sim, b":J1\r") == b"=\r"
    assert _exchange(sim, b":f1\r") == b"=110\r"

    clock.advance(10)

    assert _exchange(sim, b":j1\r") == b"=" + encode_revu24(_OFFSET + 5000).encode() + b"\r"


def test_set_position_while_running_is_accepted_and_drops_the_init_flag() -> None:
    """§11, the costliest firmware defect found: `:E` on the move.

    The spec says "motor must be full stopped" and the board does *not* enforce
    it — it answers `=`, applies the position, and then a few hundred
    milliseconds later the initialization flag is gone (5 out of 5 replays).
    The delay is why the same sequence looked flaky on hardware: a driver that
    re-reads `:f1` at once still sees the flag set. The guard has to be on the
    host side, and this is the only place it can be tested.
    """
    clock = Clock()
    sim = SkyWatcherSim(clock, cpr=8_000_000, timer_freq=1000)

    assert _exchange(sim, b":F1\r") == b"=\r"
    assert _exchange(sim, b":G110\r") == b"=\r"
    assert _exchange(sim, b":I1" + encode_revu24(2).encode() + b"\r") == b"=\r"
    assert _exchange(sim, b":J1\r") == b"=\r"

    assert _exchange(sim, b":E1" + encode_revu24(_OFFSET + 4000).encode() + b"\r") == b"=\r"
    # Immediately afterwards the flag is still up, and the position took.
    assert _exchange(sim, b":f1\r") == b"=111\r"
    assert _exchange(sim, b":j1\r") == b"=" + encode_revu24(_OFFSET + 4000).encode() + b"\r"

    clock.advance(0.5)

    assert _exchange(sim, b":f1\r") == b"=110\r"


def test_set_position_on_a_stopped_axis_keeps_the_init_flag() -> None:
    """The counterpart of §11: only `:E` sent *on the move* is the trigger.

    Every setter was tried on a stopped axis and none cleared the flag, so a
    simulator that dropped it after any `:E` would make the driver's "stop
    first, then set position" rule untestable — both orders would look alike.
    """
    clock = Clock()
    sim = SkyWatcherSim(clock)

    assert _exchange(sim, b":F1\r") == b"=\r"
    assert _exchange(sim, b":E1" + encode_revu24(_OFFSET + 4000).encode() + b"\r") == b"=\r"

    clock.advance(5)

    assert _exchange(sim, b":f1\r") == b"=101\r"


def test_set_goto_target_while_running_is_accepted() -> None:
    """§13 #8: `S` too is taken on the move, `=` and no `!2`."""
    clock = Clock()
    sim = SkyWatcherSim(clock, cpr=8_000_000, timer_freq=1000)

    assert _exchange(sim, b":F1\r") == b"=\r"
    assert _exchange(sim, b":G110\r") == b"=\r"
    assert _exchange(sim, b":J1\r") == b"=\r"

    assert _exchange(sim, b":S1" + encode_revu24(_OFFSET + 1000).encode() + b"\r") == b"=\r"
    assert _exchange(sim, b":f1\r") == b"=111\r"


def test_second_colon_resets_partial_command() -> None:
    sim = SkyWatcherSim(Clock())

    assert _exchange(sim, b":a1:f1\r") == b"=100\r"


def test_voltage_inquiry_does_not_exist_on_this_board() -> None:
    """§6: `:fL#` is a command of ours, not of the mount.

    It is missing from the command set PDF, from the EQMod reference and from
    the SynScan binary. On the live board `#` is not a terminator, so `:fL#`
    gets no answer at all — and once a `\\r` arrives, the accumulated `fL` is
    parsed as channel `L`, which is not a hex digit: `!3 Invalid Character`.
    A simulator that answered `<2 hex>#` here taught the driver a protocol no
    board speaks. The voltage is read through the `:C`/`:n` window instead —
    see below.
    """
    sim = SkyWatcherSim(Clock())

    assert _exchange(sim, b":fL#") == b""
    assert _exchange(sim, b":f1#") == b""
    # The pending bytes are dropped by the next `:`, as on hardware (§8).
    assert _exchange(sim, b":f1\r") == b"=100\r"


def test_memory_window_returns_the_two_voltages_byte_by_byte() -> None:
    """§6.8, verbatim from the wire of 2026-07-25 18:53.

    `:C1<lo><hi>` sets the address low byte first — `:C11C00` is 0x001C, not
    0x1C00 — and `:n1` answers with exactly four bytes: `=`, two hex digits,
    `\\r`. Battery 604 = 0x025C -> 6.04 V, USB 470 = 0x01D6 -> 4.70 V.
    """
    sim = SkyWatcherSim(Clock())

    assert _exchange(sim, b":C10400\r") == b"=\r"
    assert _exchange(sim, b":n1\r") == b"=5C\r"
    assert _exchange(sim, b":C10500\r") == b"=\r"
    assert _exchange(sim, b":n1\r") == b"=02\r"

    assert _exchange(sim, b":C11C00\r") == b"=\r"
    assert _exchange(sim, b":n1\r") == b"=D6\r"
    assert _exchange(sim, b":C11D00\r") == b"=\r"
    assert _exchange(sim, b":n1\r") == b"=01\r"


def test_memory_window_does_not_auto_increment() -> None:
    """The vendor re-sends `:C` before every `:n`, and this is why.

    A driver that set the address once and read twice would decode the low byte
    as both halves: 0x5C5C = 23644 -> 236.44 V instead of 6.04 V.
    """
    sim = SkyWatcherSim(Clock())

    assert _exchange(sim, b":C10400\r") == b"=\r"
    assert _exchange(sim, b":n1\r") == b"=5C\r"
    assert _exchange(sim, b":n1\r") == b"=5C\r"
    assert _exchange(sim, b":n1\r") == b"=5C\r"


def test_memory_window_follows_the_configured_voltages() -> None:
    sim = SkyWatcherSim(Clock(), battery_volt_hundredths=1234, usb_volt_hundredths=0)

    assert _exchange(sim, b":C10400\r") == b"=\r"
    assert _exchange(sim, b":n1\r") == b"=D2\r"
    assert _exchange(sim, b":C10500\r") == b"=\r"
    assert _exchange(sim, b":n1\r") == b"=04\r"

    assert _exchange(sim, b":C11C00\r") == b"=\r"
    assert _exchange(sim, b":n1\r") == b"=00\r"


def test_memory_window_reads_zero_where_nothing_lives() -> None:
    """§6.2: the dump of addresses 0x00..0x1F had non-zero bytes only at the
    two voltages. Everything else in the window answers `=00`, and so does a
    `:n1` sent before any `:C1` (address 0)."""
    sim = SkyWatcherSim(Clock())

    assert _exchange(sim, b":n1\r") == b"=00\r"
    assert _exchange(sim, b":C10800\r") == b"=\r"
    assert _exchange(sim, b":n1\r") == b"=00\r"


def test_memory_window_address_errors_follow_the_board() -> None:
    sim = SkyWatcherSim(Clock())

    assert _exchange(sim, b":C1\r") == b"!1\r"
    assert _exchange(sim, b":C104000\r") == b"!1\r"
    assert _exchange(sim, b":C1ZZZZ\r") == b"!3\r"


def test_step_period_is_clamped_at_a_hundred_times_sidereal() -> None:
    """§10.5: whatever is written with `:I`, the board keeps at least 1103.

    Eighteen values were written to the live controller and read back: 1103 and
    above survive, everything below it — down to zero — reads back as 1103.
    The limit is `:D1 // 100`, i.e. the axis is not allowed past 100x sidereal;
    dividing by zero cannot be arranged either. Without the `:i1` read-back the
    clamp is invisible, which is why the query is here too.
    """
    sim = SkyWatcherSim(Clock())

    for written in (110_359, 20_000, 2_000, 1_536, 1_103):
        assert _exchange(sim, b":I1" + encode_revu24(written).encode() + b"\r") == b"=\r"
        assert _exchange(sim, b":i1\r") == b"=" + encode_revu24(written).encode() + b"\r"

    for written in (1_000, 100, 1, 0):
        assert _exchange(sim, b":I1" + encode_revu24(written).encode() + b"\r") == b"=\r"
        assert _exchange(sim, b":i1\r") == b"=" + encode_revu24(1103).encode() + b"\r"


def test_clamped_period_caps_the_axis_at_a_hundred_times_sidereal() -> None:
    """The clamp is not cosmetic: the axis really refuses to go faster.

    A period of 1 would mean 16 000 000 steps/s. The board runs 14 506 instead
    — `timer_freq / 1103` — which is exactly 100x the sidereal rate.
    """
    clock = Clock()
    sim = SkyWatcherSim(clock)

    assert _exchange(sim, b":F1\r") == b"=\r"
    assert _exchange(sim, b":G110\r") == b"=\r"
    assert _exchange(sim, b":I1" + encode_revu24(1).encode() + b"\r") == b"=\r"
    assert _exchange(sim, b":J1\r") == b"=\r"

    clock.advance(1)

    answer = _exchange(sim, b":j1\r")
    steps = decode_revu24(answer[1:-1].decode("ascii")) - _OFFSET
    assert steps == pytest.approx(16_000_000 / 1103, abs=1)
    assert steps == pytest.approx(100 * 12_492_146 / float(STELLAR_DAY), rel=0.001)


def test_stop_brakes_along_a_ramp_instead_of_freezing_the_axis() -> None:
    """§10.3: `K` answers `=` long before the axis is done moving.

    Measured on hardware at the board's top speed: 0.6s after `:K1` the status
    still reads Running, and the axis has overshot ~1148 counts. At 64x
    sidereal the same 0.6s is enough for the ramp to finish. Both are checked
    here, because a driver that trusts the `=` from `K` (as `stop()` does)
    reports a mount as parked while it is still turning.
    """
    clock = Clock()
    sim = SkyWatcherSim(clock)

    assert _exchange(sim, b":F1\r") == b"=\r"
    assert _exchange(sim, b":G110\r") == b"=\r"
    # 128x sidereal was requested on hardware; the board clamps it to 1103.
    assert _exchange(sim, b":I1" + encode_revu24(862).encode() + b"\r") == b"=\r"
    assert _exchange(sim, b":J1\r") == b"=\r"
    clock.advance(1)

    assert _exchange(sim, b":K1\r") == b"=\r"
    at_stop_command = decode_revu24(_exchange(sim, b":j1\r")[1:-1].decode("ascii"))

    clock.advance(0.6)
    status = _exchange(sim, b":f1\r")
    assert status[2:3] == b"1", f"the axis must still be Running 0.6s after K, got {status!r}"

    clock.advance(1)
    assert _exchange(sim, b":f1\r") == b"=101\r"
    overshoot = decode_revu24(_exchange(sim, b":j1\r")[1:-1].decode("ascii")) - at_stop_command
    assert overshoot == pytest.approx(1148, rel=0.05)


def test_stop_ramp_is_short_enough_at_sixty_four_times_sidereal() -> None:
    """The other half of §10.3: at 64x the axis is stopped within 0.6s.

    Without this the "brake ramp" could be a fixed delay long enough to hide a
    driver that never polls `:f1` after `K`.
    """
    clock = Clock()
    sim = SkyWatcherSim(clock)

    assert _exchange(sim, b":F1\r") == b"=\r"
    assert _exchange(sim, b":G110\r") == b"=\r"
    assert _exchange(sim, b":I1" + encode_revu24(1724).encode() + b"\r") == b"=\r"
    assert _exchange(sim, b":J1\r") == b"=\r"
    clock.advance(1)

    assert _exchange(sim, b":K1\r") == b"=\r"
    clock.advance(0.6)

    assert _exchange(sim, b":f1\r") == b"=101\r"


def test_fault_empty_and_truncated_replies() -> None:
    sim = SkyWatcherSim(Clock())

    sim.faults.push(FaultKind.EMPTY)
    assert _exchange(sim, b":f1\r") == b""
    assert _exchange(sim, b":f1\r") == b"=100\r"

    sim.faults.push(FaultKind.TRUNCATE)
    assert _exchange(sim, b":f1\r") == b"=100"


def test_fault_targets_specific_command() -> None:
    sim = SkyWatcherSim(Clock())

    sim.faults.push(FaultKind.EMPTY, command="j")
    assert _exchange(sim, b":f1\r") == b"=100\r"
    assert _exchange(sim, b":j1\r") == b""
    assert _exchange(sim, b":j1\r") == b"=000080\r"


def test_fault_dead_motor_and_revive() -> None:
    sim = SkyWatcherSim(Clock())

    sim.faults.make_dead()
    assert _exchange(sim, b":F1\r") == b""
    assert _exchange(sim, b":f1\r") == b""

    sim.faults.revive()
    assert _exchange(sim, b":f1\r") == b"=101\r"


# --------------------------------------------------------------------------- #
# behaviours taken from the live board on 2026-07-25 that the simulator used to
# get wrong (RA_PROTOCOL_STEP_2.md §3)
# --------------------------------------------------------------------------- #


def test_fast_bit_is_the_boards_verdict_on_the_period_not_an_echo_of_g() -> None:
    """§10.2, the most surprising status finding on the live board.

    Three measurements, all on the real RA constants:

    * `:G110` (slow requested) with period 1724 -> `=101` at rest, `=511` moving;
    * `:G110` with period 6897 -> `=111` both at rest and moving;
    * `:G130` (fast requested) with period 1724 -> `=101` at rest.

    So the bit is not what `G` asked for: it reports the step mode the board
    picked from the period, and it is only up while the axis actually runs.
    """
    clock = Clock()
    sim = SkyWatcherSim(clock)

    assert _exchange(sim, b":F1\r") == b"=\r"
    assert _exchange(sim, b":I1" + encode_revu24(1724).encode() + b"\r") == b"=\r"
    # Fast requested, axis stopped: the bit stays down.
    assert _exchange(sim, b":G130\r") == b"=\r"
    assert _exchange(sim, b":f1\r") == b"=101\r"
    # Slow requested, same period: as soon as it runs, the board says Fast.
    assert _exchange(sim, b":G110\r") == b"=\r"
    assert _exchange(sim, b":f1\r") == b"=101\r"
    assert _exchange(sim, b":J1\r") == b"=\r"
    assert _exchange(sim, b":f1\r") == b"=511\r"

    assert _exchange(sim, b":K1\r") == b"=\r"
    clock.advance(2)
    assert _exchange(sim, b":f1\r") == b"=101\r"

    # 16x sidereal: slow at rest and slow on the move.
    assert _exchange(sim, b":I1" + encode_revu24(6897).encode() + b"\r") == b"=\r"
    assert _exchange(sim, b":f1\r") == b"=101\r"
    assert _exchange(sim, b":J1\r") == b"=\r"
    assert _exchange(sim, b":f1\r") == b"=111\r"


def test_every_hex_channel_answers_and_only_channel_one_is_an_axis() -> None:
    """§3.2: mistyping the channel is silent, which is why the driver pins it.

    Verbatim from the live board: `:f2`, `:f3`, `:f0`, `:f4`, `:f9` all answer
    `=000`, `:j2` answers `=000080`, `:e2` and `:a2` answer exactly what channel
    1 answers, and only a *non-hex* channel is an error (`:f?`, `:fL` -> `!3`).
    Channel `3` ("Both" in the spec) gets no special treatment either.
    """
    clock = Clock()
    sim = SkyWatcherSim(clock)

    assert _exchange(sim, b":F1\r") == b"=\r"
    for channel in (b"0", b"2", b"3", b"4", b"9", b"A", b"F"):
        assert _exchange(sim, b":f" + channel + b"\r") == b"=000\r"
    assert _exchange(sim, b":e2\r") == b"=03110A\r"
    assert _exchange(sim, b":a2\r") == b"=729DBE\r"
    assert _exchange(sim, b":j2\r") == b"=000080\r"

    assert _exchange(sim, b":f?\r") == b"!3\r"
    assert _exchange(sim, b":fL\r") == b"!3\r"

    # And the phantom axis is inert: starting it moves nothing.
    assert _exchange(sim, b":J2\r") == b"=\r"
    clock.advance(60)
    assert _exchange(sim, b":f1\r") == b"=101\r"
    assert _exchange(sim, b":j1\r") == b"=000080\r"


def test_error_codes_on_malformed_input_are_the_ones_the_board_returns() -> None:
    """§7 and §8.2, every line of the table that needs no write command.

    The parser recognizes the letter first and checks the argument second: a
    packet too short to hold a channel is "unknown command", a known letter with
    a wrong argument length is "command length", and a bad character — lower
    case hex included — is "invalid character".
    """
    sim = SkyWatcherSim(Clock())

    for command, answer in (
        (b":\r", b"!0\r"),                     # empty command
        (b":f\r", b"!0\r"),                    # no channel byte
        (b":X1\r", b"!0\r"),                   # unknown letter
        (b":k10\r", b"!0\r"),                  # documented but not implemented
        (b":q1FF0000\r", b"!0\r"),             # unsupported extended id
        (b":I1\r", b"!1\r"),                   # setter without an argument
        (b":I112\r", b"!1\r"),                 # 2 hex chars instead of 6
        (b":I100000000\r", b"!1\r"),           # 8 hex chars instead of 6
        (b":I1" + b"0" * 40 + b"\r", b"!1\r"),  # oversized argument
        (b":" + b"A" * 60 + b"\r", b"!1\r"),   # 60 chars of junk after a real letter
        (b":G1\r", b"!1\r"),
        (b":G1123456\r", b"!1\r"),
        (b":j1000000\r", b"!1\r"),             # argument on a query that takes none
        (b":f11\r", b"!1\r"),                  # one char too many
        (b":I1ZZZZZZ\r", b"!3\r"),
        (b":I1abcdef\r", b"!3\r"),             # lower case hex is refused
        (b":I1-00000\r", b"!3\r"),
        (b":I1 00000\r", b"!3\r"),
    ):
        assert _exchange(sim, command) == answer, command

    # None of that moved the step period off its power-on value.
    assert _exchange(sim, b":i1\r") == b"=17AF01\r"


def test_second_command_of_a_single_write_is_lost() -> None:
    """§8.1, reproduced four times on hardware.

    Two complete commands in one `write()` produce one answer: the board is not
    buffering while it replies, so everything that arrives in that window is
    dropped. The same two commands sent as two writes answer twice. Any attempt
    to pipeline requests on this board fails silently, which is why this is
    modelled rather than left to the driver's good manners.
    """
    sim = SkyWatcherSim(Clock())

    assert _exchange(sim, b":f1\r:j1\r") == b"=100\r"
    assert _exchange(sim, b":f1\r") == b"=100\r"
    assert _exchange(sim, b":j1\r") == b"=000080\r"

    # A command left half-written behind the first one is lost too, so the next
    # write starts from a clean parser.
    assert _exchange(sim, b":f1\r:j") == b"=100\r"
    assert _exchange(sim, b"1\r") == b""
    assert _exchange(sim, b":j1\r") == b"=000080\r"


def test_goto_runs_to_an_absolute_target_set_with_s() -> None:
    """`:S` writes the same target register `:H` does, only absolutely.

    The simulator used to record the target and never move to it, so a driver
    that used the absolute form saw an axis that ran forever. `:h1` reads the
    register back, and whichever of `:S`/`:H` came last is the one that counts.
    """
    clock = Clock()
    sim = SkyWatcherSim(clock, cpr=8_000_000, timer_freq=1000)

    assert _exchange(sim, b":h1\r") == b"=000080\r"
    assert _exchange(sim, b":F1\r") == b"=\r"
    assert _exchange(sim, b":G120\r") == b"=\r"
    assert _exchange(sim, b":I1" + encode_revu24(2).encode() + b"\r") == b"=\r"
    assert _exchange(sim, b":S1" + encode_revu24(_OFFSET + 1500).encode() + b"\r") == b"=\r"
    assert _exchange(sim, b":h1\r") == b"=" + encode_revu24(_OFFSET + 1500).encode() + b"\r"
    assert _exchange(sim, b":J1\r") == b"=\r"

    # 500 steps/s, so 1500 steps take 3 s; ten more seconds change nothing.
    clock.advance(3)
    assert _exchange(sim, b":j1\r") == b"=" + encode_revu24(_OFFSET + 1500).encode() + b"\r"
    assert _exchange(sim, b":f1\r") == b"=101\r"
    clock.advance(10)
    assert _exchange(sim, b":j1\r") == b"=" + encode_revu24(_OFFSET + 1500).encode() + b"\r"

    # `:H` after `:S` takes the register over: relative to where the axis is now.
    assert _exchange(sim, b":G120\r") == b"=\r"
    assert _exchange(sim, b":H1" + encode_revu24(500).encode() + b"\r") == b"=\r"
    assert _exchange(sim, b":h1\r") == b"=" + encode_revu24(_OFFSET + 2000).encode() + b"\r"
    assert _exchange(sim, b":J1\r") == b"=\r"
    clock.advance(1)
    assert _exchange(sim, b":j1\r") == b"=" + encode_revu24(_OFFSET + 2000).encode() + b"\r"


def test_inquiries_the_board_answers_but_the_simulator_used_to_reject() -> None:
    """§2 and §3: the whole query table of the live RA board, verbatim.

    Every one of these answered `!0` in the simulator while the board answers
    with data, which made the simulator useless for testing anything that reads
    the board's own idea of brake steps, goto target or extended status.
    """
    sim = SkyWatcherSim(Clock())

    assert _exchange(sim, b":c1\r") == b"=544200\r"      # brake steps 16980
    assert _exchange(sim, b":d1\r") == b"=000080\r"      # tele. axis position
    assert _exchange(sim, b":h1\r") == b"=000080\r"      # goto target
    assert _exchange(sim, b":s1\r") == b"=000000\r"      # PEC period, no PEC
    assert _exchange(sim, b":D1\r") == b"=17AF01\r"      # 1x tracking period
    assert _exchange(sim, b":r1\r") == b"=00\r"          # register file is a stub
    assert _exchange(sim, b":z1\r") == b"=\r"            # set debug flag

    # §3.1: the brake point is a derived value — position offset by exactly the
    # brake steps `:c1` reports, on the side the axis is facing.
    assert _exchange(sim, b":m1\r") == b"=" + encode_revu24(_OFFSET - 16980).encode() + b"\r"
    assert _exchange(sim, b":G111\r") == b"=\r"
    assert _exchange(sim, b":m1\r") == b"=" + encode_revu24(_OFFSET + 16980).encode() + b"\r"

    # §5: only extended ids 1..5 exist, and id 3 is the USB rail of the `:C`/`:n`
    # window with a constant high byte — an identity checked byte by byte on the
    # wire, so the two must not be able to drift apart in the simulator.
    assert _exchange(sim, b":q1010000\r") == b"=008000\r"
    assert _exchange(sim, b":q1020000\r") == b"=100000\r"
    assert _exchange(sim, b":q1030000\r") == b"=D60102\r"
    assert _exchange(sim, b":q1040000\r") == b"=A3C9CA\r"
    assert _exchange(sim, b":q1050000\r") == b"=49DB48\r"
    assert _exchange(sim, b":q1000000\r") == b"!0\r"


def test_extended_inquire_three_follows_the_usb_rail() -> None:
    """§6.3/§6.8: `:q1030000` and window bytes 0x1C/0x1D are the same number."""
    sim = SkyWatcherSim(Clock(), usb_volt_hundredths=0x01A6)

    assert _exchange(sim, b":q1030000\r") == b"=A60102\r"
    assert _exchange(sim, b":C11C00\r") == b"=\r"
    assert _exchange(sim, b":n1\r") == b"=A6\r"
    assert _exchange(sim, b":C11D00\r") == b"=\r"
    assert _exchange(sim, b":n1\r") == b"=01\r"
