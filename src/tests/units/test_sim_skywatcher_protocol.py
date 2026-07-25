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

    # Braking mid-flight, yet the mode nibble is back to tracking (bit0 set)
    # while the Running bit is still up: the channel changes mode at once, the
    # motion only winds down.
    assert _exchange(sim, b":f1\r") == b"=511\r"
    clock.advance(1)
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
