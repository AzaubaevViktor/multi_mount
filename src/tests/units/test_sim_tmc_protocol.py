from sim import Clock, FakeSerial, FaultKind, TMC2209Sim
from sim.tmc_sim import RX_FIFO_CAPACITY, TX_RING_CAPACITY
from tmc2209.protocol import Frame, Op, decode_response, encode_frame, response_values


def _exchange(sim: TMC2209Sim, line: str) -> str:
    sim.feed(line.encode("ascii"))
    return sim.drain().decode("ascii")


def _make_sim(clock: Clock | None = None) -> TMC2209Sim:
    sim = TMC2209Sim(clock or Clock())
    assert sim.drain() == b"ready\r\n"
    return sim


def test_power_on_emits_ready_line() -> None:
    """The banner is `Serial.println(F("ready"))` (main.cpp:452).

    Two independent sources pin it: Arduino's `println` terminates with CR LF
    (unlike `outFlushLineV2`, which appends a bare LF to command replies), and
    `TMC2209Motor.connect` accepts the line only if `line.strip() == "ready"`.
    """
    sim = TMC2209Sim(Clock())

    banner = sim.drain()
    assert banner.endswith(b"\r\n")
    assert banner.decode("ascii").strip() == "ready"
    assert sim.drain() == b""


def test_dtr_reset_reboots_and_delays_ready_by_one_drain() -> None:
    clock = Clock()
    sim = _make_sim(clock)
    port = FakeSerial(sim, clock)

    assert _exchange(sim, "position 500\n") == "1;position=500;\n"

    port.dtr = False
    port.dtr = True

    # First drain models boot time (host's read_all right after reset sees
    # nothing), the next poll receives `ready`; position is back to 0.
    assert port.read_all() == b""
    assert port.read_until(b"\n") == b"ready\r\n"
    assert _exchange(sim, "status\n").find("position=0;") > 0


def test_status_snapshot_matches_firmware_key_order() -> None:
    """Byte for byte what the reflashed board answered on the bench (task #19).

    The safety and microstep fields follow the original motion snapshot without
    changing the established field order.
    """
    sim = _make_sim()

    assert _exchange(sim, "status\n") == (
        "1;initialised=1;enabled=0;mode=target;position=0;phase=idle;"
        "target=0;target_set=0;speed=500.00;actual_speed=0.00;"
        "accel_per_s=1000.00;power_v=12.00;tx_overflow=0;"
        "drv_flags=0;safety=normal;mres=16;fine_mres=16;limit=40000;\n"
    )


def test_worst_case_line_status_still_fits_the_tx_ring() -> None:
    sim = TMC2209Sim(Clock(), power_v=26.39, tx_ring_capacity=TX_RING_CAPACITY)
    sim.drain()
    sim.position = -2147483648
    sim.target = 2147483647
    sim.speed_sps = 40000
    sim.actual_sps = 40000
    sim.accel_sps2 = 100000
    sim.driver_flags = 0xFF
    sim.safety = "shutdown"
    sim.active_microsteps = 256
    sim.microsteps = 256
    sim.safety_events = 65535

    answer = _exchange(sim, "status\n")

    assert len(answer.encode("ascii")) <= TX_RING_CAPACITY
    assert sim.tx_overflow == 0
    assert "events=" not in answer


def test_the_firmware_in_the_field_reports_no_tx_overflow_counter() -> None:
    """The board before the flash: no frame parser and no counter, one and the same commit.

    Measured before flashing: none of the `status` replies carried the key, which
    is why `_Status.tx_overflow` is `None`-able and the driver's warning cannot
    fire on that firmware.
    """
    sim = TMC2209Sim(Clock(), framed=False)
    sim.drain()

    assert "tx_overflow" not in _exchange(sim, "status\n")


def test_simple_setters_reply_with_written_value() -> None:
    sim = _make_sim()

    assert _exchange(sim, "position 100\n") == "1;position=100;\n"
    assert _exchange(sim, "speed 1000\n") == "1;speed=1000.00;\n"
    assert _exchange(sim, "acceleration 2000\n") == "1;accel_per_s=2000.00;\n"
    assert _exchange(sim, "direction 1\n") == "1;direction=1;\n"
    assert _exchange(sim, "delta 500\n") == "1;delta=500;target=600;target_set=1;\n"
    assert _exchange(sim, "mode free_ride\n") == "1;mode=free_ride;\n"
    assert _exchange(sim, "mode target\n") == "1;mode=target;\n"
    assert _exchange(sim, "set microsteps=32\n") == "1;microsteps=32;\n"
    assert _exchange(sim, "get microsteps\n") == "1;microsteps=32;\n"
    assert _exchange(sim, "enabled 1\n") == "1;enabled=1;\n"
    assert _exchange(sim, "run\n") == "1;running=1;\n"
    assert _exchange(sim, "stop\n") == "1;stopping=1;\n"


def test_error_responses_match_firmware() -> None:
    sim = _make_sim()

    assert _exchange(sim, "frobnicate\n") == "0;error=unknown_cmd;\n"
    assert _exchange(sim, "speed abc\n") == "0;error=bad_value;\n"
    assert _exchange(sim, "speed 50000\n") == "0;error=range;\n"
    assert _exchange(sim, "acceleration 200000\n") == "0;error=range;\n"
    assert _exchange(sim, "mode banana\n") == "0;error=bad_value;\n"
    assert _exchange(sim, "mode target extra\n") == "0;error=single_param;\n"
    assert _exchange(sim, "set microsteps=3\n") == "0;error=invalid_microsteps;\n"
    assert _exchange(sim, "set foo=1\n") == "0;error=unknown_param;\n"
    assert _exchange(sim, "set\n") == "0;error=missing_param;\n"
    assert _exchange(sim, "set microsteps\n") == "0;error=bad_param;\n"


def test_free_ride_motion_and_stop_over_virtual_time() -> None:
    clock = Clock()
    sim = _make_sim(clock)

    _exchange(sim, "acceleration 0\n")
    _exchange(sim, "speed 100\n")
    _exchange(sim, "mode free_ride\n")
    _exchange(sim, "run\n")

    clock.advance(10)

    status = _exchange(sim, "status\n")
    assert "position=1000;" in status
    assert "phase=running;" in status
    assert "actual_speed=100.00;" in status

    _exchange(sim, "stop\n")
    clock.advance(0.1)

    status = _exchange(sim, "status\n")
    assert "position=1000;" in status
    assert "phase=hold;" in status


def test_target_mode_ramps_and_stops_on_target() -> None:
    clock = Clock()
    sim = _make_sim(clock)

    _exchange(sim, "acceleration 1000\n")
    _exchange(sim, "speed 1000\n")
    _exchange(sim, "mode target\n")
    _exchange(sim, "delta 5000\n")
    _exchange(sim, "run\n")

    clock.advance(0.5)
    status = _exchange(sim, "status\n")
    assert "phase=acceleration;" in status or "phase=running;" in status

    clock.advance(6)
    status = _exchange(sim, "status\n")
    assert "position=5000;" in status
    assert "target_set=0;" in status
    assert "phase=hold;" in status


def test_target_mode_brakes_before_reaching_the_target() -> None:
    """`updateMotionStateV2` (main.cpp:818-825) drops the desired speed to 0 once
    the remaining distance fits inside `actual^2 / (2*accel)`.

    Watching only the final position hides that ramp: the firmware snaps onto
    the target either way. So the run is sampled while it is still in flight,
    at 1000 steps/s and 1000 steps/s^2 the braking distance is 500 steps, i.e.
    braking starts at position 4500 (t=5s) and the axis coasts down over the
    last second.
    """
    clock = Clock()
    sim = _make_sim(clock)

    _exchange(sim, "acceleration 1000\n")
    _exchange(sim, "speed 1000\n")
    _exchange(sim, "mode target\n")
    _exchange(sim, "delta 5000\n")
    _exchange(sim, "run\n")

    # 1s ramp up over 500 steps, then 4s of cruising to position 4500.
    clock.advance(4.5)
    status = _exchange(sim, "status\n")
    assert "phase=running;" in status
    assert "actual_speed=1000.00;" in status

    clock.advance(1.0)
    status = _exchange(sim, "status\n")
    assert "phase=deceleration;" in status
    assert "target_set=1;" in status
    position = int(status.split("position=")[1].split(";")[0])
    actual_speed = float(status.split("actual_speed=")[1].split(";")[0])
    assert 4500 < position < 5000, status
    assert 0.0 < actual_speed < 1000.0, status

    clock.advance(1.0)
    assert "position=5000;" in _exchange(sim, "status\n")


def test_disabling_stops_a_moving_motor_immediately() -> None:
    """`enabled 0` zeroes running/stopRequested/speeds on the spot (main.cpp:927-943).

    The reply is observed with no virtual time in between, so only the command
    handler itself can have stopped the axis — not the next integration step.
    """
    clock = Clock()
    sim = _make_sim(clock)

    _exchange(sim, "acceleration 0\n")
    _exchange(sim, "speed 100\n")
    _exchange(sim, "mode free_ride\n")
    _exchange(sim, "run\n")

    clock.advance(10)
    assert "actual_speed=100.00;" in _exchange(sim, "status\n")

    assert _exchange(sim, "enabled 0\n") == "1;enabled=0;\n"

    status = _exchange(sim, "status\n")
    assert "enabled=0;" in status
    assert "phase=idle;" in status
    assert "actual_speed=0.00;" in status
    assert "position=1000;" in status

    clock.advance(10)
    assert "position=1000;" in _exchange(sim, "status\n")


def test_negative_delta_moves_backward() -> None:
    clock = Clock()
    sim = _make_sim(clock)

    _exchange(sim, "acceleration 0\n")
    _exchange(sim, "speed 1000\n")
    _exchange(sim, "delta -2000\n")
    _exchange(sim, "run\n")

    clock.advance(5)

    status = _exchange(sim, "status\n")
    assert "position=-2000;" in status
    assert "target_set=0;" in status


def test_deceleration_phase_is_visible_while_stopping() -> None:
    clock = Clock()
    sim = _make_sim(clock)

    _exchange(sim, "acceleration 100\n")
    _exchange(sim, "speed 1000\n")
    _exchange(sim, "mode free_ride\n")
    _exchange(sim, "run\n")

    clock.advance(2)
    assert "phase=acceleration;" in _exchange(sim, "status\n")

    _exchange(sim, "stop\n")
    clock.advance(0.5)
    assert "phase=deceleration;" in _exchange(sim, "status\n")

    clock.advance(3)
    assert "phase=hold;" in _exchange(sim, "status\n")


def test_faults_empty_truncated_and_dead() -> None:
    sim = _make_sim()

    sim.faults.push(FaultKind.EMPTY)
    assert _exchange(sim, "status\n") == ""

    sim.faults.push(FaultKind.TRUNCATE)
    assert _exchange(sim, "run\n") == "1;running=1;"

    sim.faults.make_dead()
    assert _exchange(sim, "status\n") == ""
    assert _exchange(sim, "status\n") == ""

    sim.faults.revive()
    assert _exchange(sim, "stop\n") == "1;stopping=1;\n"


def test_ready_can_be_suppressed_for_fault_scenarios() -> None:
    sim = TMC2209Sim(Clock(), ready_on_reset=False)

    assert sim.drain() == b""
    sim.on_dtr(False)
    sim.on_dtr(True)
    assert sim.drain() == b""
    assert sim.drain() == b""


# ---------------------------------------------------------------------------
# The framed dialect (protocol v3) and the board's own ways of losing data.
# ---------------------------------------------------------------------------


def _frame(sim: TMC2209Sim, op: int, seq: int, payload: bytes = b"") -> Frame:
    sim.feed(encode_frame(op, seq, payload).encode("ascii"))
    return decode_response(sim.drain().decode("ascii"), op=op, seq=seq)


def test_framed_hello_answers_with_the_protocol_version() -> None:
    sim = _make_sim()

    assert response_values(_frame(sim, Op.HELLO, 1)) == {"protocol": "3", "firmware": "3"}


def test_framed_status_carries_the_same_snapshot_as_the_line_dialect() -> None:
    """Both dialects are rendered from one `_apply`, and this is what pins them together."""
    clock = Clock()
    sim = _make_sim(clock)
    _exchange(sim, "speed 1000\n")
    _exchange(sim, "acceleration 2000\n")
    _exchange(sim, "delta 5000\n")
    _exchange(sim, "run\n")
    clock.advance(1.0)

    framed = response_values(_frame(sim, Op.STATUS, 7))
    line = _exchange(sim, "status\n")

    # The framed snapshot also carries the safety event counter. It is the only
    # omitted line field: a worst-case textual status must still fit the 255-byte ring.
    line_fields = {key: value for key, value in framed.items() if key != "events"}
    assert line == "1;" + "".join(f"{key}={value};" for key, value in line_fields.items()) + "\n"


def test_target_motion_uses_full_steps_then_returns_to_fine_steps() -> None:
    clock = Clock()
    sim = _make_sim(clock)
    _exchange(sim, "speed 1000\n")
    _exchange(sim, "acceleration 0\n")
    _exchange(sim, "delta 2000\n")
    _exchange(sim, "run\n")

    clock.advance(1.0)
    assert response_values(_frame(sim, Op.STATUS, 1))["mres"] == "1"

    clock.advance(0.5)
    assert response_values(_frame(sim, Op.STATUS, 2))["mres"] == "16"

    clock.advance(0.5)
    final = response_values(_frame(sim, Op.STATUS, 3))
    assert final["position"] == "2000"
    assert final["mres"] == final["fine_mres"] == "16"


def test_overtemperature_warning_derates_and_critical_fault_shuts_down() -> None:
    clock = Clock()
    sim = _make_sim(clock)
    _exchange(sim, "acceleration 0\n")
    _exchange(sim, "speed 40000\n")
    _exchange(sim, "mode free_ride\n")
    _exchange(sim, "run\n")

    sim.driver_flags = 0x01
    clock.advance(0.1)
    derated = response_values(_frame(sim, Op.STATUS, 1))
    assert derated["safety"] == "derated"
    assert derated["limit"] == "1000"
    assert derated["actual_speed"] == "1000.00"
    assert derated["events"] == "1"

    sim.driver_flags = 0x04
    clock.advance(0.1)
    shutdown = response_values(_frame(sim, Op.STATUS, 2))
    assert shutdown["safety"] == "shutdown"
    assert shutdown["enabled"] == "0"
    assert response_values(_frame(sim, Op.RUN, 3)) == {"error": "driver_fault"}

    sim.driver_flags = 0
    clock.advance(5.1)
    recovered = response_values(_frame(sim, Op.STATUS, 4))
    assert recovered["safety"] == "normal"
    assert recovered["limit"] == "40000"


def test_confirmed_stall_is_latched_reported_and_requires_explicit_acknowledgement() -> None:
    clock = Clock()
    sim = _make_sim(clock)
    _exchange(sim, "acceleration 0\n")
    _exchange(sim, "speed 1000\n")
    _exchange(sim, "mode free_ride\n")
    _exchange(sim, "run\n")

    sim.sg_result = 2
    clock.advance(1.3)
    stalled = response_values(_frame(sim, Op.STATUS, 1))
    assert stalled["drv_flags"] == "16"
    assert stalled["safety"] == "shutdown"
    assert stalled["enabled"] == "0"
    assert stalled["events"] == "1"
    assert response_values(_frame(sim, Op.RUN, 2)) == {"error": "driver_fault"}

    assert response_values(_frame(sim, Op.ENABLED, 3, b"\x00")) == {"enabled": "0"}
    acknowledged = response_values(_frame(sim, Op.STATUS, 4))
    assert acknowledged["drv_flags"] == "0"
    assert acknowledged["safety"] == "normal"
    assert acknowledged["events"] == "1"
    assert response_values(_frame(sim, Op.RUN, 5)) == {"running": "1"}


def test_framed_setters_echo_the_value_that_was_applied() -> None:
    sim = _make_sim()

    assert response_values(_frame(sim, Op.SPEED, 1, (1000).to_bytes(4, "big"))) == {"speed": "1000.00"}
    assert response_values(_frame(sim, Op.POSITION, 2, (-42).to_bytes(4, "big", signed=True))) == {"position": "-42"}
    assert response_values(_frame(sim, Op.MODE, 3, b"\x01")) == {"mode": "free_ride"}
    assert response_values(_frame(sim, Op.MICROSTEPS, 4, (32).to_bytes(2, "big"))) == {"microsteps": "32"}
    assert response_values(_frame(sim, Op.MICROSTEPS, 5)) == {"microsteps": "32"}
    assert response_values(_frame(sim, Op.RUN, 6)) == {"running": "1"}
    # `target` is relative to the position set two commands ago: -42 + 500.
    assert response_values(_frame(sim, Op.DELTA, 7, (500).to_bytes(4, "big"))) == {
        "delta": "500",
        "target": "458",
        "target_set": "1",
    }


def test_a_command_frame_with_a_damaged_byte_is_refused_not_executed() -> None:
    """The direction the line protocol could never protect: host -> board."""
    sim = _make_sim()
    damaged = encode_frame(Op.SPEED, 9, (1000).to_bytes(4, "big")).replace("3E8", "3E9")

    sim.feed(damaged.encode("ascii"))

    assert response_values(decode_response(sim.drain().decode("ascii"), op=Op.SPEED, seq=9)) == {"error": "bad_crc"}
    assert sim.speed_sps == 500.0


def test_a_frame_whose_payload_is_the_wrong_size_is_refused() -> None:
    sim = _make_sim()

    assert response_values(_frame(sim, Op.SPEED, 1, b"\x01\x02")) == {"error": "bad_value"}
    assert response_values(_frame(sim, Op.STATUS, 2, b"\x01")) == {"error": "bad_value"}
    assert response_values(_frame(sim, 0x7F, 3)) == {"error": "unknown_cmd"}


def test_a_board_that_was_not_reflashed_answers_a_frame_with_unknown_cmd() -> None:
    """Exactly what firmware 300a439 does with a word it does not know (§5.1)."""
    sim = TMC2209Sim(Clock(), framed=False)
    sim.drain()

    sim.feed(encode_frame(Op.HELLO, 1).encode("ascii"))

    assert sim.drain() == b"0;error=unknown_cmd;\n"


def test_the_tx_ring_cuts_the_second_line_reply_exactly_as_the_board_did() -> None:
    """DEC_PROTOCOL.md §6.1, reproduced byte for byte: 147 + 108 = 255."""
    sim = TMC2209Sim(Clock(), power_v=3.88, framed=False, tx_ring_capacity=TX_RING_CAPACITY,
                     tx_overflow_truncates=True)
    sim.drain()

    sim.feed(b"status\nstatus\n")
    out = sim.drain()

    first, second = out.split(b"\n")[0] + b"\n", out.split(b"\n")[1]
    assert len(first) == 147
    assert len(second) == 108
    assert len(out) == TX_RING_CAPACITY
    assert not second.endswith(b"\n"), "the stump keeps no terminator, which is why the next reply glues onto it"


def test_the_same_two_commands_do_not_overflow_the_ring_when_framed() -> None:
    """The compactness of the frame is not cosmetic: it removes the overflow."""
    sim = TMC2209Sim(Clock(), power_v=3.88, tx_ring_capacity=TX_RING_CAPACITY, tx_overflow_truncates=True)
    sim.drain()

    sim.feed((encode_frame(Op.STATUS, 1) + encode_frame(Op.STATUS, 2)).encode("ascii"))
    out = sim.drain()

    assert len(out) == 176
    assert sim.tx_overflow == 0
    assert response_values(decode_response(out.decode("ascii"), op=Op.STATUS, seq=2))["position"] == "0"


def test_the_current_firmware_replaces_an_overflowing_reply_with_a_marker() -> None:
    sim = TMC2209Sim(Clock(), tx_ring_capacity=110)
    sim.drain()

    sim.feed((encode_frame(Op.STATUS, 1) + encode_frame(Op.STATUS, 2)).encode("ascii"))
    out = sim.drain().decode("ascii")

    assert sim.tx_overflow == 1
    assert response_values(decode_response(out, op=Op.STATUS, seq=2)) == {"error": "tx_overflow"}
    assert response_values(_frame(sim, Op.STATUS, 3))["tx_overflow"] == "1"


def test_the_receive_fifo_swallows_commands_and_poisons_the_next_one() -> None:
    """DEC_PROTOCOL.md §6.2: 6 of 13 commands vanish, then one command is wrecked."""
    sim = TMC2209Sim(Clock(), framed=False, rx_capacity=RX_FIFO_CAPACITY)
    sim.drain()

    sim.feed(b"enabled 0\n" * 13)
    answers = sim.drain()

    assert answers.count(b"\n") == 6
    assert sim.rx_dropped == 66

    sim.feed(b"status\n")
    assert sim.drain() == b"0;error=unknown_cmd;\n", "the stump of the cut line glued itself onto the next command"


def test_the_board_resynchronises_on_the_newest_frame_after_the_fifo_cut_one() -> None:
    """The same lost bytes as above, and no command is wrecked by the stump.

    The line protocol had no way to tell a leftover from a command; the frame
    marker gives the board one, so the write that follows an overflow is executed
    instead of being glued into nonsense.
    """
    sim = TMC2209Sim(Clock(), rx_capacity=RX_FIFO_CAPACITY)
    sim.drain()

    sim.feed(encode_frame(Op.SPEED, 1, (1000).to_bytes(4, "big")).encode("ascii") * 4)
    sim.drain()
    sim.feed(encode_frame(Op.SPEED, 2, (2000).to_bytes(4, "big")).encode("ascii"))
    answer = sim.drain().decode("ascii")

    assert sim.rx_dropped > 0, "the FIFO really did cut a command in half"
    assert response_values(decode_response(answer, op=Op.SPEED, seq=2)) == {"speed": "2000.00"}
    assert sim.speed_sps == 2000.0


def test_a_frame_behind_a_stump_is_still_a_frame() -> None:
    """Measured live: junk in front of the marker does not hide the frame.

    The firmware selects the dialect with `strchr(lineBufV2, '#')`, not by looking
    at position 0, precisely because the RX FIFO leaves a stump in front of the
    next write. On the bench each of these prefixes was answered 10 times out of
    10 with the status frame the sequence number asked for.
    """
    sim = _make_sim()

    for prefix in ("garbage", "garbage#00FF", "1;initialised=1;enabled=0;"):
        sim.feed(prefix.encode("ascii") + encode_frame(Op.STATUS, 9).encode("ascii"))
        answer = sim.drain().decode("ascii")

        assert response_values(decode_response(answer, op=Op.STATUS, seq=9))["position"] == "0", prefix


def test_a_frame_behind_a_nul_is_answered_normally() -> None:
    """The regression the length-driven parse exists for, measured both ways.

    The firmware used to pick the dialect with `strchr` over a C string, so a NUL
    in front of the marker ended the search, the frame was handed to the line
    parser, and the board answered nothing at all: 0 replies out of 10 on the
    bench. With `memchr` over the received length it is 10 out of 10, for a bare
    NUL and for a NUL followed by junk and a stump alike.
    """
    sim = _make_sim()

    for prefix in (b"\x00", b"\x00garbage#00FF", b"garbage\x00#00FF"):
        sim.feed(prefix + encode_frame(Op.STATUS, 11).encode("ascii"))
        answer = sim.drain().decode("ascii")

        assert response_values(decode_response(answer, op=Op.STATUS, seq=11))["position"] == "0", prefix


def test_a_nul_inside_the_hex_is_refused_as_damage() -> None:
    """The other half: seeing the whole line must not mean accepting a broken one.

    A NUL among the hex digits is a byte that was altered on the way in, and the
    parser says so rather than stopping at it and calling what came before a frame.

    The damage here lands on the sequence number itself, so the error comes back
    with `seq=0` -- the parser never got far enough to read one. The board answered
    exactly this, byte for byte: `#0100000C33F8` for a `seq=12` request
    (logs/protocol/dec_nul_inside_hex-*.jsonl). A host that sent 12 rejects that
    reply as stale, which is the safe outcome: no answer beats a wrong one.
    """
    sim = _make_sim()
    frame = encode_frame(Op.STATUS, 12)
    damaged = (frame[:6] + "\x00" + frame[7:]).encode("ascii")

    sim.feed(damaged)
    answer = sim.drain().decode("ascii")

    assert answer == "#0100000C33F8\n"
    assert response_values(decode_response(answer, op=Op.STATUS, seq=0)) == {"error": "bad_frame"}


def test_a_leading_nul_still_silences_a_line_command() -> None:
    """The line dialect keeps its C string, and with it the behaviour of 5.8.

    Nothing was fixed here on purpose: the line protocol has no marker and no
    length, so there is no framing to salvage -- and the driver that speaks it is
    the one on boards that were never reflashed.
    """
    sim = _make_sim()

    sim.feed(b"\x00status\n")

    assert sim.drain() == b""


def test_the_board_in_the_field_treats_a_frame_as_one_more_unknown_word() -> None:
    """Confirmed on the bench immediately before flashing: `#000101EF8C` -> unknown_cmd.

    No special case models this: the un-reflashed board has no marker rule at all,
    so the frame reaches the line parser as a single unrecognised token. That reply
    is the *only* positive proof the driver accepts for downgrading to v2.
    """
    sim = TMC2209Sim(Clock(), framed=False)
    sim.drain()

    sim.feed(encode_frame(Op.HELLO, 1).encode("ascii"))

    assert sim.drain() == b"0;error=unknown_cmd;\n"
