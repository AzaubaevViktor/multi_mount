from sim import Clock, FakeSerial, FaultKind, TMC2209Sim


def _exchange(sim: TMC2209Sim, line: str) -> str:
    sim.feed(line.encode("ascii"))
    return sim.drain().decode("ascii")


def _make_sim(clock: Clock | None = None) -> TMC2209Sim:
    sim = TMC2209Sim(clock or Clock())
    assert sim.drain() == b"ready\n"
    return sim


def test_power_on_emits_ready_line() -> None:
    sim = TMC2209Sim(Clock())

    assert sim.drain() == b"ready\n"
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
    assert port.read_until(b"\n") == b"ready\n"
    assert _exchange(sim, "status\n").find("position=0;") > 0


def test_status_snapshot_matches_firmware_key_order() -> None:
    sim = _make_sim()

    assert _exchange(sim, "status\n") == (
        "1;initialised=1;enabled=0;mode=target;position=0;phase=idle;"
        "target=0;target_set=0;speed=500.00;actual_speed=0.00;"
        "accel_per_s=1000.00;power_v=12.00;\n"
    )


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
