"""Chaos sessions: the real drivers, over the real ``SerialLine``, over a port
whose bytes are being eaten (PLAN.md §3, stage 3b).

Only the transport lies here — every layer above it is production code, and the
simulator is the ground truth the driver's answers are compared against. A run
that merely "did not crash" proves nothing, so each session checks invariants
that must hold no matter how much of the traffic was destroyed:

INV1 *no garbage as data*: a driver either answers correctly (agreeing with the
     simulator) or raises. A damaged frame decoded into a plausible number is
     worse than no answer at all — the mount then slews to a wrong position.
INV2 *no half-open line*: after any transport error the ``SerialLine`` is
     CLOSED, and no call ever succeeds while the port behind it is dead. This
     bug was fixed three times in three code paths and recurred five months
     later under a different exception type (DECISIONS.md §4.3).
INV3 *no hang*: every call finishes within a bounded amount of virtual time,
     and every test within a bounded amount of real time.
INV4 *no silent command loss*: a command that did not reach the controller
     surfaces as an error; a command reported as applied really is applied.
INV5 *position does not jump*: at rest the reported position equals the
     simulator's true position, and a completed GOTO lands on the target asked
     for.

Three violations that live in the DEC driver's line protocol are reproduced
with fixed seeds and marked ``xfail(strict=True)`` rather than asserted away —
see the "known defects" section at the bottom.
"""

import os
import time

import pytest

from serial_wrapper.wrapper import SerialLineClosedError, SerialLineError, SerialLineState
from sim import Chaos, ChaosProfile, Clock, FaultKind, SimSerialLine, SkyWatcherSim, TMC2209Sim, Transport
from sky.constants import STELLAR_SPEED
from sky.motor import MotionMode, MotorDirection, MotorStateError, MotorStopRequire
from sky.physics import Ha
from skywatcher.motor import SkyWatcherMotor, SkyWatcherMotorError
from tmc2209.motor import TMC2209Motor, TMC2209MotorError

_RA_CPR = 8_000_000
_RA_TIMER_FREQ = 64935
_RA_HIGHSPEED_RATIO = 16

# Fault mixes. `_LOSSY` is the everyday flaky-cable case, `_UNPLUGGING` adds the
# device vanishing mid-transfer, `_BRUTAL` is far past anything a real line does
# and exists to make sure nothing deadlocks when almost nothing gets through.
_LOSSY = ChaosProfile(lose_write_byte=0.01, lose_read_byte=0.03, short_write=0.02, short_read=0.05)
_UNPLUGGING = ChaosProfile(lose_read_byte=0.01, short_read=0.02, unplug=0.004)
_BRUTAL = ChaosProfile(lose_write_byte=0.1, lose_read_byte=0.2, short_write=0.1, short_read=0.2)

# Exceptions a caller of the axis layer is expected to handle. Anything else
# escaping a driver (KeyError, ValueError, IndexError...) means a damaged frame
# reached code that assumed it was well formed.
_EXPECTED_RA_ERRORS = (SkyWatcherMotorError, SerialLineError, MotorStopRequire, MotorStateError, OSError)
_EXPECTED_DEC_ERRORS = (TMC2209MotorError, SerialLineError, MotorStopRequire, MotorStateError, TimeoutError, OSError)

# A single driver call may retry three times, each retry sleeping and draining
# with a 0.5s read timeout; the TMC handshake gets its full ready-timeout budget
# (3 attempts x 10s). Anything above that is a wait loop that lost its exit.
_CALL_BUDGET_S = 5.0
_CONNECT_BUDGET_S = 40.0

_SEED_ENV = "MULTI_MOUNT_CHAOS_SEED"


class _Guard:
    """Runs driver calls and records every invariant violation it sees.

    Violations are collected rather than raised: under chaos most calls are
    *supposed* to fail, and what the test is about is the state of the stack
    afterwards, so the session has to keep running to the end.
    """

    def __init__(
        self,
        chaos: Chaos,
        line: SimSerialLine,
        clock: Clock,
        expected_errors: tuple[type[BaseException], ...],
        driver_error: type[BaseException],
    ) -> None:
        self.chaos = chaos
        self.line = line
        self.clock = clock
        self.expected_errors = expected_errors
        self.driver_error = driver_error
        self.violations: list[str] = []
        self.leaked: list[str] = []
        self.calls: list[str] = []
        self.last_ok = False

    def call(self, name, action, budget_s: float = _CALL_BUDGET_S, touches_line: bool = True):
        started_at = self.clock.now
        port = self.line.serial
        port_was_dead = port is not None and port.is_unplugged
        try:
            value = action()
        except BaseException as error:  # noqa: BLE001 - classifying is the whole point
            self.last_ok = False
            value = None
            self.calls.append(f"{name}:{type(error).__name__}")
            if not isinstance(error, self.expected_errors):
                # INV1: an unexpected type means a damaged frame was handed to
                # code that trusted its shape.
                self.leaked.append(f"{name}: {type(error).__name__}: {error}")
            if isinstance(error, OSError) and not isinstance(error, self.driver_error) and self.line.state != SerialLineState.CLOSED:
                # INV2: the recurrence guard. A dead port always takes the line
                # down with it, whatever code path noticed it first.
                self.violations.append(f"{name}: transport error {type(error).__name__} left the line {self.line.state.value}")
        else:
            self.last_ok = True
            self.calls.append(f"{name}:ok")
            if touches_line:
                # INV2: talking to a line that is closed, or to a port that was
                # already dead when the call started, cannot possibly succeed.
                if self.line.state != SerialLineState.OPEN:
                    self.violations.append(f"{name}: succeeded while the line is {self.line.state.value}")
                if port_was_dead and self.line.serial is port:
                    # `connect` legitimately replaces a dead port with a fresh one;
                    # anything else answering through the same dead port is a lie.
                    self.violations.append(f"{name}: succeeded on an unplugged port")

        # INV3: virtual time makes "it hung" a measurable assertion.
        elapsed = self.clock.now - started_at
        if elapsed > budget_s:
            self.violations.append(f"{name}: spent {elapsed:.1f}s of virtual time (budget {budget_s}s)")

        return value

    def check(self, condition: bool, message: str) -> None:
        if not condition:
            self.violations.append(message)

    def report(self, seed: int) -> str:
        problems = "\n  ".join(self.violations + self.leaked)
        return (
            f"seed={seed}, chaos bites={self.chaos.stats.bites} ({self.chaos.stats})\n"
            f"  {problems}\n"
            f"calls: {self.calls}\n"
            f"reproduce: {_SEED_ENV}={seed} .venv/bin/python -m pytest "
            f"src/tests/units/test_chaos_sessions.py -k chaos_seed_from_env"
        )


def _settle(sim, probe: bytes) -> None:
    """Force the device to integrate its motion up to the current clock.

    The simulators only advance their kinematics when a command arrives, so a
    test that just advanced the clock would otherwise compare against a stale
    ``position``. The probe is a plain protocol read, not private state.
    """
    sim.feed(probe)
    sim.drain()


def _ra_session(chaos: Chaos) -> _Guard:
    """Connect, track, guide, halt, GOTO, halt, reconnect — all under chaos."""
    clock = Clock()
    sim = SkyWatcherSim(clock, cpr=_RA_CPR, timer_freq=_RA_TIMER_FREQ, highspeed_ratio=_RA_HIGHSPEED_RATIO)
    line = SimSerialLine(sim, clock, transport=chaos.transport, port="sim://ra", timeout_s=0, name="chaos-ra", terminator="\r")
    chaos.bind(line)
    motor = SkyWatcherMotor(line, clock)
    guard = _Guard(chaos, line, clock, _EXPECTED_RA_ERRORS, SkyWatcherMotorError)

    guard.call("connect", motor.connect, budget_s=_CONNECT_BUDGET_S)
    if guard.last_ok:
        # INV1: the geometry every axis conversion is built on. A truncated
        # answer accepted here poisons the whole session.
        guard.check(
            (motor._steps_360, motor._steps_worm, motor._highspeed_ratio) == (_RA_CPR, _RA_TIMER_FREQ, _RA_HIGHSPEED_RATIO),
            f"connect accepted a damaged mount config: {motor._steps_360}/{motor._steps_worm}/{motor._highspeed_ratio}",
        )

    # Pure conversions never touch the port, so they are allowed to answer on a
    # closed line — but they must still refuse to work on a half-read geometry.
    sidereal_sps = guard.call("convert_speed", lambda: motor.convert_speed_to_steps_per_second(STELLAR_SPEED), touches_line=False)
    if sidereal_sps:
        applied_sps = guard.call("set_speed", lambda: motor.set_speed(sidereal_sps))
        if guard.last_ok:
            # INV4: a step period the board never received must not come back as
            # "applied".
            guard.check(
                abs(sim.speed_sps() - applied_sps) <= 1,
                f"set_speed reported {applied_sps} sps while the mount runs at {sim.speed_sps():.1f} sps",
            )

        guard.call("set_direction", lambda: motor.set_direction(MotorDirection.FORWARD))
        guard.call("set_motion_mode", lambda: motor.set_motion_mode(MotionMode.RUN))
        guard.call("run", motor.run)
        clock.advance(30)
        guard.call("status_tracking", motor.status)

        # A guide pulse reaches this layer as a tracking-rate change.
        guard.call("guide_speed", lambda: motor.set_speed(int(sidereal_sps * 1.5)))
        clock.advance(4)
        guard.call("guide_back", lambda: motor.set_speed(sidereal_sps))

    guard.call("halt", motor.stop)
    if guard.last_ok:
        clock.advance(1)
        _settle(sim, b":f1\r")
        # INV4/INV5: an acknowledged halt really stops the axis, and a stopped
        # axis does not creep.
        guard.check(not sim.running, "stop() succeeded but the mount is still running")
        resting_at = sim.position
        clock.advance(10)
        _settle(sim, b":f1\r")
        guard.check(sim.position == resting_at, f"mount moved {sim.position - resting_at:.0f} steps after a successful halt")

        status = guard.call("status_at_rest", motor.status)
        if guard.last_ok and not sim.running:
            # INV5: at rest the driver's position is the mount's position.
            guard.check(
                _ra_distance(status.steps, sim.position) <= 2,
                f"position at rest is {status.steps}, mount is at {int(round(sim.position)) % _RA_CPR}",
            )

    delta_steps = guard.call("convert_delta", lambda: motor.convert_position_to_steps(Ha(1800)), touches_line=False)
    if delta_steps:
        _settle(sim, b":f1\r")
        started_at = sim.position
        was_resting = not sim.running

        guard.call("goto", lambda: motor.set_delta(delta_steps))
        goto_set = guard.last_ok
        guard.call("goto_run", motor.run)
        clock.advance(60)
        _settle(sim, b":f1\r")

        if goto_set and guard.last_ok and was_resting and not sim.running:
            # INV5: a GOTO accepted end to end lands on the requested target — no
            # undershoot from a half-applied increment, and no overshoot from a
            # `:J1` re-sent after its answer was lost (the board would re-arm the
            # increment from wherever the axis is by then).
            travelled = sim.position - started_at
            guard.check(abs(travelled - delta_steps) <= 2, f"GOTO travelled {travelled:.0f} steps, was asked for {delta_steps}")
            status = guard.call("status_after_goto", motor.status)
            if guard.last_ok:
                guard.check(
                    _ra_distance(status.steps, sim.position) <= 2,
                    f"position after GOTO is {status.steps}, mount is at {int(round(sim.position)) % _RA_CPR}",
                )

    guard.call("final_halt", motor.stop)

    # Reconnect on a healthy port: whatever chaos did, the line must be usable
    # again after a fresh connect.
    chaos.profile = ChaosProfile()
    guard.call("reconnect", motor.connect, budget_s=_CONNECT_BUDGET_S)
    guard.check(guard.last_ok and line.state == SerialLineState.OPEN, "reconnect failed on a healthy port")
    guard.call("status_after_reconnect", motor.status)

    return guard


def _ra_distance(reported_steps: int, true_position: float) -> int:
    true_steps = int(round(true_position)) % _RA_CPR
    return min((reported_steps - true_steps) % _RA_CPR, (true_steps - reported_steps) % _RA_CPR)


def _dec_session(chaos: Chaos) -> _Guard:
    """Connect (DTR reset + ``ready``), GOTO, guide pulse, halt, reconnect."""
    clock = Clock()
    sim = TMC2209Sim(clock)
    line = SimSerialLine(sim, clock, transport=chaos.transport, port="sim://dec", timeout_s=2, name="chaos-dec", terminator="\n")
    chaos.bind(line)
    motor = TMC2209Motor(line, clock)
    guard = _Guard(chaos, line, clock, _EXPECTED_DEC_ERRORS, TMC2209MotorError)

    guard.call("connect", motor.connect, budget_s=_CONNECT_BUDGET_S)
    guard.check(not guard.last_ok or motor._is_connected, "connect returned without marking the motor connected")
    guard.call("status", motor.status)

    guard.call("mode_target", lambda: motor.set_motion_mode(MotionMode.TARGET))
    guard.call("set_acceleration", lambda: motor.set_acceleration(1000))
    guard.call("set_speed", lambda: motor.set_speed(1000))
    guard.call("set_delta", lambda: motor.set_delta(5000))
    guard.call("run", motor.run)
    clock.advance(15)
    guard.call("status_after_goto", motor.status)

    # Guide pulse: free ride at guide rate for a few seconds.
    guard.call("mode_free_ride", lambda: motor.set_motion_mode(MotionMode.RUN))
    guard.call("guide_speed", lambda: motor.set_speed(25))
    guard.call("guide_direction", lambda: motor.set_direction(MotorDirection.FORWARD))
    guard.call("guide_run", motor.run)
    clock.advance(4)

    guard.call("halt", motor.stop)
    if guard.last_ok:
        clock.advance(5)
        _settle(sim, b"status\n")
        # INV4: `stop` carries no argument, so a damaged one cannot be mistaken
        # for another valid command — an acknowledged halt really stops the axis.
        guard.check(sim.actual_sps == 0.0, f"stop() succeeded but the axis still runs at {sim.actual_sps:.1f} sps")
        resting_at = sim.position
        clock.advance(10)
        _settle(sim, b"status\n")
        guard.check(sim.position == resting_at, f"axis moved {sim.position - resting_at:.0f} steps after a successful halt")

    chaos.profile = ChaosProfile()
    guard.call("reconnect", motor.connect, budget_s=_CONNECT_BUDGET_S)
    guard.check(guard.last_ok and line.state == SerialLineState.OPEN, "reconnect failed on a healthy port")
    guard.call("status_after_reconnect", motor.status)

    return guard


# ---------------------------------------------------------------------------
# Scenario tests
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("profile_name,profile", [("lossy", _LOSSY), ("unplugging", _UNPLUGGING), ("brutal", _BRUTAL)])
def test_ra_session_holds_every_invariant(profile_name: str, profile: ChaosProfile) -> None:
    chaos = Chaos(seed=20260725, profile=profile)

    guard = _ra_session(chaos)

    assert chaos.stats.bites > 0, f"{profile_name}: chaos never fired, this run proves nothing"
    assert not guard.violations, guard.report(20260725)
    assert not guard.leaked, guard.report(20260725)


@pytest.mark.parametrize("profile_name,profile", [("lossy", _LOSSY), ("unplugging", _UNPLUGGING), ("brutal", _BRUTAL)])
def test_dec_session_holds_the_transport_invariants(profile_name: str, profile: ChaosProfile) -> None:
    chaos = Chaos(seed=20260725, profile=profile)

    guard = _dec_session(chaos)

    assert chaos.stats.bites > 0, f"{profile_name}: chaos never fired, this run proves nothing"
    assert not guard.violations, guard.report(20260725)


def test_unplug_during_a_retry_drain_closes_the_line() -> None:
    """INV2, the candidate fourth recurrence: the unplug lands on ``drop_buffers``.

    Both drivers call ``drop_buffers`` from their retry handler, i.e. exactly
    when the line is already misbehaving. That method used to be the only I/O
    path without the disconnect guard, so an unplug there threw a bare OSError
    past the driver and left the ``SerialLine`` OPEN around a dead port.
    """
    clock = Clock()
    sim = SkyWatcherSim(clock, cpr=_RA_CPR)
    line = SimSerialLine(sim, clock, port="sim://ra", timeout_s=0, name="chaos-ra", terminator="\r")
    motor = SkyWatcherMotor(line, clock)
    motor.connect()

    port = line.serial
    assert port is not None
    sim.faults.make_dead()  # no answer ever comes, so the driver enters its retry handler
    drains = 0
    original_reset_input_buffer = port.reset_input_buffer

    def reset_input_buffer() -> None:
        nonlocal drains
        drains += 1
        if drains == 2:  # 1 = the query itself, 2 = the drain inside the retry handler
            port.unplug()
        original_reset_input_buffer()

    port.reset_input_buffer = reset_input_buffer

    with pytest.raises(OSError) as unplugged:
        motor.status()

    assert unplugged.value.errno == 6
    assert line.state == SerialLineState.CLOSED
    assert line.serial is None
    assert line.last_close_meta is not None
    assert line.last_close_meta.reason == "error_in_drop_buffers"


def test_a_port_that_fails_to_close_still_leaves_the_line_closed() -> None:
    """INV2, the last way out: ``close()`` itself blowing up.

    A yanked USB-serial device can fail its final ``termios`` calls, and this is
    the one method every error path ends in. If it raises before the bookkeeping
    it both hides the failure that caused the close and leaves the line OPEN
    around a port nobody can use.
    """

    class _StuckPort:
        is_open = True

        def close(self) -> None:
            raise OSError(5, "Input/output error")

    line = SimSerialLine(SkyWatcherSim(Clock(), cpr=_RA_CPR), Clock(), port="sim://ra", timeout_s=0, name="chaos-ra", terminator="\r")
    line.serial = _StuckPort()

    line.close(reason="unplug")

    assert line.state == SerialLineState.CLOSED
    assert line.serial is None
    assert line.last_close_meta is not None
    assert line.last_close_meta.reason == "unplug"


def test_line_killed_by_chaos_never_answers_again() -> None:
    """INV2: a dead line fails fast and says why, instead of pretending to work."""
    chaos = Chaos(seed=7, profile=ChaosProfile(unplug=0.2, lose_read_byte=0.05))
    clock = Clock()
    sim = SkyWatcherSim(clock, cpr=_RA_CPR)
    line = SimSerialLine(sim, clock, transport=chaos.transport, port="sim://ra", timeout_s=0, name="chaos-ra", terminator="\r")
    chaos.bind(line)
    motor = SkyWatcherMotor(line, clock)

    for _ in range(50):
        try:
            motor.connect()
            motor.status()
        except (SkyWatcherMotorError, SerialLineError, OSError):
            pass
        if line.state == SerialLineState.CLOSED:
            break

    assert chaos.stats.unplugs > 0
    assert line.state == SerialLineState.CLOSED
    assert line.serial is None
    with pytest.raises((SerialLineClosedError, SkyWatcherMotorError)):
        motor.status()


def test_ra_truncated_status_is_rejected_instead_of_zero_padded() -> None:
    """INV1 (regression): a lost byte in ``=411`` used to read as "stopped".

    ``_Status.from_bytes`` zero-padded a short body, so a slewing axis was
    reported as idle — and ``wait_till_stop`` returned while the mount kept
    moving. Seed 2 destroys exactly the bytes that made the truncated body look
    like a plausible status.
    """
    chaos = Chaos(seed=2, profile=ChaosProfile())
    clock = Clock()
    sim = SkyWatcherSim(clock, cpr=_RA_CPR)
    line = SimSerialLine(sim, clock, transport=chaos.transport, port="sim://ra", timeout_s=0, name="chaos-ra", terminator="\r")
    chaos.bind(line)
    motor = SkyWatcherMotor(line, clock)
    motor.connect()
    motor.set_speed(motor.convert_speed_to_steps_per_second(STELLAR_SPEED))
    motor.set_direction(MotorDirection.FORWARD)
    motor.set_motion_mode(MotionMode.RUN)
    motor.run()
    clock.advance(10)
    assert sim.running is True

    chaos.profile = ChaosProfile(lose_read_byte=0.12)
    with pytest.raises(SkyWatcherMotorError):
        motor.status()

    assert chaos.stats.lost_read_bytes > 0
    assert sim.running is True


def test_ra_truncated_position_is_rejected_instead_of_zero_extended() -> None:
    """INV1 (regression): ``=A5B2C3`` losing four digits used to decode fine.

    ``_Revu24.from_mount`` zero-extended any two-char body, which is only right
    for the 8-bit high-speed ratio; for a 24-bit position it turned a shredded
    frame into a valid position half a revolution away. The hook keeps the frame
    intact (``digits=6``) for the handshake and then eats four digits out of the
    position answer, leaving the terminator in place.
    """
    clock = Clock()
    sim = SkyWatcherSim(clock, cpr=_RA_CPR, position_steps=4_000_000)
    keep = {"digits": 6}

    def eat_the_body(data: bytes) -> bytes:
        if len(data) == 7 and data.endswith(b"\r"):
            return data[: keep["digits"]] + b"\r"
        return data

    line = SimSerialLine(
        sim,
        clock,
        transport=Transport(on_read=eat_the_body),
        port="sim://ra",
        timeout_s=0,
        name="chaos-ra",
        terminator="\r",
    )
    motor = SkyWatcherMotor(line, clock)
    motor.connect()
    assert motor.status().steps == 4_000_000

    keep["digits"] = 2
    clock.advance(1)  # past the 0.25s position cache, so the next status really asks the board
    with pytest.raises(SkyWatcherMotorError):
        motor.status()


# ---------------------------------------------------------------------------
# `:J1` is the one command that must not be repeated blindly. The driver may
# only re-send it after establishing that the board did *not* execute it, and
# each of the three tests below is the sole witness of one half of that
# reasoning — the running flag, the distance travelled, and the freshness of the
# position it is measured against.
# ---------------------------------------------------------------------------


def _ra_start_motion_rig(drop_first_start: bool = False) -> tuple[Clock, SkyWatcherSim, SkyWatcherMotor, list[bytes]]:
    """A connected RA motor plus the log of every ``:J1`` that reached the board.

    ``drop_first_start`` eats the first start on the way *to* the board, which is
    the other half of the pair: with ``FaultKind.EMPTY`` the board runs and only
    its ``=`` is lost, here the board never hears the command at all. The driver
    cannot tell the two apart from the silence alone, and it has to get both right.
    """
    clock = Clock()
    sim = SkyWatcherSim(clock, cpr=_RA_CPR, timer_freq=_RA_TIMER_FREQ, highspeed_ratio=_RA_HIGHSPEED_RATIO)
    starts: list[bytes] = []
    to_drop = [b":J1\r"] if drop_first_start else []

    def watch_starts(data: bytes) -> bytes:
        if data != b":J1\r":
            return data
        if to_drop:
            to_drop.pop()
            return b""
        starts.append(data)
        return data

    line = SimSerialLine(
        sim, clock, transport=Transport(on_write=watch_starts), port="sim://ra", timeout_s=0, name="chaos-ra", terminator="\r"
    )
    motor = SkyWatcherMotor(line, clock)
    motor.connect()
    return clock, sim, motor, starts


def test_ra_start_motion_is_not_re_sent_while_the_axis_is_running() -> None:
    """The running flag: the axis is moving, but too slowly to prove it by position.

    A GOTO slowed down to 2 steps/s covers a single step during the half second
    the driver spends draining the line, i.e. less than the noise floor of the
    position comparison. Only ``:f1`` can tell that the board is already running,
    and if the driver misses that it re-arms the increment from wherever the axis
    is — which at this speed is not visible in the final position either, so the
    number of starts that reached the board is what the invariant is about.
    """
    clock, sim, motor, starts = _ra_start_motion_rig()
    delta_steps = motor.convert_position_to_steps(Ha(60))
    motor.set_delta(delta_steps)
    motor.set_speed(2)
    started_at = sim.position
    sim.faults.push(FaultKind.EMPTY, command="J")

    assert motor.run() is True

    assert starts == [b":J1\r"], "the board was told to start a second time while it was already running"
    clock.advance(delta_steps)  # 2 steps/s, so a second per two steps
    _settle(sim, b":f1\r")
    assert sim.position - started_at == pytest.approx(delta_steps, abs=2)


def test_ra_start_motion_is_not_re_sent_after_a_goto_that_already_finished() -> None:
    """The distance travelled: the run is over before the driver gets to ask.

    A GOTO of half a minute of hour angle takes a quarter of a second, and the
    retry handler drains the line for half a second before it looks. ``:f1`` then
    says "stopped, tracking" — exactly what it says about a start that never
    happened. The distance is the only difference, and without it the driver
    re-runs the whole move.
    """
    clock, sim, motor, starts = _ra_start_motion_rig()
    delta_steps = motor.convert_position_to_steps(Ha(30))
    motor.set_delta(delta_steps)
    started_at = sim.position
    sim.faults.push(FaultKind.EMPTY, command="J")

    assert motor.run() is True

    assert starts == [b":J1\r"], "a finished GOTO was mistaken for a start that never happened"
    clock.advance(60)
    _settle(sim, b":f1\r")
    assert sim.position - started_at == pytest.approx(delta_steps, abs=2)


def test_ra_start_motion_that_never_reached_the_board_is_re_sent() -> None:
    """The freshness: a 0.25s-old position cache is not evidence about the board.

    The command is lost on the way out, so the axis stands still and the driver
    must send it again. What can make it believe otherwise is its own position
    cache: `status()` fills it, and the board may move — or, after the brownout
    reboot of §12, jump — between that fill and the start. Comparing the fresh
    reading against a cached one then shows a shift that this command never
    caused, and the GOTO is dropped on the floor while `run()` reports success.
    """
    clock, sim, motor, starts = _ra_start_motion_rig(drop_first_start=True)
    delta_steps = motor.convert_position_to_steps(Ha(1800))
    motor.set_delta(delta_steps)
    motor.status()  # fills the 0.25s position cache
    sim.position += 100_000
    started_at = sim.position

    assert motor.run() is True

    assert starts == [b":J1\r"], "the lost start was never re-sent, so the axis stayed put"
    clock.advance(60)
    _settle(sim, b":f1\r")
    assert sim.position - started_at == pytest.approx(delta_steps, abs=2)


# ---------------------------------------------------------------------------
# Fuzz: many seeds, invariants only, failing seed printed
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("profile_name,profile,seeds", [("lossy", _LOSSY, 150), ("unplugging", _UNPLUGGING, 120), ("brutal", _BRUTAL, 60)])
def test_fuzz_ra_sessions(profile_name: str, profile: ChaosProfile, seeds: int) -> None:
    started_at = time.perf_counter()
    bites = 0

    for seed in range(seeds):
        chaos = Chaos(seed=seed, profile=profile)
        guard = _ra_session(chaos)
        bites += chaos.stats.bites
        assert not guard.violations, guard.report(seed)
        assert not guard.leaked, guard.report(seed)

    assert bites > seeds, f"{profile_name}: chaos barely fired ({bites} bites over {seeds} sessions)"
    assert time.perf_counter() - started_at < 5.0, "virtual time is supposed to keep this cheap"


@pytest.mark.parametrize("profile_name,profile,seeds", [("lossy", _LOSSY, 150), ("unplugging", _UNPLUGGING, 120), ("brutal", _BRUTAL, 60)])
def test_fuzz_dec_sessions(profile_name: str, profile: ChaosProfile, seeds: int) -> None:
    started_at = time.perf_counter()
    bites = 0

    for seed in range(seeds):
        chaos = Chaos(seed=seed, profile=profile)
        guard = _dec_session(chaos)
        bites += chaos.stats.bites
        assert not guard.violations, guard.report(seed)

    assert bites > seeds, f"{profile_name}: chaos barely fired ({bites} bites over {seeds} sessions)"
    assert time.perf_counter() - started_at < 5.0, "virtual time is supposed to keep this cheap"


def test_chaos_seed_from_env() -> None:
    """One-command reproduction of any seed a fuzz failure printed.

    ``MULTI_MOUNT_CHAOS_SEED=<seed> .venv/bin/python -m pytest
    src/tests/units/test_chaos_sessions.py -k chaos_seed_from_env``
    """
    seed = int(os.environ.get(_SEED_ENV, "20260725"))

    for profile in (_LOSSY, _UNPLUGGING, _BRUTAL):
        ra = _ra_session(Chaos(seed=seed, profile=profile))
        assert not ra.violations, ra.report(seed)
        assert not ra.leaked, ra.report(seed)

        dec = _dec_session(Chaos(seed=seed, profile=profile))
        assert not dec.violations, dec.report(seed)


# ---------------------------------------------------------------------------
# Known defects: reproduced here, not fixed (src/tmc2209/motor.py belongs to a
# change in flight). Each states the invariant that *should* hold and is marked
# xfail(strict=True), so the day the protocol grows an integrity check the
# marker fails loudly and gets removed together with this comment.
# ---------------------------------------------------------------------------


@pytest.mark.xfail(strict=True, reason="DEFECT: the TMC line protocol has no integrity check, so a digit lost inside a value parses as a valid position")
def test_dec_never_reports_a_position_the_controller_does_not_have() -> None:
    for seed in range(60):
        chaos = Chaos(seed=seed, profile=ChaosProfile())
        clock = Clock()
        sim = TMC2209Sim(clock)
        line = SimSerialLine(sim, clock, transport=chaos.transport, port="sim://dec", timeout_s=2, name="chaos-dec", terminator="\n")
        chaos.bind(line)
        motor = TMC2209Motor(line, clock)
        motor.connect()

        sim.position = 12345.0
        chaos.profile = ChaosProfile(lose_read_byte=0.03)
        for _ in range(6):
            try:
                steps = motor.status().steps
            except (TMC2209MotorError, SerialLineError, KeyError, ValueError, OSError):
                continue
            assert steps == 12345, f"seed={seed}: driver reports position {steps}, controller is at 12345"


@pytest.mark.xfail(strict=True, reason="DEFECT: set_speed returns the requested value instead of the `speed=` the controller echoed back")
def test_dec_never_reports_a_speed_the_controller_never_applied() -> None:
    for seed in range(20):
        chaos = Chaos(seed=seed, profile=ChaosProfile())
        clock = Clock()
        sim = TMC2209Sim(clock)
        line = SimSerialLine(sim, clock, transport=chaos.transport, port="sim://dec", timeout_s=2, name="chaos-dec", terminator="\n")
        chaos.bind(line)
        motor = TMC2209Motor(line, clock)
        motor.connect()

        chaos.profile = ChaosProfile(lose_write_byte=0.05)
        for _ in range(6):
            try:
                applied = motor.set_speed(1000)
            except (TMC2209MotorError, SerialLineError, KeyError, ValueError, OSError):
                continue
            assert abs(sim.speed_sps - applied) <= 1, f"seed={seed}: set_speed reported {applied}, controller runs at {sim.speed_sps}"


def test_ra_retried_start_motion_does_not_extend_the_goto_target() -> None:
    """INV5 (was a defect until ``_NON_IDEMPOTENT``): ``:J1`` is not re-sent blindly.

    ``_transact`` re-sends after any unusable answer, which is right for the
    queries and the set-value commands but not for ``:J1``: the board latches
    ``position + increment`` when it starts, so a resend two tenths of a second
    later moved the target that far ahead. Losing just the answer to ``:J1`` is
    enough — no chaos profile needed, a single scripted empty reply reproduces it.
    The driver now confirms the start with ``:f1`` and the position instead.
    """
    clock = Clock()
    sim = SkyWatcherSim(clock, cpr=_RA_CPR)
    line = SimSerialLine(sim, clock, port="sim://ra", timeout_s=0, name="chaos-ra", terminator="\r")
    motor = SkyWatcherMotor(line, clock)
    motor.connect()

    delta_steps = motor.convert_position_to_steps(Ha(1800))
    motor.set_delta(delta_steps)
    started_at = sim.position
    sim.faults.push(FaultKind.EMPTY, command="J")  # the board starts, only its answer is lost

    motor.run()
    clock.advance(60)
    _settle(sim, b":f1\r")

    assert sim.position - started_at == pytest.approx(delta_steps, abs=2)


@pytest.mark.xfail(strict=True, reason="DEFECT: a damaged TMC status frame escapes as a raw KeyError/ValueError instead of a TMC2209MotorError")
def test_dec_only_raises_its_own_error_types() -> None:
    for seed in range(40):
        chaos = Chaos(seed=seed, profile=_LOSSY)
        guard = _dec_session(chaos)
        assert not guard.leaked, f"seed={seed}: {guard.leaked}"
