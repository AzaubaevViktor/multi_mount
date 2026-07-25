"""Record one fixed session against the mount and write down what it answered.

Why this exists
---------------
``src/sim`` was reconstructed from logs, i.e. from a *guess* about the two
protocols, and the drivers on top of it have never been refactored against
anything but their own unit tests. Both problems need the same artifact: one
byte-exact trace of a real session plus a statement of what the two boards said
about themselves. This script produces exactly that pair — a JSON Lines trace
(``src/serial_wrapper/recorder.py``) and a JSON manifest — from a scenario fixed
in code, so that two runs differ only in what the hardware did.

Modes
-----
``--sim`` runs the identical scenario against ``src/sim``; only the transport
construction differs (``SimSerialLine`` instead of ``SerialLine``). That mode is
not a convenience: a session script executed for the first time on live hardware
is untested code inside a one-shot window, and the axes are what pays for a
mistake. Run ``--sim`` first, every time.

Layer
-----
The scenario drives the *motors*, not ``sky.axis`` / ``sky.combiner``. The trace
is evidence about the two serial protocols and their drivers, while the axis
layer inserts a command queue and a background thread between the scenario and
the wire — the recorded byte order would then depend on thread scheduling. The
axis-level concepts the scenario must cover (sync during tracking, guide pulses,
halt) are reproduced here as the motor call sequences the axis performs.

Safety
------
Every exit path — normal end, exception, Ctrl+C — goes through :func:`_shutdown`,
which stops both axes *before* dropping either port, reopening a port that an
error path has already closed if that is what it takes to deliver the stop.

Usage::

    PYTHONPATH=src .venv/bin/python -m tools.hw_session --sim
    PYTHONPATH=src .venv/bin/python -m tools.hw_session --ra-baud 115200
"""

import argparse
import dataclasses
import datetime
import json
import logging
import sys
from collections.abc import Callable, Sequence
from pathlib import Path
from typing import Any

from clock import REAL_CLOCK, Clock
from logging_setup import setup_logging
from serial_wrapper.recorder import (
    DEFAULT_FLUSH_KINDS,
    JsonlRecorder,
    Recorder,
    TraceEvent,
    TraceKind,
)
from serial_wrapper.wrapper import (
    SerialLine,
    SerialLineSearchError,
    SerialLineState,
)
from sim import Clock as VirtualClock
from sim import SimSerialLine, SkyWatcherSim, TMC2209Sim
from sky.constants import STELLAR_SPEED
from sky.motor import MotionMode, MotorDirection, MotorStatus
from sky.physics import DecPerSecond, Ha
from skywatcher.motor import SkyWatcherMotor
from skywatcher.protocol import Protocol
from tmc2209.motor import TMC2209Motor

LOGGER = logging.getLogger("hw_session")

# Defaults taken from the existing entry points, so a run without flags records
# what production actually does: src/__main__.py:46 for the ports and the RA
# baud, src/tests/hw/test_1_tmc2209_motor_hw.py:11 for the DEC pattern.
RA_DEVICE_PATTERN = "PL2303G"
DEC_DEVICE_PATTERN = r"^tty\.usbserial.*$"
RA_BAUD = 115200
DEC_BAUD = 115200
RA_TIMEOUT_S = 0.2
DEC_TIMEOUT_S = 2.0
RA_TERMINATOR = "\r"
DEC_TERMINATOR = "\n"
RA_NAME = "ra"
DEC_NAME = "dec"

# Scenario shape. Kept as module constants rather than CLI options: two traces
# are only comparable if the scenario between them is byte-for-byte the same
# decision sequence, and the one knob that legitimately varies (baud) is a flag.
TRACK_SAMPLES = 4
TRACK_DWELL_S = 3.0
# Below `SkyWatcherMotor._LOWSPEED_MARGIN` (10 min of RA) the driver picks the
# low-speed goto rate, above it the high-speed one; the pair covers both.
GOTO_LOWSPEED_HA = Ha(300)
GOTO_HIGHSPEED_HA = Ha(1800)
DEC_GOTO_SLOW_STEPS = 800
DEC_GOTO_SLOW_SPS = 400
DEC_GOTO_FAST_STEPS = 6000
DEC_GOTO_FAST_SPS = 3000
DEC_ACCEL_SPS2 = 1000
# Time to let an axis actually leave standstill before it is sampled or waited
# on; both controllers report "not moving" for the first instants after `run`.
MOTION_SETTLE_S = 0.3
# Guide rate as a fraction of sidereal, the usual LX200 0.5x.
GUIDE_RATE = 0.5
# Sidereal rate expressed in DEC units (arcseconds of arc per second of time);
# `STELLAR_SPEED` is in RA units (seconds of hour angle per second) and would be
# off by a factor of 15 on this axis.
SIDEREAL_ARCSEC_PER_S = 15.0410686
GUIDE_PULSE_S = 1.0
SYNC_DWELL_S = 2.0
HALT_DWELL_S = 2.0
STATUS_POLLS = 3
# Longer than `SkyWatcherMotor._POWER_CACHE_TTL_S`, otherwise every poll after
# the first is answered from the driver cache and never reaches the wire.
STATUS_POLL_INTERVAL_S = 2.5

STOP_TIMEOUT_S = 15.0
GOTO_TIMEOUT_S = 90.0
SHUTDOWN_STOP_TIMEOUT_S = 10.0
SHUTDOWN_STOP_ATTEMPTS = 3


class SessionError(Exception):
    pass


class SessionSetupError(SessionError):
    """The session could not be built: no device, bad arguments."""


class SessionProtocolError(SessionError):
    """A board answered something the manifest cannot be built from."""


@dataclasses.dataclass(frozen=True)
class LineConfig:
    name: str
    port: str
    baud: int
    timeout_s: float
    terminator: str

    def as_manifest(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "port": self.port,
            "baud": self.baud,
            "timeout_s": self.timeout_s,
            "terminator": self.terminator.encode("ascii").hex(),
        }


@dataclasses.dataclass(frozen=True)
class SessionConfig:
    mode: str
    session_id: str
    out_dir: Path
    ra: LineConfig
    dec: LineConfig


@dataclasses.dataclass
class Session:
    config: SessionConfig
    clock: Clock
    recorder: Recorder
    ra_line: SerialLine
    dec_line: SerialLine
    ra_motor: SkyWatcherMotor
    dec_motor: TMC2209Motor
    manifest: dict[str, Any] = dataclasses.field(default_factory=dict)
    # Present only in --sim mode; the tests assert on them to prove the axes are
    # really stopped, which no amount of manifest reading can show.
    ra_device: SkyWatcherSim | None = None
    dec_device: TMC2209Sim | None = None


@dataclasses.dataclass(frozen=True)
class Step:
    name: str
    run: Callable[[Session], None]


# --------------------------------------------------------------------------- #
# trace markers
# --------------------------------------------------------------------------- #


def mark(session: Session, step: str, phase: str, **detail: Any) -> None:
    """Write a byte-less ``mark`` event so a trace reader can segment the run.

    ``port`` is empty because a marker belongs to the session, not to one line:
    both the RA and the DEC frames that follow it are inside the same step.
    """
    session.recorder.record(
        TraceEvent(
            at_s=session.clock.monotonic(),
            name="session",
            port="",
            kind=TraceKind.MARK,
            detail={"step": step, "phase": phase, **detail},
        )
    )


# --------------------------------------------------------------------------- #
# raw board queries (manifest only)
# --------------------------------------------------------------------------- #


def _decode_revu24(body: str) -> int:
    """Decode a byte-reversed hex value as the SkyWatcher boards send it.

    Deliberately not reusing ``skywatcher.motor._Revu24``: the manifest must say
    what the *board* answered, so a later refactor of the driver cannot quietly
    rewrite the reference it is supposed to be checked against.
    """
    if len(body) == 2:
        body = f"{body}0000"
    if len(body) != 6:
        raise SessionProtocolError(f"expected 6 hex chars from the board, got {body!r}")
    try:
        return int(body[4:6] + body[2:4] + body[0:2], 16)
    except ValueError as exc:
        raise SessionProtocolError(f"board answered non-hex data: {body!r}") from exc


def _ra_inquire(line: SerialLine, command: str) -> str:
    response = line.query(
        f"{Protocol.COMMAND_PREFIX}{command}1{Protocol.COMMAND_TERMINATOR}",
        response_prefixes=(Protocol.RESPONSE_PREFIX_BYTE, Protocol.COMMAND_ERROR_PREFIX_BYTE),
    )
    if not response or not response.endswith(Protocol.ANSWER_END):
        raise SessionProtocolError(f"RA board did not answer :{command}1 — got {response!r}")
    if response[0] == Protocol.COMMAND_ERROR_PREFIX:
        raise SessionProtocolError(f"RA board rejected :{command}1 with {response!r}")
    return response[1:-len(Protocol.ANSWER_END)]


def _parse_tmc_line(raw: str) -> tuple[bool | None, dict[str, str]]:
    """Split a ``1;key=value;`` / ``0;error=...;`` reply without judging it.

    Returns ``(None, {})`` for anything unrecognised instead of raising: an old
    or wedged controller answering garbage is a fact worth recording, not a
    reason to abort a session that is otherwise fine.
    """
    tokens = [token for token in raw.strip().split(";") if token]
    if not tokens or tokens[0] not in {"0", "1"}:
        return None, {}
    values: dict[str, str] = {}
    for token in tokens[1:]:
        key, separator, value = token.partition("=")
        if separator and key:
            values[key] = value
    return tokens[0] == "1", values


def _dec_inquire(line: SerialLine, command: str) -> dict[str, Any]:
    raw = line.query(f"{command}\n")
    ok, values = _parse_tmc_line(raw)
    return {"command": command, "raw": raw.strip(), "ok": ok, "values": values}


# --------------------------------------------------------------------------- #
# scenario helpers
# --------------------------------------------------------------------------- #


def _status_row(session: Session, status: MotorStatus) -> dict[str, Any]:
    return {
        "at_s": session.clock.monotonic(),
        "steps": status.steps,
        "motion_mode": str(status.motion_mode),
        "direction": str(status.direction),
        "speed_sps": status.speed_sps,
        "target": status.target,
    }


def _sample(session: Session, motor: SkyWatcherMotor | TMC2209Motor) -> dict[str, Any]:
    """Snapshot a motor *now*.

    The row has to be built at the moment of the poll, not when the step
    assembles its manifest entry: ``at_s`` is read from the clock, so a row
    built later would stamp every sample of a move with its end time.
    """
    return _status_row(session, motor.status())


def _ra_sidereal_sps(session: Session) -> int:
    return max(1, session.ra_motor.convert_speed_to_steps_per_second(STELLAR_SPEED))


def _ra_enter_tracking(session: Session) -> int:
    """The motor-level form of "the axis is tracking".

    Idempotent on purpose — every step that needs tracking calls it, including
    the ones that just halted the axis, so the scenario never depends on which
    state the previous step happened to leave behind.
    """
    motor = session.ra_motor
    motor.wait_till_stop(do_stop=True, timeout_s=STOP_TIMEOUT_S)
    speed_sps = _ra_sidereal_sps(session)
    motor.set_motion_mode(MotionMode.RUN)
    applied_sps = motor.set_speed(speed_sps)
    motor.set_direction(MotorDirection.FORWARD)
    motor.run()
    return applied_sps


def _ra_goto(session: Session, label: str, offset: Ha) -> None:
    motor = session.ra_motor
    motor.wait_till_stop(do_stop=True, timeout_s=STOP_TIMEOUT_S)
    start = _sample(session, motor)
    delta_steps = motor.convert_position_to_steps(offset)
    motor.set_delta(delta_steps)
    motor.run()
    session.clock.sleep(MOTION_SETTLE_S)
    # Sampled while the axis is still moving: once it stops, the board is back
    # in tracking mode and the speed-mode flag no longer describes the goto.
    moving = _sample(session, motor)
    speed_mode = motor.protocol_monitor()["speed_mode"]
    motor.wait_till_stop(do_stop=False, timeout_s=GOTO_TIMEOUT_S)
    session.manifest["goto"].append(
        {
            "axis": "ra",
            "label": label,
            "delta_steps": delta_steps,
            "start": start,
            "moving": moving,
            "end": _sample(session, motor),
            "speed_mode": str(speed_mode),
        }
    )


def _dec_goto(session: Session, label: str, delta_steps: int, speed_sps: int) -> None:
    motor = session.dec_motor
    motor.wait_till_stop(do_stop=True, timeout_s=STOP_TIMEOUT_S)
    start = _sample(session, motor)
    motor.set_motion_mode(MotionMode.TARGET)
    motor.set_acceleration(DEC_ACCEL_SPS2)
    motor.set_speed(speed_sps)
    motor.set_delta(delta_steps)
    motor.run()
    # Without this the controller is still in `hold` (the acceleration ramp has
    # not produced a step yet) and `wait_till_stop` would report the move
    # finished before it began.
    session.clock.sleep(MOTION_SETTLE_S)
    moving = _sample(session, motor)
    motor.wait_till_stop(do_stop=False, timeout_s=GOTO_TIMEOUT_S)
    session.manifest["goto"].append(
        {
            "axis": "dec",
            "label": label,
            "delta_steps": delta_steps,
            "speed_sps": speed_sps,
            "start": start,
            "moving": moving,
            "end": _sample(session, motor),
        }
    )


# --------------------------------------------------------------------------- #
# scenario steps
# --------------------------------------------------------------------------- #


def _step_connect(session: Session) -> None:
    session.ra_motor.connect()
    session.dec_motor.connect()


def _step_read_board_config(session: Session) -> None:
    line = session.ra_line
    raw_version = _ra_inquire(line, "e")
    # Same byte shuffle as SkyWatcherMotor.connect: the board sends the version
    # with its outer bytes swapped relative to the Revu24 order.
    revu = _decode_revu24(raw_version)
    version = ((revu & 0xFF) << 16) | (revu & 0xFF00) | ((revu & 0xFF0000) >> 16)
    power_v = session.ra_motor.get_power_v()
    session.manifest["ra_board"] = {
        "raw_version": raw_version,
        "board_version": f"0x{version >> 8:04X}",
        "mount_code": version & 0xFF,
        "cpr": _decode_revu24(_ra_inquire(line, "a")),
        "timer_freq": _decode_revu24(_ra_inquire(line, "b")),
        "highspeed_ratio": _decode_revu24(_ra_inquire(line, "g")),
        # The RA board has no supply-voltage query at all (RA_PROTOCOL.md §6),
        # so None is the only honest answer here, not a failure of the run.
        "power_v": power_v,
        "voltage_supported": power_v is not None,
    }

    status = _dec_inquire(session.dec_line, "status")
    full_status = _dec_inquire(session.dec_line, "full_status")
    session.manifest["dec_board"] = {
        "status": status,
        "full_status": full_status,
        "full_status_supported": bool(full_status["ok"]),
        # Firmware older than the TX-ring fix does not report it at all.
        "tx_overflow": _optional_int(status["values"].get("tx_overflow")),
        "microsteps": session.dec_motor.status().microsteps,
        "power_v": session.dec_motor.get_power_v(),
    }


def _step_track_sidereal(session: Session) -> None:
    applied_sps = _ra_enter_tracking(session)
    samples = []
    for _ in range(TRACK_SAMPLES):
        session.clock.sleep(TRACK_DWELL_S)
        samples.append(_sample(session, session.ra_motor))
    session.manifest["tracking"] = {
        "requested_sps": _ra_sidereal_sps(session),
        "applied_sps": applied_sps,
        "samples": samples,
    }


def _step_goto_ra_lowspeed_forward(session: Session) -> None:
    _ra_goto(session, "ra_lowspeed_forward", GOTO_LOWSPEED_HA)


def _step_goto_ra_lowspeed_backward(session: Session) -> None:
    _ra_goto(session, "ra_lowspeed_backward", -GOTO_LOWSPEED_HA)


def _step_goto_ra_highspeed_forward(session: Session) -> None:
    _ra_goto(session, "ra_highspeed_forward", GOTO_HIGHSPEED_HA)


def _step_goto_ra_highspeed_backward(session: Session) -> None:
    _ra_goto(session, "ra_highspeed_backward", -GOTO_HIGHSPEED_HA)


def _step_goto_dec_slow(session: Session) -> None:
    _dec_goto(session, "dec_slow_forward", DEC_GOTO_SLOW_STEPS, DEC_GOTO_SLOW_SPS)
    _dec_goto(session, "dec_slow_backward", -DEC_GOTO_SLOW_STEPS, DEC_GOTO_SLOW_SPS)


def _step_goto_dec_fast(session: Session) -> None:
    _dec_goto(session, "dec_fast_forward", DEC_GOTO_FAST_STEPS, DEC_GOTO_FAST_SPS)
    _dec_goto(session, "dec_fast_backward", -DEC_GOTO_FAST_STEPS, DEC_GOTO_FAST_SPS)


def _step_sync_while_tracking(session: Session) -> None:
    """П6 regression: a sync arriving during tracking must leave it tracking.

    The historical bug left the motor in ``idle`` while the axis still believed
    it was tracking (PLAN.md §1 П6). At motor level a sync is exactly this
    sequence, so a trace that ends with ``motion_mode != run`` reproduces it.
    """
    motor = session.ra_motor
    _ra_enter_tracking(session)
    session.clock.sleep(SYNC_DWELL_S)
    before = _sample(session, motor)

    motor.wait_till_stop(do_stop=True, timeout_s=STOP_TIMEOUT_S)
    motor.set_steps(0)
    _ra_enter_tracking(session)
    session.clock.sleep(SYNC_DWELL_S)
    after_status = motor.status()
    after = _status_row(session, after_status)

    resumed = after_status.motion_mode is MotionMode.RUN and after_status.direction is MotorDirection.FORWARD
    session.manifest["sync"] = {"before": before, "after": after}
    session.manifest["checks"]["sync_resumed_tracking"] = resumed
    if not resumed:
        LOGGER.error("P6 REGRESSION: after sync the RA axis is %s, not tracking", after_status.motion_mode)


def _step_guide_pulses(session: Session) -> None:
    motor = session.ra_motor
    sidereal_sps = _ra_sidereal_sps(session)
    _ra_enter_tracking(session)
    pulses = []
    for label, factor in (("ra_west", 1.0 + GUIDE_RATE), ("ra_east", 1.0 - GUIDE_RATE)):
        # A guide pulse in RA is a tracking-rate nudge, not a separate move: the
        # axis keeps running and only its step period changes.
        applied_sps = motor.set_speed(max(1, round(sidereal_sps * factor)))
        start = _sample(session, motor)
        session.clock.sleep(GUIDE_PULSE_S)
        end = _sample(session, motor)
        motor.set_speed(sidereal_sps)
        pulses.append(
            {
                "axis": "ra",
                "label": label,
                "applied_sps": applied_sps,
                "start": start,
                "end": end,
            }
        )

    dec = session.dec_motor
    guide_sps = max(1, dec.convert_speed_to_steps_per_second(DecPerSecond(GUIDE_RATE * SIDEREAL_ARCSEC_PER_S)))
    for label, direction in (("dec_north", MotorDirection.FORWARD), ("dec_south", MotorDirection.BACKWARD)):
        dec.wait_till_stop(do_stop=True, timeout_s=STOP_TIMEOUT_S)
        dec.set_motion_mode(MotionMode.RUN)
        dec.set_acceleration(DEC_ACCEL_SPS2)
        dec.set_speed(guide_sps)
        dec.set_direction(direction)
        start = _sample(session, dec)
        dec.run()
        session.clock.sleep(GUIDE_PULSE_S)
        dec.wait_till_stop(do_stop=True, timeout_s=STOP_TIMEOUT_S)
        pulses.append(
            {
                "axis": "dec",
                "label": label,
                "applied_sps": guide_sps,
                "start": start,
                "end": _sample(session, dec),
            }
        )
    session.manifest["guide"] = pulses


def _step_halt_and_resume_tracking(session: Session) -> None:
    motor = session.ra_motor
    _ra_enter_tracking(session)
    session.clock.sleep(HALT_DWELL_S)
    moving = _sample(session, motor)

    motor.wait_till_stop(do_stop=True, timeout_s=STOP_TIMEOUT_S)
    session.dec_motor.wait_till_stop(do_stop=True, timeout_s=STOP_TIMEOUT_S)
    halted_status = motor.status()
    halted = _status_row(session, halted_status)

    _ra_enter_tracking(session)
    session.clock.sleep(HALT_DWELL_S)
    resumed_status = motor.status()
    resumed = _status_row(session, resumed_status)

    session.manifest["halt"] = {"moving": moving, "halted": halted, "resumed": resumed}
    session.manifest["checks"]["halt_stops_axis"] = halted_status.motion_mode is MotionMode.IDLE
    session.manifest["checks"]["tracking_resumed_after_halt"] = resumed_status.motion_mode is MotionMode.RUN


def _step_poll_status_and_voltage(session: Session) -> None:
    polls = []
    for _ in range(STATUS_POLLS):
        polls.append(
            {
                "ra": _sample(session, session.ra_motor),
                "ra_power_v": session.ra_motor.get_power_v(),
                "dec": _sample(session, session.dec_motor),
                "dec_power_v": session.dec_motor.get_power_v(),
            }
        )
        session.clock.sleep(STATUS_POLL_INTERVAL_S)
    session.manifest["status_polls"] = polls


def _step_dec_status_and_full_status(session: Session) -> None:
    line = session.dec_line
    status = _dec_inquire(line, "status")
    full_status = _dec_inquire(line, "full_status")
    # A second reading of the same two commands, kept separate from the one in
    # `dec_board`: `tx_overflow` is a counter, so the pair is what says whether
    # the controller dropped anything *during* the session.
    session.manifest["dec_final"] = {
        "status": status,
        "full_status": full_status,
        "tx_overflow": _optional_int(status["values"].get("tx_overflow")),
    }


SCENARIO: tuple[Step, ...] = (
    Step("connect", _step_connect),
    Step("read_board_config", _step_read_board_config),
    Step("track_sidereal", _step_track_sidereal),
    Step("goto_ra_lowspeed_forward", _step_goto_ra_lowspeed_forward),
    Step("goto_ra_lowspeed_backward", _step_goto_ra_lowspeed_backward),
    Step("goto_ra_highspeed_forward", _step_goto_ra_highspeed_forward),
    Step("goto_ra_highspeed_backward", _step_goto_ra_highspeed_backward),
    Step("goto_dec_slow", _step_goto_dec_slow),
    Step("goto_dec_fast", _step_goto_dec_fast),
    Step("sync_while_tracking", _step_sync_while_tracking),
    Step("guide_pulses", _step_guide_pulses),
    Step("halt_and_resume_tracking", _step_halt_and_resume_tracking),
    Step("poll_status_and_voltage", _step_poll_status_and_voltage),
    Step("dec_status_and_full_status", _step_dec_status_and_full_status),
)

SCENARIO_STEP_NAMES: tuple[str, ...] = tuple(step.name for step in SCENARIO)


# --------------------------------------------------------------------------- #
# shutdown
# --------------------------------------------------------------------------- #


def _force_stop(session: Session, axis: str, motor: SkyWatcherMotor | TMC2209Motor, line: SerialLine) -> None:
    """Stop one axis, no matter what the session did on its way here.

    Reopening the port is not paranoia: ``SerialLine`` closes itself on every
    I/O error, so by the time this runs the only handle to a possibly still
    slewing axis may already be gone — and a closed port stops nothing.
    """
    for attempt in range(SHUTDOWN_STOP_ATTEMPTS):
        try:
            if line.state is not SerialLineState.OPEN:
                line.connect()
            motor.wait_till_stop(do_stop=True, timeout_s=SHUTDOWN_STOP_TIMEOUT_S)
        # BaseException on purpose: a second Ctrl+C landing on the stop itself
        # must cost one attempt, not the whole shutdown.
        except BaseException as exc:
            LOGGER.exception("Could not stop the %s axis (attempt %d/%d)", axis, attempt + 1, SHUTDOWN_STOP_ATTEMPTS)
            mark(
                session,
                "shutdown",
                "stop_failed",
                axis=axis,
                attempt=attempt + 1,
                error_type=type(exc).__name__,
                error_message=str(exc),
            )
        else:
            mark(session, "shutdown", "stopped", axis=axis, attempt=attempt + 1)
            return
    LOGGER.critical("THE %s AXIS MAY STILL BE MOVING: every stop attempt failed, cut the power", axis.upper())
    mark(session, "shutdown", "stop_gave_up", axis=axis)
    session.manifest["checks"][f"{axis}_stopped"] = False


def _shutdown(session: Session) -> None:
    """Stop both axes first, drop both ports second. Never raises."""
    mark(session, "shutdown", "begin")
    session.manifest["checks"].setdefault("ra_stopped", True)
    session.manifest["checks"].setdefault("dec_stopped", True)
    _force_stop(session, "ra", session.ra_motor, session.ra_line)
    _force_stop(session, "dec", session.dec_motor, session.dec_line)
    for axis, motor in (("ra", session.ra_motor), ("dec", session.dec_motor)):
        try:
            motor.disconnect()
        except BaseException as exc:  # the port is being dropped anyway
            LOGGER.exception("Could not disconnect the %s axis cleanly", axis)
            mark(session, "shutdown", "disconnect_failed", axis=axis, error_type=type(exc).__name__, error_message=str(exc))
    mark(session, "shutdown", "end")


# --------------------------------------------------------------------------- #
# session assembly and run
# --------------------------------------------------------------------------- #


def open_session(config: SessionConfig, recorder: Recorder) -> Session:
    if config.mode == "sim":
        return _open_sim_session(config, recorder)
    return _open_hw_session(config, recorder)


def _open_sim_session(config: SessionConfig, recorder: Recorder) -> Session:
    # One clock for both devices and both lines: two clocks would let the RA
    # axis integrate over a different amount of time than the DEC one and make
    # the single `at_s` timeline of the trace a lie.
    clock = VirtualClock()
    ra_device = SkyWatcherSim(clock)
    dec_device = TMC2209Sim(clock)
    ra_line = SimSerialLine(
        ra_device,
        clock,
        port=config.ra.port,
        baud=config.ra.baud,
        timeout_s=config.ra.timeout_s,
        name=config.ra.name,
        terminator=config.ra.terminator,
        recorder=recorder,
    )
    dec_line = SimSerialLine(
        dec_device,
        clock,
        port=config.dec.port,
        baud=config.dec.baud,
        timeout_s=config.dec.timeout_s,
        name=config.dec.name,
        terminator=config.dec.terminator,
        recorder=recorder,
    )
    return Session(
        config=config,
        clock=clock,
        recorder=recorder,
        ra_line=ra_line,
        dec_line=dec_line,
        ra_motor=SkyWatcherMotor(ra_line, clock),
        dec_motor=TMC2209Motor(dec_line, clock),
        ra_device=ra_device,
        dec_device=dec_device,
    )


def _open_hw_session(config: SessionConfig, recorder: Recorder) -> Session:
    ra_line = SerialLine(
        port=config.ra.port,
        baud=config.ra.baud,
        timeout_s=config.ra.timeout_s,
        name=config.ra.name,
        terminator=config.ra.terminator,
        clock=REAL_CLOCK,
        recorder=recorder,
    )
    dec_line = SerialLine(
        port=config.dec.port,
        baud=config.dec.baud,
        timeout_s=config.dec.timeout_s,
        name=config.dec.name,
        terminator=config.dec.terminator,
        clock=REAL_CLOCK,
        recorder=recorder,
    )
    return Session(
        config=config,
        clock=REAL_CLOCK,
        recorder=recorder,
        ra_line=ra_line,
        dec_line=dec_line,
        ra_motor=SkyWatcherMotor(ra_line, REAL_CLOCK),
        dec_motor=TMC2209Motor(dec_line, REAL_CLOCK),
    )


def init_manifest(session: Session) -> dict[str, Any]:
    manifest = session.manifest
    manifest.update(
        {
            "session_id": session.config.session_id,
            "mode": session.config.mode,
            "started_at": _utc_now(),
            "finished_at": None,
            "code": git_revision(Path(__file__).resolve()),
            "lines": {"ra": session.config.ra.as_manifest(), "dec": session.config.dec.as_manifest()},
            "scenario": list(SCENARIO_STEP_NAMES),
            "steps": [],
            "goto": [],
            "checks": {},
            "error": None,
        }
    )
    return manifest


def run_scenario(session: Session, steps: Sequence[Step] | None = None) -> dict[str, Any]:
    """Run the scenario end to end; stop and release the mount whatever happens.

    The recorder is *not* closed here: it belongs to the caller, and the
    shutdown markers written from the ``finally`` block have to reach it.
    """
    scenario = SCENARIO if steps is None else tuple(steps)
    manifest = session.manifest
    if "checks" not in manifest:
        init_manifest(session)
    try:
        for index, step in enumerate(scenario):
            LOGGER.info("step %d/%d: %s", index + 1, len(scenario), step.name)
            mark(session, step.name, "begin", index=index)
            started_at = session.clock.monotonic()
            try:
                step.run(session)
            except BaseException as exc:  # includes KeyboardInterrupt: the mount must still be stopped
                mark(session, step.name, "failed", index=index, error_type=type(exc).__name__, error_message=str(exc))
                manifest["steps"].append(
                    {
                        "name": step.name,
                        "at_s": started_at,
                        "duration_s": session.clock.monotonic() - started_at,
                        "status": "failed",
                    }
                )
                manifest["error"] = {"step": step.name, "type": type(exc).__name__, "message": str(exc)}
                raise
            mark(session, step.name, "end", index=index)
            manifest["steps"].append(
                {
                    "name": step.name,
                    "at_s": started_at,
                    "duration_s": session.clock.monotonic() - started_at,
                    "status": "ok",
                }
            )
    finally:
        _shutdown(session)
        manifest["finished_at"] = _utc_now()
    return manifest


# --------------------------------------------------------------------------- #
# metadata
# --------------------------------------------------------------------------- #


def _utc_now() -> str:
    return datetime.datetime.now(datetime.UTC).isoformat()


def _optional_int(value: str | None) -> int | None:
    if value is None:
        return None
    try:
        return int(value)
    except ValueError:
        return None


def git_revision(start: Path) -> dict[str, Any]:
    """Resolve HEAD by reading ``.git`` directly, without spawning ``git``.

    ``subprocess`` would be the obvious way, but ruff's S603/S607 flag it and
    the project forbids inline ``# noqa``; reading two files is also faster and
    works when no ``git`` binary is on PATH.
    """
    try:
        git_dir = _find_git_dir(start)
        if git_dir is None:
            return {"commit": None, "ref": None}
        head = (git_dir / "HEAD").read_text(encoding="utf-8").strip()
        if not head.startswith("ref: "):
            return {"commit": head, "ref": None}
        ref = head.removeprefix("ref: ").strip()
        return {"commit": _resolve_ref(git_dir, ref), "ref": ref}
    except OSError:
        return {"commit": None, "ref": None}


def _find_git_dir(start: Path) -> Path | None:
    for directory in (start, *start.parents):
        candidate = directory / ".git"
        if candidate.is_dir():
            return candidate
        if candidate.is_file():
            # Worktree / submodule: the file holds `gitdir: <path>`.
            target = candidate.read_text(encoding="utf-8").strip().removeprefix("gitdir:").strip()
            resolved = (candidate.parent / target).resolve()
            return resolved if resolved.is_dir() else None
    return None


def _resolve_ref(git_dir: Path, ref: str) -> str | None:
    loose = git_dir / ref
    if loose.is_file():
        return loose.read_text(encoding="utf-8").strip()
    packed = git_dir / "packed-refs"
    if packed.is_file():
        for line in packed.read_text(encoding="utf-8").splitlines():
            if not line or line.startswith(("#", "^")):
                continue
            commit, _, name = line.partition(" ")
            if name.strip() == ref:
                return commit.strip()
    return None


# --------------------------------------------------------------------------- #
# CLI
# --------------------------------------------------------------------------- #


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="tools.hw_session",
        description="Record one fixed mount session: byte trace + board manifest.",
    )
    parser.add_argument("--sim", action="store_true", help="run against src/sim instead of the hardware")
    parser.add_argument("--ra-port", default=None, help="RA serial port; skips the /dev search")
    parser.add_argument("--dec-port", default=None, help="DEC serial port; skips the /dev search")
    parser.add_argument("--ra-pattern", default=RA_DEVICE_PATTERN, help="regex for the RA device in /dev")
    parser.add_argument("--dec-pattern", default=DEC_DEVICE_PATTERN, help="regex for the DEC device in /dev")
    parser.add_argument("--ra-baud", type=int, default=RA_BAUD, help="RA baud; 112500 vs 115200 is the open question")
    parser.add_argument("--dec-baud", type=int, default=DEC_BAUD, help="DEC baud")
    parser.add_argument("--ra-timeout", type=float, default=RA_TIMEOUT_S, help="RA read timeout, seconds")
    parser.add_argument("--dec-timeout", type=float, default=DEC_TIMEOUT_S, help="DEC read timeout, seconds")
    parser.add_argument("--out-dir", default="logs/sessions", help="directory for the trace and the manifest")
    parser.add_argument("--session-id", default=None, help="basename of both artifacts; defaults to a UTC stamp")
    parser.add_argument("--logs-root", default="logs", help="root of the per-run text log directories")
    return parser


def _resolve_port(axis: str, explicit: str | None, pattern: str) -> str:
    if explicit:
        return explicit
    try:
        return SerialLine.search(pattern)
    except SerialLineSearchError as exc:
        raise SessionSetupError(
            f"{axis.upper()} device not found: nothing in /dev matched the pattern {pattern!r} ({exc}). "
            f"Plug the board in, or name the port with --{axis}-port."
        ) from exc


def config_from_args(args: argparse.Namespace) -> SessionConfig:
    mode = "sim" if args.sim else "hw"
    if mode == "sim":
        ra_port, dec_port = "sim://ra", "sim://dec"
    else:
        ra_port = _resolve_port("ra", args.ra_port, args.ra_pattern)
        dec_port = _resolve_port("dec", args.dec_port, args.dec_pattern)
    session_id = args.session_id or f"{mode}-{datetime.datetime.now(datetime.UTC).strftime('%Y%m%dT%H%M%SZ')}"
    return SessionConfig(
        mode=mode,
        session_id=session_id,
        out_dir=Path(args.out_dir),
        ra=LineConfig(RA_NAME, ra_port, args.ra_baud, args.ra_timeout, RA_TERMINATOR),
        dec=LineConfig(DEC_NAME, dec_port, args.dec_baud, args.dec_timeout, DEC_TERMINATOR),
    )


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    setup_logging(logs_root=args.logs_root)
    config = config_from_args(args)
    config.out_dir.mkdir(parents=True, exist_ok=True)
    trace_path = config.out_dir / f"{config.session_id}.jsonl"
    manifest_path = config.out_dir / f"{config.session_id}.manifest.json"

    # Markers are flushed like the session-ending events: a killed run must still
    # show which step it died in.
    recorder = JsonlRecorder(trace_path, flush_kinds=DEFAULT_FLUSH_KINDS | {TraceKind.MARK})
    session = open_session(config, recorder)
    init_manifest(session)
    session.manifest["trace_path"] = str(trace_path)

    exit_code = 0
    try:
        run_scenario(session)
    except BaseException as exc:  # the mount is already stopped by run_scenario's finally
        LOGGER.exception("Session aborted")
        session.manifest.setdefault("error", {})
        if not session.manifest["error"]:
            session.manifest["error"] = {"step": None, "type": type(exc).__name__, "message": str(exc)}
        exit_code = 1
    finally:
        manifest_path.write_text(
            json.dumps(session.manifest, indent=2, ensure_ascii=False, default=str) + "\n",
            encoding="utf-8",
        )
        recorder.close()

    LOGGER.info("trace:    %s", trace_path)
    LOGGER.info("manifest: %s", manifest_path)
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
