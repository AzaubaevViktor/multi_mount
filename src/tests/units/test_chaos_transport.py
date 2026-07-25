"""Tests of the chaos generator itself (``src/sim/chaos.py``).

A chaos suite is only worth its runtime if the chaos is (a) reproducible and
(b) actually applied. Both are properties of this module, not of the drivers,
so they are pinned here: same seed -> byte-identical traffic, different seeds ->
different traffic, a quiet profile -> not a single altered byte, and a loaded
profile -> every fault kind observed at least once.
"""

import errno

import pytest

from serial_wrapper.wrapper import SerialLineState
from sim import Chaos, ChaosProfile, Clock, SimSerialLine, SkyWatcherSim
from sky.motor import MotionMode, MotorDirection
from skywatcher.motor import SkyWatcherMotor

_RA_CPR = 8_000_000

_QUIET = ChaosProfile()
_LOSSY = ChaosProfile(lose_write_byte=0.01, lose_read_byte=0.03, short_write=0.02, short_read=0.05)
_UNPLUGGING = ChaosProfile(lose_read_byte=0.01, unplug=0.01)


def _drive_ra(chaos: Chaos) -> tuple[SimSerialLine, SkyWatcherSim, list[str]]:
    """Run a short RA session under `chaos`, swallowing whatever it breaks."""
    clock = Clock()
    sim = SkyWatcherSim(clock, cpr=_RA_CPR)
    line = SimSerialLine(sim, clock, transport=chaos.transport, port="sim://ra", timeout_s=0, name="chaos-ra", terminator="\r")
    chaos.bind(line)
    motor = SkyWatcherMotor(line, clock)
    outcomes: list[str] = []

    for name, action in (
        ("connect", motor.connect),
        ("speed", lambda: motor.set_speed(100)),
        ("direction", lambda: motor.set_direction(MotorDirection.FORWARD)),
        ("mode", lambda: motor.set_motion_mode(MotionMode.RUN)),
        ("run", motor.run),
        ("status", motor.status),
        ("stop", motor.stop),
    ):
        try:
            action()
        except Exception as error:  # noqa: BLE001 - the point is to classify, not to handle
            outcomes.append(f"{name}:{type(error).__name__}")
        else:
            outcomes.append(f"{name}:ok")
        clock.advance(1)

    return line, sim, outcomes


def test_quiet_profile_leaves_every_byte_alone() -> None:
    """The zero profile is the control group: no bites, and a healthy session."""
    chaos = Chaos(seed=1, profile=_QUIET)

    line, _, outcomes = _drive_ra(chaos)

    assert chaos.stats.bites == 0
    assert chaos.stats.writes > 0
    assert chaos.stats.written_bytes > 0
    assert all(outcome.endswith(":ok") for outcome in outcomes), outcomes
    assert line.state == SerialLineState.OPEN


def test_same_seed_replays_byte_for_byte() -> None:
    """Reproducibility: the seed alone determines the whole run."""
    first = Chaos(seed=20260725, profile=_LOSSY)
    second = Chaos(seed=20260725, profile=_LOSSY)

    _, _, first_outcomes = _drive_ra(first)
    _, _, second_outcomes = _drive_ra(second)

    assert first.trace == second.trace
    assert first.stats == second.stats
    assert first_outcomes == second_outcomes


def test_different_seeds_produce_different_runs() -> None:
    """Guard against a degenerate generator that always damages the same byte."""
    traces = []
    for seed in range(6):
        chaos = Chaos(seed=seed, profile=_LOSSY)
        _drive_ra(chaos)
        traces.append(tuple(chaos.trace))

    assert len(set(traces)) == len(traces)


def test_loaded_profile_applies_every_fault_kind() -> None:
    """Anti-self-deception: a green chaos test needs proof that chaos happened."""
    totals = {"lost_write_bytes": 0, "lost_read_bytes": 0, "short_writes": 0, "short_reads": 0, "unplugs": 0}

    for seed in range(20):
        chaos = Chaos(seed=seed, profile=ChaosProfile(lose_write_byte=0.05, lose_read_byte=0.05, short_write=0.05, short_read=0.05, unplug=0.01))
        _drive_ra(chaos)
        for field in totals:
            totals[field] += getattr(chaos.stats, field)

    assert all(count > 0 for count in totals.values()), totals


def test_unplug_kills_the_live_port_and_the_line_reconnects() -> None:
    chaos = Chaos(seed=3, profile=ChaosProfile(unplug=1.0))
    clock = Clock()
    sim = SkyWatcherSim(clock, cpr=_RA_CPR)
    line = SimSerialLine(sim, clock, transport=chaos.transport, port="sim://ra", timeout_s=0, name="chaos-ra", terminator="\r")
    chaos.bind(line)
    line.connect()

    dead_port = line.serial
    with pytest.raises(OSError) as unplugged:
        line.query(":f1\r")

    assert dead_port is not None
    assert dead_port.is_unplugged is True
    assert unplugged.value.errno == errno.ENXIO
    # Read the state into a local: asserting on `line.state` directly would pin
    # the narrowed type down for the rest of the test, past the reconnect below.
    state_after_unplug = line.state
    assert state_after_unplug == SerialLineState.CLOSED

    # The transport survives the port: a reconnect installs a fresh one and the
    # very same chaos keeps biting it.
    chaos.profile = _QUIET
    line.connect()
    assert line.state == SerialLineState.OPEN
    assert line.serial is not None
    assert line.serial.is_unplugged is False
    assert line.query(":f1\r").endswith("\r")


def test_unplug_without_a_bound_line_is_a_configuration_error() -> None:
    """Silently skipping the fault would be the worst kind of green test."""
    chaos = Chaos(seed=0, profile=ChaosProfile(unplug=1.0))

    with pytest.raises(RuntimeError, match=r"Chaos\.bind"):
        chaos.transport.on_write(b":f1\r")
