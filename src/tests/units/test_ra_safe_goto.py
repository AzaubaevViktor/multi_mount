"""§27: a GOTO the board is never allowed to finish.

The failure this is built around is not a guess. Five times out of five, a GOTO
left to its own devices reached a point 110…225 counts short of its target,
stepped its rate down from the ~4 650 steps/s plateau it had been holding to
whatever ``:I1`` asked for — 4 550 -> 145 steps/s in one control period on the
one run that survived the step — and the controller reset
(``docs/protocol/RA_PROTOCOL_STEP_2.md`` §24). Nothing the host sends moves that
point: ``:I1`` is not obeyed on the cruise, ``:M1`` is inert at every value, and
``:m1`` is always "target - `:c1`", so a GOTO shorter than 16 980 counts has its
brake point behind its own start.

So the driver does the last part itself. These tests pin the three properties
that make that safe, each one against the simulator's
``reboot_on_goto_arrival`` mode — which is the failure, modelled:

1. the board never gets within the danger window of its own target;
2. a move too short for the board's geometry is not given to the board at all;
3. the axis still ends up where it was asked to go.
"""

from typing import Any

import pytest

from sim import Clock, SimSerialLine, SkyWatcherSim
from sky.motor import MotionMode, MotorDirection
from sky.physics import Ha
from skywatcher.motor import SkyWatcherMotor

# Long enough to be a board GOTO several times over: `:c1` is 16 980 counts.
_LONG = Ha(1800)
# 4 337 counts — under `:c1`, i.e. the degenerate geometry of §11.
_SHORT = Ha(30)


def _make_ra(**config: Any) -> tuple[Clock, SkyWatcherSim, SkyWatcherMotor]:
    clock = Clock()
    sim = SkyWatcherSim(clock, **config)
    line = SimSerialLine(sim, clock, port="sim://ra", timeout_s=0, name="sim-ra", terminator="\r")
    return clock, sim, SkyWatcherMotor(line, clock)


def _goto(clock: Clock, motor: SkyWatcherMotor, delta: Ha, budget_s: float = 300.0) -> float:
    """One `set_delta` + `run` + the polling `Axis` does, to the end of the move."""
    motor.set_delta(motor.convert_position_to_steps(delta))
    assert motor.run() is True
    started_at_s = clock.monotonic()
    while clock.monotonic() - started_at_s < budget_s:
        if motor.status().motion_mode != MotionMode.TARGET:
            return clock.monotonic() - started_at_s
        clock.advance(0.5)
    raise AssertionError("the safe GOTO did not finish inside its budget")


def test_a_board_that_dies_on_arrival_never_gets_the_chance() -> None:
    """The whole point, end to end, against the failure itself.

    With ``reboot_on_goto_arrival`` the simulated controller resets the moment a
    GOTO of its own comes within 200 counts of the target — the observed danger
    window (§24.2). A driver that hands the board the move and waits loses the
    position, the initialization flag and the target; this one arrives with all
    three intact.
    """
    clock, sim, motor = _make_ra(reboot_on_goto_arrival=True)
    motor.connect()
    delta_steps = motor.convert_position_to_steps(_LONG)

    _goto(clock, motor, _LONG)

    assert motor.protocol_monitor()["reboots"] == 0, "the board was allowed to finish its own GOTO"
    assert sim.initialized is True
    assert sim.position == pytest.approx(delta_steps, abs=SkyWatcherMotor._GOTO_TOLERANCE_TICKS)


def test_the_board_leg_is_stopped_a_long_way_short_of_the_target() -> None:
    """Where the `:K1` goes, in counts, measured on the wire.

    The margin has to clear the danger window (110…225 counts) and the coast
    that follows a `:K1` from the plateau (831 counts by the board's own measured
    ramp). This pins the fact that the board leg ends inside the margin and
    outside the window — the two-sided condition the number was chosen for.
    """
    clock, sim, motor = _make_ra()
    motor.connect()
    delta_steps = motor.convert_position_to_steps(_LONG)

    braked_at: list[float] = []
    original_stop = motor._session.request_stop

    def _watch_stop() -> None:
        braked_at.append(sim.position)
        original_stop()

    motor._session.request_stop = _watch_stop  # type: ignore[method-assign]

    _goto(clock, motor, _LONG)

    assert braked_at, "the board leg was never braked by the driver"
    remaining_at_brake = delta_steps - braked_at[0]
    assert remaining_at_brake <= SkyWatcherMotor._GOTO_BRAKE_MARGIN_TICKS
    # The margin exists to be big; a `:K1` sent inside the danger window would
    # be no better than letting the board arrive.
    assert remaining_at_brake > 225, "the driver braked inside the window the board dies in"


def test_a_move_shorter_than_the_brake_distance_never_becomes_a_board_goto() -> None:
    """§11: under `:c1` the brake point lies behind the start, so the board is not asked.

    The evidence on the wire is the motion mode: a move this short must be run
    as a slew (`:G11x`/`:G13x`), never as a goto (`:G10x`/`:G12x`), because a
    goto is the only thing that can arrive.
    """
    clock, sim, motor = _make_ra(reboot_on_goto_arrival=True)
    motor.connect()
    delta_steps = motor.convert_position_to_steps(_SHORT)
    assert delta_steps < sim.brake_steps

    modes: list[str] = []
    original_feed = sim.feed

    def _watch(data: bytes) -> None:
        text = data.decode("ascii", errors="replace")
        if text.startswith(":G1"):
            modes.append(text[3:5])
        original_feed(data)

    sim.feed = _watch  # type: ignore[method-assign]

    _goto(clock, motor, _SHORT)

    assert modes, "no motion mode was ever set"
    assert all(mode[0] in "13" for mode in modes), f"a degenerate GOTO was handed to the board: {modes}"
    assert motor.protocol_monitor()["reboots"] == 0
    assert sim.position == pytest.approx(delta_steps, abs=SkyWatcherMotor._GOTO_TOLERANCE_TICKS)


def test_the_creep_closes_an_overshoot_by_reversing() -> None:
    """A leg that went too far is corrected, not measured again from where it landed.

    The creep aims at the absolute count `run()` worked out, so an axis pushed
    past the target comes back to it. Without this the driver would report the
    move done wherever the overshoot left it.
    """
    clock, sim, motor = _make_ra()
    motor.connect()
    delta_steps = motor.convert_position_to_steps(_LONG)
    motor.set_delta(delta_steps)
    assert motor.run() is True

    # Shove the axis past its target while the board leg is still running, the
    # way a lost `:K1` or a slipping mount would.
    clock.advance(1.0)
    motor.status()
    sim.position = float(delta_steps + 3_000)

    started_at_s = clock.monotonic()
    while clock.monotonic() - started_at_s < 300.0:
        if motor.status().motion_mode != MotionMode.TARGET:
            break
        clock.advance(0.5)

    assert sim.position == pytest.approx(delta_steps, abs=SkyWatcherMotor._GOTO_TOLERANCE_TICKS)
    assert sim.position < delta_steps + 3_000, "the overshoot was accepted instead of being closed"


def test_a_goto_in_progress_locks_out_the_setters_through_the_creep() -> None:
    """The creep runs the board in tracking mode, and that must not read as "idle".

    `sky/axis.py` decides a GOTO is over the moment the motor stops reporting
    `MotionMode.TARGET`, and answers that with a `wait_till_stop` — which would
    cut the creep short every single time. Reporting the *plan* rather than the
    board's raw mode is what keeps the two layers from fighting.
    """
    clock, sim, motor = _make_ra()
    motor.connect()
    motor.set_delta(motor.convert_position_to_steps(_SHORT))
    assert motor.run() is True

    status = motor.status()
    assert sim.tracking_mode is True, "the short move was not run as a slew"
    assert status.motion_mode == MotionMode.TARGET, "a creep leg was reported as something other than a GOTO"

    with pytest.raises(Exception, match="GOTO is in progress"):
        motor.set_direction(MotorDirection.BACKWARD)


def test_stopping_cancels_the_plan_instead_of_letting_it_creep_on() -> None:
    """`stop()` has to mean stopped: a surviving plan would restart the axis."""
    clock, sim, motor = _make_ra()
    motor.connect()
    motor.set_delta(motor.convert_position_to_steps(_LONG))
    assert motor.run() is True
    clock.advance(1.0)

    motor.stop()
    position_after_stop = sim.position

    for _ in range(10):
        motor.status()
        clock.advance(0.5)

    assert sim.running is False
    assert sim.position == pytest.approx(position_after_stop, abs=2)
    assert motor.status().motion_mode != MotionMode.TARGET
