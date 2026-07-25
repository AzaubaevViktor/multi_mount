"""§12: telling a rebooted board from a board that is merely being driven.

The mechanism is the owner's: a command loads the motor, the supply sags, the
controller resets. The reset itself was never observed on the wire — this
session had no brownout (§12: "в этой сессии броунаут не наблюдался") — but its
*signs* are fully observable, and every one of them has a legal explanation on
its own (§12.2):

* the position is exactly `0x800000` — so is any correct `:E1000080`;
* the initialization flag is down — the `:E`-on-the-move quirk of §11 drops it
  without any reboot;
* the step period is back at the 1x tracking value — so it is after the
  driver's own `:I1`.

Only the three together, *and none of them the driver's own doing*, mean a
reboot. That last clause is why the session has to remember what it wrote
(§12.4.4), and it is what these tests pin: the same board state is a reboot or
a normal state of affairs depending on what the driver did before.

The recovery path (`:F1`, position back at a stopped axis, period again) is
exercised here against ``SkyWatcherSim.reboot()`` and **only** there: the board
was disconnected while this was written, so the live-hardware half of it is
stage Э5 of ``docs/RA_REWRITE_PLAN.md``.
"""

import pytest

from sim import Clock, SimSerialLine, SkyWatcherSim
from sky.constants import STELLAR_SPEED
from sky.motor import MotionMode, MotorDirection
from sky.physics import Ha
from skywatcher.codec import Status
from skywatcher.motor import SkyWatcherMotor, SkyWatcherMotorRebootError

_TRACKING_MULTIPLE = 128


def _make_ra() -> tuple[Clock, SkyWatcherSim, SkyWatcherMotor]:
    clock = Clock()
    sim = SkyWatcherSim(clock)
    line = SimSerialLine(sim, clock, port="sim://ra", timeout_s=0, name="sim-ra", terminator="\r")
    return clock, sim, SkyWatcherMotor(line, clock)


def _tracking_motor() -> tuple[Clock, SkyWatcherSim, SkyWatcherMotor]:
    """A connected driver that has moved the axis away from the logical zero.

    Both memories §12.2 needs are filled by doing this and nothing else: the
    step period is no longer the one the board powers up with, and the position
    the driver last saw is not zero.
    """
    clock, sim, motor = _make_ra()
    motor.connect()
    motor.set_speed(motor.convert_speed_to_steps_per_second(STELLAR_SPEED * _TRACKING_MULTIPLE))
    motor.set_direction(MotorDirection.FORWARD)
    motor.set_motion_mode(MotionMode.RUN)
    motor.run()
    clock.advance(10.0)
    motor.stop()
    # The axis layer polls the position constantly, and that poll is where the
    # session learns where the axis stands. Without it the driver's last word on
    # the position is the zero it started from, and §12.4.4 applies.
    motor.status()
    return clock, sim, motor


def test_a_reboot_is_detected_and_the_board_is_put_back(caplog: pytest.LogCaptureFixture) -> None:
    """The whole of §12.2 plus §12.4.2, end to end through the public API.

    Without this the driver carries on from a false zero: the board says the
    axis is at `0x800000`, the axis is really ten seconds of 128x tracking away
    from it, and every position the mount reports from then on is wrong by that
    distance — silently, because every command still answers normally (§12.3).
    """
    clock, sim, motor = _tracking_motor()
    position_before = motor.status().steps
    period_before = sim.step_period
    assert position_before > 0
    assert period_before != sim.tracking_period_1x

    sim.reboot()
    assert sim.position == 0.0
    assert sim.initialized is False
    assert sim.step_period == sim.tracking_period_1x

    status = motor.status()

    assert motor.protocol_monitor()["reboots"] == 1
    assert sim.initialized is True, "the rebooted board was left uninitialized"
    assert sim.position == pytest.approx(position_before), "the position the driver knew was not restored"
    assert sim.step_period == period_before, "the step period was not put back"
    assert status.steps == pytest.approx(position_before)
    assert any("rebooted" in record.message.lower() for record in caplog.records), (
        "a reboot was handled without a word in the log"
    )


def test_a_reboot_cancels_a_pending_motion_instead_of_starting_it() -> None:
    """§12.4.2: "do not carry on moving".

    The GOTO registers do not survive a reboot, so a `:J1` sent after one runs
    the axis to whatever `:H1` reset to. `run()` reports that nothing started.
    """
    clock, sim, motor = _tracking_motor()
    delta_steps = motor.convert_position_to_steps(Ha(1800))
    motor.set_delta(delta_steps)

    sim.reboot()

    assert motor.run() is False
    assert sim.running is False, "the axis was started on a board that had just rebooted"
    assert motor.protocol_monitor()["reboots"] == 1


def test_a_reboot_between_the_status_and_the_start_stops_the_start() -> None:
    """The same rule one layer down, for the gap `run()` cannot see.

    `run()` checks the status first, so a board that reboots *after* that check
    is caught by the position read `:J1` does on its way out (§7). Nothing is
    sent, and the caller is told with an error rather than a `True`.
    """
    clock, sim, motor = _tracking_motor()
    motor.set_delta(motor.convert_position_to_steps(Ha(1800)))

    original_status = motor._session.status
    armed = [True]

    def _status_then_reboot() -> Status:
        status = original_status()
        if armed:
            # Once, right after `run()` has looked at the board and found it
            # healthy — the window the status check cannot cover.
            armed.clear()
            sim.reboot()
        return status

    motor._session.status = _status_then_reboot  # type: ignore[method-assign]

    with pytest.raises(SkyWatcherMotorRebootError, match="motion not started"):
        motor.run()

    assert sim.running is False


def test_a_position_of_zero_the_driver_asked_for_is_not_a_reboot() -> None:
    """§12.2 line 1: `:E1000080` leaves exactly the same counter behind.

    The driver has just put the axis at the logical zero, so the position sign
    carries no information — and acting on the other two would mean re-syncing
    a healthy board to a position it already has, and re-initializing it for no
    reason.
    """
    clock, sim, motor = _tracking_motor()
    motor.set_steps(0)
    period_before = sim.step_period

    # The flag goes down for some other reason: the board is at the logical zero
    # because the driver put it there, and only two signs of three are present.
    sim.initialized = False

    motor.status()

    assert motor.protocol_monitor()["reboots"] == 0
    assert sim.step_period == period_before
    assert sim.initialized is False, "a healthy board was re-initialized on one sign"


def test_a_dropped_flag_alone_is_not_a_reboot() -> None:
    """§12.2 line 2: §11 drops the same flag on a board that never rebooted.

    The axis is nowhere near the logical zero, so whatever took the flag down it
    was not a reset — and nothing may be written to the board on that evidence.
    """
    clock, sim, motor = _tracking_motor()
    position_before = sim.position
    period_before = sim.step_period

    sim.initialized = False

    status = motor.status()

    assert motor.protocol_monitor()["reboots"] == 0
    assert status.initialized is False, "the dropped flag must still reach the layer above"
    assert sim.position == position_before, "the driver wrote a position on one sign of three"
    assert sim.step_period == period_before


def test_a_period_the_driver_never_changed_cannot_be_a_sign() -> None:
    """§12.2 line 4, and §12.4.4 behind it.

    Freshly connected, the driver has not set a step period, so the board
    holding the 1x tracking value is what it was holding all along. The two
    remaining signs are the ones §12.2 calls insufficient, and a driver that
    ruled on them alone would "restore" a board that is simply idle.
    """
    clock, sim, motor = _make_ra()
    motor.connect()
    assert sim.step_period == sim.tracking_period_1x

    sim.initialized = False

    motor.status()

    assert motor.protocol_monitor()["reboots"] == 0
    assert sim.initialized is False


def test_a_reboot_before_anything_was_written_is_reported_not_guessed(
    caplog: pytest.LogCaptureFixture,
) -> None:
    """§12.4.4 stated as a limit, not as a gap in the tests.

    A driver that has written nothing has nothing to compare against, so the
    board's state is not interpretable at all. The honest answer is a line in
    the log — not a restore to a position the driver does not know.
    """
    clock, sim, motor = _make_ra()
    motor.connect()

    sim.reboot()
    motor.status()

    assert motor.protocol_monitor()["reboots"] == 0
    assert sim.initialized is False


def test_the_axis_keeps_reporting_the_right_position_after_a_reboot() -> None:
    """What the whole detection is for: the counter above the driver stays true.

    Tracking is resumed on the restored board and the position keeps growing
    from where it was, instead of from the false zero the reboot left behind.
    """
    clock, sim, motor = _tracking_motor()
    position_before = motor.status().steps

    sim.reboot()
    motor.status()

    motor.set_direction(MotorDirection.FORWARD)
    motor.set_motion_mode(MotionMode.RUN)
    assert motor.run() is True
    clock.advance(10.0)

    assert motor.status().steps > position_before
    assert motor.status().steps == pytest.approx(2 * position_before, rel=0.05)
