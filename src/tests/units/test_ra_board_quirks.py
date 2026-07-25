"""The RA driver against the quirks of the live board (``docs/protocol/RA_PROTOCOL.md``).

Every test here pins a *conclusion of that document*, not a constant of the
driver: the numbers on the board may differ, the behaviour must not.

- §10.3 `:K1` is a braking ramp: the board answers `=` while the axis still runs.
- §10.5 the board clamps the step period at 100x sidereal, whatever is written.
- §11 `:E` sent on the move is accepted with `=` and drops the initialization
  flag a few hundred milliseconds later.
"""

import pytest

from sim import Clock, SimSerialLine, SkyWatcherSim
from sky.constants import STELLAR_SPEED
from sky.motor import MotionMode, MotorDirection
from sky.physics import Ha
from skywatcher.motor import SkyWatcherMotor, SkyWatcherMotorTimeoutError


def _make_ra(**config: int) -> tuple[Clock, SkyWatcherSim, SkyWatcherMotor]:
    clock = Clock()
    sim = SkyWatcherSim(clock, **config)
    line = SimSerialLine(sim, clock, port="sim://ra", timeout_s=0, name="sim-ra", terminator="\r")
    return clock, sim, SkyWatcherMotor(line, clock)


def _spin_up(clock: Clock, motor: SkyWatcherMotor, times_sidereal: int = 128) -> None:
    """Run the axis at the fastest rate the board allows, for a second."""
    motor.set_speed(motor.convert_speed_to_steps_per_second(STELLAR_SPEED * times_sidereal))
    motor.set_direction(MotorDirection.FORWARD)
    motor.set_motion_mode(MotionMode.RUN)
    motor.run()
    clock.advance(1.0)


class _StuckAxisSerial:
    """Board that acknowledges everything and never clears the Running bit."""

    terminator = b"\r"

    def __init__(self) -> None:
        self.payloads: list[str] = []

    def connect(self) -> None:
        pass

    def close(self) -> None:
        pass

    def query(
        self,
        payload: str | None,
        timeout: float | None = None,
        response_prefixes: tuple[bytes, ...] | None = None,
        response_terminator: bytes | str | None = None,
    ) -> str:
        self.payloads.append(payload or "")
        if payload is not None and payload.startswith(":f"):
            return "=111\r"
        return "=\r"

    def drop_buffers(self) -> None:
        pass

    def read_all_data(self, timeout: float | None = None) -> list[str] | None:
        return None


# ---------------------------------------------------------------------------
# §10.3 -- stop() must not lie about the axis being stopped
# ---------------------------------------------------------------------------


def test_stop_returns_only_after_the_axis_has_really_stopped() -> None:
    """`K` answers `=` mid-ramp; `stop()` may not return on that answer.

    `Axis.disconnect` goes reset -> stop -> disconnect, so a `stop()` that
    returns early closes the serial port on a physically moving mount.
    """
    clock, sim, motor = _make_ra()
    motor.connect()
    _spin_up(clock, motor)
    assert sim.running is True

    started_at = clock.now
    assert motor.stop() is True

    assert sim.running is False, "stop() returned while the axis was still running"
    assert clock.now - started_at >= 0.5, (
        f"stop() returned after {clock.now - started_at:.3f}s of virtual time; the measured "
        f"brake ramp at this speed takes ~0.6s, so nothing was actually waited for"
    )


def test_stopped_axis_makes_stop_cheap() -> None:
    """No ramp, no waiting: the guard must not cost time on an idle axis."""
    clock, sim, motor = _make_ra()
    motor.connect()

    started_at = clock.now
    assert motor.stop() is True

    assert sim.running is False
    assert clock.now - started_at < 0.2


def test_stop_gives_up_with_a_timeout_error_when_running_never_clears() -> None:
    """A board stuck at Running must surface as an error, not as a hang."""
    clock = Clock()
    serial = _StuckAxisSerial()
    motor = SkyWatcherMotor(serial, clock)  # type: ignore[arg-type]

    with pytest.raises(SkyWatcherMotorTimeoutError, match="did not stop"):
        motor.stop()

    assert clock.now >= SkyWatcherMotor._STOP_TIMEOUT_S
    assert serial.payloads[0] == ":K1\r"
    assert serial.payloads.count(":f1\r") > 1, "the driver never re-read the status while waiting"


def test_wait_till_stop_brakes_and_waits_for_the_axis() -> None:
    """`wait_till_stop(do_stop=True)` is the same contract, one call up."""
    clock, sim, motor = _make_ra()
    motor.connect()
    _spin_up(clock, motor)

    started_at = clock.now
    motor.wait_till_stop(do_stop=True, timeout_s=10)

    assert sim.running is False, "wait_till_stop returned while the axis was still running"
    assert clock.now - started_at >= 0.5


def test_wait_till_stop_without_braking_waits_for_the_goto_to_finish() -> None:
    """`do_stop=False` is how a GOTO is awaited: it must not return early."""
    clock, sim, motor = _make_ra()
    motor.connect()

    delta_steps = motor.convert_position_to_steps(Ha(300))
    motor.set_delta(delta_steps)
    motor.run()
    assert sim.running is True

    motor.wait_till_stop(do_stop=False, timeout_s=600)

    assert sim.running is False, "wait_till_stop(do_stop=False) returned mid-GOTO"
    assert motor.status().steps == delta_steps
