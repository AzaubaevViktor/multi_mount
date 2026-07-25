"""Red test for П2 (USB-serial unplug kills the axis silently), PLAN.md §1 П2.

Both healthy real sessions ended with ``OSError: [Errno 6] Device not
configured`` on both axes: the ``_motion_convertor`` threads died, there was no
hot-path reconnect, and the session simply stopped.

Desired contract: a motor operation raising ``OSError(ENXIO)`` mid-work must not
silently crash the axis. The convertor step handles the exception and the axis
reaches an observable "disconnected/degraded" state instead of re-raising into
the thread and leaving an opaque ``_motion_convertor_error``.

Today ``OSError`` is not in ``EXCEPTIONS_TO_CLOSE``, so it escapes every inner
handler, hits the outer ``except Exception`` while ``_connected`` is still True,
and is re-raised — the thread dies. This test drives one convertor iteration
synchronously (no live thread, no sleep) and asserts on that desired contract.
"""

import errno
import queue

import pytest

from sky.axis import AxisRA
from sky.motor import MotionMode, MotorDirection, MotorStatus
from sky.physics import Ha, HaPerSecond


class _UnpluggedMotor:
    """Stub motor that raises OSError(ENXIO) on the first real operation.

    ENXIO is raised from ``set_direction`` (the first motor call inside
    ``_run_change_speed``), mimicking a USB-serial unplug mid-work. The app-level
    ``_connected`` flag is intentionally left True (the OS unplug does not flip
    it), so the current re-raise-and-die behaviour is exercised.
    """

    FORWARD_POSITION_SIGN = 1

    def __init__(self) -> None:
        self._status = MotorStatus(
            is_connected=True,
            steps=0,
            motion_mode=MotionMode.IDLE,
            speed_sps=0,
            accel_sps=None,
            direction=MotorDirection.STOP,
            target=None,
            microsteps=None,
        )

    def connect(self):
        self._status.is_connected = True

    def disconnect(self) -> bool:
        self._status.is_connected = False
        return True

    def status(self) -> MotorStatus:
        return self._status

    def convert_speed_to_steps_per_second(self, speed: HaPerSecond) -> int:
        return int(abs(float(speed)))

    def convert_steps_to_position(self, steps: int) -> Ha:
        return Ha(steps)

    def set_direction(self, direction: MotorDirection) -> bool:
        raise OSError(errno.ENXIO, "Device not configured")

    def set_speed(self, steps_per_second: int) -> int:
        return steps_per_second

    def run(self) -> bool:
        return True

    def stop(self) -> bool:
        return True

    def wait_till_stop(self, do_stop: bool = True, timeout_s: float | None = None) -> None:
        pass

    def reset(self) -> None:
        pass


def test_usb_unplug_degrades_axis_instead_of_silent_crash(monkeypatch: pytest.MonkeyPatch) -> None:
    """П2: ENXIO during a motion step is handled, not re-raised into the thread.

    See PLAN.md §1 П2 / §3 item 8. Drives a single convertor iteration
    synchronously: the command that hits the dead motor is dequeued first, then
    a sentinel op flips ``_connected`` off so the ``while self._connected`` loop
    can only run one iteration regardless of how the ENXIO is handled.
    """
    motor = _UnpluggedMotor()
    axis = AxisRA(motor)  # type: ignore[arg-type]

    # Prepare a "connected" axis without starting the background thread.
    axis._connected = True
    axis._mc_logger = axis.logger.getChild("_motion_convertor")

    # Enqueue exactly one command that reaches the dead motor.
    axis.change_speed(axis.FORWARD_DIRECTION, HaPerSecond(10), update_sky_speed=True)

    # Ensure the loop terminates after processing that one command: once the
    # queue is empty, stop the loop by detaching. We do this by wrapping the
    # queue.get so the second call (post-command) stops the loop.
    real_get = axis._queue.get
    calls = {"n": 0}

    def _get_then_stop(*args: object, **kwargs: object) -> object:
        calls["n"] += 1
        if calls["n"] == 1:
            return real_get(block=False)
        axis._connected = False
        raise queue.Empty()

    monkeypatch.setattr(axis._queue, "get", _get_then_stop)

    crashed: BaseException | None = None
    try:
        axis._motion_convertor()
    except BaseException as exc:  # noqa: BLE001 - capture the crash to assert on it
        crashed = exc

    assert crashed is None, (
        "USB unplug (OSError ENXIO) crashed the convertor step instead of being "
        f"handled: {crashed!r}"
    )
    assert axis._motion_convertor_error is None, (
        "ENXIO was stored as an opaque convertor crash instead of a clean "
        f"disconnect: {axis._motion_convertor_error!r}"
    )
