"""What the RA axis sees: the :class:`sky.motor.Motor` contract, and nothing else.

This class used to be the whole driver — codec, board constants, caches and
quirks in one place. It is now the top of four layers
(``docs/RA_REWRITE_PLAN.md`` §2), and it owns exactly the things the layers
below have no business knowing:

* the sky units (``Ha``/``HaPerSecond``) and the conversions to counts;
* the speed *policy* — when a move is worth high speed, what a GOTO is run at;
* the motion bookkeeping the ``Motor`` contract requires (last target, last
  direction, the "delta of zero" case).

Everything else is delegated: framing and parsing to
:class:`skywatcher.codec.SkyWatcherCodec`, board constants and the maths that
follows from them to :class:`skywatcher.board.SkyWatcherBoard`, and the line,
the expected state and every firmware quirk to
:class:`skywatcher.session.SkyWatcherSession`.
"""

import logging

from clock import REAL_CLOCK, Clock
from serial_wrapper.wrapper import SerialLine
from sky.constants import STELLAR_SPEED
from sky.motor import MotionMode, Motor, MotorDirection, MotorStateError, MotorStatus, MotorStopRequire
from sky.physics import Ha, HaPerSecond
from skywatcher.board import SkyWatcherBoard
from skywatcher.codec import (
    Command,
    Direction,
    MotionStatus,
    SkyWatcherCodec,
    SkyWatcherMotorCommandError,
    SkyWatcherMotorError,
    SkyWatcherMotorProtocolError,
    SkyWatcherMotorRebootError,
    SkyWatcherMotorTimeoutError,
    SlewMode,
    SpeedMode,
    Status,
)
from skywatcher.session import SkyWatcherSession

# Re-exported: callers and tests have always imported the error hierarchy from this
# module, and splitting the codec out must not move their imports.
__all__ = [
    "SkyWatcherMotor",
    "SkyWatcherMotorCommandError",
    "SkyWatcherMotorError",
    "SkyWatcherMotorProtocolError",
    "SkyWatcherMotorRebootError",
    "SkyWatcherMotorTimeoutError",
]


class SkyWatcherMotor(Motor[Ha, HaPerSecond]):
    FORWARD_POSITION_SIGN = -1

    _LOWSPEED_MARGIN = Ha(10 * 60)
    _LOWSPEED_SPEED = STELLAR_SPEED * 128
    _HIGHSPEED_SPEED = STELLAR_SPEED * 800

    # Policy, not transport: how long the axis is given to obey a `K` before the
    # driver calls it stuck. The measured brake ramp is 0.6s at the fastest the
    # board goes (§10.3), so this is an order of magnitude above it and only
    # fires when the axis is really stuck. The waiting itself is the session's.
    _STOP_TIMEOUT_S = 10.0
    # One transaction's retry budget. Pinned here because `connect()` has to
    # out-retry it: an app-level retry that gives up sooner adds nothing.
    REPEATS = SkyWatcherSession.REPEATS

    def __init__(self, serial: SerialLine, clock: Clock = REAL_CLOCK) -> None:
        self._session = SkyWatcherSession(serial, clock)
        self._logger = logging.getLogger(type(self).__name__)
        self._is_connected = False
        self._last_speed_sps = 0
        self._last_direction = MotorDirection.STOP
        self._last_target: int | None = None
        self._zero_target_pending = False

    # ------------------------------------------------------------------
    # connection
    # ------------------------------------------------------------------

    def connect(self) -> None:
        self._session.open()
        self._is_connected = True

    def disconnect(self) -> bool:
        self._is_connected = False
        self._session.close()
        return True

    @property
    def board(self) -> SkyWatcherBoard | None:
        """The snapshot taken at connect, or None while the mount is unknown."""
        return self._session.board

    @property
    def steps_per_revolution(self) -> int:
        """Counts per full turn of the axis (`:a1`), as the board reports them."""
        return self._board().cpr

    def _board(self) -> SkyWatcherBoard:
        return self._session.require_board()

    # ------------------------------------------------------------------
    # reporting
    # ------------------------------------------------------------------

    def status(self) -> MotorStatus:
        status = self._session.status()
        if status.running and status.slew_mode == SlewMode.GOTO:
            motion_mode = MotionMode.TARGET
        elif status.running:
            motion_mode = MotionMode.RUN
        else:
            motion_mode = MotionMode.IDLE
        if not status.running:
            direction = MotorDirection.STOP
        elif status.direction == Direction.FORWARD:
            direction = MotorDirection.FORWARD
        else:
            direction = MotorDirection.BACKWARD

        return MotorStatus(
            is_connected=self._is_connected,
            steps=self.convert_position_to_steps(self._get_position()),
            motion_mode=motion_mode,
            speed_sps=self._last_speed_sps,
            accel_sps=None,
            direction=direction,
            target=self._last_target if status.slew_mode == SlewMode.GOTO else None,
            microsteps=None,
            # Whatever the last poll brought, never a poll of its own: `status()` is
            # called far more often than the voltage changes, and it must not turn
            # into eight extra commands per call.
            power_v=self._session.cached_power_v(),
            initialized=status.initialized,
        )

    def get_power_v(self) -> float | None:
        """Supply voltage of the RA board, or None while it is unknown.

        §6.8: the board does have a voltage query after all — not a command of its
        own but the `:C`/`:n` memory window, found by disassembling the vendor's
        SAM Console app and then confirmed on the wire. The number reported is
        `max(battery, usb)`, as the vendor's own UI shows it: the board is fed
        from whichever source is higher, so that maximum is the rail the regulator
        actually sees. Both channels stay separately visible in
        :meth:`protocol_monitor`, so a battery sagging under a healthy USB is not
        hidden by the maximum.
        """
        if not self._is_connected:
            return None
        return self._session.power_v()

    def protocol_monitor(self) -> dict[str, object]:
        last_status = self._session.last_status
        board = self._session.board
        return {
            "speed_mode": "-" if last_status is None else f"{last_status.speed_mode.name.lower()}({int(last_status.speed_mode)})",
            "highspeed_ratio": board.highspeed_ratio if board is not None else "-",
            "initialized": "-" if last_status is None else ("yes" if last_status.initialized else "NO"),
            # Both rails, because `get_power_v` reports only the higher one (§6.8).
            "battery_v": "-" if self._session.battery_v is None else f"{self._session.battery_v:.2f}",
            "usb_v": "-" if self._session.usb_v is None else f"{self._session.usb_v:.2f}",
            # §12: silence here means the board has not been caught rebooting. It is
            # the one counter a human watching the mount needs to see grow.
            "reboots": self._session.reboots_detected,
        }

    # ------------------------------------------------------------------
    # setters
    # ------------------------------------------------------------------

    def set_steps(self, steps: int) -> bool:
        # Two `:f1` reads, on purpose: the GOTO guard is this layer's rule and the
        # "never `:E` on the move" guard is the session's (§11). Neither is allowed
        # to rely on a status the other one read.
        self._ensure_not_goto(self._session.status(), "cannot change steps while GOTO is in progress")
        self._session.set_position_ticks(steps)
        return True

    def set_speed(self, steps_per_second: int) -> int:
        status = self._session.status()
        self._ensure_not_goto(status, "cannot change speed while GOTO is in progress")
        if steps_per_second <= 0:
            raise ValueError(f"steps_per_second must be positive, got {steps_per_second}")
        board = self._board()
        target_speed_mode = self._get_speed_mode_for_speed_sps(steps_per_second)
        self._set_motion(
            MotionStatus(
                slew_mode=status.slew_mode,
                direction=status.direction,
                speed_mode=target_speed_mode,
            ),
            status,
        )
        period = board.clamp_period(self._period_from_speed_sps(steps_per_second))
        self._session.set_period(period)
        self._last_speed_sps = board.speed_sps_from_period(period, target_speed_mode)
        return self._last_speed_sps

    def set_acceleration(self, steps_per_second_square: float) -> bool:
        self._ensure_not_goto(self._session.status(), "cannot change acceleration while GOTO is in progress")
        return False

    def set_direction(self, direction: MotorDirection) -> bool:
        status = self._session.status()
        self._ensure_not_goto(status, "cannot change direction while GOTO is in progress")
        if status.running:
            raise MotorStopRequire("cannot change direction while motor is moving")
        if direction == MotorDirection.STOP:
            self._last_direction = direction
            return True
        self._set_motion(
            MotionStatus(
                slew_mode=SlewMode.SLEW,
                direction=Direction.FORWARD if direction == MotorDirection.FORWARD else Direction.BACKWARD,
                speed_mode=self._get_preferred_speed_mode(status.speed_mode),
            ),
            status,
        )
        self._last_direction = direction
        return True

    def set_delta(self, delta_steps: int) -> bool:
        status = self._session.status()
        self._ensure_not_goto(status, "cannot change target while GOTO is in progress")
        board = self._board()
        delta = self.convert_steps_to_position(delta_steps).moving_wrap()
        if delta == Ha(0):
            self._last_target = None
            self._zero_target_pending = True
            return True
        self._zero_target_pending = False
        speed = self._get_goto_speed(delta)
        requested_speed_sps = self.convert_speed_to_steps_per_second(abs(speed))
        target_speed_mode = self._get_speed_mode_for_speed_sps(requested_speed_sps)
        self._set_motion(
            MotionStatus(
                slew_mode=SlewMode.GOTO,
                direction=Direction.FORWARD if speed > HaPerSecond(0) else Direction.BACKWARD,
                speed_mode=target_speed_mode,
            ),
            status,
        )
        period = board.clamp_period(self._period_from_speed_sps(requested_speed_sps))
        self._last_speed_sps = board.speed_sps_from_period(period, target_speed_mode)
        self._session.set_period(period)
        self._last_target = abs(delta_steps) % board.cpr
        self._session.transact(Command.SET_GOTO_TARGET_INCREMENT, SkyWatcherCodec.encode_revu24(self._last_target))
        self._session.transact(Command.SET_BREAK_POINT_INCREMENT, SkyWatcherCodec.encode_revu24(min(200, self._last_target)))
        self._last_direction = MotorDirection.FORWARD if speed > HaPerSecond(0) else MotorDirection.BACKWARD
        return True

    def set_motion_mode(self, motion_mode: MotionMode) -> bool:
        status = self._session.status()
        self._ensure_not_goto(status, "cannot change motion mode while GOTO is in progress")
        if motion_mode == MotionMode.RUN:
            self._set_motion(
                MotionStatus(
                    slew_mode=SlewMode.SLEW,
                    direction=Direction.FORWARD if self._last_direction != MotorDirection.BACKWARD else Direction.BACKWARD,
                    speed_mode=self._get_preferred_speed_mode(status.speed_mode),
                ),
                status,
            )
            return True
        if motion_mode == MotionMode.TARGET:
            self._set_motion(
                MotionStatus(
                    slew_mode=SlewMode.GOTO,
                    direction=Direction.FORWARD if self._last_direction != MotorDirection.BACKWARD else Direction.BACKWARD,
                    speed_mode=self._get_preferred_speed_mode(status.speed_mode),
                ),
                status,
            )
            return True
        if motion_mode == MotionMode.IDLE:
            return True
        raise MotorStateError(f"unsupported motion mode: {motion_mode}")

    def set_microsteps(self, microsteps: int) -> bool:
        status = self._session.status()
        self._ensure_not_goto(status, "cannot change microsteps while GOTO is in progress")
        if status.running:
            raise MotorStopRequire("cannot change microsteps while motor is moving")
        return False

    # ------------------------------------------------------------------
    # motion
    # ------------------------------------------------------------------

    def run(self) -> bool:
        reboots_before = self._session.reboots_detected
        status = self._session.status()
        self._ensure_not_goto(status, "cannot run while GOTO is in progress")
        if self._session.reboots_detected != reboots_before:
            # §12.4.2: "do not carry on moving". The board that was told the target,
            # the direction and the period no longer exists — the session has put the
            # position and the period back, but the GOTO registers went with the
            # reboot, so starting now would run the axis to whatever `:H1` reset to.
            # Nothing is sent, and the caller is told the axis did not start.
            self._logger.error("RA board rebooted, the pending motion is cancelled instead of started")
            self._last_target = None
            self._zero_target_pending = False
            return False
        if self._zero_target_pending:
            self._zero_target_pending = False
            return True
        if self._last_target is not None and status.slew_mode != SlewMode.GOTO:
            raise MotorStateError("cannot run before motor motion mode is switched")
        self._session.transact(Command.START_MOTION)
        return True

    def stop(self) -> bool:
        # §10.3: `K` starts a braking *ramp*. The board answers `=` immediately, but at
        # the top speed it allows the axis keeps running for another 0.6s and coasts
        # ~1148 counts. Returning True on the `=` alone made `Axis.disconnect` (reset ->
        # stop -> disconnect) close the port on a physically moving axis, so the caller
        # gets control back only once the Running bit is really down.
        self._request_stop()
        self._session.wait_till_running_clears(self._STOP_TIMEOUT_S)
        return True

    def wait_till_stop(self, do_stop: bool = True, timeout_s: float | None = None) -> None:
        if do_stop:
            self._request_stop()
        self._session.wait_till_running_clears(timeout_s)

    def _request_stop(self) -> None:
        self._session.request_stop()
        self._last_target = None
        self._zero_target_pending = False

    def reset(self) -> None:
        self.stop()
        self._last_speed_sps = 0
        self._last_direction = MotorDirection.STOP
        self._last_target = None
        self._zero_target_pending = False

    # ------------------------------------------------------------------
    # units
    # ------------------------------------------------------------------

    def convert_position_to_steps(self, position: Ha) -> int:
        return int(round(float(position) / (24 * 60 * 60) * self._board().cpr))

    def convert_steps_to_position(self, steps: int) -> Ha:
        return Ha(steps / self._board().cpr * 24 * 60 * 60)

    def convert_speed_to_steps_per_second(self, speed: HaPerSecond) -> int:
        return int(round(abs(float(speed)) * self._board().cpr / (24 * 60 * 60)))

    def get_speed_sps_by_delta(self, delta_steps: int) -> int:
        # What the axis above gets is what the mount will really do, not what the driver
        # would like it to do. `Axis._run_goto_to` turns this number into the GOTO ETA and
        # subtracts the sky drift over that ETA, so an optimistic answer is not cosmetic:
        # asking for 800x and getting 100x made the move take 8 times longer than the axis
        # expected and cut the sky compensation by the same factor — an undershoot and a
        # second GOTO. The request therefore goes through the same round trip the board
        # imposes (speed -> period -> clamp -> speed) before it is reported.
        requested_speed = self._get_goto_speed(self.convert_steps_to_position(delta_steps).moving_wrap())
        return self._achievable_speed_sps(self.convert_speed_to_steps_per_second(requested_speed))

    def get_speed_by_speed_sps(self, speed_sps: int) -> HaPerSecond:
        if speed_sps < 0:
            raise ValueError(f"speed_sps must be non-negative, got {speed_sps}")
        return HaPerSecond(float(speed_sps) * 24 * 60 * 60 / self._board().cpr)

    def _achievable_speed_sps(self, speed_sps: int) -> int:
        board = self._board()
        speed_mode = self._get_speed_mode_for_speed_sps(speed_sps)
        return board.speed_sps_from_period(board.clamp_period(self._period_from_speed_sps(speed_sps)), speed_mode)

    def _period_from_speed_sps(self, speed_sps: int) -> int:
        return self._board().period_from_speed_sps(speed_sps, self._get_speed_mode_for_speed_sps(speed_sps))

    def _get_speed_mode_for_speed_sps(self, speed_sps: int) -> SpeedMode:
        return SpeedMode.HIGHSPEED if speed_sps > self.convert_speed_to_steps_per_second(self._LOWSPEED_SPEED) else SpeedMode.LOWSPEED

    def _get_preferred_speed_mode(self, fallback: SpeedMode) -> SpeedMode:
        if self._last_speed_sps > 0:
            return self._get_speed_mode_for_speed_sps(self._last_speed_sps)
        return fallback

    def _get_goto_speed(self, delta: Ha) -> HaPerSecond:
        speed = self._HIGHSPEED_SPEED if abs(delta) > self._LOWSPEED_MARGIN else self._LOWSPEED_SPEED
        return speed if delta >= Ha(0) else -speed

    # ------------------------------------------------------------------
    # helpers
    # ------------------------------------------------------------------

    def _get_position(self, fresh: bool = False) -> Ha:
        return self.convert_steps_to_position(self._session.position_ticks(fresh=fresh)).wrap()

    def _set_motion(self, target: MotionStatus, current: Status) -> None:
        if current.running and (
            current.slew_mode != target.slew_mode
            or current.direction != target.direction
            or current.speed_mode != target.speed_mode
        ):
            raise MotorStopRequire("cannot change motion mode while motor is moving")
        self._session.transact(Command.SET_MOTION_MODE, target.to_command())

    def _ensure_not_goto(self, status: Status, message: str) -> None:
        if status.running and status.slew_mode == SlewMode.GOTO:
            raise MotorStopRequire(message)
