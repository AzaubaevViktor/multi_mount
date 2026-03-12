from dataclasses import dataclass
from enum import StrEnum
import logging
import queue
import threading
from typing import Callable, ParamSpec, Sequence, TypeVar, cast

from serial_wrapper.wrapper import EXCEPTIONS_TO_CLOSE
from sky.motor import MotionMode, Motor, MotorDirection, MotorStopRequire
from sky.physics import AxisPos, AxisSpeed, Dec, DecPerSecond, Ha, HaPerSecond, Second, SkyDirection
from utils.method_call_chain import log_method_call_chain


class AxisName(StrEnum):
    RA = 'ra'
    DEC = 'dec'


class AxisMotionMode(StrEnum):
    STOP = 'stop'
    TRACK = 'track'
    SLEW = 'slew'
    GOTO = 'goto'


class AxisCommandType(StrEnum):
    SET_POSITION = 'set_position'
    CHANGE_SPEED = 'change_speed'
    MOVE = 'move'
    GOTO_TO = 'goto_to'
    HALT_DIRECTION = 'halt_direction'
    HALT_ALL = 'halt_all'


@dataclass
class AxisCommand:
    type: AxisCommandType
    position: PointCoordinates | None = None
    speed: AxisSpeed | None = None
    update_sky_speed: bool = False
    direction: SkyDirection | None = None


@dataclass
class PointCoordinates:
    ra: Ha
    dec: Dec


_AXIS_PARAMS = ParamSpec("_AXIS_PARAMS")
_AXIS_RETURN = TypeVar("_AXIS_RETURN")


def _raise_if_thread_failed(method: Callable[_AXIS_PARAMS, _AXIS_RETURN]):
    def wrapped(
        self: "Axis[AxisPos, AxisSpeed]",
        *args: _AXIS_PARAMS.args,
        **kwargs: _AXIS_PARAMS.kwargs,
    ) -> _AXIS_RETURN:
        if self._motion_convertor_error is not None:
            raise RuntimeError(f"{self.axis.value} motion convertor thread crashed") from self._motion_convertor_error
        return method(self, *args, **kwargs)

    return wrapped


class Axis[_POS_CLS: AxisPos, _SPEED_CLS: AxisSpeed]:
    """Motor axis with position tracking and motion conversion.

    Invariants:
    - After any action (SLEW, GOTO, HALT) completes, the axis returns to TRACK mode
      when _sky_speed != 0. The mount resumes tracking automatically.
    - _last_motor_position and _last_motor_position_update_s are kept in sync for
      compensation; they are updated on GOTO completion and in TRACK mode.
    """

    axis: AxisName
    POS_CLS: type[_POS_CLS]
    SPEED_CLS: type[_SPEED_CLS]
    FORWARD_DIRECTION: SkyDirection
    BACKWARD_DIRECTION: SkyDirection
    def __init__(self, motor: Motor[_POS_CLS, _SPEED_CLS]) -> None:
        self.logger = logging.getLogger(f"{type(self).__name__}.{self.axis.value}")
        self.DIRECTIONS: Sequence[SkyDirection] = (self.FORWARD_DIRECTION, self.BACKWARD_DIRECTION)

        self._motor = motor
        self._mode: AxisMotionMode = AxisMotionMode.STOP
        self._connected = False

        self._motion_convertor_thread: threading.Thread | None = None
        self._motion_convertor_error: Exception | None = None
        self._motor_lock = threading.RLock()
        self._queue: queue.Queue[AxisCommand] = queue.Queue()

        self._sky_speed: _SPEED_CLS = self.SPEED_CLS(0)

        self._ra_position = Ha(0)
        """ Ra Mount point position """
        self._dec_position = Dec(0)
        """ Dec Mount point position """

        self._move_direction: SkyDirection | None = None
        self._goto_target: _POS_CLS | None = None
        self._goto_direction: SkyDirection | None = None

        self._last_motor_position: _POS_CLS = self.POS_CLS(0)
        self._last_motor_position_update_s: Second = Second.monotonic()

    @_raise_if_thread_failed
    def connect(self):
        if self._connected:
            return
        
        with self._motor_lock:
            self._motor.connect()
            self._motor.reset()

        self._connected = True
        self._motion_convertor_error = None
        if self._motion_convertor_thread is None or not self._motion_convertor_thread.is_alive():
            self._motion_convertor_thread = threading.Thread(
                target=self._motion_convertor,
                name=f"{self.axis.value}_motion_convertor",
            )

        self._motion_convertor_thread.start()

    @_raise_if_thread_failed
    def is_connected(self) -> bool:
        return self._connected

    @_raise_if_thread_failed
    def disconnect(self):
        if not self._connected:
            return
        
        self._connected = False

        with self._motor_lock:  # Wait while motor available
            self._motor.reset()
            self._motor.disconnect()
        
        if self._motion_convertor_thread is not None:
            self._motion_convertor_thread.join(float(self.THREAD_ITERATION_DELAY_S))
    
    @_raise_if_thread_failed
    def mode(self) -> AxisMotionMode:
        return self._mode

    def _get_motor_direction_and_speed(self, direction: SkyDirection, speed: AxisSpeed) -> tuple[MotorDirection, int]:
        if not isinstance(speed, self.SPEED_CLS):
            raise ValueError(f"Speed should be of type {self.SPEED_CLS} for {self.axis.value} axis, got {type(speed)}")
        
        if direction not in self.DIRECTIONS:
            raise ValueError(f"Direction should be one of {self.DIRECTIONS} for {self.axis.value} axis, got {direction}")

        if (direction == self.FORWARD_DIRECTION) ^ (float(speed) > 0):
            return (MotorDirection.BACKWARD, self._motor.convert_speed_to_steps_per_second(abs(speed)))
        else:
            return (MotorDirection.FORWARD, self._motor.convert_speed_to_steps_per_second(abs(speed)))

    def _get_current_position(self, position: PointCoordinates | None = None) -> _POS_CLS:
        if position is None:
            position = self.get_position()
        if not isinstance(position, PointCoordinates):
            raise ValueError(f"Position should be of type PointCoordinates for {self.axis.value} axis, got {type(position)}")

        return cast(_POS_CLS, position.ra if self.axis == AxisName.RA else position.dec)

    def _set_current_position(self, pos: _POS_CLS) -> None:
        with self._motor_lock:
            if self.axis == AxisName.RA:
                self._ra_position = cast(Ha, pos)
            elif self.axis == AxisName.DEC:
                self._dec_position = cast(Dec, pos)
            else:
                raise ValueError(f"Invalid axis: {self.axis}")
    
    def _get_forward_relative_speed(self, direction: SkyDirection, speed: _SPEED_CLS) -> _SPEED_CLS:
        """ Get relative speed to forward direction """
        if direction not in self.DIRECTIONS:
            raise ValueError(f"Direction should be one of {self.DIRECTIONS} for {self.axis.value} axis, got {direction}")

        if direction == self.FORWARD_DIRECTION:
            return speed
        elif direction == self.BACKWARD_DIRECTION:
            return -speed
        else:
            raise ValueError(f"Invalid direction: {direction} for {self.axis.value} axis")

    def _do_resume_to_tracking(self) -> None:
        if float(self._sky_speed) == 0:
            with self._motor_lock:
                self._motor.wait_till_stop()
            self._mode = AxisMotionMode.TRACK
            return
        if float(self._sky_speed) > 0:
            direction = MotorDirection.FORWARD
        elif float(self._sky_speed) < 0:
            direction = MotorDirection.BACKWARD
        else:
            direction = MotorDirection.STOP

        with self._motor_lock:
            speed = self._motor.convert_speed_to_steps_per_second(abs(self._sky_speed))

            while True:
                try:
                    self._motor.set_direction(direction)
                    self._motor.set_speed(speed)
                    self._motor.run()
                    
                    self._mode = AxisMotionMode.TRACK
                    break
                except MotorStopRequire:
                    self._mode = AxisMotionMode.STOP
                    self._motor.wait_till_stop()

    def _run_set_position(self, position: PointCoordinates) -> bool:
        return_to_tracking = False
        new_position = self._get_current_position(position)
        
        while True:
            try:
                self._motor.set_steps(
                    self._motor.convert_position_to_steps(new_position)
                )
                break
            except MotorStopRequire:
                prev_mode = self._mode
                self._mode = AxisMotionMode.STOP
                self._motor.wait_till_stop()

                if prev_mode != AxisMotionMode.STOP:
                    return_to_tracking = True

        self._ra_position = position.ra
        self._dec_position = position.dec

        return return_to_tracking
        
    def _run_change_speed(self, sky_direction: SkyDirection, new_speed: _SPEED_CLS, update_sky_speed: bool) -> None:
        motor_direction, motor_speed = self._get_motor_direction_and_speed(sky_direction, new_speed)

        self._mc_logger.info("Run change speed: %s, %s, %s", sky_direction, new_speed, update_sky_speed)

        while True:
            try:
                if motor_speed != 0:
                    self._motor.set_direction(motor_direction)
                    self._motor.set_speed(motor_speed)

                prev_sky_speed = self._sky_speed
                self._sky_speed = self._get_forward_relative_speed(sky_direction, new_speed) if update_sky_speed else self._sky_speed
                
                self._mc_logger.info("New sky speed: %s -> %s", prev_sky_speed, self._sky_speed)
                
                if motor_speed == 0:
                    self._mode = AxisMotionMode.STOP
                elif update_sky_speed:
                    self._mode = AxisMotionMode.TRACK
                    self._motor.run()
                else:
                    self._mode = AxisMotionMode.SLEW
                    self._motor.run()
                
                break
            except MotorStopRequire:
                self._mode = AxisMotionMode.STOP
                self._motor.wait_till_stop()

    def _run_move(self, sky_direction: SkyDirection, move_speed: _SPEED_CLS) -> None:
        motor_direction, motor_speed = self._get_motor_direction_and_speed(sky_direction, move_speed)
        
        while True:
            try:
                self._motor.set_direction(motor_direction)
                self._motor.set_speed(motor_speed)

                if motor_speed == 0:
                    self._mode = AxisMotionMode.STOP
                else:
                    self._mode = AxisMotionMode.SLEW
                    self._move_direction = sky_direction
                    self._motor.run()
                
                break
            except MotorStopRequire:
                self._mode = AxisMotionMode.STOP
                self._motor.wait_till_stop()

    def _run_goto_to(self, position: _POS_CLS) -> None:
        # This is fastest path; It will be calculated at higher level
        delta = self._get_current_position() - position
        signed = self._motor.FORWARD_POSITION_SIGN * float(delta)
        if signed < 0:
            direction = MotorDirection.FORWARD
            self._goto_direction = self.FORWARD_DIRECTION
        elif signed > 0:
            direction = MotorDirection.BACKWARD
            self._goto_direction = self.BACKWARD_DIRECTION
        else:
            direction = MotorDirection.STOP
            self._goto_direction = None
        
        if float(delta) != 0:
            speed_sps = self._motor.get_speed_sps_by_delta(self._motor.convert_position_to_steps(delta))
            speed = self._motor.get_speed_by_speed_sps(speed_sps)
            moving_approx_time = abs(delta / speed)
            delta -= self.POS_CLS(float(self._sky_speed) * float(moving_approx_time) * self._motor.FORWARD_POSITION_SIGN)
            
            while True:
                try:
                    self._motor.set_direction(direction)
                    self._motor.set_speed(speed_sps)
                    self._motor.set_delta(-self._motor.FORWARD_POSITION_SIGN * self._motor.convert_position_to_steps(delta))
                    self._motor.run()

                    self._goto_target = position
                    self._mode = AxisMotionMode.GOTO
                    break
                except MotorStopRequire:
                    self._mode = AxisMotionMode.STOP
                    self._motor.wait_till_stop()

    def _run_halt_direction(self) -> bool:
        self._motor.wait_till_stop()
        self._mode = AxisMotionMode.STOP
        self._move_direction = None
        
        return True

    THREAD_ITERATION_DELAY_S = Second(.5)
    _GOTO_SECONDS_TOLERANCE = 10
    def _motion_convertor(self):
        self._mc_logger = self.logger.getChild("_motion_convertor")
        self._mc_logger.info("Start working")

        while self._connected:
            try:
                try:
                    command = self._queue.get(timeout=float(self.THREAD_ITERATION_DELAY_S))
                except queue.Empty:
                    command = None

                return_to_tracking: None | bool = None  # None - not decided, True - return to tracking, False - do not return to tracking

                if command:
                    try:
                        self._mc_logger.info("Processing command: %s", command)
                        match command.type:
                            case AxisCommandType.SET_POSITION:
                                if not command.position:
                                    raise ValueError("Position is required for SET_POSITION command")

                                with self._motor_lock:
                                    return_to_tracking = self._run_set_position(command.position)

                            case AxisCommandType.CHANGE_SPEED:
                                if not command.direction:
                                    raise ValueError("Direction is required for CHANGE_SPEED command")
                                if not command.speed:
                                    raise ValueError("Speed is required for CHANGE_SPEED command")
                                if not isinstance(command.speed, self.SPEED_CLS):
                                    raise ValueError(f"Speed should be of type {self.SPEED_CLS} for {self.axis.value} axis, got {type(command.speed)}")

                                with self._motor_lock:
                                    self._run_change_speed(command.direction, command.speed, command.update_sky_speed)
                            
                            case AxisCommandType.MOVE:
                                if not command.direction:
                                    raise ValueError("Direction is required for MOVE command")
                                if not isinstance(command.speed, self.SPEED_CLS):
                                    raise ValueError(f"Speed should be of type {self.SPEED_CLS} for {self.axis.value} axis, got {type(command.speed)}")

                                with self._motor_lock:
                                    self._run_move(command.direction, command.speed)
                            
                            case AxisCommandType.GOTO_TO:
                                if not command.position:
                                    raise ValueError("Position is required for GOTO_TO command")

                                with self._motor_lock:
                                    self._run_goto_to(self._get_current_position(command.position))
                            
                            case AxisCommandType.HALT_DIRECTION:
                                if not command.direction:
                                    raise ValueError("Direction is required for HALT_DIRECTION command")
                                
                                if command.direction == self._move_direction:
                                    with self._motor_lock:
                                        return_to_tracking = self._run_halt_direction()
                            
                            case AxisCommandType.HALT_ALL:
                                with self._motor_lock:
                                    return_to_tracking = self._run_halt_direction()
                        
                        self._mc_logger.info("Command precessed, return to tracking required: %s", return_to_tracking)

                        if return_to_tracking is not None and return_to_tracking:
                            self._do_resume_to_tracking()
                    except EXCEPTIONS_TO_CLOSE:
                        self._mc_logger.exception("While processing command, skip: %s", command)

                try:
                    need_to_compensate = False

                    self._mc_logger.info("Mode: %s", self._mode)

                    match self._mode:
                        case AxisMotionMode.STOP:
                            if self._sky_speed == 0:
                                self._mode = AxisMotionMode.TRACK
                            else:
                                need_to_compensate = True
                        case AxisMotionMode.TRACK:
                            with self._motor_lock:
                                # Don't need to change pointing position — its tracking
                                self._last_motor_position = self._motor.convert_steps_to_position(self._motor.status().steps)
                                self._last_motor_position_update_s = Second.monotonic()
                        case AxisMotionMode.SLEW:
                            need_to_compensate = True
                        case AxisMotionMode.GOTO:
                            need_to_compensate = True

                            with self._motor_lock:
                                if self._goto_target is None or self._goto_direction is None:
                                    self._mode = AxisMotionMode.STOP
                                else:
                                    current_position = self._get_current_position()

                                    need_to_stop = None

                                    if abs(current_position - self._goto_target) < self.POS_CLS(self._GOTO_SECONDS_TOLERANCE):
                                        need_to_stop = "Tolerance good enough"

                                    if self._motor.status().motion_mode != MotionMode.TARGET:
                                        need_to_stop = "Motor stopped before target"

                                    overshoot = self._motor.FORWARD_POSITION_SIGN * float(current_position - self._goto_target)
                                    if self._goto_direction == self.FORWARD_DIRECTION and overshoot >= 0:
                                        need_to_stop = "Overshoot in forward direction"
                                    elif self._goto_direction == self.BACKWARD_DIRECTION and overshoot <= 0:
                                        need_to_stop = "Overshoot in backward direction"
                                    
                                    if need_to_stop is not None:
                                        self.logger.info("GOTO to %s need to be stopped: %s", self._goto_target, need_to_stop)
                                        self._motor.wait_till_stop()
                                        self._last_motor_position = self._motor.convert_steps_to_position(self._motor.status().steps)
                                        self._last_motor_position_update_s = Second.monotonic()

                                        if abs(current_position - self._goto_target) < self.POS_CLS(self._GOTO_SECONDS_TOLERANCE):
                                            self._mode = AxisMotionMode.STOP
                                            self._goto_target = None
                                            self._goto_direction = None
                                            self.logger.info("GOTO to %s stopped, resume to tracking", self._goto_target)
                                            self._do_resume_to_tracking()
                                        else:
                                            self.logger.info("GOTO rerun to %s", self._goto_target)
                                            self._run_goto_to(self._goto_target)

                    self._mc_logger.debug("Processing mode done, need to compensate: %s", need_to_compensate)

                    if need_to_compensate:
                        self._mc_logger.info("Compensating tracking")
                        with self._motor_lock:
                            current_motor_position = self._motor.convert_steps_to_position(self._motor.status().steps)
                            motor_position_update_s = Second.monotonic()
                            elapsed_s = motor_position_update_s - self._last_motor_position_update_s

                            expected_delta = self._sky_speed * elapsed_s
                            actual_delta = (current_motor_position - self._last_motor_position).moving_wrap()

                            delta = self.POS_CLS(self._motor.FORWARD_POSITION_SIGN * float(actual_delta - expected_delta))
                            
                            # TODO: Add wrapping, if wrap happens in dec axis, we need to add to RA axis +12 hours
                            self._set_current_position(
                                self._get_current_position() + delta
                            )
                            
                            self._last_motor_position = current_motor_position
                            self._last_motor_position_update_s = motor_position_update_s
                        
                        self._mc_logger.debug("Compensating tracking done")

                except EXCEPTIONS_TO_CLOSE:
                    self._mc_logger.exception("While processing mode: %s", self._mode)
            
            except Exception as error:
                if not self._connected:
                    self._mc_logger.debug("Connection is closed, stop working")
                    break
                else:
                    self._motion_convertor_error = error
                    self._mc_logger.exception("While processing")
                    raise

        self._mc_logger.info("Stop working")

    @_raise_if_thread_failed
    def get_position(self) -> PointCoordinates:
        return PointCoordinates(ra=self._ra_position, dec=self._dec_position)
    
    @_raise_if_thread_failed
    def set_position(self, position: PointCoordinates) -> None:
        self._queue.put(AxisCommand(AxisCommandType.SET_POSITION, position=position))

    @_raise_if_thread_failed
    @log_method_call_chain()
    def change_speed(self, direction: SkyDirection, speed: _SPEED_CLS, update_sky_speed: bool = False) -> None:
        if direction not in self.DIRECTIONS:
            return
        self._queue.put(AxisCommand(AxisCommandType.CHANGE_SPEED, direction=direction, speed=speed, update_sky_speed=update_sky_speed))

    @_raise_if_thread_failed
    def move(self, direction: SkyDirection, speed: _SPEED_CLS) -> None:
        if direction not in self.DIRECTIONS:
            return
        self._queue.put(AxisCommand(AxisCommandType.MOVE, direction=direction, speed=speed))

    @_raise_if_thread_failed
    def goto_to(self, position: PointCoordinates) -> None:
        self._queue.put(AxisCommand(AxisCommandType.GOTO_TO, position=position))
    
    @_raise_if_thread_failed
    def halt_direction(self, direction: SkyDirection) -> None:
        if direction not in self.DIRECTIONS:
            return
        self._queue.put(AxisCommand(AxisCommandType.HALT_DIRECTION, direction=direction))
    
    @_raise_if_thread_failed
    def halt_all(self) -> None:
        self._queue.put(AxisCommand(AxisCommandType.HALT_ALL))

    @_raise_if_thread_failed
    def is_moving_to(self) -> bool:
        return self._mode == AxisMotionMode.GOTO


class AxisRA(Axis[Ha, HaPerSecond]):
    axis = AxisName.RA
    POS_CLS = Ha
    SPEED_CLS = HaPerSecond
    FORWARD_DIRECTION = SkyDirection.EAST
    BACKWARD_DIRECTION = SkyDirection.WEST


class AxisDEC(Axis[Dec, DecPerSecond]):
    axis = AxisName.DEC
    POS_CLS = Dec
    SPEED_CLS = DecPerSecond
    FORWARD_DIRECTION = SkyDirection.NORTH
    BACKWARD_DIRECTION = SkyDirection.SOUTH
