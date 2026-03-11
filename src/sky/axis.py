from dataclasses import dataclass
from enum import StrEnum
import logging
import queue
import threading
from typing import Sequence, cast

from sky.motor import Motor, MotorDirection, MotorStopRequire
from sky.physics import AxisPos, AxisSpeed, Dec, Ha, Second, SkyDirection


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
    

class Axis[_POS_CLS: AxisPos, _SPEED_CLS: AxisSpeed]:
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

        self._motion_convertor_thread = threading.Thread(
            target=self._motion_convertor,
            name=f"{self.axis.value}_motion_convertor",
        )
        self._motor_lock = threading.Lock()
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

    def connect(self):
        if self._connected:
            return
        
        with self._motor_lock:
            self._motor.connect()
            self._motor.reset()

        self._connected = True

        self._motion_convertor_thread.start()

    def disconnect(self):
        if not self._connected:
            return
        
        self._connected = False

        with self._motor_lock:  # Wait while motor available
            self._motor.reset()
            self._motor.disconnect()
        
        self._motion_convertor_thread.join(float(self.THREAD_ITERATION_DELAY_S))

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
                self._ra_position = pos
            elif self.axis == AxisName.DEC:
                self._dec_position = pos
            else:
                raise ValueError(f"Invalid axis: {self.axis}")
    
    def _get_forward_relative_speed(self, direction: SkyDirection, speed: _SPEED_CLS) -> _SPEED_CLS:
        if direction not in self.DIRECTIONS:
            raise ValueError(f"Direction should be one of {self.DIRECTIONS} for {self.axis.value} axis, got {direction}")

        if direction == self.FORWARD_DIRECTION:
            return speed
        elif direction == self.BACKWARD_DIRECTION:
            return -speed

    def _do_resume_to_tracking(self) -> None:
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

    def _run_goto_to(self, position: _POS_CLS) -> None:
        with self._motor_lock:
            # This is fastest path; It will be calculated at higher level
            delta = self._get_current_position() - position
            if float(delta) > 0:
                direction = MotorDirection.FORWARD
                self._goto_direction = self.FORWARD_DIRECTION
            elif float(delta) < 0:
                direction = MotorDirection.BACKWARD
                self._goto_direction = self.BACKWARD_DIRECTION
            else:
                direction = MotorDirection.STOP
                self._goto_direction = None
            
            if float(delta) != 0:
                speed_sps = self._motor.get_speed_sps_by_delta(self._motor.convert_position_to_steps(delta))
                speed = self._motor.get_speed_by_speed_sps(speed_sps)
                moving_approx_time = abs(delta / speed)
                delta += self._sky_speed * moving_approx_time
                
                while True:
                    try:
                        self._motor.set_direction(direction)
                        self._motor.set_speed(speed_sps)
                        self._motor.set_delta(self._motor.convert_position_to_steps(delta))
                        self._motor.run()

                        self._goto_target = position
                        self._mode = AxisMotionMode.GOTO
                        break
                    except MotorStopRequire:
                        self._mode = AxisMotionMode.STOP
                        self._motor.wait_till_stop()

    THREAD_ITERATION_DELAY_S = Second(.5)
    _GOTO_SECONDS_TOLERANCE = 10
    def _motion_convertor(self):
        logger = self.logger.getChild("_motion_convertor")
        logger.info("Start working")

        while self._connected:
            try:
                command = self._queue.get(timeout=float(self.THREAD_ITERATION_DELAY_S))
            except queue.Empty:
                command = None

            return_to_tracking: None | bool = None  # None - not decided, True - return to tracking, False - do not return to tracking

            if command:
                match command.type:
                    case AxisCommandType.SET_POSITION:
                        if not command.position:
                            raise ValueError("Position is required for SET_POSITION command")

                        with self._motor_lock:
                            new_position = self._get_current_position(command.position)
                            
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

                            self._ra_position = command.position.ra
                            self._dec_position = command.position.dec

                    case AxisCommandType.CHANGE_SPEED:
                        if not command.direction:
                            raise ValueError("Direction is required for CHANGE_SPEED command")
                        if not command.speed:
                            raise ValueError("Speed is required for CHANGE_SPEED command")
                        if not isinstance(command.speed, self.SPEED_CLS):
                            raise ValueError(f"Speed should be of type {self.SPEED_CLS} for {self.axis.value} axis, got {type(command.speed)}")

                        with self._motor_lock:
                            direction, speed = self._get_motor_direction_and_speed(command.direction, command.speed)
                            
                            while True:
                                try:
                                    self._motor.set_direction(direction)
                                    self._motor.set_speed(speed)
                                    self._sky_speed = self._get_forward_relative_speed(command.direction, command.speed) if command.update_sky_speed else self._sky_speed
                                    
                                    if speed == 0:
                                        self._mode = AxisMotionMode.STOP
                                    elif command.update_sky_speed:
                                        self._mode = AxisMotionMode.TRACK
                                        self._motor.run()
                                    else:
                                        self._mode = AxisMotionMode.SLEW
                                        self._motor.run()
                                    
                                    break
                                except MotorStopRequire:
                                    self._mode = AxisMotionMode.STOP
                                    self._motor.wait_till_stop()
                    
                    case AxisCommandType.MOVE:
                        if not command.direction:
                            raise ValueError("Direction is required for MOVE command")
                        if not isinstance(command.speed, self.SPEED_CLS):
                            raise ValueError(f"Speed should be of type {self.SPEED_CLS} for {self.axis.value} axis, got {type(command.speed)}")

                        with self._motor_lock:
                            direction, speed = self._get_motor_direction_and_speed(command.direction, command.speed)
                            
                            while True:
                                try:
                                    self._motor.set_direction(direction)
                                    self._motor.set_speed(speed)

                                    if speed == 0:
                                        self._mode = AxisMotionMode.STOP
                                    else:
                                        self._mode = AxisMotionMode.SLEW
                                        self._move_direction = command.direction
                                        self._motor.run()

                                    
                                    break
                                except MotorStopRequire:
                                    self._mode = AxisMotionMode.STOP
                                    self._motor.wait_till_stop()
                    
                    case AxisCommandType.GOTO_TO:
                        if not command.position:
                            raise ValueError("Position is required for GOTO_TO command")

                        self._run_goto_to(self._get_current_position(command.position))
                    
                    case AxisCommandType.HALT_DIRECTION:
                        if not command.direction:
                            raise ValueError("Direction is required for HALT_DIRECTION command")
                        
                        if command.direction == self._move_direction:
                            with self._motor_lock:
                                self._motor.wait_till_stop()
                                self._mode = AxisMotionMode.STOP
                    
                    case AxisCommandType.HALT_ALL:
                        with self._motor_lock:
                            self._motor.wait_till_stop()
                            self._mode = AxisMotionMode.STOP

                if return_to_tracking is not None and return_to_tracking:
                    self._do_resume_to_tracking()

            try:
                need_to_compensate = False

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

                                need_to_stop = False

                                if abs(current_position - self._goto_target) < self.POS_CLS(self._GOTO_SECONDS_TOLERANCE):
                                    need_to_stop = True

                                # Stop if we reached the target
                                if self._goto_direction == SkyDirection.EAST and current_position >= self._goto_target:
                                    need_to_stop = True
                                elif self._goto_direction == SkyDirection.WEST and current_position <= self._goto_target:
                                    need_to_stop = True
                                
                                if need_to_stop:
                                    self._motor.wait_till_stop()
                                
                                if need_to_stop and abs(current_position - self._goto_target) < self.POS_CLS(self._GOTO_SECONDS_TOLERANCE):
                                    self._mode = AxisMotionMode.STOP
                                    self._goto_target = None
                                    self._goto_direction = None
                                else:
                                    # Need to rerun GOTO to the target
                                    self._run_goto_to(self._goto_target)
                
                if need_to_compensate:
                    with self._motor_lock:
                        current_motor_position = self._motor.convert_steps_to_position(self._motor.status().steps)
                        motor_position_update_s = Second.monotonic()
                        elapsed_s = motor_position_update_s - self._last_motor_position_update_s

                        expected_delta = self._sky_speed * elapsed_s
                        actual_delta = current_motor_position - self._last_motor_position

                        delta = expected_delta - actual_delta

                        self._set_current_position(
                            self._get_current_position() + delta
                        )
                        
                        self._last_motor_position = current_motor_position + delta
                        self._last_motor_position_update_s = motor_position_update_s

            except:
                logger.exception("While processing step: ")

        logger.info("Stop working")

    def get_position(self) -> PointCoordinates:
        return PointCoordinates(self._ra_position, self._dec_position)
    
    def set_position(self, position: PointCoordinates) -> None:
        self._queue.put(AxisCommand(AxisCommandType.SET_POSITION, position=position))

    def change_speed(self, direction: SkyDirection, speed: _SPEED_CLS, update_sky_speed: bool = False) -> None:
        self._queue.put(AxisCommand(AxisCommandType.CHANGE_SPEED, direction=direction, speed=speed, update_sky_speed=update_sky_speed))

    def move(self, direction: SkyDirection, speed: _SPEED_CLS) -> None:
        self._queue.put(AxisCommand(AxisCommandType.MOVE, direction=direction, speed=speed))

    def goto_to(self, position: PointCoordinates) -> None:
        self._queue.put(AxisCommand(AxisCommandType.GOTO_TO, position=position))
    
    def halt_direction(self, direction: SkyDirection) -> None:
        self._queue.put(AxisCommand(AxisCommandType.HALT_DIRECTION, direction=direction))
    
    def halt_all(self) -> None:
        self._queue.put(AxisCommand(AxisCommandType.HALT_ALL))

    def is_moving_to(self) -> bool:
        return self._mode == AxisMotionMode.GOTO
