from abc import ABC, abstractmethod
from dataclasses import dataclass
from enum import StrEnum

from sky.physics import AxisPos, AxisSpeed


class MotionMode(StrEnum):
    IDLE = 'idle'
    ACCELERATION = 'acceleration'
    RUN = 'run'
    DECELERATION = 'deceleration'
    TARGET = 'target'


class MotorDirection(StrEnum):
    FORWARD = 'forward'
    STOP = 'stop'
    BACKWARD = 'backward'


@dataclass
class MotorStatus:
    is_connected: bool
    steps: int
    motion_mode: MotionMode
    speed_sps: int
    accel_sps: int | None
    direction: MotorDirection
    target: int | None
    microsteps: int | None


class MotorStopRequire(Exception):
    pass


class MotorStateError(Exception):
    pass



class Motor[POS_CLS: AxisPos, SPEED_CLS: AxisSpeed](ABC):
    """
    Abstract class for motor interface. 
    All methods should just inquire action, wait for answer from motor and return answer.
    Expect methods implemented in this class
    Methods should not wait until inqured action happens!
    Standart session looks like bunch of set's, which should change behaviour, wait till stop if method wants to stop, and send run command if need to run
    """
    @abstractmethod
    def __init__(self) -> None:
        ...

    @abstractmethod
    def connect(self):
        ...
    
    @abstractmethod
    def disconnect(self) -> bool:
        ...

    @abstractmethod
    def status(self) -> MotorStatus:
        """ Get actual status from motor """
        ...
    
    @abstractmethod
    def set_steps(self, steps: int) -> bool:
        """ Update currrent steps position for the motor; can raise MotorStopRequire """
        ...

    @abstractmethod
    def set_speed(self, steps_per_second: int) -> int:
        """ Change current motor speed, absolute value; can raise MotorStopRequire"""
        ...
    
    @abstractmethod
    def set_acceleration(self, steps_per_second_square: float) -> bool:
        """ Change acceleration, absolute value; can return False if not supported or raise MotorStopRequire """
        ...

    @abstractmethod
    def set_direction(self, direction: MotorDirection) -> bool:
        """ Change motion direction; can raise MotorStopRequire """
        ...

    @abstractmethod
    def set_delta(self, delta_steps: int) -> bool:
        """ Set delta for moving; can raise MotorStopRequire """
        ...

    @abstractmethod
    def get_speed_sps_by_delta(self, delta_steps: int) -> int:
        """ Get speed in steps per second by delta """
        ...

    @abstractmethod
    def get_speed_by_speed_sps(self, speed_sps: int) -> AxisSpeed:
        """ Get speed in steps per second by speed in steps per second """
        ...

    @abstractmethod
    def set_motion_mode(self, motion_mode: MotionMode) -> bool:
        """ Set new motion mode; can raise MotorStopRequire """

    @abstractmethod
    def set_microsteps(self, microsteps: int) -> bool:
        """ Update microsteps; can raise MotorStopRequire """
        ...

    @abstractmethod
    def convert_position_to_steps(self, position: POS_CLS) -> int:
        """ Convert position to steps """
        ...

    @abstractmethod
    def convert_steps_to_position(self, steps: int) -> POS_CLS:
        """ Convert steps to position """
        ...

    @abstractmethod
    def convert_speed_to_steps_per_second(self, speed: SPEED_CLS) -> int:
        """ Convert speed to steps per second, absolute value """
        ...

    @abstractmethod
    def run(self) -> bool:
        """ Ask to run motor """
        ...
    
    @abstractmethod
    def stop(self) -> bool:
        """ Ask to stop motor """
        ...

    def wait_till_stop(self, do_stop: bool = True, timeout_s: float | None = None) -> None:
        raise NotImplementedError()
    
    def reset(self) -> None:
        """ Stops all motion, reset all parameters """
        raise NotImplementedError()
