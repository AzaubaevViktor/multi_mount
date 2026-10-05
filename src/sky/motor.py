from abc import ABC, abstractmethod
from dataclasses import dataclass
from enum import StrEnum
from typing import Any

from pointing.sensor import SensorReading, SensorState
from sky.physics import AxisPos, AxisSpeed, StepsPerSecond


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
class MotorStatus[SPEED_CLS: AxisSpeed]:
    """What one motor reports about itself right now.

    Parameterised by the axis' sky-speed unit for the sake of one field:
    ``speed_sps``. A step rate is only meaningful on the axis that produced it
    (the two axes have different counts per revolution), and the status object
    is the one place where a rate travels away from its driver -- into the
    dashboard, the hardware session tool and the axis loop, all of which hold
    both axes at once.
    """

    is_connected: bool
    steps: int
    motion_mode: MotionMode
    speed_sps: StepsPerSecond[SPEED_CLS]
    accel_sps: int | None
    direction: MotorDirection
    target: int | None
    microsteps: int | None
    power_v: float | None = None
    initialized: bool | None = None
    """Board-reported "initialization done" flag, or None if the board has no such flag.

    On the SkyWatcher RA board it is not decoration: `:E` sent while the axis moves is
    accepted with `=` and clears this flag a few hundred milliseconds later
    (RA_PROTOCOL.md §11), and the same flag is one of the three signs of a controller
    reboot (§12.2). A driver that parses it and never reports it cannot detect either.
    """


class MotorStopRequire(Exception):
    pass


class MotorStateError(Exception):
    pass



class Motor[POS_CLS: AxisPos[Any], SPEED_CLS: AxisSpeed](ABC):
    """
    Abstract class for motor interface. 
    All methods should just inquire action, wait for answer from motor and return answer.
    Method can raise MotorStopRequire if motor need to be stopped before performing the action.
    Expect methods implemented in this class
    Methods should not wait until inqured action happens!
    Standart session looks like bunch of set's, which should change behaviour, wait till stop if method wants to stop, and send run command if need to run
    
    Some invariants:
    If motor is in GOTO mode, we can only check status and position
    If motor is moving, we can't change direction and microsteps
    If error happens, we should raise MotorStateError or MotorStopRequire
    """
    FORWARD_POSITION_SIGN: int
    """
    +1 or -1. How motor step increase translates to sky coordinate change.

    The Axis operates in two coordinate frames:
    - "motor frame": _sky_speed and motor position deltas are positive
      when Motor goes FORWARD (step count grows).
    - "sky frame": reported celestial position (Ha, Dec).

    FORWARD_POSITION_SIGN = 1 means these frames agree:
      Motor FORWARD → steps ↑ → position ↑ (e.g. DEC: forward = north = Dec grows).
    FORWARD_POSITION_SIGN = -1 means they disagree:
      Motor FORWARD → steps ↑ → position ↑ in encoder units,
      but the corresponding sky motion is opposite to FORWARD_DIRECTION
      (e.g. RA: forward = Ha grows = telescope tracks west,
      while FORWARD_DIRECTION = EAST because "East button" = faster tracking).

    Used in Axis to convert between these frames: goto direction,
    overshoot detection, sky-speed compensation, and position drift correction.
    """

    @abstractmethod
    def __init__(self) -> None:
        ...

    def read_orientation_sensor(self) -> SensorReading:
        return SensorReading(SensorState.UNSUPPORTED)

    def protocol_monitor(self) -> dict[str, object]:
        """Optional board diagnostics, without bypassing the motor interface."""
        return {}

    @abstractmethod
    def connect(self) -> None:
        ...
    
    @abstractmethod
    def disconnect(self) -> bool:
        ...

    @abstractmethod
    def status(self) -> MotorStatus[SPEED_CLS]:
        """ Get actual status from motor """
        ...

    @abstractmethod
    def get_power_v(self) -> float | None:
        """ Get current motor supply voltage if supported """
        ...
    
    @abstractmethod
    def set_steps(self, steps: int) -> bool:
        """ Update currrent steps position for the motor; can raise MotorStopRequire """
        ...

    @abstractmethod
    def set_speed(self, steps_per_second: StepsPerSecond[SPEED_CLS]) -> StepsPerSecond[SPEED_CLS]:
        """ Change current motor speed, absolute value; can raise MotorStopRequire

        Returns the rate the axis will really run at, which is not always the one
        asked for: the RA board clamps the step period, so the request goes
        through speed -> period -> clamp -> speed before it comes back.
        """
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
    def get_speed_sps_by_delta(self, delta_steps: int) -> StepsPerSecond[SPEED_CLS]:
        """ Get speed in steps per second by delta """
        ...

    @abstractmethod
    def get_speed_by_speed_sps(self, speed_sps: StepsPerSecond[SPEED_CLS]) -> SPEED_CLS:
        """ Turn a step rate of *this* axis back into a sky speed.

        The conversion needs the axis geometry (counts per revolution, gear
        ratios), which is why it is a method of the driver and not arithmetic on
        the units themselves.
        """
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
    def convert_speed_to_steps_per_second(self, speed: SPEED_CLS) -> StepsPerSecond[SPEED_CLS]:
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
