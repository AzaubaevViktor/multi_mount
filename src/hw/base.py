from abc import ABC, abstractmethod
from dataclasses import dataclass
from enum import StrEnum

from lx200.base import AxisPos
from serial_wrapper.wrapper import SerialLine


class Axis(StrEnum):
    RA = "RA"
    DEC = "DEC"


class Motion(StrEnum):
    IDLE = "idle"
    """ Not fixed """
    HOLD = "hold"
    """ Fixed """
    ACCELERATION = "acceleration"
    RUNNING = "running"
    DECELERATION = "deceleration"


@dataclass
class RawStatus:
    motor_steps: float
    motion: Motion


@dataclass
class Status:
    axis: Axis  # TODO: AxisName?
    motor_steps: float
    motor_position: float
    motion: Motion
    _raw_status: RawStatus

    @property
    def running(self) -> bool:
        return self.motion not in {Motion.IDLE, Motion.HOLD}



@dataclass
class MoveProfile[T: AxisPos]:
    name: str
    speed_sps: float
    """Ha sec/Dec arcsec per second"""
    accel: float
    microsteps: int


class AxisBase[T: AxisPos](ABC):
    def __init__(self, device: Device, axis: Axis) -> None:
        self.axis = axis
        self.device = device
        self.connected = False

    def connect(self):
        if self.connected:
            return 
        
        self.device.connect()
        self.connected = True

    def disconnect(self):
        if not self.connected:
            return
        
        self.device.disconnect()
        self.connected = False

    def reset(self):
        self.device.reset()
    
    def set_profile(self, profile: MoveProfile[T], rate: float) -> None:
        desired_steps_per_sec = self.device.to_steps(profile.speed_sps * rate)
        current_steps_per_sec = self.device.current_speed()

        # Definetly stop when direction changes or if device want to stop
        if ((desired_steps_per_sec < 0) ^ (current_steps_per_sec > 0)) or self.device.need_stop(desired_steps_per_sec):
            self.device.stop()
        
        # Now we can change speed
        self.device.change_speed(desired_steps_per_sec)
    
    def run(self):
        self.device.start_motor()
    
    def stop(self):
        self.device.stop_motor()

    def set_position(self, position: T):
        self.device.set_position(self.device.to_steps(position.to_raw()))

    def _motor_status(self) -> RawStatus:
        return self.device.status()
    
    def status(self) -> Status:
        raw_status = self.device.status()
        return Status(
            axis=self.axis,
            motor_steps=raw_status.motor_steps,
            motor_position=self.device.from_steps(raw_status.motor_steps),
            motion=raw_status.motion,
            _raw_status=raw_status,
        )


class Device(ABC):
    def __init__(self, serial: SerialLine) -> None:
        self.serial = serial
    
    def connect(self) -> None:
        self.serial.connect()

    def disconnect(self) -> None:
        self.serial.close()

    @abstractmethod
    def to_steps(self, value: float) -> float:
        ...

    @abstractmethod
    def from_steps(self, steps: float) -> float:
        ...

    @abstractmethod
    def current_speed(self) -> float:
        ...

    @abstractmethod
    def need_stop(self, desired_steps_per_sec: float) -> bool:
        ...

    def stop(self) -> None:
        self.stop_motor()

    @abstractmethod
    def change_speed(self, speed_steps_per_sec: float) -> None:
        ...

    @abstractmethod
    def start_motor(self) -> None:
        ...

    @abstractmethod
    def stop_motor(self) -> None:
        ...

    @abstractmethod
    def set_position(self, position_steps: float) -> None:
        ...
    
    @abstractmethod
    def reset(self) -> None:
        ...

    @abstractmethod
    def status(self) -> RawStatus:
        ...
