from abc import ABC, abstractmethod
from dataclasses import dataclass
from enum import StrEnum
import time

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


class MoveMode(StrEnum):
    STOP = "stop"
    MOVING = "moving"
    TARGET = "target"


@dataclass
class RawStatus:
    motor_steps: float
    motion: Motion
    mode: MoveMode

    @property
    def running(self) -> bool:
        return self.motion not in {Motion.IDLE, Motion.HOLD}


@dataclass
class Status:
    axis: Axis  # TODO: AxisName?
    motor_steps: float
    motor_position: float
    motion: Motion
    mode: MoveMode
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
    _WAIT_POLL_INTERVAL_S = 0.2

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

    def wait_till_stop(self, timeout_s: float | None = None):
        if timeout_s is not None and timeout_s <= 0:
            raise ValueError("timeout_s must be positive")

        self.stop()

        deadline = None
        if timeout_s is not None:
            deadline = time.monotonic() + timeout_s

        while True:
            if not self._motor_status().running:
                return
            if deadline is not None and time.monotonic() > deadline:
                raise TimeoutError(f"motor did not stop within {timeout_s}s")

            time.sleep(self._WAIT_POLL_INTERVAL_S)
    
    def set_profile(self, profile: MoveProfile[T], rate: float) -> None:
        desired_steps_per_sec = self.device.to_steps(profile.speed_sps * rate)
        current_steps_per_sec = self.device.current_speed()

        # Definetly stop when direction changes or if device want to stop
        if ((desired_steps_per_sec < 0) ^ (current_steps_per_sec > 0)) or self.device.need_stop(desired_steps_per_sec):
            self.wait_till_stop()
        
        # Now we can change speed
        self.device.change_speed(desired_steps_per_sec)
    
    def run(self):
        self.device.start_motor()
    
    def stop(self):
        self.device.stop_motor()

    def move_to_target(self, delta: T):
        self.wait_till_stop()
        self.device.move_to_target(self.device.to_steps(delta.to_raw()))
        self.run()

    def set_position(self, position: T):
        if self._motor_status().mode == MoveMode.TARGET:
            self.wait_till_stop()
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
            mode=raw_status.mode,
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
    def move_to_target(self, delta_steps: float) -> None:
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
