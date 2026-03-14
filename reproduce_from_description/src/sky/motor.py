from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from enum import Enum
from typing import Any


class MotorDirection(Enum):
    NEGATIVE = -1
    POSITIVE = 1


class MotorMotionMode(str, Enum):
    STOPPED = "stopped"
    FREE_RIDE = "free_ride"
    TARGET = "target"


class MotorPhase(str, Enum):
    IDLE = "idle"
    ACCEL = "accel"
    CRUISE = "cruise"
    DECEL = "decel"
    TARGET_REACHED = "target_reached"
    ERROR = "error"


@dataclass(slots=True)
class MotorStatus:
    connected: bool
    position_steps: int
    mode: MotorMotionMode
    phase: MotorPhase
    direction: MotorDirection
    speed_sps: float
    actual_speed_sps: float
    target_steps: int | None = None
    target_set: bool = False
    enabled: bool = True
    raw: dict[str, Any] | None = None

    @property
    def running(self) -> bool:
        return self.phase not in {MotorPhase.IDLE, MotorPhase.TARGET_REACHED, MotorPhase.ERROR}


class Motor(ABC):
    @abstractmethod
    def connect(self) -> None:
        raise NotImplementedError

    @abstractmethod
    def disconnect(self) -> None:
        raise NotImplementedError

    @abstractmethod
    def status(self) -> MotorStatus:
        raise NotImplementedError

    @abstractmethod
    def set_steps(self, steps: int) -> None:
        raise NotImplementedError

    @abstractmethod
    def set_speed(self, steps_per_second: float) -> None:
        raise NotImplementedError

    @abstractmethod
    def set_acceleration(self, steps_per_second_square: float) -> None:
        raise NotImplementedError

    @abstractmethod
    def set_direction(self, direction: MotorDirection) -> None:
        raise NotImplementedError

    @abstractmethod
    def set_delta(self, delta_steps: int) -> None:
        raise NotImplementedError

    @abstractmethod
    def get_speed_sps_by_delta(self, delta_steps: int) -> float:
        raise NotImplementedError

    @abstractmethod
    def get_speed_by_speed_sps(self, steps_per_second: float) -> Any:
        raise NotImplementedError

    @abstractmethod
    def set_motion_mode(self, mode: MotorMotionMode) -> None:
        raise NotImplementedError

    @abstractmethod
    def set_microsteps(self, microsteps: int) -> None:
        raise NotImplementedError

    @abstractmethod
    def convert_position_to_steps(self, position: Any) -> int:
        raise NotImplementedError

    @abstractmethod
    def convert_steps_to_position(self, steps: int) -> Any:
        raise NotImplementedError

    @abstractmethod
    def convert_speed_to_steps_per_second(self, speed: Any) -> float:
        raise NotImplementedError

    @abstractmethod
    def run(self) -> None:
        raise NotImplementedError

    @abstractmethod
    def stop(self) -> None:
        raise NotImplementedError

    @abstractmethod
    def wait_till_stop(self, timeout: float | None = None) -> MotorStatus:
        raise NotImplementedError

    @abstractmethod
    def reset(self) -> None:
        raise NotImplementedError
