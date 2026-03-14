from __future__ import annotations

from dataclasses import dataclass

from sky.axis import AxisSnapshot, AxisMotionMode
from sky.motor import Motor, MotorDirection, MotorMotionMode, MotorPhase, MotorStatus
from sky.physics import Dec, DecPerSecond, Ha, HaPerSecond, PointCoordinates


class FakeRAMotor(Motor):
    def __init__(self, steps_per_hour: int = 1000) -> None:
        self.steps_per_hour = steps_per_hour
        self.connected = False
        self.position_steps = 0
        self.direction = MotorDirection.POSITIVE
        self.mode = MotorMotionMode.STOPPED
        self.phase = MotorPhase.IDLE
        self.speed_sps = 0.0
        self.target_steps: int | None = None

    def connect(self) -> None:
        self.connected = True

    def disconnect(self) -> None:
        self.connected = False

    def status(self) -> MotorStatus:
        if self.mode == MotorMotionMode.TARGET and self.phase == MotorPhase.CRUISE and self.target_steps is not None:
            signed_target = self.target_steps if self.direction == MotorDirection.POSITIVE else -self.target_steps
            self.position_steps += signed_target
            self.phase = MotorPhase.TARGET_REACHED
        return MotorStatus(
            connected=self.connected,
            position_steps=self.position_steps,
            mode=self.mode,
            phase=self.phase,
            direction=self.direction,
            speed_sps=self.speed_sps,
            actual_speed_sps=self.speed_sps if self.phase == MotorPhase.CRUISE else 0.0,
            target_steps=self.target_steps,
            target_set=self.target_steps is not None,
        )

    def set_steps(self, steps: int) -> None:
        self.position_steps = steps

    def set_speed(self, steps_per_second: float) -> None:
        self.speed_sps = steps_per_second

    def set_acceleration(self, steps_per_second_square: float) -> None:
        pass

    def set_direction(self, direction: MotorDirection) -> None:
        self.direction = direction

    def set_delta(self, delta_steps: int) -> None:
        self.target_steps = delta_steps

    def get_speed_sps_by_delta(self, delta_steps: int) -> float:
        return float(max(1, abs(delta_steps)))

    def get_speed_by_speed_sps(self, steps_per_second: float) -> HaPerSecond:
        return HaPerSecond(steps_per_second / self.steps_per_hour)

    def set_motion_mode(self, mode: MotorMotionMode) -> None:
        self.mode = mode

    def set_microsteps(self, microsteps: int) -> None:
        pass

    def convert_position_to_steps(self, position: Ha) -> int:
        return int(round(position.hours * self.steps_per_hour))

    def convert_steps_to_position(self, steps: int) -> Ha:
        return Ha(steps / self.steps_per_hour)

    def convert_speed_to_steps_per_second(self, speed: HaPerSecond) -> float:
        return speed.hours_per_second * self.steps_per_hour

    def run(self) -> None:
        self.phase = MotorPhase.CRUISE

    def stop(self) -> None:
        self.phase = MotorPhase.IDLE
        self.mode = MotorMotionMode.STOPPED

    def wait_till_stop(self, timeout: float | None = None) -> MotorStatus:
        return self.status()

    def reset(self) -> None:
        self.stop()


class FakeDECMotor(Motor):
    def __init__(self, steps_per_degree: int = 100) -> None:
        self.steps_per_degree = steps_per_degree
        self.connected = False
        self.position_steps = 0
        self.direction = MotorDirection.POSITIVE
        self.mode = MotorMotionMode.STOPPED
        self.phase = MotorPhase.IDLE
        self.speed_sps = 0.0
        self.target_steps: int | None = None

    def connect(self) -> None:
        self.connected = True

    def disconnect(self) -> None:
        self.connected = False

    def status(self) -> MotorStatus:
        if self.mode == MotorMotionMode.TARGET and self.phase == MotorPhase.CRUISE and self.target_steps is not None:
            signed_target = self.target_steps if self.direction == MotorDirection.POSITIVE else -self.target_steps
            self.position_steps += signed_target
            self.phase = MotorPhase.TARGET_REACHED
        return MotorStatus(
            connected=self.connected,
            position_steps=self.position_steps,
            mode=self.mode,
            phase=self.phase,
            direction=self.direction,
            speed_sps=self.speed_sps,
            actual_speed_sps=self.speed_sps if self.phase == MotorPhase.CRUISE else 0.0,
            target_steps=self.target_steps,
            target_set=self.target_steps is not None,
        )

    def set_steps(self, steps: int) -> None:
        self.position_steps = steps

    def set_speed(self, steps_per_second: float) -> None:
        self.speed_sps = steps_per_second

    def set_acceleration(self, steps_per_second_square: float) -> None:
        pass

    def set_direction(self, direction: MotorDirection) -> None:
        self.direction = direction

    def set_delta(self, delta_steps: int) -> None:
        self.target_steps = delta_steps

    def get_speed_sps_by_delta(self, delta_steps: int) -> float:
        return float(max(1, abs(delta_steps)))

    def get_speed_by_speed_sps(self, steps_per_second: float) -> DecPerSecond:
        return DecPerSecond(steps_per_second / self.steps_per_degree)

    def set_motion_mode(self, mode: MotorMotionMode) -> None:
        self.mode = mode

    def set_microsteps(self, microsteps: int) -> None:
        pass

    def convert_position_to_steps(self, position: Dec) -> int:
        return int(round(position.degrees * self.steps_per_degree))

    def convert_steps_to_position(self, steps: int) -> Dec:
        return Dec(steps / self.steps_per_degree)

    def convert_speed_to_steps_per_second(self, speed: DecPerSecond) -> float:
        return speed.degrees_per_second * self.steps_per_degree

    def run(self) -> None:
        self.phase = MotorPhase.CRUISE

    def stop(self) -> None:
        self.phase = MotorPhase.IDLE
        self.mode = MotorMotionMode.STOPPED

    def wait_till_stop(self, timeout: float | None = None) -> MotorStatus:
        return self.status()

    def reset(self) -> None:
        self.stop()


@dataclass
class AxisRecorder:
    position: Ha | Dec
    tracking_speed: HaPerSecond | DecPerSecond
    moves: list[tuple[object, AxisMotionMode]]
    halted: int = 0
    mode: AxisMotionMode = AxisMotionMode.STOPPED

    def connect(self) -> None:
        return None

    def disconnect(self) -> None:
        return None

    def sync_to(self, position: Ha | Dec) -> None:
        self.position = position

    def goto_to(self, position: Ha | Dec) -> None:
        self.position = position
        self.mode = AxisMotionMode.GOTO

    def change_speed(self, speed: HaPerSecond | DecPerSecond) -> None:
        self.tracking_speed = speed

    def move(self, speed: HaPerSecond | DecPerSecond, mode: AxisMotionMode = AxisMotionMode.SLEW) -> None:
        self.moves.append((speed, mode))
        self.mode = mode

    def halt(self) -> None:
        self.halted += 1
        self.mode = AxisMotionMode.TRACK

    def status(self) -> AxisSnapshot:
        return AxisSnapshot(
            position=self.position,
            mode=self.mode,
            tracking_speed=self.tracking_speed,
            active_speed=self.tracking_speed,
            motor_status=None,
        )
