from sky.motor import MotionMode, Motor, MotorDirection, MotorStatus
from sky.physics import AxisPos, AxisSpeed


class UnavailableMotor[_POS_CLS: AxisPos, _SPEED_CLS: AxisSpeed](Motor[_POS_CLS, _SPEED_CLS]):
    def __init__(
        self,
        pos_cls: type[_POS_CLS],
        speed_cls: type[_SPEED_CLS],
        forward_position_sign: int,
        reason: str,
    ) -> None:
        self._pos_cls = pos_cls
        self._speed_cls = speed_cls
        self.FORWARD_POSITION_SIGN = forward_position_sign
        self._reason = reason

    def connect(self):
        raise ConnectionError(self._reason)

    def disconnect(self) -> bool:
        return True

    def status(self) -> MotorStatus:
        return MotorStatus(
            is_connected=False,
            steps=0,
            motion_mode=MotionMode.IDLE,
            speed_sps=0,
            accel_sps=None,
            direction=MotorDirection.STOP,
            target=None,
            microsteps=None,
            power_v=None,
        )

    def get_power_v(self) -> float | None:
        return None

    def set_steps(self, steps: int) -> bool:
        raise ConnectionError(self._reason)

    def set_speed(self, steps_per_second: int) -> int:
        raise ConnectionError(self._reason)

    def set_acceleration(self, steps_per_second_square: float) -> bool:
        raise ConnectionError(self._reason)

    def set_direction(self, direction: MotorDirection) -> bool:
        raise ConnectionError(self._reason)

    def set_delta(self, delta_steps: int) -> bool:
        raise ConnectionError(self._reason)

    def get_speed_sps_by_delta(self, delta_steps: int) -> int:
        return abs(delta_steps)

    def get_speed_by_speed_sps(self, speed_sps: int) -> _SPEED_CLS:
        return self._speed_cls(float(speed_sps))

    def set_motion_mode(self, motion_mode: MotionMode) -> bool:
        raise ConnectionError(self._reason)

    def set_microsteps(self, microsteps: int) -> bool:
        raise ConnectionError(self._reason)

    def convert_position_to_steps(self, position: _POS_CLS) -> int:
        return int(round(float(position)))

    def convert_steps_to_position(self, steps: int) -> _POS_CLS:
        return self._pos_cls(float(steps))

    def convert_speed_to_steps_per_second(self, speed: _SPEED_CLS) -> int:
        return int(round(abs(float(speed))))

    def run(self) -> bool:
        raise ConnectionError(self._reason)

    def stop(self) -> bool:
        return True

    def wait_till_stop(self, do_stop: bool = True, timeout_s: float | None = None) -> None:
        return None

    def reset(self) -> None:
        return None
