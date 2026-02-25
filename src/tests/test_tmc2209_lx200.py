from types import SimpleNamespace

from tmc2209.tmc2209_lx200 import TMC2209LX200


class _FakeAdapter:
    def __init__(self) -> None:
        self.steps_per_rev = 1000.0
        self.position = 0

        self.speed_calls: list[int] = []
        self.direction_calls: list[bool] = []
        self.run_calls = 0
        self.halt_calls = 0

    def status(self):
        return SimpleNamespace(position=self.position, phase="idle")

    def set_microsteps(self, _microsteps: int) -> bool:
        return True

    def set_position(self, position: int) -> int:
        self.position = position
        return position

    def set_acceleration_steps_per_ms(self, _accel: int) -> float:
        return 0.0

    def set_speed_sps(self, speed_sps: int) -> float:
        self.speed_calls.append(speed_sps)
        return float(speed_sps)

    def set_free_ride_mode(self):
        return "free_ride"

    def set_direction(self, direction: bool) -> bool:
        self.direction_calls.append(direction)
        return direction

    def run(self) -> bool:
        self.run_calls += 1
        return True

    def halt(self) -> bool:
        self.halt_calls += 1
        return True

    def close(self) -> None:
        return None


def test_set_tracking_rate_negative_uses_positive_speed_and_north_direction() -> None:
    adapter = _FakeAdapter()
    axis = TMC2209LX200(adapter)
    try:
        axis._set_tracking_rate(-0.5)
        assert adapter.speed_calls[-1] == 50
        assert adapter.direction_calls[-1] is False
        assert adapter.run_calls == 1
    finally:
        axis.stop()


def test_set_tracking_rate_positive_uses_positive_speed_and_south_direction() -> None:
    adapter = _FakeAdapter()
    axis = TMC2209LX200(adapter)
    try:
        axis._set_tracking_rate(0.5)
        assert adapter.speed_calls[-1] == 50
        assert adapter.direction_calls[-1] is True
        assert adapter.run_calls == 1
    finally:
        axis.stop()

