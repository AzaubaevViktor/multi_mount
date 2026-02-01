from __future__ import annotations

import dataclasses

from lx200.models import LX200Ra
from lx200.protocol import LX200GotoResult, LX200MoveDirection, LX200SlewRate
from tmc2209_lx200.common import (
    TMC2209AxisConfig,
    TMC2209AxisMapping,
    TMC2209LX200Constants,
    TMC2209MountConfig,
)
from tmc2209_lx200.mount import TMC2209Mount


class TMC2209LX200TestConstants:
    ZERO_INT = 0
    RA_WRAP_START_HOURS = 23.0
    RA_WRAP_TARGET_HOURS = 1.0
    RA_WRAP_STEPS_PER_DEG = 1.0
    RA_WRAP_EXPECTED_STEPS = 30
    RA_SYNC_START_POS = 100
    RA_SYNC_TARGET_HOURS = 10.0
    STEPS_PER_DEG = 2.0
    GUIDE_SPS = 10
    CENTER_SPS = 20
    FIND_SPS = 40
    SLEW_SPS = 80
    GOTO_SPS = 60
    TOLERANCE_STEPS = 1


@dataclasses.dataclass
class _FakeProxy:
    position: int = TMC2209LX200TestConstants.ZERO_INT
    last_run_sps: int | None = None
    last_move: tuple[int, int] | None = None
    stopped: bool = False
    enabled: bool = False

    def get_position(self) -> int:
        return self.position

    def enable(self, enabled: bool) -> bool:
        self.enabled = enabled
        return enabled

    def run(self, steps_per_second: int) -> list[str]:
        self.last_run_sps = steps_per_second
        return []

    def move(self, steps: int, steps_per_second: int | None = None) -> list[str]:
        if steps_per_second is None:
            raise ValueError("steps per second required")
        self.last_move = (steps, steps_per_second)
        self.position += steps
        return []

    def stop(self) -> list[str]:
        self.stopped = True
        return []

    def close(self) -> None:
        return None


def _axis_config() -> TMC2209AxisConfig:
    return TMC2209AxisConfig(
        steps_per_degree=TMC2209LX200TestConstants.STEPS_PER_DEG,
        guide_sps=TMC2209LX200TestConstants.GUIDE_SPS,
        center_sps=TMC2209LX200TestConstants.CENTER_SPS,
        find_sps=TMC2209LX200TestConstants.FIND_SPS,
        slew_sps=TMC2209LX200TestConstants.SLEW_SPS,
        goto_sps=TMC2209LX200TestConstants.GOTO_SPS,
        tolerance_steps=TMC2209LX200TestConstants.TOLERANCE_STEPS,
    )


def test_start_move_direction_mapping() -> None:
    ra_proxy = _FakeProxy()
    mapping = TMC2209AxisMapping(ra_forward_is_east=True, dec_forward_is_north=False)
    config = TMC2209MountConfig(
        axis_mapping=mapping,
        ra_axis_config=_axis_config(),
    )
    mount = TMC2209Mount(ra_proxy=ra_proxy, config=config)
    mount.set_slew_rate(LX200SlewRate.FIND)

    mount.start_move(LX200MoveDirection.EAST)
    assert ra_proxy.last_run_sps == TMC2209LX200TestConstants.FIND_SPS

    mount.start_move(LX200MoveDirection.WEST)
    assert ra_proxy.last_run_sps == -TMC2209LX200TestConstants.FIND_SPS


def test_slew_to_target_wraps_ra() -> None:
    ra_proxy = _FakeProxy()
    ra_config = TMC2209AxisConfig(
        steps_per_degree=TMC2209LX200TestConstants.RA_WRAP_STEPS_PER_DEG,
        guide_sps=TMC2209LX200Constants.DEFAULT_GUIDE_SPS,
        center_sps=TMC2209LX200Constants.DEFAULT_CENTER_SPS,
        find_sps=TMC2209LX200Constants.DEFAULT_FIND_SPS,
        slew_sps=TMC2209LX200Constants.DEFAULT_SLEW_SPS,
        goto_sps=TMC2209LX200TestConstants.GOTO_SPS,
        tolerance_steps=TMC2209LX200TestConstants.TOLERANCE_STEPS,
    )
    config = TMC2209MountConfig(
        ra_axis_config=ra_config,
        initial_ra=LX200Ra(hours=TMC2209LX200TestConstants.RA_WRAP_START_HOURS),
    )
    mount = TMC2209Mount(ra_proxy=ra_proxy, config=config)
    mount.set_target_ra(LX200Ra(hours=TMC2209LX200TestConstants.RA_WRAP_TARGET_HOURS))

    result = mount.slew_to_target()

    assert result == LX200GotoResult.OK
    assert ra_proxy.last_move == (
        TMC2209LX200TestConstants.RA_WRAP_EXPECTED_STEPS,
        TMC2209LX200TestConstants.GOTO_SPS,
    )


def test_sync_to_target_updates_zero() -> None:
    ra_proxy = _FakeProxy(position=TMC2209LX200TestConstants.RA_SYNC_START_POS)
    config = TMC2209MountConfig(
        ra_axis_config=_axis_config(),
        initial_ra=LX200Ra(hours=TMC2209LX200Constants.ZERO_FLOAT),
    )
    mount = TMC2209Mount(ra_proxy=ra_proxy, config=config)
    mount.set_target_ra(LX200Ra(hours=TMC2209LX200TestConstants.RA_SYNC_TARGET_HOURS))

    mount.sync_to_target()

    assert mount.get_current_ra().hours == TMC2209LX200TestConstants.RA_SYNC_TARGET_HOURS
