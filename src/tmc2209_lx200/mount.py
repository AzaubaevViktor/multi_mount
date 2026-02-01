from __future__ import annotations

import dataclasses
import logging
from typing import Optional

from lx200.coords import clamp, wrap_deg
from lx200.models import LX200Dec, LX200Ra
from lx200.protocol import LX200Constants, LX200GotoResult, LX200MoveDirection, LX200SlewRate, LX200SyncResult
from tmc2209.proxy import TMC2209ArduinoProxy

from .common import (
    TMC2209Axis,
    TMC2209AxisConfig,
    TMC2209AxisMapping,
    TMC2209AxisState,
    TMC2209ConfigError,
    TMC2209DirectionSign,
    TMC2209LX200Constants,
    TMC2209MountConfig,
    TMC2209OperationError,
)


@dataclasses.dataclass
class _AxisRuntime:
    state: TMC2209AxisState
    config: TMC2209AxisConfig
    proxy: Optional[TMC2209ArduinoProxy] = None
    virtual_steps: int = 0


class _TMC2209MountConstants:
    RA_DIRECTIONS = frozenset({LX200MoveDirection.EAST, LX200MoveDirection.WEST})
    POSITIVE_DIRECTIONS = frozenset({LX200MoveDirection.EAST, LX200MoveDirection.NORTH})


class TMC2209Mount:
    def __init__(
        self,
        *,
        dec_proxy: TMC2209ArduinoProxy | None = None,
        config: TMC2209MountConfig | None = None,
        logger: Optional[logging.Logger] = None,
    ) -> None:
        self._log = logger or logging.getLogger(TMC2209LX200Constants.LOGGER_NAME)
        self._config = config or TMC2209MountConfig()
        if self._config.ra_axis_config is not None:
            raise TMC2209ConfigError("RA axis is not supported for DEC-only mount")
        self._dec_proxy = dec_proxy
        self._slew_rate = LX200SlewRate.CENTER
        self._target_ra = self._config.initial_ra
        self._target_dec = self._config.initial_dec
        self._dec_axis: _AxisRuntime | None = None
        self._initialize_axes()

    def close(self) -> None:
        if self._dec_proxy is not None:
            self._dec_proxy.close()

    def set_slew_rate(self, rate: LX200SlewRate) -> None:
        self._slew_rate = rate

    def get_current_ra(self) -> LX200Ra:
        return self._target_ra

    def get_current_dec(self) -> LX200Dec:
        runtime = self._dec_axis
        if runtime is None:
            return self._target_dec
        degrees = self._axis_current_degrees(runtime, wrap=False)
        degrees = clamp(degrees, LX200Constants.MIN_LAT_DEG, LX200Constants.MAX_LAT_DEG)
        return LX200Dec(degrees=degrees)

    def set_target_ra(self, ra: LX200Ra) -> bool:
        self._target_ra = ra
        return True

    def set_target_dec(self, dec: LX200Dec) -> bool:
        self._target_dec = dec
        return True

    def slew_to_target(self) -> LX200GotoResult:
        ra_result = self._slew_axis_to_target(TMC2209Axis.RA, self._target_ra.hours, wrap=True)
        dec_result = self._slew_axis_to_target(TMC2209Axis.DEC, self._target_dec.degrees, wrap=False)
        if ra_result and dec_result:
            return LX200GotoResult.ALREADY_THERE
        return LX200GotoResult.OK

    def sync_to_target(self) -> LX200SyncResult:
        self._sync_axis(TMC2209Axis.RA, self._target_ra.hours, wrap=True)
        self._sync_axis(TMC2209Axis.DEC, self._target_dec.degrees, wrap=False)
        return LX200SyncResult.OK

    def stop_all(self) -> None:
        if self._dec_axis is None:
            return None
        if self._dec_axis.proxy is not None:
            self._dec_axis.proxy.stop()

    def start_move(self, direction: LX200MoveDirection) -> None:
        axis = self._axis_for_direction(direction)
        if axis == TMC2209Axis.RA:
            return None
        runtime = self._require_axis(axis, direction.name)
        sps = self._speed_for_axis(runtime.config, self._slew_rate)
        signed_sps = self._signed_speed_for_direction(runtime.state, direction, sps)
        self._ensure_enabled(runtime)
        runtime.proxy.run(signed_sps)

    def stop_move(self, direction: LX200MoveDirection) -> None:
        axis = self._axis_for_direction(direction)
        if axis == TMC2209Axis.RA:
            return None
        runtime = self._require_axis(axis, direction.name)
        runtime.proxy.stop()

    def _initialize_axes(self) -> None:
        if self._dec_proxy is not None and self._config.dec_axis_config is None:
            raise TMC2209ConfigError("DEC axis config is required when DEC proxy is set")
        if self._config.dec_axis_config is not None:
            if self._dec_proxy is None:
                raise TMC2209ConfigError("DEC proxy is required when DEC axis config is set")
            state = self._create_axis_state(
                axis=TMC2209Axis.DEC,
                config=self._config.dec_axis_config,
                mapping=self._config.axis_mapping,
                zero_deg=self._config.initial_dec.degrees,
                proxy=self._dec_proxy,
            )
            self._dec_axis = _AxisRuntime(
                state=state,
                config=self._config.dec_axis_config,
                proxy=self._dec_proxy,
            )

    def _create_axis_state(
        self,
        *,
        axis: TMC2209Axis,
        config: TMC2209AxisConfig,
        mapping: TMC2209AxisMapping,
        zero_deg: float,
        proxy: TMC2209ArduinoProxy,
    ) -> TMC2209AxisState:
        if axis == TMC2209Axis.RA:
            direction_sign = (
                TMC2209DirectionSign.POSITIVE
                if mapping.ra_forward_is_east
                else TMC2209DirectionSign.NEGATIVE
            )
        else:
            direction_sign = (
                TMC2209DirectionSign.POSITIVE
                if mapping.dec_forward_is_north
                else TMC2209DirectionSign.NEGATIVE
            )
        zero_steps = proxy.get_position()
        return TMC2209AxisState(
            axis=axis,
            steps_per_degree=config.steps_per_degree,
            zero_steps=zero_steps,
            zero_deg=zero_deg,
            direction_sign=direction_sign,
        )

    def _axis_current_degrees(self, runtime: _AxisRuntime, *, wrap: bool) -> float:
        steps = self._axis_current_steps(runtime)
        delta = runtime.state.degrees_from_steps(steps - runtime.state.zero_steps)
        degrees = runtime.state.zero_deg + delta
        if wrap:
            return wrap_deg(degrees)
        return degrees

    def _axis_current_steps(self, runtime: _AxisRuntime) -> int:
        if runtime.proxy is None:
            return runtime.virtual_steps
        return runtime.proxy.get_position()

    def _slew_axis_to_target(self, axis: TMC2209Axis, target_value: float, *, wrap: bool) -> bool:
        if axis == TMC2209Axis.RA:
            return True
        runtime = self._dec_axis
        if runtime is None:
            return True
        # TODO: deduplicate target degree computation shared with _sync_axis.
        target_deg = clamp(target_value, LX200Constants.MIN_LAT_DEG, LX200Constants.MAX_LAT_DEG)
        current_deg = self._axis_current_degrees(runtime, wrap=wrap)
        delta_deg = target_deg - current_deg
        if wrap:
            delta_deg = self._wrap_delta_degrees(delta_deg)
        steps = runtime.state.steps_from_degrees(delta_deg)
        if abs(steps) <= runtime.config.tolerance_steps:
            return True
        self._ensure_enabled(runtime)
        runtime.proxy.move(steps, runtime.config.goto_sps)
        return False

    def _sync_axis(self, axis: TMC2209Axis, target_value: float, *, wrap: bool) -> None:
        if axis == TMC2209Axis.RA:
            return None
        runtime = self._dec_axis
        if runtime is None:
            return None
        target_deg = clamp(target_value, LX200Constants.MIN_LAT_DEG, LX200Constants.MAX_LAT_DEG)
        current_steps = self._axis_current_steps(runtime)
        runtime.state.zero_steps = current_steps
        runtime.state.zero_deg = target_deg if not wrap else wrap_deg(target_deg)
        if runtime.proxy is None:
            runtime.virtual_steps = current_steps

    def _speed_for_axis(self, config: TMC2209AxisConfig, rate: LX200SlewRate) -> int:
        if rate == LX200SlewRate.GUIDE:
            return config.guide_sps
        if rate == LX200SlewRate.CENTER:
            return config.center_sps
        if rate == LX200SlewRate.FIND:
            return config.find_sps
        return config.slew_sps

    def _axis_for_direction(self, direction: LX200MoveDirection) -> TMC2209Axis:
        if direction in _TMC2209MountConstants.RA_DIRECTIONS:
            return TMC2209Axis.RA
        return TMC2209Axis.DEC

    def _signed_speed_for_direction(
        self,
        state: TMC2209AxisState,
        direction: LX200MoveDirection,
        sps: int,
    ) -> int:
        if direction in _TMC2209MountConstants.POSITIVE_DIRECTIONS:
            return sps * int(state.direction_sign)
        return -sps * int(state.direction_sign)

    def _ensure_enabled(self, runtime: _AxisRuntime) -> None:
        if runtime.proxy is None:
            return None
        if runtime.config.auto_enable:
            runtime.proxy.enable(True)

    def _wrap_delta_degrees(self, delta: float) -> float:
        full = TMC2209LX200Constants.DEGREES_PER_REV
        half = TMC2209LX200Constants.HALF_DEGREES_PER_REV
        wrapped = (delta + half) % full - half
        if wrapped == -half:
            return half
        return wrapped

    def _require_axis(self, axis: TMC2209Axis, op: str) -> _AxisRuntime:
        if axis == TMC2209Axis.RA:
            raise TMC2209OperationError(f"{op} requires {axis.value} axis")
        runtime = self._dec_axis
        if runtime is None or runtime.proxy is None:
            raise TMC2209OperationError(f"{op} requires {axis.value} axis")
        return runtime
