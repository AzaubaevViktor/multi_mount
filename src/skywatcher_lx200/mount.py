from __future__ import annotations

import logging
from typing import Optional

from lx200.coords import clamp, wrap_deg, wrap_hours
from lx200.models import LX200Dec, LX200Ra
from lx200.protocol import (
    LX200Constants,
    LX200GotoResult,
    LX200MoveDirection,
    LX200SlewRate,
    LX200SyncResult,
)
from lib.serial_prims import SerialLineDevice
from lib.skywatcher import (
    SkyWatcherAxis,
    SkyWatcherDirection,
    SkyWatcherMC,
    SkyWatcherMotionMode,
    SkyWatcherSlewMode,
    SkyWatcherSpeedMode,
    SkyWatcherTrackingError,
)

from .common import (
    SkyWatcherAxisState,
    SkyWatcherAxisStateError,
    SkyWatcherBackendConstants,
    SkyWatcherInitializationError,
    SkyWatcherMountConfig,
    SkyWatcherOperationError,
    SkyWatcherSerialConfig,
)


class SkyWatcherMount:
    def __init__(
        self,
        mc: SkyWatcherMC,
        config: SkyWatcherMountConfig | None = None,
        *,
        logger: Optional[logging.Logger] = None,
    ) -> None:
        self._log = logger or logging.getLogger(SkyWatcherBackendConstants.LOGGER_NAME)
        self._mc = mc
        self._config = config or SkyWatcherMountConfig()
        self._log.info("skywatcher mount init config=%r", self._config)
        self._device: Optional[SerialLineDevice] = None
        self._axis_states: dict[SkyWatcherAxis, SkyWatcherAxisState] = {}
        self._slew_rate = LX200SlewRate.CENTER
        self._target_ra = self._config.initial_ra
        self._target_dec = self._config.initial_dec
        self._initialized = False
        self._refresh_axis_states()
        if self._config.auto_initialize:
            self.initialize()

    @classmethod
    def from_serial(
        cls,
        serial_config: SkyWatcherSerialConfig,
        mount_config: SkyWatcherMountConfig | None = None,
        *,
        logger: Optional[logging.Logger] = None,
    ) -> "SkyWatcherMount":
        log = logger or logging.getLogger(SkyWatcherBackendConstants.LOGGER_NAME)
        log.info(
            "skywatcher connect port=%s baud=%s timeout_s=%s",
            serial_config.port,
            serial_config.baud,
            serial_config.timeout_s,
        )
        dev = SerialLineDevice(
            serial_config.port,
            serial_config.baud,
            serial_config.timeout_s,
            serial_config.device_name,
        )
        mc = SkyWatcherMC(dev, logger=logger)
        mount = cls(mc=mc, config=mount_config, logger=logger)
        mount._device = dev
        return mount

    def initialize(self) -> None:
        self._log.info("initializing skywatcher axes")
        timed_out_axes: list[SkyWatcherAxis] = []
        for axis in (self._config.axis_mapping.ra_axis, self._config.axis_mapping.dec_axis):
            try:
                self._mc.do_initialize(
                    axis,
                    timeout_s=self._config.init_config.timeout_s,
                    poll_interval_s=self._config.init_config.poll_interval_s,
                )
            except TimeoutError:
                timed_out_axes.append(axis)
        if len(timed_out_axes) >= SkyWatcherBackendConstants.TWO_INT:
            raise SkyWatcherInitializationError("skywatcher init timeout for all axes")
        if timed_out_axes:
            timed_out = ",".join(axis.name for axis in timed_out_axes)
            self._log.warning("skywatcher init timeout for axes=%s; continuing", timed_out)
        self._initialized = True
        self._refresh_axis_states(reset_zero=True)

    def close(self) -> None:
        self._log.info("skywatcher mount close")
        if self._device is not None:
            self._device.close()

    def set_slew_rate(self, rate: LX200SlewRate) -> None:
        self._slew_rate = rate
        self._log.info("skywatcher slew rate set=%s", rate)

    def get_current_ra(self) -> LX200Ra:
        state = self._axis_states[self._config.axis_mapping.ra_axis]
        ticks = self._mc.inquire_position(state.axis)
        deg = self._axis_ticks_to_deg(state, ticks, wrap=True)
        hours = wrap_hours(deg / SkyWatcherBackendConstants.RA_DEG_PER_HOUR)
        return LX200Ra(hours=hours)

    def get_current_dec(self) -> LX200Dec:
        state = self._axis_states[self._config.axis_mapping.dec_axis]
        ticks = self._mc.inquire_position(state.axis)
        deg = self._axis_ticks_to_deg(state, ticks, wrap=False)
        deg = clamp(deg, LX200Constants.MIN_LAT_DEG, LX200Constants.MAX_LAT_DEG)
        return LX200Dec(degrees=deg)

    def set_target_ra(self, ra: LX200Ra) -> bool:
        self._target_ra = ra
        self._log.info("skywatcher target ra=%s", ra)
        return True

    def set_target_dec(self, dec: LX200Dec) -> bool:
        self._target_dec = dec
        self._log.info("skywatcher target dec=%s", dec)
        return True

    def slew_to_target(self) -> LX200GotoResult:
        ra_delta = self._compute_target_delta(self._config.axis_mapping.ra_axis, self._target_ra.hours)
        dec_delta = self._compute_target_delta(self._config.axis_mapping.dec_axis, self._target_dec.degrees)
        self._log.info("skywatcher slew_to_target ra_delta=%s dec_delta=%s", ra_delta, dec_delta)
        if self._is_delta_within_tolerance(ra_delta) and self._is_delta_within_tolerance(dec_delta):
            return LX200GotoResult.ALREADY_THERE
        self._run_goto(self._config.axis_mapping.ra_axis, ra_delta)
        self._run_goto(self._config.axis_mapping.dec_axis, dec_delta)
        return LX200GotoResult.OK

    def sync_to_target(self) -> LX200SyncResult:
        self._log.info("skywatcher sync_to_target ra=%s dec=%s", self._target_ra, self._target_dec)
        self._sync_axis(self._config.axis_mapping.ra_axis, self._target_ra.hours, wrap=True)
        self._sync_axis(self._config.axis_mapping.dec_axis, self._target_dec.degrees, wrap=False)
        return LX200SyncResult.OK

    def stop_all(self) -> None:
        self._log.info("skywatcher stop_all")
        self._mc.stop_motion(self._config.axis_mapping.ra_axis)
        self._mc.stop_motion(self._config.axis_mapping.dec_axis)

    def start_move(self, direction: LX200MoveDirection) -> None:
        axis, sky_dir = self._map_direction(direction)
        rate = self._rate_for_direction(sky_dir)
        self._log.info(
            "skywatcher start_move axis=%s direction=%s sky_dir=%s rate=%s",
            axis,
            direction,
            sky_dir,
            rate,
        )
        try:
            self._mc.set_ra_rate(rate, axis=axis)
        except SkyWatcherTrackingError as exc:
            raise SkyWatcherOperationError(str(exc)) from exc
        self._mc.start_motion(axis)

    def stop_move(self, direction: LX200MoveDirection) -> None:
        axis, _ = self._map_direction(direction)
        self._log.info("skywatcher stop_move axis=%s direction=%s", axis, direction)
        self._mc.stop_motion(axis)

    def _refresh_axis_states(self, *, reset_zero: bool = False) -> None:
        for axis in (self._config.axis_mapping.ra_axis, self._config.axis_mapping.dec_axis):
            cpr = self._mc.inquire_cpr(axis)
            if cpr <= SkyWatcherBackendConstants.ZERO_INT:
                raise SkyWatcherAxisStateError("axis CPR must be positive")
            ticks = self._mc.inquire_position(axis)
            if axis in self._axis_states:
                state = self._axis_states[axis]
                state.cpr = cpr
                if reset_zero:
                    state.zero_ticks = ticks
            else:
                zero_deg = self._initial_zero_deg(axis)
                self._axis_states[axis] = SkyWatcherAxisState(
                    axis=axis,
                    cpr=cpr,
                    zero_ticks=ticks,
                    zero_deg=zero_deg,
                )
            self._log.info(
                "skywatcher axis_state axis=%s cpr=%s zero_ticks=%s zero_deg=%s",
                axis,
                self._axis_states[axis].cpr,
                self._axis_states[axis].zero_ticks,
                self._axis_states[axis].zero_deg,
            )

    def _axis_ticks_to_deg(self, state: SkyWatcherAxisState, ticks: int, *, wrap: bool) -> float:
        delta_ticks = self._wrap_delta_ticks(ticks - state.zero_ticks)
        deg = state.zero_deg + state.degrees_from_ticks(delta_ticks)
        if wrap:
            return wrap_deg(deg)
        return deg

    def _initial_zero_deg(self, axis: SkyWatcherAxis) -> float:
        if axis == self._config.axis_mapping.ra_axis:
            return wrap_deg(self._config.initial_ra.hours * SkyWatcherBackendConstants.RA_DEG_PER_HOUR)
        return self._config.initial_dec.degrees

    def _compute_target_delta(self, axis: SkyWatcherAxis, target_value: float) -> int:
        state = self._axis_states[axis]
        if axis == self._config.axis_mapping.ra_axis:
            target_deg = wrap_deg(target_value * SkyWatcherBackendConstants.RA_DEG_PER_HOUR)
        else:
            target_deg = clamp(target_value, LX200Constants.MIN_LAT_DEG, LX200Constants.MAX_LAT_DEG)
        delta_deg = target_deg - state.zero_deg
        raw_ticks = state.ticks_from_degrees(delta_deg)
        return self._wrap_delta_ticks(raw_ticks)

    def _run_goto(self, axis: SkyWatcherAxis, delta_ticks: int) -> None:
        direction = self._skywatcher_direction(delta_ticks)
        steps = abs(delta_ticks)
        if steps <= SkyWatcherBackendConstants.ZERO_INT:
            self._log.info("skywatcher goto axis=%s skipped: no steps", axis)
            return
        mode = SkyWatcherMotionMode(
            slew_mode=SkyWatcherSlewMode.GOTO,
            direction=direction,
            speed_mode=self._config.goto_config.speed_mode,
        )
        self._log.info(
            "skywatcher goto axis=%s direction=%s steps=%s speed_mode=%s",
            axis,
            direction,
            steps,
            self._config.goto_config.speed_mode,
        )
        self._mc.set_motion_mode(axis, mode)
        self._mc.set_step_period(axis, self._config.goto_config.step_period)
        break_count = min(self._config.goto_config.break_max, steps)
        self._mc.set_target_breaks(axis, break_count)
        self._mc.set_goto_target_increment(axis, steps)
        self._mc.set_motion_mode(axis, mode)
        self._mc.start_motion(axis)

    def _sync_axis(self, axis: SkyWatcherAxis, target_value: float, *, wrap: bool) -> None:
        state = self._axis_states[axis]
        if axis == self._config.axis_mapping.ra_axis:
            target_deg = wrap_deg(target_value * SkyWatcherBackendConstants.RA_DEG_PER_HOUR)
        else:
            target_deg = clamp(target_value, LX200Constants.MIN_LAT_DEG, LX200Constants.MAX_LAT_DEG)
        target_ticks = state.ticks_from_degrees(target_deg - state.zero_deg)
        target_ticks = self._wrap_ticks(state.zero_ticks + target_ticks)
        self._log.info(
            "skywatcher sync axis=%s target_deg=%s target_ticks=%s",
            axis,
            target_deg,
            target_ticks,
        )
        self._mc.set_axis_position(axis, target_ticks)
        state.zero_ticks = target_ticks
        state.zero_deg = target_deg if wrap else target_deg

    def _wrap_ticks(self, ticks: int) -> int:
        return ticks % SkyWatcherBackendConstants.REVU24_MOD

    def _wrap_delta_ticks(self, ticks: int) -> int:
        mod = SkyWatcherBackendConstants.REVU24_MOD
        half = SkyWatcherBackendConstants.REVU24_HALF
        return ((ticks + half) % mod) - half

    def _skywatcher_direction(self, delta_ticks: int) -> SkyWatcherDirection:
        if delta_ticks < SkyWatcherBackendConstants.ZERO_INT:
            return SkyWatcherDirection.BACKWARD
        return SkyWatcherDirection.FORWARD

    def _is_delta_within_tolerance(self, delta_ticks: int) -> bool:
        return abs(delta_ticks) <= self._config.goto_config.tolerance_ticks

    def _map_direction(self, direction: LX200MoveDirection) -> tuple[SkyWatcherAxis, SkyWatcherDirection]:
        if direction in (LX200MoveDirection.EAST, LX200MoveDirection.WEST):
            axis = self._config.axis_mapping.ra_axis
            sky_dir = self._east_west_dir(direction)
        else:
            axis = self._config.axis_mapping.dec_axis
            sky_dir = self._north_south_dir(direction)
        return axis, sky_dir

    def _east_west_dir(self, direction: LX200MoveDirection) -> SkyWatcherDirection:
        forward = self._config.axis_mapping.ra_forward_is_east
        if direction == LX200MoveDirection.EAST:
            return SkyWatcherDirection.FORWARD if forward else SkyWatcherDirection.BACKWARD
        return SkyWatcherDirection.BACKWARD if forward else SkyWatcherDirection.FORWARD

    def _north_south_dir(self, direction: LX200MoveDirection) -> SkyWatcherDirection:
        forward = self._config.axis_mapping.dec_forward_is_north
        if direction == LX200MoveDirection.NORTH:
            return SkyWatcherDirection.FORWARD if forward else SkyWatcherDirection.BACKWARD
        return SkyWatcherDirection.BACKWARD if forward else SkyWatcherDirection.FORWARD

    def _rate_for_direction(self, direction: SkyWatcherDirection) -> float:
        sign = -SkyWatcherBackendConstants.ONE_INT
        if direction == SkyWatcherDirection.FORWARD:
            sign = SkyWatcherBackendConstants.ONE_INT
        mult = self._config.slew_rate_config.multiplier_for(self._slew_rate)
        return float(sign) * mult
