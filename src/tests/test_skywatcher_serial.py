from __future__ import annotations

import dataclasses
import logging
import os
import time
from typing import Callable, Optional

import pytest

LOGGER = logging.getLogger("tests.skywatcher.serial")

from coords import clamp
from serial_prims import SerialLineDevice
from skywatcher import (
    SkyWatcherAxis,
    SkyWatcherDirection,
    SkyWatcherMC,
    SkyWatcherMotionMode,
    SkyWatcherSlewMode,
    SkyWatcherSpeedMode,
    SkyWatcherStatus,
)


@dataclasses.dataclass(frozen=True)
class SkyWatcherTestConfig:
    port: str
    baud: int
    timeout_s: float
    axis: SkyWatcherAxis
    poll_interval_s: float
    running_timeout_s: float
    goto_timeout_s: float
    slew_duration_s: float
    settle_delay_s: float
    manual_rate_deg_s: float

    def __post_init__(self) -> None:
        if not self.port:
            raise ValueError("Serial port is required.")
        if self.baud <= 0:
            raise ValueError("Baud rate must be positive.")
        if self.timeout_s <= 0:
            raise ValueError("Serial timeout must be positive.")
        if not isinstance(self.axis, SkyWatcherAxis):
            raise TypeError("Axis must be SkyWatcherAxis.")
        if self.poll_interval_s <= 0:
            raise ValueError("Poll interval must be positive.")
        if self.running_timeout_s <= 0:
            raise ValueError("Running timeout must be positive.")
        if self.goto_timeout_s <= 0:
            raise ValueError("Goto timeout must be positive.")
        if self.slew_duration_s <= 0:
            raise ValueError("Slew duration must be positive.")
        if self.settle_delay_s < 0:
            raise ValueError("Settle delay must be non-negative.")
        if self.manual_rate_deg_s <= 0:
            raise ValueError("Manual rate must be positive.")


def _mask_ticks(value: int) -> int:
    return int(value) & 0xFFFFFF


def _tick_delta(a: int, b: int) -> int:
    forward = _mask_ticks(a - b)
    backward = _mask_ticks(b - a)
    return min(forward, backward)


def _compute_goto_delta(cpr: int) -> int:
    if cpr <= 0:
        return 50
    scaled = cpr // 1000
    return max(50, int(scaled))


def _compute_step_period(cpr: int, timer_freq: int, rate_deg_s: float) -> int:
    if cpr <= 0:
        raise ValueError("CPR must be positive.")
    if timer_freq <= 0:
        raise ValueError("Timer frequency must be positive.")
    counts_per_s = rate_deg_s * cpr / 360.0
    preset = int(round(timer_freq / counts_per_s))
    bounded = int(clamp(preset, 1, (1 << 24) - 1))
    return bounded


def _wait_for_status(
    mc: SkyWatcherMC,
    axis: SkyWatcherAxis,
    predicate: Callable[[SkyWatcherStatus], bool],
    *,
    timeout_s: float,
    poll_interval_s: float,
    note: str,
) -> SkyWatcherStatus:
    start = time.monotonic()
    LOGGER.info("WAIT %s timeout_s=%s", note, timeout_s)
    while True:
        status = mc.inquire_status(axis)
        LOGGER.info(
            "STATUS %s raw=%s running=%s initialized=%s mode=%s dir=%s speed=%s note=%s",
            axis.name,
            status.raw,
            status.running,
            status.initialized,
            status.slew_mode.name,
            status.direction.name,
            status.speed_mode.name,
            note,
        )
        if predicate(status):
            return status
        elapsed = time.monotonic() - start
        if elapsed >= timeout_s:
            LOGGER.info(
                "WAIT %s elapsed_s=%s timeout_s=%s",
                note,
                elapsed,
                timeout_s,
            )
            pytest.fail(note)
        time.sleep(poll_interval_s)


def _wait_for_position_change(
    mc: SkyWatcherMC,
    axis: SkyWatcherAxis,
    start_pos: int,
    *,
    min_delta: int,
    timeout_s: float,
    poll_interval_s: float,
    note: str,
) -> int:
    start = time.monotonic()
    LOGGER.info(
        "WAIT %s timeout_s=%s start_pos=%s",
        note,
        timeout_s,
        start_pos,
    )
    while True:
        pos = mc.inquire_position(axis)
        LOGGER.info(
            "POSITION %s pos=%s note=%s",
            axis.name,
            pos,
            note,
        )
        if _tick_delta(pos, start_pos) >= min_delta:
            return pos
        elapsed = time.monotonic() - start
        if elapsed >= timeout_s:
            LOGGER.info(
                "WAIT %s elapsed_s=%s timeout_s=%s pos=%s",
                note,
                elapsed,
                timeout_s,
                pos,
            )
            pytest.fail(note)
        time.sleep(poll_interval_s)


def _assert_position_stable(
    mc: SkyWatcherMC,
    axis: SkyWatcherAxis,
    *,
    duration_s: float,
    poll_interval_s: float,
    max_delta: int,
    note: str,
) -> int:
    start = time.monotonic()
    start_pos = mc.inquire_position(axis)
    LOGGER.info(
        "CHECK POSITION %s pos=%s note=%s",
        axis.name,
        start_pos,
        note,
    )
    while True:
        elapsed = time.monotonic() - start
        if elapsed >= duration_s:
            return start_pos
        time.sleep(poll_interval_s)
        pos = mc.inquire_position(axis)
        LOGGER.info(
            "CHECK POSITION %s pos=%s note=%s",
            axis.name,
            pos,
            note,
        )
        if (delta := _tick_delta(pos, start_pos)) > max_delta:
            pytest.fail(note)
        else:
            LOGGER.info(
                "POSITION STABLE %s delta=%s max_delta=%s note=%s",
                axis.name,
                delta,
                max_delta,
                note,
            )


def _safe_stop(mc: SkyWatcherMC, axis: SkyWatcherAxis) -> None:
    LOGGER.info("STEP stop_motion axis=%s", axis.name)
    try:
        mc.stop_motion(axis)
    except Exception:
        LOGGER.exception("ACTION stop_motion_failed")
    try:
        mc.instant_stop(axis)
    except Exception:
        LOGGER.exception("ACTION instant_stop_failed")


@pytest.fixture(scope="session")
def skywatcher_config() -> SkyWatcherTestConfig:
    port = os.environ.get("SKYWATCHER_PORT", "/dev/tty.PL2303G-USBtoUART2120")
    if not port:
        LOGGER.info(
            "SKIP SKYWATCHER_PORT is not set; skipping serial tests."
        )
        pytest.skip("SKYWATCHER_PORT is not set; skipping serial tests.")
    baud = int(os.environ.get("SKYWATCHER_BAUD", "115200"))
    timeout_s = float(os.environ.get("SKYWATCHER_TIMEOUT_S", "0.5"))
    axis_value = os.environ.get("SKYWATCHER_AXIS", "1")
    axis = SkyWatcherAxis.from_channel(axis_value)
    return SkyWatcherTestConfig(
        port=port,
        baud=baud,
        timeout_s=timeout_s,
        axis=axis,
        poll_interval_s=0.2,
        running_timeout_s=6.0,
        goto_timeout_s=20.0,
        slew_duration_s=0.6,
        settle_delay_s=0.8,
        manual_rate_deg_s=0.1,
    )


@pytest.fixture(scope="session")
def skywatcher_mc(skywatcher_config: SkyWatcherTestConfig) -> SkyWatcherMC:
    LOGGER.info(
        "STEP connect axis=%s port=%s",
        skywatcher_config.axis.name,
        skywatcher_config.port,
    )
    try:
        dev = SerialLineDevice(
            skywatcher_config.port,
            skywatcher_config.baud,
            skywatcher_config.timeout_s,
            name="tests.skywatcher.serial.device",
        )
    except ImportError:
        LOGGER.info(
            "SKIP pyserial is not available; skipping serial tests."
        )
        pytest.skip("pyserial is not available; skipping serial tests.")
    mc = SkyWatcherMC(dev, logger=logging.getLogger("tests.skywatcher.serial.mc"))
    mc.do_initialize(
        skywatcher_config.axis,
        timeout_s=skywatcher_config.running_timeout_s,
        poll_interval_s=skywatcher_config.poll_interval_s,
    )
    return mc


def test_connect_and_status(skywatcher_mc: SkyWatcherMC, skywatcher_config: SkyWatcherTestConfig) -> None:
    axis = skywatcher_config.axis
    LOGGER.info("STEP read_status axis=%s", axis.name)
    cpr = skywatcher_mc.inquire_cpr(axis)
    timer_freq = skywatcher_mc.inquire_timer_freq(axis)
    status = skywatcher_mc.inquire_status(axis)
    LOGGER.info(
        "ACTION axis=%s cpr=%s timer_freq=%s",
        axis.name,
        cpr,
        timer_freq,
    )
    LOGGER.info(
        "STATUS %s raw=%s running=%s initialized=%s mode=%s dir=%s speed=%s note=read_status",
        axis.name,
        status.raw,
        status.running,
        status.initialized,
        status.slew_mode.name,
        status.direction.name,
        status.speed_mode.name,
    )
    assert cpr >= 1
    assert timer_freq >= 1
    assert isinstance(status, SkyWatcherStatus)


def test_check_position(skywatcher_mc: SkyWatcherMC, skywatcher_config: SkyWatcherTestConfig) -> None:
    axis = skywatcher_config.axis
    LOGGER.info("STEP read_position axis=%s", axis.name)
    pos = skywatcher_mc.inquire_position(axis)
    LOGGER.info(
        "POSITION %s pos=%s note=read_position",
        axis.name,
        pos,
    )
    assert 0 <= pos <= 0xFFFFFF


def test_enable_target_mode_and_update_pos(
    skywatcher_mc: SkyWatcherMC,
    skywatcher_config: SkyWatcherTestConfig,
) -> None:
    axis = skywatcher_config.axis
    start_pos = skywatcher_mc.inquire_position(axis)
    LOGGER.info(
        "POSITION %s pos=%s note=check_position",
        axis.name,
        start_pos,
    )
    cpr = skywatcher_mc.inquire_cpr(axis)
    delta = _compute_goto_delta(cpr)
    target = _mask_ticks(start_pos + delta)
    LOGGER.info(
        "STEP set_goto_mode axis=%s target=%s delta=%s",
        axis.name,
        target,
        delta,
    )
    try:
        skywatcher_mc.instant_stop(axis)
        time.sleep(skywatcher_config.settle_delay_s)
        mode = SkyWatcherMotionMode(
            slew_mode=SkyWatcherSlewMode.GOTO,
            direction=SkyWatcherDirection.FORWARD,
            speed_mode=SkyWatcherSpeedMode.LOWSPEED,
        )
        skywatcher_mc.set_motion_mode(axis, mode)
        skywatcher_mc.set_goto_target(axis, target)
        skywatcher_mc.start_motion(axis)
        _wait_for_status(
            skywatcher_mc,
            axis,
            lambda s: s.running,
            timeout_s=skywatcher_config.running_timeout_s,
            poll_interval_s=skywatcher_config.poll_interval_s,
            note="wait_running",
        )
        _wait_for_position_change(
            skywatcher_mc,
            axis,
            start_pos,
            min_delta=1,
            timeout_s=skywatcher_config.running_timeout_s,
            poll_interval_s=skywatcher_config.poll_interval_s,
            note="check_position",
        )
        _wait_for_status(
            skywatcher_mc,
            axis,
            lambda s: not s.running,
            timeout_s=skywatcher_config.goto_timeout_s,
            poll_interval_s=skywatcher_config.poll_interval_s,
            note="wait_stopped",
        )
        _assert_position_stable(
            skywatcher_mc,
            axis,
            duration_s=skywatcher_config.settle_delay_s,
            poll_interval_s=skywatcher_config.poll_interval_s,
            max_delta=100,
            note="stop_stable",
        )
    finally:
        _safe_stop(skywatcher_mc, axis)


def test_do_goto_check_happens(skywatcher_mc: SkyWatcherMC, skywatcher_config: SkyWatcherTestConfig) -> None:
    axis = skywatcher_config.axis
    start_pos = skywatcher_mc.inquire_position(axis)
    LOGGER.info(
        "POSITION %s pos=%s note=check_position",
        axis.name,
        start_pos,
    )
    cpr = skywatcher_mc.inquire_cpr(axis)
    delta = _compute_goto_delta(cpr)
    target = _mask_ticks(start_pos + delta)
    LOGGER.info(
        "STEP start_goto axis=%s target=%s delta=%s",
        axis.name,
        target,
        delta,
    )
    try:
        skywatcher_mc.instant_stop(axis)
        time.sleep(skywatcher_config.settle_delay_s)
        mode = SkyWatcherMotionMode(
            slew_mode=SkyWatcherSlewMode.GOTO,
            direction=SkyWatcherDirection.FORWARD,
            speed_mode=SkyWatcherSpeedMode.LOWSPEED,
        )
        skywatcher_mc.set_motion_mode(axis, mode)
        skywatcher_mc.set_goto_target(axis, target)
        skywatcher_mc.start_motion(axis)
        _wait_for_status(
            skywatcher_mc,
            axis,
            lambda s: s.running,
            timeout_s=skywatcher_config.running_timeout_s,
            poll_interval_s=skywatcher_config.poll_interval_s,
            note="wait_running",
        )
        _wait_for_position_change(
            skywatcher_mc,
            axis,
            start_pos,
            min_delta=1,
            timeout_s=skywatcher_config.running_timeout_s,
            poll_interval_s=skywatcher_config.poll_interval_s,
            note="check_position",
        )
    finally:
        _safe_stop(skywatcher_mc, axis)


def test_set_target_and_goto_reaches_target(
    skywatcher_mc: SkyWatcherMC,
    skywatcher_config: SkyWatcherTestConfig,
) -> None:
    axis = skywatcher_config.axis
    start_pos = skywatcher_mc.inquire_position(axis)
    LOGGER.info(
        "POSITION %s pos=%s note=goto_start",
        axis.name,
        start_pos,
    )
    cpr = skywatcher_mc.inquire_cpr(axis)
    delta = _compute_goto_delta(cpr)
    target = _mask_ticks(start_pos + delta)
    LOGGER.info(
        "STEP goto_target axis=%s target=%s delta=%s",
        axis.name,
        target,
        delta,
    )
    try:
        skywatcher_mc.instant_stop(axis)
        time.sleep(skywatcher_config.settle_delay_s)
        mode = SkyWatcherMotionMode(
            slew_mode=SkyWatcherSlewMode.GOTO,
            direction=SkyWatcherDirection.FORWARD,
            speed_mode=SkyWatcherSpeedMode.LOWSPEED,
        )
        skywatcher_mc.set_motion_mode(axis, mode)
        skywatcher_mc.set_goto_target(axis, target)
        skywatcher_mc.start_motion(axis)
        _wait_for_status(
            skywatcher_mc,
            axis,
            lambda s: s.running,
            timeout_s=skywatcher_config.running_timeout_s,
            poll_interval_s=skywatcher_config.poll_interval_s,
            note="wait_running",
        )
        _wait_for_position_change(
            skywatcher_mc,
            axis,
            start_pos,
            min_delta=1,
            timeout_s=skywatcher_config.running_timeout_s,
            poll_interval_s=skywatcher_config.poll_interval_s,
            note="move_check",
        )
        _wait_for_status(
            skywatcher_mc,
            axis,
            lambda s: not s.running,
            timeout_s=skywatcher_config.goto_timeout_s,
            poll_interval_s=skywatcher_config.poll_interval_s,
            note="wait_stopped",
        )
        end_pos = skywatcher_mc.inquire_position(axis)
        LOGGER.info(
            "POSITION %s pos=%s note=goto_end",
            axis.name,
            end_pos,
        )
        tolerance = max(1, delta // 10)
        assert _tick_delta(end_pos, target) <= tolerance
        _assert_position_stable(
            skywatcher_mc,
            axis,
            duration_s=skywatcher_config.settle_delay_s,
            poll_interval_s=skywatcher_config.poll_interval_s,
            max_delta=100,
            note="stop_stable",
        )
    finally:
        _safe_stop(skywatcher_mc, axis)


def test_do_goto_backwards_check_statuses(
    skywatcher_mc: SkyWatcherMC,
    skywatcher_config: SkyWatcherTestConfig,
) -> None:
    axis = skywatcher_config.axis
    start_pos = skywatcher_mc.inquire_position(axis)
    LOGGER.info(
        "POSITION %s pos=%s note=check_position",
        axis.name,
        start_pos,
    )
    cpr = skywatcher_mc.inquire_cpr(axis)
    delta = _compute_goto_delta(cpr)
    target = _mask_ticks(start_pos - delta)
    LOGGER.info(
        "STEP start_goto axis=%s target=%s delta=%s",
        axis.name,
        target,
        delta,
    )
    try:
        skywatcher_mc.instant_stop(axis)
        time.sleep(skywatcher_config.settle_delay_s)
        mode = SkyWatcherMotionMode(
            slew_mode=SkyWatcherSlewMode.GOTO,
            direction=SkyWatcherDirection.BACKWARD,
            speed_mode=SkyWatcherSpeedMode.LOWSPEED,
        )
        skywatcher_mc.set_motion_mode(axis, mode)
        skywatcher_mc.set_goto_target(axis, target)
        skywatcher_mc.start_motion(axis)
        _wait_for_status(
            skywatcher_mc,
            axis,
            lambda s: s.running,
            timeout_s=skywatcher_config.running_timeout_s,
            poll_interval_s=skywatcher_config.poll_interval_s,
            note="wait_running",
        )
        _wait_for_status(
            skywatcher_mc,
            axis,
            lambda s: s.direction == SkyWatcherDirection.BACKWARD,
            timeout_s=skywatcher_config.running_timeout_s,
            poll_interval_s=skywatcher_config.poll_interval_s,
            note="check_status",
        )
        _wait_for_position_change(
            skywatcher_mc,
            axis,
            start_pos,
            min_delta=1,
            timeout_s=skywatcher_config.running_timeout_s,
            poll_interval_s=skywatcher_config.poll_interval_s,
            note="check_position",
        )
        _wait_for_status(
            skywatcher_mc,
            axis,
            lambda s: not s.running,
            timeout_s=skywatcher_config.goto_timeout_s,
            poll_interval_s=skywatcher_config.poll_interval_s,
            note="wait_stopped",
        )
        _assert_position_stable(
            skywatcher_mc,
            axis,
            duration_s=skywatcher_config.settle_delay_s,
            poll_interval_s=skywatcher_config.poll_interval_s,
            max_delta=100,
            note="stop_stable",
        )
    finally:
        _safe_stop(skywatcher_mc, axis)


def test_move_left_right_ra(skywatcher_mc: SkyWatcherMC, skywatcher_config: SkyWatcherTestConfig) -> None:
    axis = skywatcher_config.axis
    cpr = skywatcher_mc.inquire_cpr(axis)
    timer_freq = skywatcher_mc.inquire_timer_freq(axis)
    LOGGER.info(
        "ACTION axis=%s cpr=%s timer_freq=%s",
        axis.name,
        cpr,
        timer_freq,
    )
    step_period = _compute_step_period(cpr, timer_freq, skywatcher_config.manual_rate_deg_s)
    for direction in (SkyWatcherDirection.FORWARD, SkyWatcherDirection.BACKWARD):
        LOGGER.info(
            "STEP start_slew axis=%s direction=%s",
            axis.name,
            direction.name,
        )
        try:
            skywatcher_mc.instant_stop(axis)
            time.sleep(skywatcher_config.settle_delay_s)
            mode = SkyWatcherMotionMode(
                slew_mode=SkyWatcherSlewMode.SLEW,
                direction=direction,
                speed_mode=SkyWatcherSpeedMode.LOWSPEED,
            )
            skywatcher_mc.set_motion_mode(axis, mode)
            skywatcher_mc.set_step_period(axis, step_period)
            skywatcher_mc.start_motion(axis)
            _wait_for_status(
                skywatcher_mc,
                axis,
                lambda s: s.running,
                timeout_s=skywatcher_config.running_timeout_s,
                poll_interval_s=skywatcher_config.poll_interval_s,
                note="wait_running",
            )
            _wait_for_position_change(
                skywatcher_mc,
                axis,
                skywatcher_mc.inquire_position(axis),
                min_delta=1,
                timeout_s=skywatcher_config.running_timeout_s,
                poll_interval_s=skywatcher_config.poll_interval_s,
                note="move_check",
            )
            time.sleep(skywatcher_config.slew_duration_s)
        finally:
            _safe_stop(skywatcher_mc, axis)
        _wait_for_status(
            skywatcher_mc,
            axis,
            lambda s: not s.running,
            timeout_s=skywatcher_config.running_timeout_s,
            poll_interval_s=skywatcher_config.poll_interval_s,
            note="wait_stopped",
        )
        _assert_position_stable(
            skywatcher_mc,
            axis,
            duration_s=skywatcher_config.settle_delay_s,
            poll_interval_s=skywatcher_config.poll_interval_s,
            max_delta=100,
            note="stop_stable",
        )


def test_enable_modes_and_check_it(skywatcher_mc: SkyWatcherMC, skywatcher_config: SkyWatcherTestConfig) -> None:
    axis = skywatcher_config.axis
    cases = [
        SkyWatcherMotionMode(
            slew_mode=SkyWatcherSlewMode.SLEW,
            direction=SkyWatcherDirection.FORWARD,
            speed_mode=SkyWatcherSpeedMode.LOWSPEED,
        ),
        SkyWatcherMotionMode(
            slew_mode=SkyWatcherSlewMode.SLEW,
            direction=SkyWatcherDirection.BACKWARD,
            speed_mode=SkyWatcherSpeedMode.HIGHSPEED,
        ),
        SkyWatcherMotionMode(
            slew_mode=SkyWatcherSlewMode.GOTO,
            direction=SkyWatcherDirection.FORWARD,
            speed_mode=SkyWatcherSpeedMode.LOWSPEED,
        ),
    ]
    for mode in cases:
        LOGGER.info(
            "STEP mode_check axis=%s mode=%s direction=%s speed=%s",
            axis.name,
            mode.slew_mode.name,
            mode.direction.name,
            mode.speed_mode.name,
        )
        try:
            skywatcher_mc.instant_stop(axis)
            time.sleep(skywatcher_config.settle_delay_s)
            skywatcher_mc.set_motion_mode(axis, mode)
            status = skywatcher_mc.inquire_status(axis)
            LOGGER.info(
                "STATUS %s raw=%s running=%s initialized=%s mode=%s dir=%s speed=%s note=mode_check",
                axis.name,
                status.raw,
                status.running,
                status.initialized,
                status.slew_mode.name,
                status.direction.name,
                status.speed_mode.name,
            )
            assert status.slew_mode == mode.slew_mode
            assert status.speed_mode == mode.speed_mode
        finally:
            _safe_stop(skywatcher_mc, axis)
