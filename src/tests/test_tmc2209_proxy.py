from __future__ import annotations

import dataclasses
import logging
import time
from enum import IntEnum, StrEnum
from typing import Iterator

import pytest

from lib.logging_setup import set_all_loggers_level
from tmc2209.proxy import (
    TMC2209ArduinoConfig,
    TMC2209ArduinoProxy,
    TMC2209ProtocolConstants,
)

LOGGER = logging.getLogger("tests.tmc2209.proxy")
set_all_loggers_level(logging.DEBUG)


class TMC2209TestConstants:
    PORT = "/dev/tty.usbserial-2110"
    BAUD = TMC2209ProtocolConstants.DEFAULT_BAUD
    TIMEOUT_S = TMC2209ProtocolConstants.DEFAULT_TIMEOUT_S
    IDLE_TIMEOUT_S = TMC2209ProtocolConstants.DEFAULT_IDLE_TIMEOUT_S
    DEFAULT_POLL_INTERVAL_S = 0.05
    MOVE_TIMEOUT_FACTOR = 3.0
    MOVE_TIMEOUT_MIN_S = 0.5
    SETTLE_DELAY_S = 0.2
    POSITION_DELTA = 1000
    POSITION_MIN_DELTA = 1
    MOVE_STEPS = 400
    STEPS_PER_SECOND = 800
    MICROSTEPS = 16
    CURRENT_MA = 600
    SGTHRS = 10


class TMC2209MoveDirection(IntEnum):
    FORWARD = 1
    BACKWARD = -1


class TMC2209StealthMode(IntEnum):
    DISABLED = 0
    ENABLED = 1


@dataclasses.dataclass(frozen=True)
class TMC2209TestConfig:
    port: str
    baud: int
    timeout_s: float
    idle_timeout_s: float
    steps_per_second: int
    move_steps: int
    poll_interval_s: float
    move_timeout_s: float
    settle_delay_s: float
    microsteps: int
    current_ma: int
    sgthrs: int

    def __post_init__(self) -> None:
        if not self.port:
            raise ValueError("Serial port is required.")
        if self.baud <= TMC2209ProtocolConstants.BOOL_FALSE:
            raise ValueError("Baud must be positive.")
        if self.timeout_s <= TMC2209ProtocolConstants.BOOL_FALSE:
            raise ValueError("Timeout must be positive.")
        if self.idle_timeout_s <= TMC2209ProtocolConstants.BOOL_FALSE:
            raise ValueError("Idle timeout must be positive.")
        if self.steps_per_second == TMC2209ProtocolConstants.BOOL_FALSE:
            raise ValueError("Steps per second must be non-zero.")
        if self.move_steps == TMC2209ProtocolConstants.BOOL_FALSE:
            raise ValueError("Move steps must be non-zero.")
        if self.poll_interval_s <= TMC2209ProtocolConstants.BOOL_FALSE:
            raise ValueError("Poll interval must be positive.")
        if self.move_timeout_s <= TMC2209ProtocolConstants.BOOL_FALSE:
            raise ValueError("Move timeout must be positive.")
        if self.settle_delay_s < TMC2209ProtocolConstants.BOOL_FALSE:
            raise ValueError("Settle delay must be non-negative.")
        if self.microsteps not in TMC2209ProtocolConstants.MICROSTEPS_ALLOWED:
            raise ValueError("Microsteps must be allowed by protocol.")
        if (
            self.current_ma < TMC2209ProtocolConstants.MIN_CURRENT_MA
            or self.current_ma > TMC2209ProtocolConstants.MAX_CURRENT_MA
        ):
            raise ValueError("Current must be within allowed range.")
        if self.sgthrs < TMC2209ProtocolConstants.SGTHRS_MIN or self.sgthrs > TMC2209ProtocolConstants.SGTHRS_MAX:
            raise ValueError("SGTHRS must be within allowed range.")


def _compute_move_timeout_s(steps: int, sps: int) -> float:
    duration = abs(steps) / max(abs(sps), TMC2209ProtocolConstants.MIN_SPS)
    scaled = duration * TMC2209TestConstants.MOVE_TIMEOUT_FACTOR
    return max(TMC2209TestConstants.MOVE_TIMEOUT_MIN_S, scaled)


def _abs_delta(a: int, b: int) -> int:
    return abs(a - b)


def _wait_for_position_change(
    proxy: TMC2209ArduinoProxy,
    start_pos: int,
    *,
    min_delta: int,
    timeout_s: float,
    poll_interval_s: float,
    note: str,
) -> int:
    start = time.monotonic()
    while True:
        pos = proxy.get_position()
        delta = _abs_delta(pos, start_pos)
        LOGGER.info("WAIT position_change pos=%s delta=%s note=%s", pos, delta, note)
        if delta >= min_delta:
            return pos
        if time.monotonic() - start >= timeout_s:
            pytest.fail(note)
        time.sleep(poll_interval_s)


def _wait_for_position_stable(
    proxy: TMC2209ArduinoProxy,
    start_pos: int,
    *,
    duration_s: float,
    poll_interval_s: float,
    max_delta: int,
    note: str,
) -> None:
    start = time.monotonic()
    while True:
        if time.monotonic() - start >= duration_s:
            return
        time.sleep(poll_interval_s)
        pos = proxy.get_position()
        delta = _abs_delta(pos, start_pos)
        LOGGER.info("WAIT position_stable pos=%s delta=%s note=%s", pos, delta, note)
        if delta > max_delta:
            pytest.fail(note)


def _safe_disable(proxy: TMC2209ArduinoProxy) -> None:
    try:
        proxy.stop()
    except Exception:
        LOGGER.exception("ACTION stop_failed")
    try:
        proxy.enable(False)
    except Exception:
        LOGGER.exception("ACTION disable_failed")


@pytest.fixture(scope="session")
def tmc2209_config() -> TMC2209TestConfig:
    port = TMC2209TestConstants.PORT
    baud = TMC2209TestConstants.BAUD
    timeout_s = TMC2209TestConstants.TIMEOUT_S
    idle_timeout_s = TMC2209TestConstants.IDLE_TIMEOUT_S
    steps_per_second = TMC2209TestConstants.STEPS_PER_SECOND
    move_steps = TMC2209TestConstants.MOVE_STEPS
    microsteps = TMC2209TestConstants.MICROSTEPS
    current_ma = TMC2209TestConstants.CURRENT_MA
    sgthrs = TMC2209TestConstants.SGTHRS
    move_timeout_s = _compute_move_timeout_s(move_steps, steps_per_second)
    return TMC2209TestConfig(
        port=port,
        baud=baud,
        timeout_s=timeout_s,
        idle_timeout_s=idle_timeout_s,
        steps_per_second=steps_per_second,
        move_steps=move_steps,
        poll_interval_s=TMC2209TestConstants.DEFAULT_POLL_INTERVAL_S,
        move_timeout_s=move_timeout_s,
        settle_delay_s=TMC2209TestConstants.SETTLE_DELAY_S,
        microsteps=microsteps,
        current_ma=current_ma,
        sgthrs=sgthrs,
    )


@pytest.fixture()
def tmc2209_proxy(tmc2209_config: TMC2209TestConfig) -> Iterator[TMC2209ArduinoProxy]:
    config = TMC2209ArduinoConfig(
        port=tmc2209_config.port,
        baud=tmc2209_config.baud,
        timeout_s=tmc2209_config.timeout_s,
        idle_timeout_s=tmc2209_config.idle_timeout_s,
        device_name="tests.tmc2209.proxy",
    )
    try:
        proxy = TMC2209ArduinoProxy.from_serial(config, logger=LOGGER)
    except ImportError:
        LOGGER.info("SKIP pyserial is not available; skipping serial tests.")
        pytest.skip("pyserial is not available; skipping serial tests.")
    LOGGER.info(
        "STEP apply_config microsteps=%s current_ma=%s sgthrs=%s",
        tmc2209_config.microsteps,
        tmc2209_config.current_ma,
        tmc2209_config.sgthrs,
    )
    proxy.set_microsteps(tmc2209_config.microsteps)
    proxy.set_current_ma(tmc2209_config.current_ma)
    proxy.set_sgthrs(tmc2209_config.sgthrs)
    try:
        yield proxy
    finally:
        _safe_disable(proxy)
        proxy.close()


def test_connection_and_status(tmc2209_proxy: TMC2209ArduinoProxy) -> None:
    help_lines = tmc2209_proxy.help()
    info_items = tmc2209_proxy.info()
    LOGGER.info("STATUS help_lines=%s info_items=%s", help_lines, info_items)
    assert help_lines
    assert info_items


def test_read_position(tmc2209_proxy: TMC2209ArduinoProxy) -> None:
    pos = tmc2209_proxy.get_position()
    LOGGER.info("STATUS position=%s", pos)
    assert isinstance(pos, int)


def test_set_position_updates_value(
    tmc2209_proxy: TMC2209ArduinoProxy,
    tmc2209_config: TMC2209TestConfig,
) -> None:
    start_pos = tmc2209_proxy.get_position()
    target = start_pos + TMC2209TestConstants.POSITION_DELTA
    LOGGER.info("STEP set_position start=%s target=%s", start_pos, target)
    new_pos = tmc2209_proxy.set_position(target)
    read_back = tmc2209_proxy.get_position()
    LOGGER.info("STATUS set_position new_pos=%s read_back=%s", new_pos, read_back)
    assert _abs_delta(new_pos, target) <= TMC2209TestConstants.POSITION_MIN_DELTA
    assert _abs_delta(read_back, target) <= TMC2209TestConstants.POSITION_MIN_DELTA


# TODO: prompt - consolidate move verification flows to reduce duplication.
def test_move_forward_changes_position(
    tmc2209_proxy: TMC2209ArduinoProxy,
    tmc2209_config: TMC2209TestConfig,
) -> None:
    start_pos = tmc2209_proxy.get_position()
    steps = TMC2209MoveDirection.FORWARD * tmc2209_config.move_steps
    LOGGER.info("STEP move_forward start=%s steps=%s sps=%s", start_pos, steps, tmc2209_config.steps_per_second)
    tmc2209_proxy.move(steps, tmc2209_config.steps_per_second)
    end_pos = _wait_for_position_change(
        tmc2209_proxy,
        start_pos,
        min_delta=TMC2209TestConstants.POSITION_MIN_DELTA,
        timeout_s=tmc2209_config.move_timeout_s,
        poll_interval_s=tmc2209_config.poll_interval_s,
        note="move_forward",
    )
    time.sleep(tmc2209_config.settle_delay_s)
    _wait_for_position_stable(
        tmc2209_proxy,
        end_pos,
        duration_s=tmc2209_config.settle_delay_s,
        poll_interval_s=tmc2209_config.poll_interval_s,
        max_delta=TMC2209TestConstants.POSITION_MIN_DELTA,
        note="move_forward_stable",
    )


def test_move_backward_changes_position(
    tmc2209_proxy: TMC2209ArduinoProxy,
    tmc2209_config: TMC2209TestConfig,
) -> None:
    start_pos = tmc2209_proxy.get_position()
    steps = TMC2209MoveDirection.BACKWARD * tmc2209_config.move_steps
    LOGGER.info("STEP move_backward start=%s steps=%s sps=%s", start_pos, steps, tmc2209_config.steps_per_second)
    tmc2209_proxy.move(steps, tmc2209_config.steps_per_second)
    end_pos = _wait_for_position_change(
        tmc2209_proxy,
        start_pos,
        min_delta=TMC2209TestConstants.POSITION_MIN_DELTA,
        timeout_s=tmc2209_config.move_timeout_s,
        poll_interval_s=tmc2209_config.poll_interval_s,
        note="move_backward",
    )
    time.sleep(tmc2209_config.settle_delay_s)
    _wait_for_position_stable(
        tmc2209_proxy,
        end_pos,
        duration_s=tmc2209_config.settle_delay_s,
        poll_interval_s=tmc2209_config.poll_interval_s,
        max_delta=TMC2209TestConstants.POSITION_MIN_DELTA,
        note="move_backward_stable",
    )


def test_set_mode_and_move_forward(
    tmc2209_proxy: TMC2209ArduinoProxy,
    tmc2209_config: TMC2209TestConfig,
) -> None:
    LOGGER.info("STEP set_stealth enabled=%s", TMC2209StealthMode.ENABLED.name)
    tmc2209_proxy.set_stealth(bool(TMC2209StealthMode.ENABLED))
    start_pos = tmc2209_proxy.get_position()
    steps = TMC2209MoveDirection.FORWARD * tmc2209_config.move_steps
    tmc2209_proxy.move(steps, tmc2209_config.steps_per_second)
    _wait_for_position_change(
        tmc2209_proxy,
        start_pos,
        min_delta=TMC2209TestConstants.POSITION_MIN_DELTA,
        timeout_s=tmc2209_config.move_timeout_s,
        poll_interval_s=tmc2209_config.poll_interval_s,
        note="mode_forward",
    )


def test_set_mode_and_move_backward(
    tmc2209_proxy: TMC2209ArduinoProxy,
    tmc2209_config: TMC2209TestConfig,
) -> None:
    LOGGER.info("STEP set_stealth enabled=%s", TMC2209StealthMode.ENABLED.name)
    tmc2209_proxy.set_stealth(bool(TMC2209StealthMode.ENABLED))
    start_pos = tmc2209_proxy.get_position()
    steps = TMC2209MoveDirection.BACKWARD * tmc2209_config.move_steps
    tmc2209_proxy.move(steps, tmc2209_config.steps_per_second)
    _wait_for_position_change(
        tmc2209_proxy,
        start_pos,
        min_delta=TMC2209TestConstants.POSITION_MIN_DELTA,
        timeout_s=tmc2209_config.move_timeout_s,
        poll_interval_s=tmc2209_config.poll_interval_s,
        note="mode_backward",
    )
