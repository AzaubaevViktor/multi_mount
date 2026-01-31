from __future__ import annotations

import dataclasses
import logging
from enum import StrEnum
from typing import Optional

from lib.serial_prims import SerialLineDevice


class TMC2209Error(Exception):
    pass


class TMC2209ConfigError(TMC2209Error):
    pass


class TMC2209ProtocolError(TMC2209Error):
    pass


class TMC2209TimeoutError(TMC2209Error):
    pass


class TMC2209ResponseError(TMC2209Error):
    pass


class TMC2209Command(StrEnum):
    HELP = "help"
    INFO = "info"
    ENABLE = "enable"
    DIR = "dir"
    RUN = "run"
    MOVE = "move"
    STOP = "stop"
    CURRENT = "current"
    MICROSTEPS = "microsteps"
    STEALTH = "stealth"
    SGTHRS = "sgthrs"
    POS = "pos"
    SET_POS = "setpos"


class TMC2209ProtocolConstants:
    ENCODING = "ascii"
    DECODE_ERRORS = "replace"
    LINE_TERMINATOR = b"\n"
    LINE_TERMINATOR_STR = "\n"
    ARG_SEPARATOR = " "
    STRIP_CHARS = "\r\n"
    DEFAULT_BAUD = 115200
    DEFAULT_TIMEOUT_S = 0.5
    DEFAULT_IDLE_TIMEOUT_S = 0.2
    DEFAULT_DEVICE_NAME = "lx200.tmc2209.serial"
    BOOL_TRUE = 1
    BOOL_FALSE = 0
    MIN_SPS = 1
    MAX_SPS = 40000
    MIN_CURRENT_MA = 50
    MAX_CURRENT_MA = 2000
    MICROSTEPS_ALLOWED = (1, 2, 4, 8, 16, 32, 64, 128, 256)
    SGTHRS_MIN = 0
    SGTHRS_MAX = 255
    INFO_SEPARATOR = " = "
    POS_PREFIX = "pos="
    ENABLE_PREFIX = "enable="
    DIR_PREFIX = "dir="
    RMS_CURRENT_PREFIX = "rms_current(mA)="
    MICROSTEPS_PREFIX = "microsteps="
    STEALTH_PREFIX = "stealth="
    SGTHRS_PREFIX = "SGTHRS="


@dataclasses.dataclass
class TMC2209ArduinoConfig:
    port: str
    baud: int = TMC2209ProtocolConstants.DEFAULT_BAUD
    timeout_s: float = TMC2209ProtocolConstants.DEFAULT_TIMEOUT_S
    idle_timeout_s: float = TMC2209ProtocolConstants.DEFAULT_IDLE_TIMEOUT_S
    device_name: str = TMC2209ProtocolConstants.DEFAULT_DEVICE_NAME

    def __post_init__(self) -> None:
        if not self.port:
            raise TMC2209ConfigError("serial port is required")
        if self.baud <= TMC2209ProtocolConstants.BOOL_FALSE:
            raise TMC2209ConfigError("baud must be positive")
        if self.timeout_s <= TMC2209ProtocolConstants.BOOL_FALSE:
            raise TMC2209ConfigError("timeout must be positive")
        if self.idle_timeout_s <= TMC2209ProtocolConstants.BOOL_FALSE:
            raise TMC2209ConfigError("idle timeout must be positive")
        if not self.device_name:
            raise TMC2209ConfigError("device name is required")


@dataclasses.dataclass
class TMC2209KeyValue:
    key: str
    value: str

    def __post_init__(self) -> None:
        if not self.key:
            raise TMC2209ProtocolError("key must be non-empty")

    @classmethod
    def from_line(cls, line: str) -> "TMC2209KeyValue":
        if TMC2209ProtocolConstants.INFO_SEPARATOR not in line:
            raise TMC2209ProtocolError(f"invalid info line: {line!r}")
        key, value = line.split(TMC2209ProtocolConstants.INFO_SEPARATOR, 1)
        key = key.strip()
        value = value.strip()
        if not key:
            raise TMC2209ProtocolError(f"invalid info key: {line!r}")
        return cls(key=key, value=value)


class TMC2209ArduinoProxy:
    def __init__(
        self,
        device: SerialLineDevice,
        config: TMC2209ArduinoConfig,
        *,
        logger: Optional[logging.Logger] = None,
    ) -> None:
        if device is None:
            raise TMC2209ConfigError("serial device is required")
        if config is None:
            raise TMC2209ConfigError("config is required")
        self._device = device
        self._config = config
        self._log = logger or logging.getLogger(TMC2209ProtocolConstants.DEFAULT_DEVICE_NAME)

    @classmethod
    def from_serial(
        cls,
        config: TMC2209ArduinoConfig,
        *,
        logger: Optional[logging.Logger] = None,
    ) -> "TMC2209ArduinoProxy":
        device = SerialLineDevice(
            config.port,
            config.baud,
            config.timeout_s,
            config.device_name,
        )
        return cls(device, config, logger=logger)

    def close(self) -> None:
        self._device.close()

    def help(self) -> list[str]:
        return self._transact_lines(TMC2209Command.HELP)

    def info(self) -> list[TMC2209KeyValue]:
        lines = self._transact_lines(TMC2209Command.INFO)
        return [TMC2209KeyValue.from_line(line) for line in lines if line.strip()]

    def get_position(self) -> int:
        line = self._transact_single_line(TMC2209Command.POS)
        return self._parse_int_prefix(line, TMC2209ProtocolConstants.POS_PREFIX)

    def set_position(self, value: int) -> int:
        line = self._transact_single_line(TMC2209Command.SET_POS, str(value))
        return self._parse_int_prefix(line, TMC2209ProtocolConstants.POS_PREFIX)

    def enable(self, enabled: bool) -> bool:
        value = self._bool_to_str(enabled)
        line = self._transact_single_line(TMC2209Command.ENABLE, value)
        parsed = self._parse_int_prefix(line, TMC2209ProtocolConstants.ENABLE_PREFIX)
        return parsed != TMC2209ProtocolConstants.BOOL_FALSE

    def set_direction(self, reverse: bool) -> bool:
        value = self._bool_to_str(reverse)
        line = self._transact_single_line(TMC2209Command.DIR, value)
        parsed = self._parse_int_prefix(line, TMC2209ProtocolConstants.DIR_PREFIX)
        return parsed != TMC2209ProtocolConstants.BOOL_FALSE

    def run(self, steps_per_second: int) -> list[str]:
        self._validate_speed(steps_per_second)
        return self._transact_lines(TMC2209Command.RUN, str(steps_per_second))

    def move(self, steps: int, steps_per_second: Optional[int] = None) -> list[str]:
        args = [str(steps)]
        if steps_per_second is not None:
            self._validate_speed(steps_per_second)
            args.append(str(steps_per_second))
        return self._transact_lines(TMC2209Command.MOVE, *args)

    def stop(self) -> list[str]:
        return self._transact_lines(TMC2209Command.STOP)

    def set_current_ma(self, milliamps: int) -> int:
        if (
            milliamps < TMC2209ProtocolConstants.MIN_CURRENT_MA
            or milliamps > TMC2209ProtocolConstants.MAX_CURRENT_MA
        ):
            raise TMC2209ProtocolError("current out of range")
        line = self._transact_single_line(TMC2209Command.CURRENT, str(milliamps))
        return self._parse_int_prefix(line, TMC2209ProtocolConstants.RMS_CURRENT_PREFIX)

    def set_microsteps(self, microsteps: int) -> int:
        allowed = self._allowed_microsteps()
        if microsteps not in allowed:
            raise TMC2209ProtocolError("invalid microsteps value")
        line = self._transact_single_line(TMC2209Command.MICROSTEPS, str(microsteps))
        return self._parse_int_prefix(line, TMC2209ProtocolConstants.MICROSTEPS_PREFIX)

    def set_stealth(self, enabled: bool) -> bool:
        value = self._bool_to_str(enabled)
        line = self._transact_single_line(TMC2209Command.STEALTH, value)
        parsed = self._parse_int_prefix(line, TMC2209ProtocolConstants.STEALTH_PREFIX)
        return parsed != TMC2209ProtocolConstants.BOOL_FALSE

    def set_sgthrs(self, value: int) -> int:
        if value < TMC2209ProtocolConstants.SGTHRS_MIN or value > TMC2209ProtocolConstants.SGTHRS_MAX:
            raise TMC2209ProtocolError("sgthrs out of range")
        line = self._transact_single_line(TMC2209Command.SGTHRS, str(value))
        return self._parse_int_prefix(line, TMC2209ProtocolConstants.SGTHRS_PREFIX)

    def read_notifications(self) -> list[str]:
        lines = self._device.read_lines_until_idle(
            TMC2209ProtocolConstants.LINE_TERMINATOR,
            self._config.idle_timeout_s,
        )
        return self._decode_lines(lines)

    def _transact_lines(self, command: TMC2209Command, *args: str) -> list[str]:
        payload = self._encode_command(command, *args)
        lines = self._device.transact_lines(
            payload,
            TMC2209ProtocolConstants.LINE_TERMINATOR,
            self._config.idle_timeout_s,
        )
        if not lines:
            raise TMC2209TimeoutError("no response from device")
        return self._decode_lines(lines)

    def _transact_single_line(self, command: TMC2209Command, *args: str) -> str:
        lines = self._transact_lines(command, *args)
        return lines[TMC2209ProtocolConstants.BOOL_FALSE]

    def _encode_command(self, command: TMC2209Command, *args: str) -> bytes:
        parts = [command.value]
        if args:
            parts.extend(args)
        line = TMC2209ProtocolConstants.ARG_SEPARATOR.join(parts) + TMC2209ProtocolConstants.LINE_TERMINATOR_STR
        return line.encode(TMC2209ProtocolConstants.ENCODING)

    @staticmethod
    def _decode_lines(lines: list[bytes]) -> list[str]:
        decoded: list[str] = []
        for line in lines:
            text = line.decode(TMC2209ProtocolConstants.ENCODING, errors=TMC2209ProtocolConstants.DECODE_ERRORS)
            decoded.append(text.rstrip(TMC2209ProtocolConstants.STRIP_CHARS))
        return decoded

    @staticmethod
    def _parse_int_prefix(line: str, prefix: str) -> int:
        if not line.startswith(prefix):
            raise TMC2209ResponseError(f"unexpected response: {line!r}")
        value_str = line[len(prefix) :].strip()
        if not value_str:
            raise TMC2209ResponseError(f"missing value in response: {line!r}")
        return int(value_str)

    @staticmethod
    def _bool_to_str(value: bool) -> str:
        if value:
            return str(TMC2209ProtocolConstants.BOOL_TRUE)
        return str(TMC2209ProtocolConstants.BOOL_FALSE)

    @staticmethod
    def _allowed_microsteps() -> set[int]:
        return set(TMC2209ProtocolConstants.MICROSTEPS_ALLOWED)

    @staticmethod
    def _validate_speed(steps_per_second: int) -> None:
        if steps_per_second == TMC2209ProtocolConstants.BOOL_FALSE:
            raise TMC2209ProtocolError("steps per second must be non-zero")
        sps = abs(steps_per_second)
        if sps < TMC2209ProtocolConstants.MIN_SPS or sps > TMC2209ProtocolConstants.MAX_SPS:
            raise TMC2209ProtocolError("steps per second out of range")
