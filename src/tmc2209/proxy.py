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

class Command(StrEnum):
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


class ProtocolConstants:
    ENCODING = "ascii"
    DECODE_ERRORS = "replace"
    LINE_TERMINATOR = b"\r"
    LINE_TERMINATOR_STR = "\r\n"
    ARG_SEPARATOR = " "
    STRIP_CHARS = "\r\n"
    DEFAULT_BAUD = 115200
    DEFAULT_TIMEOUT_S = 0.5
    DEFAULT_IDLE_TIMEOUT_S = .5
    DEFAULT_DEVICE_NAME = "tmc2209"
    BOOL_TRUE = 1
    BOOL_FALSE = 0
    ZERO = 0
    FIRST_LINE_INDEX = 0
    MIN_SPS = 1
    MAX_SPS = 40000
    MIN_CURRENT_MA = 50
    MAX_CURRENT_MA = 2000
    MICROSTEPS_ALLOWED = frozenset((1, 2, 4, 8, 16, 32, 64, 128, 256))
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
    baud: int = ProtocolConstants.DEFAULT_BAUD
    timeout_s: float = ProtocolConstants.DEFAULT_TIMEOUT_S
    idle_timeout_s: float = ProtocolConstants.DEFAULT_IDLE_TIMEOUT_S
    device_name: str = ProtocolConstants.DEFAULT_DEVICE_NAME

    def __post_init__(self) -> None:
        if not self.port:
            raise TMC2209ConfigError("serial port is required")
        if self.baud <= 0:
            raise TMC2209ConfigError("baud must be positive")
        if self.timeout_s <= 0:
            raise TMC2209ConfigError("timeout must be positive")
        if self.idle_timeout_s <= 0:
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
        if ProtocolConstants.INFO_SEPARATOR not in line:
            raise TMC2209ProtocolError(f"invalid info line: {line!r}")
        key, value = line.split(ProtocolConstants.INFO_SEPARATOR, 1)
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
        self._log = logger or logging.getLogger(ProtocolConstants.DEFAULT_DEVICE_NAME)

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
        proxy = cls(device, config, logger=logger)
        proxy._read_all_input_after_connect()
        return proxy

    def close(self) -> None:
        self._device.close()

    def help(self) -> list[str]:
        return self._transact_lines(Command.HELP)

    def info(self) -> dict[str, int | str]:
        lines = self._transact_lines(Command.INFO)
        info: dict[str, int | str] = {}
        for line in lines:
            if not line.strip():
                continue
            item = TMC2209KeyValue.from_line(line)
            info[item.key] = self._parse_info_value(item.value)
        return info

    def get_position(self) -> int:
        line = self._transact_single_line(Command.POS)
        return self._parse_int_prefix(line, ProtocolConstants.POS_PREFIX)

    def set_position(self, value: int) -> int:
        line = self._transact_single_line(Command.SET_POS, str(value))
        return self._parse_int_prefix(line, ProtocolConstants.POS_PREFIX)

    def enable(self, enabled: bool) -> bool:
        value = self._bool_to_str(enabled)
        line = self._transact_single_line(Command.ENABLE, value)
        parsed = self._parse_int_prefix(line, ProtocolConstants.ENABLE_PREFIX)
        return parsed != ProtocolConstants.BOOL_FALSE

    def set_direction(self, reverse: bool) -> bool:
        value = self._bool_to_str(reverse)
        line = self._transact_single_line(Command.DIR, value)
        parsed = self._parse_int_prefix(line, ProtocolConstants.DIR_PREFIX)
        return parsed != ProtocolConstants.BOOL_FALSE

    def run(self, steps_per_second: int) -> list[str]:
        self._validate_speed(steps_per_second)
        return self._transact_lines(Command.RUN, str(steps_per_second))

    def move(self, steps: int, steps_per_second: Optional[int] = None) -> list[str]:
        args = [str(steps)]
        if steps_per_second is not None:
            self._validate_speed(steps_per_second)
            args.append(str(steps_per_second))
        return self._transact_lines(Command.MOVE, *args)

    def stop(self) -> list[str]:
        return self._transact_lines(Command.STOP)

    def set_current_ma(self, milliamps: int) -> int:
        if (
            milliamps < ProtocolConstants.MIN_CURRENT_MA
            or milliamps > ProtocolConstants.MAX_CURRENT_MA
        ):
            raise TMC2209ProtocolError("current out of range")
        line = self._transact_single_line(Command.CURRENT, str(milliamps))
        return self._parse_int_prefix(line, ProtocolConstants.RMS_CURRENT_PREFIX)

    def set_microsteps(self, microsteps: int) -> int:
        if microsteps not in ProtocolConstants.MICROSTEPS_ALLOWED:
            raise TMC2209ProtocolError("invalid microsteps value")
        line = self._transact_single_line(Command.MICROSTEPS, str(microsteps))
        return self._parse_int_prefix(line, ProtocolConstants.MICROSTEPS_PREFIX)

    def set_stealth(self, enabled: bool) -> bool:
        value = self._bool_to_str(enabled)
        line = self._transact_single_line(Command.STEALTH, value)
        parsed = self._parse_int_prefix(line, ProtocolConstants.STEALTH_PREFIX)
        return parsed != ProtocolConstants.BOOL_FALSE

    def set_sgthrs(self, value: int) -> int:
        if value < ProtocolConstants.SGTHRS_MIN or value > ProtocolConstants.SGTHRS_MAX:
            raise TMC2209ProtocolError("sgthrs out of range")
        line = self._transact_single_line(Command.SGTHRS, str(value))
        return self._parse_int_prefix(line, ProtocolConstants.SGTHRS_PREFIX)

    def read_notifications(self) -> list[str]:
        lines = self._device.read_lines_until_idle(
            ProtocolConstants.LINE_TERMINATOR,
            self._config.idle_timeout_s,
        )
        return self._decode_lines(lines)

    def _read_all_input_after_connect(self) -> None:
        self._device.read_lines_until_idle(
            ProtocolConstants.LINE_TERMINATOR,
            self._config.idle_timeout_s * 5,
        )

    def _transact_lines(self, command: Command, *args: str) -> list[str]:
        payload = self._encode_command(command, *args)
        lines = self._device.transact_lines(
            payload,
            ProtocolConstants.LINE_TERMINATOR,
            self._config.idle_timeout_s,
        )
        if not lines:
            raise TMC2209TimeoutError("no response from device")
        return self._decode_lines(lines)

    def _transact_single_line(self, command: Command, *args: str) -> str:
        lines = self._transact_lines(command, *args)
        return lines[ProtocolConstants.FIRST_LINE_INDEX]

    def _encode_command(self, command: Command, *args: str) -> bytes:
        parts = [command.value]
        if args:
            parts.extend(args)
        line = ProtocolConstants.ARG_SEPARATOR.join(parts) + ProtocolConstants.LINE_TERMINATOR_STR
        return line.encode(ProtocolConstants.ENCODING)

    @staticmethod
    def _parse_info_value(value: str) -> int | str:
        try:
            return int(value)
        except ValueError:
            return value

    @staticmethod
    def _decode_lines(lines: list[bytes]) -> list[str]:
        decoded: list[str] = []
        for line in lines:
            text = line.decode(ProtocolConstants.ENCODING, errors=ProtocolConstants.DECODE_ERRORS)
            decoded.append(text.rstrip(ProtocolConstants.STRIP_CHARS))
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
            return str(ProtocolConstants.BOOL_TRUE)
        return str(ProtocolConstants.BOOL_FALSE)

    @staticmethod
    def _validate_speed(steps_per_second: int) -> None:
        if steps_per_second == ProtocolConstants.ZERO:
            raise TMC2209ProtocolError("steps per second must be non-zero")
        sps = abs(steps_per_second)
        if sps < ProtocolConstants.MIN_SPS or sps > ProtocolConstants.MAX_SPS:
            raise TMC2209ProtocolError("steps per second out of range")
