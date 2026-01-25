from __future__ import annotations

import argparse
import dataclasses
import json
import logging
import os
import time
from typing import Any, Mapping

from lib.logging_setup import setup_logging

from .constants import (
    PolarAlignmentCliConstants,
    PolarAlignmentConstants,
    PolarAlignmentFileKey,
)
from .models import (
    PolarAlignmentDataError,
    PolarAlignmentMathError,
    PolarAlignmentParseError,
    PolarAlignmentResult,
    apply_tracking_correction,
    compute_axis,
    parse_samples,
)


@dataclasses.dataclass(frozen=True)
class PolarAlignmentConfig:
    input_path: str
    poll_interval_s: float
    tracking_sign: int

    def __post_init__(self) -> None:
        if not self.input_path:
            raise PolarAlignmentDataError("input path is required")
        if self.poll_interval_s <= PolarAlignmentConstants.VECTOR_EPS:
            raise PolarAlignmentDataError("poll interval must be positive")


def main() -> int:
    args = _parse_args()
    setup_logging(level=logging.DEBUG if args.verbose else logging.INFO)
    config = PolarAlignmentConfig(
        input_path=args.input,
        poll_interval_s=args.poll,
        tracking_sign=args.tracking_sign,
    )
    logger = logging.getLogger(PolarAlignmentConstants.LOGGER_NAME)
    monitor = PolarAlignmentFileMonitor(config=config, logger=logger)
    monitor.run()
    return 0


@dataclasses.dataclass
class PolarAlignmentFileState:
    mtime_ns: int = 0
    size: int = 0


class PolarAlignmentFileMonitor:
    def __init__(self, config: PolarAlignmentConfig, logger: logging.Logger) -> None:
        self._config = config
        self._logger = logger
        self._state = PolarAlignmentFileState()

    def run(self) -> None:
        self._logger.info("monitoring %s", self._config.input_path)
        while True:
            if self._has_changed():
                self._handle_update()
            time.sleep(self._config.poll_interval_s)

    def _has_changed(self) -> bool:
        try:
            stat = os.stat(self._config.input_path)
        except FileNotFoundError:
            return False
        if stat.st_mtime_ns != self._state.mtime_ns or stat.st_size != self._state.size:
            self._state.mtime_ns = stat.st_mtime_ns
            self._state.size = stat.st_size
            return True
        return False

    def _handle_update(self) -> None:
        try:
            payload = _load_payload(self._config.input_path)
            samples = parse_samples(payload)
            corrected = apply_tracking_correction(samples, tracking_sign=self._config.tracking_sign)
            axis = compute_axis(corrected)
            result = PolarAlignmentResult(axis=axis, corrected_samples=corrected)
        except (PolarAlignmentParseError, PolarAlignmentMathError, PolarAlignmentDataError) as exc:
            self._logger.error("polar alignment parse error: %s", exc)
            return
        self._log_result(result)

    def _log_result(self, result: PolarAlignmentResult) -> None:
        axis = result.axis
        self._logger.info(
            "axis ra=%s dec=%s error_deg=%.6f",
            axis.ra.to_string().strip(),
            axis.dec.to_string().strip(),
            axis.error_deg,
        )
        for key in PolarAlignmentFileKey:
            sample = result.corrected_samples[key]
            self._logger.debug(
                "corrected %s ra=%s dec=%s",
                key.value,
                sample.ra.to_string().strip(),
                sample.dec.to_string().strip(),
            )


def _load_payload(path: str) -> Mapping[str, Mapping[str, str]]:
    try:
        with open(path, "r", encoding=PolarAlignmentConstants.FILE_ENCODING) as handle:
            payload = json.load(handle)
    except FileNotFoundError as exc:
        raise PolarAlignmentParseError(f"file not found: {path!r}") from exc
    except json.JSONDecodeError as exc:
        raise PolarAlignmentParseError(f"invalid json in {path!r}") from exc
    if not isinstance(payload, dict):
        raise PolarAlignmentParseError("payload must be a json object")
    return _ensure_mapping(payload)


def _ensure_mapping(payload: Mapping[str, Any]) -> Mapping[str, Mapping[str, str]]:
    validated: dict[str, Mapping[str, str]] = {}
    for key, value in payload.items():
        if not isinstance(value, dict):
            raise PolarAlignmentParseError("sample payload must be object")
        validated[key] = value
    return validated


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Polar alignment file monitor")
    parser.add_argument(
        PolarAlignmentCliConstants.ARG_INPUT,
        required=True,
        help="Path to JSON file with t1/t2/t3 samples",
    )
    parser.add_argument(
        PolarAlignmentCliConstants.ARG_POLL,
        type=float,
        default=PolarAlignmentConstants.DEFAULT_POLL_INTERVAL_S,
        help="Polling interval in seconds",
    )
    parser.add_argument(
        PolarAlignmentCliConstants.ARG_TRACKING_SIGN,
        type=int,
        default=PolarAlignmentCliConstants.DEFAULT_TRACKING_SIGN,
        help="Tracking sign: 1 or -1",
    )
    parser.add_argument(
        PolarAlignmentCliConstants.ARG_VERBOSE,
        action="store_true",
        help="Enable debug logging",
    )
    return parser.parse_args()


if __name__ == "__main__":
    raise SystemExit(main())
