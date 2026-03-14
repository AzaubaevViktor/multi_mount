from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from threading import Event, RLock, Thread, Timer
from time import monotonic, sleep
from typing import Any

from .axis import AxisDEC, AxisMotionMode, AxisRA
from .constants import (
    CENTER_RATE_MULTIPLIER,
    FIND_RATE_MULTIPLIER,
    GUIDE_RATE_MULTIPLIER,
    GUIDE_SPEED_BASELINE_MS,
    GUIDE_SPEED_MAX_SCALE,
    GUIDE_SPEED_MIN_SCALE,
    MAX_RATE_MULTIPLIER,
    SIDEREAL_RATE_DEGREES_PER_SECOND,
    SIDEREAL_RATE_HOURS_PER_SECOND,
)
from .physics import DecPerSecond, HaPerSecond, PointCoordinates
from .polar_compensator import PolarCompensator


class GuideDirection(str, Enum):
    EAST = "e"
    WEST = "w"
    NORTH = "n"
    SOUTH = "s"


@dataclass(frozen=True, slots=True)
class SlewPreset:
    name: str
    ra_rate: HaPerSecond
    dec_rate: DecPerSecond


class Combiner:
    def __init__(self, ra_axis: AxisRA, dec_axis: AxisDEC, polar_compensator: PolarCompensator | None = None) -> None:
        self.ra_axis = ra_axis
        self.dec_axis = dec_axis
        self.polar_compensator = polar_compensator or PolarCompensator()
        self._lock = RLock()
        self._stop_event = Event()
        self._polar_thread: Thread | None = None
        self._external_guiding_until = 0.0
        self._polar_ra_correction = HaPerSecond(0.0)
        self._polar_dec_correction = DecPerSecond(0.0)
        self._preset = "guide"
        self._presets = {
            "guide": SlewPreset(
                name="guide",
                ra_rate=HaPerSecond(SIDEREAL_RATE_HOURS_PER_SECOND * GUIDE_RATE_MULTIPLIER),
                dec_rate=DecPerSecond(SIDEREAL_RATE_DEGREES_PER_SECOND * GUIDE_RATE_MULTIPLIER),
            ),
            "center": SlewPreset(
                name="center",
                ra_rate=HaPerSecond(SIDEREAL_RATE_HOURS_PER_SECOND * CENTER_RATE_MULTIPLIER),
                dec_rate=DecPerSecond(SIDEREAL_RATE_DEGREES_PER_SECOND * CENTER_RATE_MULTIPLIER),
            ),
            "find": SlewPreset(
                name="find",
                ra_rate=HaPerSecond(SIDEREAL_RATE_HOURS_PER_SECOND * FIND_RATE_MULTIPLIER),
                dec_rate=DecPerSecond(SIDEREAL_RATE_DEGREES_PER_SECOND * FIND_RATE_MULTIPLIER),
            ),
            "max": SlewPreset(
                name="max",
                ra_rate=HaPerSecond(SIDEREAL_RATE_HOURS_PER_SECOND * MAX_RATE_MULTIPLIER),
                dec_rate=DecPerSecond(SIDEREAL_RATE_DEGREES_PER_SECOND * MAX_RATE_MULTIPLIER),
            ),
        }

    def connect(self) -> None:
        self.ra_axis.connect()
        self.dec_axis.connect()
        with self._lock:
            if self._polar_thread and self._polar_thread.is_alive():
                return
            self._stop_event.clear()
            self._polar_thread = Thread(target=self._polar_loop, name="PolarCompensatorLoop", daemon=True)
            self._polar_thread.start()

    def disconnect(self) -> None:
        self._stop_event.set()
        thread = self._polar_thread
        if thread is not None:
            thread.join(timeout=1.0)
        self.ra_axis.disconnect()
        self.dec_axis.disconnect()

    def position(self) -> PointCoordinates:
        return PointCoordinates(ra=self.ra_axis.status().position, dec=self.dec_axis.status().position)

    def sync_to(self, coordinates: PointCoordinates) -> None:
        self.polar_compensator.reset()
        self._polar_ra_correction = HaPerSecond(0.0)
        self._polar_dec_correction = DecPerSecond(0.0)
        self.ra_axis.sync_to(coordinates.ra)
        self.dec_axis.sync_to(coordinates.dec)

    def goto_to(self, coordinates: PointCoordinates) -> None:
        self.polar_compensator.reset()
        self._polar_ra_correction = HaPerSecond(0.0)
        self._polar_dec_correction = DecPerSecond(0.0)
        self.ra_axis.goto_to(coordinates.ra)
        self.dec_axis.goto_to(coordinates.dec)

    def set_rate_preset(self, preset: str) -> None:
        if preset not in self._presets:
            raise ValueError(f"Unknown preset: {preset}")
        self._preset = preset

    def current_preset(self) -> SlewPreset:
        return self._presets[self._preset]

    def set_sky_speed(self, ra_speed: HaPerSecond, dec_speed: DecPerSecond) -> None:
        self.ra_axis.change_speed(HaPerSecond(ra_speed.hours_per_second + self._polar_ra_correction.hours_per_second))
        self.dec_axis.change_speed(DecPerSecond(dec_speed.degrees_per_second + self._polar_dec_correction.degrees_per_second))

    def move(self, direction: GuideDirection) -> None:
        preset = self.current_preset()
        if direction == GuideDirection.EAST:
            self.ra_axis.move(HaPerSecond(-preset.ra_rate.hours_per_second))
        elif direction == GuideDirection.WEST:
            self.ra_axis.move(HaPerSecond(preset.ra_rate.hours_per_second))
        elif direction == GuideDirection.NORTH:
            self.dec_axis.move(DecPerSecond(preset.dec_rate.degrees_per_second))
        elif direction == GuideDirection.SOUTH:
            self.dec_axis.move(DecPerSecond(-preset.dec_rate.degrees_per_second))
        else:
            raise ValueError(f"Unsupported direction: {direction}")

    def halt_all(self) -> None:
        self.ra_axis.halt()
        self.dec_axis.halt()

    def halt_ra(self) -> None:
        self.ra_axis.halt()

    def halt_dec(self) -> None:
        self.dec_axis.halt()

    def guide_speed(self, direction: GuideDirection, milliseconds: int) -> HaPerSecond | DecPerSecond:
        scale = max(GUIDE_SPEED_MIN_SCALE, min(GUIDE_SPEED_MAX_SCALE, milliseconds / GUIDE_SPEED_BASELINE_MS))
        preset = self._presets["guide"]
        if direction in {GuideDirection.EAST, GuideDirection.WEST}:
            sign = -1.0 if direction == GuideDirection.EAST else 1.0
            return HaPerSecond(preset.ra_rate.hours_per_second * scale * sign)

        sign = 1.0 if direction == GuideDirection.NORTH else -1.0
        return DecPerSecond(preset.dec_rate.degrees_per_second * scale * sign)

    def guide(self, direction: GuideDirection, milliseconds: int) -> None:
        now = monotonic()
        self._external_guiding_until = max(self._external_guiding_until, now + milliseconds / 1000.0)
        current_position = self.position()
        speed = self.guide_speed(direction, milliseconds)
        if direction in {GuideDirection.EAST, GuideDirection.WEST}:
            self.polar_compensator.record_guide_speeds(speed, DecPerSecond(0.0), current_position)
            self.ra_axis.move(speed, mode=AxisMotionMode.GUIDE)
            Timer(milliseconds / 1000.0, self.ra_axis.halt).start()
        else:
            self.polar_compensator.record_guide_speeds(HaPerSecond(0.0), speed, current_position)
            self.dec_axis.move(speed, mode=AxisMotionMode.GUIDE)
            Timer(milliseconds / 1000.0, self.dec_axis.halt).start()

    def monitor_groups(self) -> list[dict[str, Any]]:
        position = self.position()
        return [
            {
                "name": "Combiner",
                "fields": [
                    {"name": "ra", "value": position.ra.to_lx200()},
                    {"name": "dec", "value": position.dec.to_lx200()},
                    {"name": "preset", "value": self._preset},
                ],
            }
        ]

    def _polar_loop(self) -> None:
        while not self._stop_event.is_set():
            sleep(0.2)
            if monotonic() < self._external_guiding_until:
                continue

            ra_status = self.ra_axis.status()
            dec_status = self.dec_axis.status()
            if ra_status.mode in {AxisMotionMode.GOTO, AxisMotionMode.SLEW, AxisMotionMode.GUIDE}:
                continue
            if dec_status.mode in {AxisMotionMode.GOTO, AxisMotionMode.SLEW, AxisMotionMode.GUIDE}:
                continue

            correction = self.polar_compensator.takeover_speeds(self.position())
            if correction is None:
                continue

            self._polar_ra_correction, self._polar_dec_correction = correction
            self.set_sky_speed(self.ra_axis.status().tracking_speed, self.dec_axis.status().tracking_speed)
