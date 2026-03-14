from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from threading import Event, RLock, Thread
from time import sleep
from typing import Any

from .axis import AxisDEC, AxisMotionMode, AxisRA
from .constants import (
    CENTER_RATE_MULTIPLIER,
    FIND_RATE_MULTIPLIER,
    GUIDE_INTERVAL_SECONDS,
    GUIDE_RATE_MULTIPLIER,
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


@dataclass(frozen=True, slots=True)
class GuideSpeedProfile:
    backward: HaPerSecond | DecPerSecond
    default: HaPerSecond | DecPerSecond
    forward: HaPerSecond | DecPerSecond

    def calculate_speed(self, direction: GuideDirection, milliseconds: int) -> HaPerSecond | DecPerSecond:
        fraction = max(0.0, min(1.0, milliseconds / (GUIDE_INTERVAL_SECONDS * 1000.0)))
        if direction in {GuideDirection.EAST, GuideDirection.NORTH}:
            start = float(self.default.hours_per_second if isinstance(self.default, HaPerSecond) else self.default.degrees_per_second)
            finish = float(self.forward.hours_per_second if isinstance(self.forward, HaPerSecond) else self.forward.degrees_per_second)
        else:
            start = float(self.default.hours_per_second if isinstance(self.default, HaPerSecond) else self.default.degrees_per_second)
            finish = float(self.backward.hours_per_second if isinstance(self.backward, HaPerSecond) else self.backward.degrees_per_second)

        value = start + (finish - start) * fraction
        if isinstance(self.default, HaPerSecond):
            return HaPerSecond(value)
        return DecPerSecond(value)


class Combiner:
    def __init__(self, ra_axis: AxisRA, dec_axis: AxisDEC, polar_compensator: PolarCompensator | None = None) -> None:
        self.ra_axis = ra_axis
        self.dec_axis = dec_axis
        self.polar_compensator = polar_compensator or PolarCompensator()
        self._lock = RLock()
        self._stop_event = Event()
        self._polar_thread: Thread | None = None
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
        guide_preset = self._presets["guide"]
        self._ra_guide_speed = GuideSpeedProfile(
            backward=HaPerSecond(SIDEREAL_RATE_HOURS_PER_SECOND - guide_preset.ra_rate.hours_per_second),
            default=HaPerSecond(SIDEREAL_RATE_HOURS_PER_SECOND),
            forward=HaPerSecond(SIDEREAL_RATE_HOURS_PER_SECOND + guide_preset.ra_rate.hours_per_second),
        )
        self._dec_guide_speed = GuideSpeedProfile(
            backward=DecPerSecond(-guide_preset.dec_rate.degrees_per_second),
            default=DecPerSecond(0.0),
            forward=DecPerSecond(guide_preset.dec_rate.degrees_per_second),
        )

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
        self.ra_axis.sync_to(coordinates.ra)
        self.dec_axis.sync_to(coordinates.dec)

    def goto_to(self, coordinates: PointCoordinates) -> None:
        self.polar_compensator.reset()
        self.ra_axis.goto_to(coordinates.ra)
        self.dec_axis.goto_to(coordinates.dec)

    def set_rate_preset(self, preset: str) -> None:
        if preset not in self._presets:
            raise ValueError(f"Unknown preset: {preset}")
        self._preset = preset

    def current_preset(self) -> SlewPreset:
        return self._presets[self._preset]

    def set_sky_speed(
        self,
        ra_speed: HaPerSecond | None,
        dec_speed: DecPerSecond | None,
        update_polar_compensator: bool = True,
    ) -> None:
        if ra_speed is not None:
            self.ra_axis.change_speed(ra_speed)
            if update_polar_compensator:
                self.polar_compensator.guide_ra(ra_speed)

        if dec_speed is not None:
            self.dec_axis.change_speed(dec_speed)
            if update_polar_compensator:
                self.polar_compensator.guide_dec(dec_speed)

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
        if direction in {GuideDirection.EAST, GuideDirection.WEST}:
            return self._ra_guide_speed.calculate_speed(direction, milliseconds)

        return self._dec_guide_speed.calculate_speed(direction, milliseconds)

    def guide(self, direction: GuideDirection, milliseconds: int) -> None:
        current_position = self.position()
        self.polar_compensator.update_position(current_position)
        speed = self.guide_speed(direction, milliseconds)
        if direction in {GuideDirection.EAST, GuideDirection.WEST}:
            self.set_sky_speed(speed, None, update_polar_compensator=True)
        else:
            self.set_sky_speed(None, speed, update_polar_compensator=True)

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

            ra_status = self.ra_axis.status()
            dec_status = self.dec_axis.status()
            if ra_status.mode in {AxisMotionMode.GOTO, AxisMotionMode.SLEW, AxisMotionMode.GUIDE}:
                continue
            if dec_status.mode in {AxisMotionMode.GOTO, AxisMotionMode.SLEW, AxisMotionMode.GUIDE}:
                continue

            current_position = self.position()
            self.polar_compensator.update_position(current_position)
            speeds = self.polar_compensator.takeover_speeds(current_position)
            if speeds is None:
                continue

            self.set_sky_speed(*speeds, update_polar_compensator=False)
