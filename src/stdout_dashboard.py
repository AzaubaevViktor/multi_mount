import sys
import threading
import time

from lx200.base import LX200Handler
from sky.axis import AxisDEC, AxisRA
from sky.combiner import Combiner
from sky.constants import STELLAR_SPEED
from sky.physics import Dec, DecPerSecond, Ha, HaPerSecond, Second


class StdoutDashboard:
    _LEFT_WIDTH = 42
    _MID_WIDTH = 42
    _RIGHT_WIDTH = 42
    _HEIGHT = 20

    def __init__(self, combiner: Combiner, lx200: LX200Handler, refresh_s: float = 0.25) -> None:
        self._combiner = combiner
        self._lx200 = lx200
        self._refresh_s = refresh_s
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._samples: list[tuple[float, Ha, Dec, Ha, Dec]] = []

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return

        self._stop_event.clear()
        self._thread = threading.Thread(target=self._serve_forever, name="STDOUT_DASHBOARD", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=self._refresh_s * 2)

    def render(self, clear_screen: bool = True) -> str:
        lines = self._render_lines()
        frame = "\n".join(lines)
        if clear_screen:
            return f"\x1b[2J\x1b[H{frame}\n"
        return f"{frame}\n"

    def _serve_forever(self) -> None:
        while not self._stop_event.is_set():
            sys.stdout.write(self.render(clear_screen=True))
            sys.stdout.flush()
            if self._stop_event.wait(self._refresh_s):
                break

    def _render_lines(self) -> list[str]:
        now = time.monotonic()
        wall_clock = time.strftime("%H:%M:%S")

        try:
            mount_position = self._combiner.get_position()
            ra_axis: AxisRA = self._combiner.ra
            dec_axis: AxisDEC = self._combiner.dec
            ra_monitor = ra_axis.command_monitor()
            dec_monitor = dec_axis.command_monitor()
            ra_motor_status = ra_axis._motor.status()
            dec_motor_status = dec_axis._motor.status()
            ra_motor_position = ra_axis._motor.convert_steps_to_position(ra_motor_status.steps)
            dec_motor_position = dec_axis._motor.convert_steps_to_position(dec_motor_status.steps)
            polar_compensator = self._combiner._polar_compensator

            self._samples.append((now, mount_position.ra, mount_position.dec, ra_motor_position, dec_motor_position))
            while len(self._samples) > 1 and now - self._samples[1][0] >= 1.0:
                self._samples.pop(0)

            if len(self._samples) < 2 or now - self._samples[0][0] < 1.0:
                mount_ra_rate = HaPerSecond(0)
                mount_dec_rate = DecPerSecond(0)
                motor_ra_rate = HaPerSecond(0)
                motor_dec_rate = DecPerSecond(0)
            else:
                sample_at, sample_mount_ra, sample_mount_dec, sample_ra_motor, sample_dec_motor = self._samples[0]
                elapsed_s = Second(now - sample_at)
                mount_ra_rate = (mount_position.ra - sample_mount_ra).moving_wrap() / elapsed_s
                mount_dec_rate = (mount_position.dec - sample_mount_dec).moving_wrap() / elapsed_s
                motor_ra_rate = (ra_motor_position - sample_ra_motor).moving_wrap() / elapsed_s
                motor_dec_rate = (dec_motor_position - sample_dec_motor).moving_wrap() / elapsed_s

            mount_state = "goto" if ra_axis.is_moving_to() or dec_axis.is_moving_to() else "track"
            polar_external_guide = Second(now) - polar_compensator.last_guide_pulse < polar_compensator.DROP_GUIDE_PULSES_COUNT_AFTER
            polar_stable_guide = (
                polar_compensator.stable_guide_ra_pulses_count >= polar_compensator.STABLE_GUIDE_PULSES_COUNT
                and polar_compensator.stable_guide_dec_pulses_count >= polar_compensator.STABLE_GUIDE_PULSES_COUNT
            )
            if polar_external_guide:
                polar_status = "external"
            elif polar_stable_guide and polar_compensator.eps_E is not None and polar_compensator.eps_N is not None:
                polar_status = "guiding"
            elif polar_stable_guide:
                polar_status = "stable"
            else:
                polar_status = "disabled"

            left_lines = [
                self._section_header("AXIS", self._LEFT_WIDTH),
                f"pos   {mount_position.ra} ax={self._abbr_axis_mode(ra_axis.mode())}",
                f"sky   {self._fmt_ha_rate(ra_axis._sky_speed)}",
                f"mount {self._fmt_ha_rate(mount_ra_rate)}",
                f"queue {ra_monitor['queue_size']:>6}",
                "",
                self._section_header("MOTOR", self._LEFT_WIDTH),
                f"pos   {ra_motor_position}",
                f"mode  {self._abbr_motor_mode(ra_motor_status.motion_mode)} {self._abbr_direction(ra_motor_status.direction)}",
                f"rate  {self._fmt_ha_rate(motor_ra_rate)}",
                f"sps   {ra_motor_status.speed_sps:>6} raw {ra_motor_status.steps}",
                "",
                self._section_header("POLAR", self._LEFT_WIDTH),
                f"guide {self._fmt_ra_guide_rate(polar_compensator.ra_speed)}",
                f"stable {polar_compensator.stable_guide_ra_pulses_count}/{polar_compensator.STABLE_GUIDE_PULSES_COUNT}",
                f"eps E {polar_compensator.eps_E or '-'}",
            ]

            middle_lines = [
                self._section_header("AXIS", self._MID_WIDTH),
                f"pos   {mount_position.dec} ax={self._abbr_axis_mode(dec_axis.mode())}",
                f"sky   {self._fmt_dec_rate(dec_axis._sky_speed)}",
                f"mount {self._fmt_dec_rate(mount_dec_rate)}",
                f"queue {dec_monitor['queue_size']:>6}",
                "",
                self._section_header("MOTOR", self._MID_WIDTH),
                f"pos   {dec_motor_position}",
                f"mode  {self._abbr_motor_mode(dec_motor_status.motion_mode)} {self._abbr_direction(dec_motor_status.direction)}",
                f"rate  {self._fmt_dec_rate(motor_dec_rate)}",
                f"sps   {dec_motor_status.speed_sps:>6} raw {dec_motor_status.steps}",
                "",
                self._section_header("POLAR", self._MID_WIDTH),
                f"guide {self._fmt_dec_rate(polar_compensator.dec_speed)}",
                f"stable {polar_compensator.stable_guide_dec_pulses_count}/{polar_compensator.STABLE_GUIDE_PULSES_COUNT}",
                f"eps N {polar_compensator.eps_N or '-'}",
            ]

            lx200_monitor = self._lx200.command_monitor()
            lx200_lines = [
                self._section_header("SYSTEM", self._RIGHT_WIDTH),
                f"clock {wall_clock}",
                f"state {mount_state}",
                f"polar {polar_status}",
                f"last  {self._fmt_age(now, polar_compensator.last_guide_pulse)}",
                "",
                self._section_header("LX200", self._RIGHT_WIDTH),
            ]
            if lx200_monitor["guide"] is not None:
                guide_at, guide_command = lx200_monitor["guide"]
                lx200_lines.append(f"guide {self._fmt_age(now, guide_at)} {guide_command}")
            for command_at, command in reversed(lx200_monitor["recent"]):
                lx200_lines.append(f"{self._fmt_age(now, command_at)} {command}")

            axis_lines = ["", self._section_header("AXIS", self._RIGHT_WIDTH)]
            processed_entries: list[tuple[float, str]] = []
            for axis_name, axis in (("RA", ra_axis), ("DEC", dec_axis)):
                for command_at, command in axis.command_monitor()["processed"]:
                    processed_entries.append((float(command_at), f"{axis_name} {self._short_axis_command(command)}"))
            for command_at, command in sorted(processed_entries, reverse=True)[:8]:
                axis_lines.append(f"{self._fmt_age(now, Second(command_at))} {command}")

            right_lines = lx200_lines + axis_lines
        except Exception as error:
            left_lines = [f"dashboard error: {type(error).__name__}", str(error)]
            middle_lines = ["DEC", "-"]
            right_lines = ["EVENTS", "-"]

        lines = [
            self._fit("RA", self._LEFT_WIDTH)
            + "|"
            + self._fit("DEC", self._MID_WIDTH)
            + "|"
            + self._fit("EVENTS", self._RIGHT_WIDTH),
            self._fit("-" * self._LEFT_WIDTH, self._LEFT_WIDTH)
            + "|"
            + self._fit("-" * self._MID_WIDTH, self._MID_WIDTH)
            + "|"
            + self._fit("-" * self._RIGHT_WIDTH, self._RIGHT_WIDTH),
        ]

        body_height = self._HEIGHT - len(lines)
        for index in range(body_height):
            left = left_lines[index] if index < len(left_lines) else ""
            middle = middle_lines[index] if index < len(middle_lines) else ""
            right = right_lines[index] if index < len(right_lines) else ""
            lines.append(
                self._fit(left, self._LEFT_WIDTH)
                + "|"
                + self._fit(middle, self._MID_WIDTH)
                + "|"
                + self._fit(right, self._RIGHT_WIDTH)
            )

        return lines[: self._HEIGHT]

    @staticmethod
    def _fit(text: object, width: int) -> str:
        rendered = str(text)
        if len(rendered) > width:
            return rendered[: max(0, width - 3)] + "..."
        return rendered.ljust(width)

    @staticmethod
    def _section_header(title: str, width: int) -> str:
        return f"-- {title} " + "-" * max(0, width - len(title) - 4)

    @staticmethod
    def _abbr_motor_mode(mode: object) -> str:
        name = str(mode).split(".")[-1].lower()
        return {
            "idle": "idle",
            "run": "run",
            "target": "tgt",
            "acceleration": "acc",
            "deceleration": "dec",
        }.get(name, name[:4])

    @staticmethod
    def _abbr_direction(direction: object) -> str:
        name = str(direction).split(".")[-1].lower()
        return {
            "forward": "fwd",
            "backward": "bwd",
            "stop": "stp",
        }.get(name, name[:3])

    @staticmethod
    def _abbr_axis_mode(mode: object) -> str:
        name = getattr(mode, "value", str(mode)).lower()
        return {
            "track": "track",
            "slew": "slew",
            "goto": "goto",
        }.get(name, name[:5])

    @staticmethod
    def _fmt_age(now: float, then: Second) -> str:
        delta = max(0.0, now - float(then))
        if delta < 10:
            return f"{delta:>4.1f}s"
        if delta < 60:
            return f"{delta:>4.0f}s"
        minutes = int(delta // 60)
        seconds = int(delta % 60)
        return f"{minutes:>2}m{seconds:02d}"

    @staticmethod
    def _fmt_ha_rate(value: HaPerSecond) -> str:
        return f"{float(value):+7.3f}hs"

    @staticmethod
    def _fmt_ra_guide_rate(value: HaPerSecond) -> str:
        return f"{float(value / STELLAR_SPEED):+6.3f}x sid"

    @staticmethod
    def _fmt_dec_rate(value: DecPerSecond) -> str:
        return f"{float(value):+7.2f}as"

    @staticmethod
    def _short_axis_command(command: str) -> str:
        shortened = command
        shortened = shortened.replace("set_position", "set")
        shortened = shortened.replace("change_speed", "chg")
        shortened = shortened.replace("halt_direction", "halt")
        shortened = shortened.replace("halt_all", "haltall")
        shortened = shortened.replace("goto_to", "goto")
        shortened = shortened.replace("move", "mov")
        shortened = shortened.replace("east", "e")
        shortened = shortened.replace("west", "w")
        shortened = shortened.replace("north", "n")
        shortened = shortened.replace("south", "s")
        return shortened
