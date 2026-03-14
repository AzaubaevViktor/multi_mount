import shutil
import sys
import threading
import time
import traceback
import textwrap

from lx200.base import LX200Handler
from sky.axis import AxisDEC, AxisRA
from sky.combiner import Combiner
from sky.constants import STELLAR_SPEED
from sky.physics import Dec, DecPerSecond, Ha, HaPerSecond, Second
from terminal_output import TERMINAL_OUTPUT_LOCK


class StdoutDashboard:
    _LEFT_WIDTH = 32
    _MID_WIDTH = 32
    _STATE_WIDTH = 34
    _HEIGHT = 30
    _TOTAL_WIDTH = _LEFT_WIDTH + _MID_WIDTH + _STATE_WIDTH + 2

    def __init__(self, combiner: Combiner, lx200: LX200Handler, refresh_s: float = 0.25) -> None:
        self._combiner = combiner
        self._lx200 = lx200
        self._refresh_s = refresh_s
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._samples: list[tuple[float, Ha, Dec, Ha, Dec]] = []
        self._screen_initialized = False

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
        if self._screen_initialized:
            with TERMINAL_OUTPUT_LOCK:
                sys.stdout.write("\x1b[r")
                sys.stdout.flush()
            self._screen_initialized = False

    def render(self, clear_screen: bool = True) -> str:
        lines = self._render_lines()
        frame = "\n".join(lines)
        if clear_screen:
            terminal_size = shutil.get_terminal_size((130, self._HEIGHT + 1))
            log_top_row = min(terminal_size.lines, self._HEIGHT + 1)
            rendered_lines = [f"\x1b[{log_top_row};{terminal_size.lines}r"]

            if self._screen_initialized:
                rendered_lines.append("\x1b7")

            for row, line in enumerate(lines, start=1):
                rendered_lines.append(f"\x1b[{row};1H\x1b[2K{line}")

            if self._screen_initialized:
                rendered_lines.append("\x1b8")
            else:
                rendered_lines.append(f"\x1b[{log_top_row};1H")
                self._screen_initialized = True

            return "".join(rendered_lines)
        return f"{frame}\n"

    def _serve_forever(self) -> None:
        while not self._stop_event.is_set():
            with TERMINAL_OUTPUT_LOCK:
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
            ra_power_v = ra_axis._motor.get_power_v()
            dec_power_v = dec_axis._motor.get_power_v()
            ra_motor_position = ra_axis._motor.convert_steps_to_position(ra_motor_status.steps)
            dec_motor_position = dec_axis._motor.convert_steps_to_position(dec_motor_status.steps)
            ra_axis_position = ra_axis.get_position().ra
            dec_axis_position = dec_axis.get_position().dec
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

            ra_mode = self._abbr_axis_mode(ra_axis.mode())
            dec_mode = self._abbr_axis_mode(dec_axis.mode())
            if "goto" in (ra_mode, dec_mode):
                mount_state = "goto"
            elif "slew" in (ra_mode, dec_mode):
                mount_state = "move"
            elif "track" in (ra_mode, dec_mode):
                mount_state = "track"
            else:
                mount_state = "stop"

            ra_set_speed = ra_axis._motor.get_speed_by_speed_sps(ra_motor_status.speed_sps) if ra_motor_status.speed_sps else HaPerSecond(0)
            dec_set_speed = dec_axis._motor.get_speed_by_speed_sps(dec_motor_status.speed_sps) if dec_motor_status.speed_sps else DecPerSecond(0)
            polar_external_guide = Second(now) - polar_compensator.last_guide_pulse < polar_compensator.DROP_GUIDE_PULSES_COUNT_AFTER
            polar_external_guide_ra = Second(now) - polar_compensator.last_ra_guide_pulse < polar_compensator.STOP_AXIS_AFTER
            polar_external_guide_dec = Second(now) - polar_compensator.last_dec_guide_pulse < polar_compensator.STOP_AXIS_AFTER
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
                self._pair("position", mount_position.ra),
                self._pair("mode", ra_mode),
                self._pair("sky", self._fmt_ha_rate(ra_axis._sky_speed)),
                self._pair("mount_1s", self._fmt_ha_rate(mount_ra_rate)),
                self._pair("queue", ra_monitor["queue_size"]),
                "",
                self._section_header("MOTOR", self._LEFT_WIDTH),
                self._pair("position", ra_motor_position),
                self._pair("mode", self._abbr_motor_mode(ra_motor_status.motion_mode)),
                self._pair("dir", self._abbr_direction(ra_motor_status.direction)),
                self._pair("motor_1s", self._fmt_ha_rate(motor_ra_rate)),
                self._pair("speed", f"{ra_motor_status.speed_sps} sps"),
                self._pair("raw", ra_motor_status.steps),
                "",
                self._section_header("POLAR", self._LEFT_WIDTH),
                self._pair("avg", self._fmt_ra_guide_rate(polar_compensator.ra_speed)),
                self._pair("samples", len(polar_compensator._ra_speeds)),
                self._pair("pulse", self._fmt_age(now, polar_compensator.last_ra_guide_pulse)),
                self._pair(
                    "flags",
                    f"e={self._fmt_flag(polar_external_guide)} s={self._fmt_flag(polar_stable_guide)} a={self._fmt_flag(polar_external_guide_ra)}",
                ),
                self._pair("current", polar_compensator.current_ha),
                self._pair("eps", polar_compensator.eps_E or "-"),
            ]

            middle_lines = [
                self._section_header("AXIS", self._MID_WIDTH),
                self._pair("position", mount_position.dec),
                self._pair("mode", dec_mode),
                self._pair("sky", self._fmt_dec_rate(dec_axis._sky_speed)),
                self._pair("mount_1s", self._fmt_dec_rate(mount_dec_rate)),
                self._pair("queue", dec_monitor["queue_size"]),
                "",
                self._section_header("MOTOR", self._MID_WIDTH),
                self._pair("position", dec_motor_position),
                self._pair("mode", self._abbr_motor_mode(dec_motor_status.motion_mode)),
                self._pair("dir", self._abbr_direction(dec_motor_status.direction)),
                self._pair("motor_1s", self._fmt_dec_rate(motor_dec_rate)),
                self._pair("speed", f"{dec_motor_status.speed_sps} sps"),
                self._pair("raw", dec_motor_status.steps),
                "",
                self._section_header("POLAR", self._MID_WIDTH),
                self._pair("avg", self._fmt_dec_rate(polar_compensator.dec_speed)),
                self._pair("samples", len(polar_compensator._dec_speeds)),
                self._pair("pulse", self._fmt_age(now, polar_compensator.last_dec_guide_pulse)),
                self._pair(
                    "flags",
                    f"e={self._fmt_flag(polar_external_guide)} s={self._fmt_flag(polar_stable_guide)} a={self._fmt_flag(polar_external_guide_dec)}",
                ),
                self._pair("current", polar_compensator.current_dec),
                self._pair("eps", polar_compensator.eps_N or "-"),
            ]

            state_lines = [
                self._section_header("STATE", self._STATE_WIDTH),
                self._pair("clock", wall_clock),
                self._pair("guide", self._fmt_age(now, polar_compensator.last_guide_pulse)),
                self._pair("mode", mount_state),
                self._pair("ra_mode", ra_mode),
                self._pair("dec_mode", dec_mode),
                self._pair("polar", polar_status),
                self._pair("ra_bat", self._fmt_voltage(ra_power_v)),
                self._pair("dec_bat", self._fmt_voltage(dec_power_v)),
                "",
            ]

            if ra_mode == "goto":
                state_lines.extend(
                    [
                        self._pair("ra_dir", ra_axis._goto_direction or "-"),
                        self._pair("ra_vset", self._fmt_ha_rate(ra_set_speed)),
                        self._pair("ra_tgt", ra_axis._goto_target or "-"),
                        self._pair("ra_left", abs((ra_axis_position - ra_axis._goto_target).moving_wrap()) if ra_axis._goto_target is not None else "-"),
                    ]
                )
            elif ra_mode == "slew":
                state_lines.extend(
                    [
                        self._pair("ra_dir", ra_axis._move_direction or "-"),
                        self._pair("ra_vset", self._fmt_ha_rate(ra_set_speed)),
                    ]
                )
            else:
                state_lines.append(self._pair("ra_track", self._fmt_ha_rate(ra_axis._sky_speed)))

            if dec_mode == "goto":
                state_lines.extend(
                    [
                        self._pair("dec_dir", dec_axis._goto_direction or "-"),
                        self._pair("dec_vset", self._fmt_dec_rate(dec_set_speed)),
                        self._pair("dec_tgt", dec_axis._goto_target or "-"),
                        self._pair("dec_left", abs((dec_axis_position - dec_axis._goto_target).moving_wrap()) if dec_axis._goto_target is not None else "-"),
                    ]
                )
            elif dec_mode == "slew":
                state_lines.extend(
                    [
                        self._pair("dec_dir", dec_axis._move_direction or "-"),
                        self._pair("dec_vset", self._fmt_dec_rate(dec_set_speed)),
                    ]
                )
            else:
                state_lines.append(self._pair("dec_track", self._fmt_dec_rate(dec_axis._sky_speed)))

        except Exception as error:
            error_lines = [
                self._fit("DASHBOARD ERROR", self._TOTAL_WIDTH),
                self._fit("-" * self._TOTAL_WIDTH, self._TOTAL_WIDTH),
            ]

            for traceback_block in traceback.format_exception(type(error), error, error.__traceback__):
                for traceback_line in traceback_block.rstrip("\n").splitlines():
                    wrapped_lines = textwrap.wrap(
                        traceback_line,
                        width=self._TOTAL_WIDTH,
                        replace_whitespace=False,
                        drop_whitespace=False,
                        subsequent_indent="  ",
                    )
                    if not wrapped_lines:
                        error_lines.append(" " * self._TOTAL_WIDTH)
                        continue

                    for wrapped_line in wrapped_lines:
                        error_lines.append(self._fit(wrapped_line, self._TOTAL_WIDTH))

            while len(error_lines) < self._HEIGHT:
                error_lines.append(" " * self._TOTAL_WIDTH)

            return error_lines[: self._HEIGHT]

        lines = [
            self._fit("RA", self._LEFT_WIDTH)
            + "|"
            + self._fit("DEC", self._MID_WIDTH)
            + "|"
            + self._fit("STATE", self._STATE_WIDTH),
            self._fit("-" * self._LEFT_WIDTH, self._LEFT_WIDTH)
            + "|"
            + self._fit("-" * self._MID_WIDTH, self._MID_WIDTH)
            + "|"
            + self._fit("-" * self._STATE_WIDTH, self._STATE_WIDTH),
        ]

        body_height = self._HEIGHT - len(lines)
        for index in range(body_height):
            left = left_lines[index] if index < len(left_lines) else ""
            middle = middle_lines[index] if index < len(middle_lines) else ""
            state = state_lines[index] if index < len(state_lines) else ""
            lines.append(
                self._fit(left, self._LEFT_WIDTH)
                + "|"
                + self._fit(middle, self._MID_WIDTH)
                + "|"
                + self._fit(state, self._STATE_WIDTH)
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
    def _pair(name: str, value: object) -> str:
        return f"{name:<8}: {value}"

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
    def _fmt_flag(value: bool) -> str:
        return "y" if value else "n"

    @staticmethod
    def _fmt_voltage(value: float | None) -> str:
        return "-" if value is None else f"{value:.2f}V"

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
