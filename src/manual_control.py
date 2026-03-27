import time
from collections.abc import Callable

from lx200.base import LX200Handler
from lx200.base_server import LX200SimpleServer


def _normalize_lx200_command(raw: str) -> str | None:
    command = raw.strip()
    if not command:
        return None
    if command.startswith(":"):
        command = command[1:]
    if command.endswith("#"):
        command = command[:-1]
    if not command or any(char.isspace() for char in command):
        return None
    return command


class ManualControlConsole:
    def __init__(
        self,
        lx200: LX200Handler,
        server: LX200SimpleServer,
        axis_status_provider: Callable[[], dict[str, bool]],
        output: Callable[[str], None] = print,
    ) -> None:
        self._lx200 = lx200
        self._server = server
        self._axis_status_provider = axis_status_provider
        self._output = output

    def run(self) -> None:
        self._output(self.banner_text())
        while True:
            try:
                line = input("manual> ")
            except EOFError:
                self._output("EOF, manual mode stopped.")
                return
            except KeyboardInterrupt:
                self._output("\nInterrupted, manual mode stopped.")
                return

            should_continue, message = self.handle_line(line)
            if message:
                self._output(message)
            if not should_continue:
                return

    def banner_text(self) -> str:
        axis_status = self._axis_status_provider()
        available_axes = [axis.upper() for axis, is_connected in axis_status.items() if is_connected]
        missing_axes = [axis.upper() for axis, is_connected in axis_status.items() if not is_connected]

        if missing_axes:
            lines = [
                f"Axis unavailable: {', '.join(missing_axes)}.",
                "Started single-axis manual mode.",
                "LX200 server is disabled by default.",
            ]
        else:
            lines = [
                "Started manual mode.",
                "LX200 server can be managed from this console.",
            ]

        if available_axes:
            lines.append(f"Available axes: {', '.join(available_axes)}.")
        else:
            lines.append("No axes are currently available.")

        lines.append("")
        lines.append(self.help_text())
        return "\n".join(lines)

    def help_text(self) -> str:
        axis_status = self._axis_status_provider()
        lines = [
            "Commands:",
            "  status      show axes and LX200 server state",
            "  lx200 on    start LX200 TCP server",
            "  lx200 off   stop LX200 TCP server",
            "  lx200 status  show LX200 TCP server state",
            "  help        show this help",
            "  quit        stop manual mode",
            "  :<cmd>#     send raw LX200 command manually",
            "  <cmd>       same raw LX200 command without wrappers",
        ]

        examples: list[str] = []
        if axis_status.get("ra"):
            examples.extend(
                [
                    "  RA examples: :Me#  :Mw#  :Q#  :Qe#  :Qw#",
                    "  RA get/set examples: :GR#  :Sr12:34:56#",
                ]
            )
        if axis_status.get("dec"):
            examples.extend(
                [
                    "  DEC examples: :Mn#  :Ms#  :Qn#  :Qs#",
                    "  DEC get/set examples: :GD#  :Sd+12*34:56#",
                ]
            )
        if not examples:
            examples.append("  No axis-specific examples yet because no axis is connected.")

        return "\n".join(lines + ["Examples:"] + examples)

    def handle_line(self, raw: str) -> tuple[bool, str | None]:
        line = raw.strip()
        if not line:
            return True, None

        lowered = line.lower()
        if lowered in {"help", "?"}:
            return True, self.help_text()
        if lowered == "status":
            return True, self.status_text()
        if lowered in {"quit", "exit"}:
            return False, "Manual mode stopped."
        if lowered == "lx200 status":
            return True, self._lx200_status_text()
        if lowered == "lx200 on":
            if self._server.is_running():
                return True, self._lx200_status_text()
            self._server.start_background()
            time.sleep(0.1)
            return True, self._lx200_status_text()
        if lowered == "lx200 off":
            if not self._server.is_running():
                return True, self._lx200_status_text()
            self._server.stop()
            return True, self._lx200_status_text()

        if (command := _normalize_lx200_command(line)) is None:
            return True, "Unknown command. Use `help`."

        try:
            result = self._lx200.handle(command)
        except Exception as exc:
            return True, f"LX200 error: {type(exc).__name__}: {exc}"

        if result is None:
            return True, f"{command} -> ∅"
        if isinstance(result, bool):
            return True, f"{command} -> {int(result)}"
        return True, f"{command} -> {result}"

    def status_text(self) -> str:
        axis_status = self._axis_status_provider()
        axis_chunks = [f"{axis.upper()}={'up' if is_connected else 'wait'}" for axis, is_connected in axis_status.items()]
        return "\n".join(
            [
                "Single-axis manual mode",
                f"Axes: {', '.join(axis_chunks)}",
                self._lx200_status_text(),
            ]
        )

    def _lx200_status_text(self) -> str:
        if self._server.is_running():
            return f"LX200 server: on ({self._server.host}:{self._server.port})"
        if self._server.last_error is not None:
            return f"LX200 server: off, last error: {type(self._server.last_error).__name__}: {self._server.last_error}"
        return f"LX200 server: off ({self._server.host}:{self._server.port})"
