"""Run the RA conformance suite against the live board (or against the simulator).

This is the hardware half of RA_REWRITE_PLAN.md §3 Э2. The case list and the
executor are shared with ``src/tests/units/test_ra_conformance.py`` — this
module only builds a transport, points the runner at it and writes the
«кейс → ожидание → факт → совпало/нет» table out as markdown.

It does nothing by default: the port has to be named, and motion has to be
asked for.

Usage::

    # dry run, no hardware involved — do this first, every time
    PYTHONPATH=src .venv/bin/python -m tools.ra_conformance --sim

    # read-only and non-motion writes against the board
    PYTHONPATH=src .venv/bin/python -m tools.ra_conformance --port /dev/tty.PL2303G-USBtoUART2120

    # the same plus the cases that actually turn the axis
    PYTHONPATH=src .venv/bin/python -m tools.ra_conformance --port /dev/tty.X --motion

    # just look at the case list
    PYTHONPATH=src .venv/bin/python -m tools.ra_conformance --list

Safety: cases classed ``NEVER`` are not executed on hardware under any flag, and
:class:`ra_conformance.HardwareWire` refuses their payloads a second time before
the write. Whatever happens, the axis is stopped and the port is closed on the
way out.
"""

import argparse
import datetime
import logging
import sys
from collections.abc import Sequence
from pathlib import Path

from clock import REAL_CLOCK
from logging_setup import setup_logging
from ra_conformance import (
    CASES,
    HARDWARE_DEFAULT_SAFETY,
    SIMULATOR_SAFETY,
    CaseResult,
    HardwareWire,
    Safety,
    SerialWire,
    Status,
    Wire,
    describe_cases,
    render_markdown,
    run_cases,
    stop_axis,
    summarize,
)
from serial_wrapper.recorder import JsonlRecorder
from serial_wrapper.wrapper import SerialLine, SerialLineSearchError
from sim import Clock as VirtualClock
from sim import SimSerialLine, SkyWatcherSim

LOGGER = logging.getLogger("ra_conformance")

# Same line parameters the protocol document ran with (§1); the 0.5 s timeout is
# load-bearing, because "the board stays silent" is a documented answer (§8).
DEFAULT_PATTERN = "PL2303G"
DEFAULT_BAUD = 115200
DEFAULT_TIMEOUT_S = 0.5


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="tools.ra_conformance",
        description="Прогнать протокольные кейсы RA против платы или симулятора.",
    )
    parser.add_argument("--port", default=None, help="последовательный порт платы RA; обязателен для железа")
    parser.add_argument("--pattern", default=None, help=f"искать порт в /dev по регулярке (например {DEFAULT_PATTERN})")
    parser.add_argument("--sim", action="store_true", help="прогон против src/sim, железо не требуется")
    parser.add_argument("--motion", action="store_true", help="разрешить кейсы, которые крутят ось")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument("--timeout", type=float, default=DEFAULT_TIMEOUT_S)
    parser.add_argument("--out", default=None, help="куда положить markdown-отчёт")
    parser.add_argument("--trace", default=None, help="куда положить байтовую трассу JSON Lines")
    parser.add_argument("--list", action="store_true", help="напечатать таблицу кейсов и выйти")
    parser.add_argument("--logs-root", default="logs", help="корень каталогов текстовых логов")
    return parser


def _resolve_port(args: argparse.Namespace) -> str:
    if args.port:
        return str(args.port)
    if args.pattern:
        try:
            return SerialLine.search(str(args.pattern))
        except SerialLineSearchError as exc:
            raise SystemExit(f"порт по регулярке {args.pattern!r} не найден: {exc}") from exc
    raise SystemExit("нужен --port (или --pattern, или --sim): вслепую на железо этот набор не ходит")


def _selected_safety(motion: bool) -> frozenset[Safety]:
    if motion:
        return HARDWARE_DEFAULT_SAFETY | {Safety.MOTION}
    return HARDWARE_DEFAULT_SAFETY


def _run_against_simulator() -> list[CaseResult]:
    clock = VirtualClock()
    device = SkyWatcherSim(clock)
    line = SimSerialLine(
        device, clock, port="sim://ra", baud=DEFAULT_BAUD, timeout_s=DEFAULT_TIMEOUT_S, name="ra", terminator="\r"
    )
    line.connect()
    return run_cases(SerialWire(line, clock), CASES, SIMULATOR_SAFETY)


def _run_against_hardware(port: str, baud: int, timeout_s: float, safety: frozenset[Safety], trace: Path | None) -> list[CaseResult]:
    recorder = JsonlRecorder(trace) if trace is not None else None
    line = SerialLine(
        port=port, baud=baud, timeout_s=timeout_s, name="ra", terminator="\r", clock=REAL_CLOCK, recorder=recorder
    )
    wire: Wire = HardwareWire(line, REAL_CLOCK, safety)
    line.connect()
    try:
        return run_cases(wire, CASES, safety)
    finally:
        # Belt and braces over `run_cases`' own finally: the axis is stopped
        # before the port is dropped, and the port is dropped whatever happened.
        if Safety.MOTION in safety and not stop_axis(wire):
            LOGGER.critical("ОСЬ МОЖЕТ ПРОДОЛЖАТЬ ДВИГАТЬСЯ: остановить не удалось, снимайте питание")
        line.close(reason="conformance_finished")
        if recorder is not None:
            recorder.close()


def _stamp() -> str:
    return datetime.datetime.now(datetime.UTC).strftime("%Y%m%dT%H%M%SZ")


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    setup_logging(logs_root=args.logs_root)

    if args.list:
        # Written out rather than logged: the log formatter escapes newlines, and
        # a 104-row table is only readable as a file.
        out = Path(args.out) if args.out else Path("logs/protocol/ra_conformance_cases.md")
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(f"# Кейсы RA-conformance ({len(CASES)})\n\n{describe_cases(CASES)}", encoding="utf-8")
        LOGGER.info("список кейсов (%d): %s", len(CASES), out)
        return 0

    mode = "sim" if args.sim else "hw"
    if mode == "sim":
        safety = SIMULATOR_SAFETY
        results = _run_against_simulator()
        title = "Симулятор"
    else:
        port = _resolve_port(args)
        safety = _selected_safety(bool(args.motion))
        trace = Path(args.trace) if args.trace else Path("logs/protocol") / f"ra_conformance_{_stamp()}.jsonl"
        trace.parent.mkdir(parents=True, exist_ok=True)
        LOGGER.info("порт %s, классы %s, трасса %s", port, sorted(item.value for item in safety), trace)
        results = _run_against_hardware(port, int(args.baud), float(args.timeout), safety, trace)
        title = f"Живая плата RA, порт {port}"

    report = render_markdown(results, title)
    out = Path(args.out) if args.out else Path("logs/protocol") / f"ra_conformance_{mode}_{_stamp()}.md"
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(report, encoding="utf-8")

    summary = summarize(results)
    LOGGER.info("отчёт: %s", out)
    LOGGER.info("%s", summary.as_line())
    # The table goes to the file; the log carries only what a human has to act
    # on. One line per divergence, so a failed run is readable in the console.
    for result in results:
        if result.status in (Status.MISMATCH, Status.ERROR):
            LOGGER.error("%s (%s): %s", result.case.name, result.case.section, result.detail)
    return 0 if summary.clean else 1


if __name__ == "__main__":
    sys.exit(main())
