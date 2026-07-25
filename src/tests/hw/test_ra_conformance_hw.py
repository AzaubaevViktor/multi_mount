"""Тот же conformance-набор, но против живой платы RA.

Не запускается ни по `pytest`, ни по `pytest src/tests/hw` без явного указания
порта: `testpaths` в pyproject.toml не включает `src/tests/hw`, а этот модуль
дополнительно требует переменную окружения.

    RA_PORT=/dev/tty.PL2303G-USBtoUART2120 \
        .venv/bin/python -m pytest src/tests/hw/test_ra_conformance_hw.py -q

Кейсы, которые крутят ось, по умолчанию выключены — плата стоит на монтировке:

    RA_PORT=... RA_MOTION=1 .venv/bin/python -m pytest src/tests/hw/test_ra_conformance_hw.py -q

Отчёт «кейс → ожидание → факт → совпало/нет» кладётся в
`logs/protocol/ra_conformance_hw_<штамп>.md`, байтовая трасса — рядом в `.jsonl`.
Класс `NEVER` не исполняется ни при каких флагах: `HardwareWire` роняет попытку
до записи в порт (см. `src/ra_conformance/runner.py`).

Тот же набор против симулятора живёт в `src/tests/units/test_ra_conformance.py`
и гоняется обычным юнит-прогоном.
"""

import datetime
import os
from collections.abc import Iterator
from pathlib import Path

import pytest

from clock import REAL_CLOCK
from ra_conformance import (
    CASES,
    HARDWARE_DEFAULT_SAFETY,
    HardwareWire,
    Safety,
    UnsafeOnHardware,
    render_markdown,
    run_cases,
    stop_axis,
    summarize,
)
from serial_wrapper.recorder import JsonlRecorder
from serial_wrapper.wrapper import SerialLine

PORT_ENV = "RA_PORT"
MOTION_ENV = "RA_MOTION"
BAUD = 115200
TIMEOUT_S = 0.5
REPORT_DIR = Path("logs/protocol")

pytestmark = pytest.mark.skipif(
    not os.environ.get(PORT_ENV),
    reason=f"нужен живой порт платы RA: {PORT_ENV}=/dev/tty...",
)


def _safety() -> frozenset[Safety]:
    if os.environ.get(MOTION_ENV):
        return HARDWARE_DEFAULT_SAFETY | {Safety.MOTION}
    return HARDWARE_DEFAULT_SAFETY


@pytest.fixture(scope="module")
def ra_wire() -> Iterator[HardwareWire]:
    port = os.environ[PORT_ENV]
    REPORT_DIR.mkdir(parents=True, exist_ok=True)
    stamp = datetime.datetime.now(datetime.UTC).strftime("%Y%m%dT%H%M%SZ")
    recorder = JsonlRecorder(REPORT_DIR / f"ra_conformance_hw_{stamp}.jsonl")
    line = SerialLine(
        port=port, baud=BAUD, timeout_s=TIMEOUT_S, name="ra", terminator="\r", clock=REAL_CLOCK, recorder=recorder
    )
    wire = HardwareWire(line, REAL_CLOCK, _safety())
    line.connect()
    try:
        yield wire
    finally:
        # Ось останавливается раньше, чем закрывается порт: закрытый порт не
        # останавливает ничего (тот же порядок, что в tools/hw_session.py).
        if Safety.MOTION in _safety():
            stop_axis(wire)
        line.close(reason="conformance_finished")
        recorder.close()


def test_ra_conformance(ra_wire: HardwareWire) -> None:
    safety = _safety()
    results = run_cases(ra_wire, CASES, safety)
    report = render_markdown(results, f"Живая плата RA, {datetime.datetime.now(datetime.UTC).isoformat()}")
    REPORT_DIR.mkdir(parents=True, exist_ok=True)
    stamp = datetime.datetime.now(datetime.UTC).strftime("%Y%m%dT%H%M%SZ")
    path = REPORT_DIR / f"ra_conformance_hw_{stamp}.md"
    path.write_text(report, encoding="utf-8")

    summary = summarize(results)
    assert summary.clean, f"отчёт: {path}\n\n{report}"


def test_never_class_is_not_executed(ra_wire: HardwareWire) -> None:
    """Проверка самой защиты, а не платы: `:W1` не уходит в порт даже вручную."""
    with pytest.raises(UnsafeOnHardware):
        ra_wire.exchange(b":W1000009\r", Safety.NEVER)
