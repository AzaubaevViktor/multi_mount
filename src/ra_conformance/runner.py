"""One executor, two transports.

The whole point of this module is that there is exactly one place where a reply
is compared to an expectation. The simulator and the live board differ only in
which :class:`Wire` is handed to :func:`run_cases`; everything downstream — the
comparison, the teardown, the report — is the same code, so the two can no
longer drift apart silently.

Safety is enforced twice, on purpose:

1. :func:`run_cases` never executes a case whose :class:`~ra_conformance.model.Safety`
   is not in the selected set, and :data:`HARDWARE_DEFAULT_SAFETY` does not
   contain ``MOTION``;
2. :class:`HardwareWire` checks every payload against the whitelist of
   ``RA_PROTOCOL_STEP_2.md`` §1.1 **before** it reaches the port and raises
   :class:`UnsafeOnHardware` if it is not there.

The second check is what makes "never send ``:W1``" a property of the code
rather than of anybody's discipline: a case that is mislabelled, a teardown that
grows a new command, or a hand-written experiment all hit the same guard, and a
raise happens before the write, not after.
"""

import dataclasses
from collections.abc import Iterable, Sequence
from typing import Protocol

from clock import Clock
from ra_conformance.model import (
    Case,
    CaseResult,
    Exchange,
    Pause,
    Safety,
    Status,
    StepResult,
    render_bytes,
)

# Command prefixes that may be written to the live board, transcribed from
# `docs/protocol/RA_PROTOCOL_STEP_2.md` §1.1. Two additions to that list, both
# pure inquiries that §3 of RA_PROTOCOL.md read off the board and that change
# nothing: `:r1` (Inquire Register Value) and `:f`-family channel probes, which
# are covered by the exact-payload set below. No write command is here: not
# `:W1`, not `:N1`, not `:R1`, not `:A1`, not `:Q1`.
HARDWARE_COMMAND_PREFIXES: tuple[str, ...] = (
    ":e1", ":a1", ":b1", ":c1", ":d1", ":D1", ":f1", ":f2", ":f3", ":g1",
    ":h1", ":i1", ":j1", ":m1", ":s1", ":r1", ":q1", ":C1", ":n1",
    ":E1", ":G1", ":H1", ":I1", ":J1", ":K1", ":M1", ":F1",
)

# Deliberately malformed input the document exercised on the live board (§7, §8)
# plus the unknown-letter probes of §4. None of them can change board state:
# the board rejects them before acting, or ignores them for want of a colon.
HARDWARE_EXACT_PAYLOADS: frozenset[str] = frozenset(
    {
        ":", ":f", ":f0", ":f4", ":f9", ":f?", ":fL", ":f11", ":fL#", ":f1#", ":e1#",
        ":IZZZZZZ", ":k10",
        ":" + "A" * 60,
        # §3.2 read the board-wide constants and the position off a phantom
        # channel. Step 2 whitelisted only `:f2`/`:f3`; these three are the same
        # kind of inquiry on the same non-existent axis and cannot write anything.
        ":e2", ":a2", ":j2",
    }
    | {f":{letter}1" for letter in "XYZloptuvwxy"}
)


class UnsafeOnHardware(RuntimeError):
    """A payload or a case that must never touch the live board tried to."""


class Line(Protocol):
    """The slice of :class:`serial_wrapper.wrapper.SerialLine` a wire needs."""

    def query(self, payload: str | None, timeout: float | None = None) -> str: ...


class Wire(Protocol):
    """Send bytes, get bytes back; let time pass. Nothing else."""

    def exchange(self, payload: bytes | None, safety: Safety, timeout_s: float | None = None) -> bytes: ...

    def pause(self, seconds: float) -> None: ...


class SerialWire:
    """A wire over any :class:`Line`. Used as-is for the simulator.

    Bytes in, bytes out: the cases are byte-exact and the report has to quote
    what actually came back, so the ``str`` that ``SerialLine`` deals in is only
    ever an internal detail of this class.
    """

    def __init__(self, line: Line, clock: Clock) -> None:
        self._line = line
        self._clock = clock

    def exchange(self, payload: bytes | None, safety: Safety, timeout_s: float | None = None) -> bytes:
        self._guard(payload, safety)
        text = self._line.query(None if payload is None else payload.decode("ascii"), timeout=timeout_s)
        return text.encode("ascii", errors="replace")

    def pause(self, seconds: float) -> None:
        self._clock.sleep(seconds)

    def _guard(self, payload: bytes | None, safety: Safety) -> None:
        """No restrictions against a simulator: the point of it is to be safe."""


class HardwareWire(SerialWire):
    """A wire that refuses to put anything unvetted on a real serial port."""

    def __init__(self, line: Line, clock: Clock, allowed: Iterable[Safety]) -> None:
        super().__init__(line, clock)
        # `NEVER` is subtracted rather than trusted to the caller: this class is
        # the last thing between a mislabelled case and the board.
        self._allowed = frozenset(allowed) - {Safety.NEVER}

    def _guard(self, payload: bytes | None, safety: Safety) -> None:
        if safety not in self._allowed:
            raise UnsafeOnHardware(
                f"класс безопасности {safety.value!r} не разрешён на живой плате "
                f"(разрешено: {sorted(item.value for item in self._allowed)})"
            )
        if payload is None:
            return
        for chunk in payload.split(b"\r"):
            if not is_allowed_on_hardware(chunk):
                raise UnsafeOnHardware(
                    f"команда {render_bytes(chunk)!r} вне белого списка RA_PROTOCOL_STEP_2 §1.1"
                )


def is_allowed_on_hardware(chunk: bytes) -> bool:
    """Whether one colon-framed chunk may be written to the live board."""
    text = chunk.decode("ascii", errors="replace")
    if not text.startswith(":"):
        # Without a leading colon the board does not react at all (§8), so the
        # framing cases (`garbage`, `f1`, a bare CR) are harmless by construction.
        return True
    if text in HARDWARE_EXACT_PAYLOADS:
        return True
    return any(text.startswith(prefix) for prefix in HARDWARE_COMMAND_PREFIXES)


# Motion is opt-in even on hardware: the axis is attached to a mount, and a
# conformance run should not start turning it because somebody forgot a flag.
HARDWARE_DEFAULT_SAFETY: frozenset[Safety] = frozenset({Safety.READ, Safety.WRITE})
SIMULATOR_SAFETY: frozenset[Safety] = frozenset({Safety.READ, Safety.WRITE, Safety.MOTION, Safety.NEVER})

# Poll interval and budget for "the Running bit is down", used by the guaranteed
# stop. §10.3 measured a ramp still running 0.6 s after `:K1` at 100x sidereal.
_STOP_POLL_S = 0.2
_STOP_ATTEMPTS = 15


def stop_axis(wire: Wire) -> bool:
    """Send ``:K1`` and wait for the Running bit to drop. Never raises."""
    stopped = False
    for attempt in range(_STOP_ATTEMPTS):
        try:
            if attempt == 0:
                wire.exchange(b":K1\r", Safety.MOTION)
            status = wire.exchange(b":f1\r", Safety.MOTION)
        # A failing stop is retried, never propagated: this is the last line of
        # defence between an exception and a turning axis.
        except Exception:
            wire.pause(_STOP_POLL_S)
            continue
        if len(status) >= 4 and status[2:3] == b"0":
            stopped = True
            break
        wire.pause(_STOP_POLL_S)
    return stopped


def run_cases(wire: Wire, cases: Sequence[Case], safety: Iterable[Safety]) -> list[CaseResult]:
    """Execute the list against one transport and report what happened.

    Whatever a case does, its teardown runs; whatever the run does, the axis is
    stopped at the end if motion was ever allowed.
    """
    selected = frozenset(safety)
    results: list[CaseResult] = []
    try:
        for case in cases:
            if case.safety not in selected:
                results.append(
                    CaseResult(case, Status.SKIPPED, detail=f"класс {case.safety.value} не выбран для этого прогона")
                )
                continue
            results.append(run_case(wire, case))
    finally:
        if Safety.MOTION in selected:
            stop_axis(wire)
    return results


def run_case(wire: Wire, case: Case) -> CaseResult:
    steps: list[StepResult] = []
    status = Status.MATCH
    detail = ""
    try:
        for step in case.steps:
            if isinstance(step, Pause):
                wire.pause(step.seconds)
                continue
            actual = wire.exchange(step.send, case.safety, step.timeout_s)
            ok = step.expect.match(actual)
            steps.append(StepResult(step.send, step.expect.text, actual, ok))
            if not ok:
                status = Status.MISMATCH
                detail = f"шаг {len(steps)}: ожидалось {step.expect.text}, получено {render_bytes(actual)}"
                break
        else:
            if case.check is not None:
                message = case.check(tuple(item.actual for item in steps))
                if message is not None:
                    status = Status.MISMATCH
                    detail = message
    except UnsafeOnHardware as exc:
        status = Status.ERROR
        detail = str(exc)
    # One broken case must not end the run: the rest of the table is still evidence.
    except Exception as exc:
        status = Status.ERROR
        detail = f"{type(exc).__name__}: {exc}"
    finally:
        _teardown(wire, case)
    return CaseResult(case, status, tuple(steps), detail)


def _teardown(wire: Wire, case: Case) -> None:
    if case.safety is Safety.MOTION:
        # The stop comes first and it *waits*: `K` is a ramp (§10.3), and the
        # restore payloads below contain `:E`, which on a still-running axis is
        # exactly the firmware quirk of §11. Without this wait the suite would
        # be committing the mistake it exists to detect — and it did, until the
        # one-session run went red on the next case.
        stop_axis(wire)
    for payload in case.teardown:
        try:
            wire.exchange(payload, case.safety)
        # Teardown is best effort by definition; the case verdict is already decided.
        except Exception:
            return


# --------------------------------------------------------------------------- #
# report
# --------------------------------------------------------------------------- #


@dataclasses.dataclass(frozen=True)
class Summary:
    total: int
    matched: int
    mismatched: int
    skipped: int
    errored: int

    @property
    def clean(self) -> bool:
        return self.mismatched == 0 and self.errored == 0

    def as_line(self) -> str:
        return (
            f"кейсов {self.total}: совпало {self.matched}, расхождений {self.mismatched}, "
            f"ошибок {self.errored}, пропущено {self.skipped}"
        )


def summarize(results: Sequence[CaseResult]) -> Summary:
    counts = dict.fromkeys(Status, 0)
    for result in results:
        counts[result.status] += 1
    return Summary(
        total=len(results),
        matched=counts[Status.MATCH],
        mismatched=counts[Status.MISMATCH],
        skipped=counts[Status.SKIPPED],
        errored=counts[Status.ERROR],
    )


_VERDICT = {
    Status.MATCH: "совпало",
    Status.MISMATCH: "**РАСХОЖДЕНИЕ**",
    Status.SKIPPED: "не исполнялся",
    Status.ERROR: "ошибка",
}


def render_markdown(results: Sequence[CaseResult], title: str) -> str:
    """The «кейс → ожидание → факт → совпало/нет» table, ready to paste."""
    summary = summarize(results)
    lines = [
        f"## {title}",
        "",
        summary.as_line(),
        "",
        "| Кейс | Раздел | Класс | Послано | Ожидание | Факт | Итог |",
        "|---|---|---|---|---|---|---|",
    ]
    for result in results:
        step = result.failed_step or (result.steps[-1] if result.steps else None)
        sent = render_bytes(step.sent) if step is not None else "—"
        expected = step.expected if step is not None else "—"
        actual = render_bytes(step.actual) if step is not None else "—"
        lines.append(
            f"| `{result.case.name}` | {result.case.section} | {result.case.safety.value} "
            f"| `{sent}` | `{expected}` | `{actual}` | {_VERDICT[result.status]} |"
        )
    mismatches = [result for result in results if result.status in (Status.MISMATCH, Status.ERROR)]
    if mismatches:
        lines.extend(["", "### Расхождения", ""])
        for result in mismatches:
            lines.append(f"* `{result.case.name}` ({result.case.section}): {result.detail}")
    else:
        lines.extend(["", "Расхождений нет.", ""])
    return "\n".join(lines) + "\n"


def _exchange_count(case: Case) -> int:
    return sum(1 for step in case.steps if isinstance(step, Exchange))


def describe_cases(cases: Sequence[Case]) -> str:
    """The case list itself as a table — the human-readable half of the suite."""
    lines = [
        "| Кейс | Раздел | Класс | Обменов | Комментарий |",
        "|---|---|---|---|---|",
    ]
    for case in cases:
        lines.append(
            f"| `{case.name}` | {case.section} | {case.safety.value} | {_exchange_count(case)} | {case.note} |"
        )
    return "\n".join(lines) + "\n"
