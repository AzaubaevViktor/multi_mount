"""Data model of one RA protocol conformance case.

A case is **data**, not code: what to put on the wire byte for byte, what the
board is supposed to answer byte for byte, which section of
``docs/protocol/RA_PROTOCOL.md`` says so, and whether it may be executed against
the live board at all. The executor in :mod:`ra_conformance.runner` is the only
place that knows how to compare anything, so the simulator and the hardware are
judged by literally the same code.

Why a case is a *sequence* of exchanges rather than one request/response pair:
half of what the document states is about state — the period clamp needs a
write and a read back, the firmware quirk of §11 needs a running axis and a
pause, framing cases of §8 need a write that is not a command at all. A single
pair could only cover §3.

Every case is written to be **self-contained and restoring**: it starts from the
power-on state described in §12.1 and puts the board back into it through
:attr:`Case.teardown`, which runs whether the case passed or not. That is what
lets the same list be executed as one long session on hardware and as one fresh
simulator per case in the unit suite.
"""

import dataclasses
import re
from collections.abc import Callable
from enum import StrEnum


class Safety(StrEnum):
    """What a case does to the board — the axis of the safety split.

    ``NEVER`` is not a severity, it is a mechanical fact: the runner refuses to
    execute such a case on a live transport, and the live wire refuses the
    payload a second time on its way to the port (:mod:`ra_conformance.runner`).
    """

    READ = "read"
    WRITE = "write"
    MOTION = "motion"
    NEVER = "never"


def render_bytes(data: bytes | None) -> str:
    """Wire bytes as they read in a report: ``:f1\\r`` instead of ``b':f1\\r'``."""
    if data is None:
        return "(только чтение)"
    if not data:
        return "(ничего)"
    text = data.decode("ascii", errors="replace")
    return text.replace("\\", "\\\\").replace("\r", "\\r").replace("\n", "\\n")


@dataclasses.dataclass(frozen=True)
class Expect:
    """A statement about one reply: how it reads in the report, and how to test it."""

    text: str
    match: Callable[[bytes], bool]


def exact(reply: bytes) -> Expect:
    return Expect(render_bytes(reply), lambda got: got == reply)


def silence() -> Expect:
    """No reply at all — the board stays quiet until the read timeout expires."""
    return Expect("(тишина)", lambda got: got == b"")


def one_of(*replies: bytes) -> Expect:
    text = " | ".join(render_bytes(reply) for reply in replies)
    accepted = frozenset(replies)
    return Expect(text, lambda got: got in accepted)


def matching(pattern: str, text: str) -> Expect:
    """A reply the document describes by shape rather than by value.

    Used only where the live board legitimately varies: a drifting voltage, a
    brake point that follows the last direction, an initialization flag whose
    state depends on the session's history.
    """
    regex = re.compile(pattern.encode("ascii"))
    return Expect(text, lambda got: regex.fullmatch(got) is not None)


@dataclasses.dataclass(frozen=True)
class Exchange:
    """Write ``send`` (or nothing, when it is ``None``) and judge what comes back."""

    send: bytes | None
    expect: Expect
    timeout_s: float | None = None


@dataclasses.dataclass(frozen=True)
class Pause:
    """Let time pass — real seconds on hardware, virtual ones on the simulator."""

    seconds: float


type Step = Exchange | Pause

# A cross-step assertion: gets every reply the case collected, returns an error
# message or None. Needed where the claim is about a *relation* between replies
# (the two bytes of a voltage, a period written and read back), which no
# per-reply predicate can express.
type Check = Callable[[tuple[bytes, ...]], str | None]


@dataclasses.dataclass(frozen=True)
class Case:
    name: str
    section: str
    safety: Safety
    steps: tuple[Step, ...]
    note: str = ""
    check: Check | None = None
    # Sent after the case whatever happened to it, replies ignored: this is what
    # returns the board to the power-on state, and for a motion case it is also
    # the guaranteed stop.
    teardown: tuple[bytes, ...] = ()

    @property
    def sent(self) -> tuple[bytes, ...]:
        return tuple(step.send for step in self.steps if isinstance(step, Exchange) and step.send is not None)


class Status(StrEnum):
    MATCH = "match"
    MISMATCH = "mismatch"
    SKIPPED = "skipped"
    ERROR = "error"


@dataclasses.dataclass(frozen=True)
class StepResult:
    sent: bytes | None
    expected: str
    actual: bytes
    ok: bool


@dataclasses.dataclass(frozen=True)
class CaseResult:
    case: Case
    status: Status
    steps: tuple[StepResult, ...] = ()
    detail: str = ""

    @property
    def failed_step(self) -> StepResult | None:
        for step in self.steps:
            if not step.ok:
                return step
        return None
