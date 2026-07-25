"""Cheap scriptable per-command faults for the simulated devices.

This is the small subset of PLAN.md 2.4 that the red P1/P3 tests need right now
and that is cheap to express at the device level:

- empty response (once for the next matching command, or forever -> "dead motor");
- truncated response (drop the terminator, like the observed ``'=1'``).

Heavier chaos (connection drop mid-response, partial writes, byte loss,
mid-session ``OSError``) belongs to the transport seam in ``fake_serial`` and is
a separate stage, so it is deliberately not modelled here.
"""

from enum import Enum, auto


class FaultKind(Enum):
    EMPTY = auto()
    """Device produces no bytes for the affected reply."""

    TRUNCATE = auto()
    """Device produces the reply body but drops the trailing terminator."""


class FaultScript:
    """A queue of faults to apply to upcoming replies of a device.

    Each queued fault optionally targets a specific command name (e.g. ``"j"``
    for the SkyWatcher position inquiry or ``"status"`` for TMC); a fault with
    ``command=None`` hits the next reply regardless of command. Queued faults
    pop after a single use; :meth:`make_dead` installs a permanent fault for
    every subsequent reply, modelling a mute/dead motor. A device calls
    :meth:`apply` right before emitting a reply.
    """

    def __init__(self) -> None:
        self._queue: list[tuple[FaultKind, str | None]] = []
        self._dead: FaultKind | None = None

    def push(self, kind: FaultKind, command: str | None = None) -> None:
        self._queue.append((kind, command))

    def make_dead(self, kind: FaultKind = FaultKind.EMPTY) -> None:
        self._dead = kind

    def revive(self) -> None:
        self._dead = None
        self._queue.clear()

    def take(self, command: str | None = None) -> FaultKind | None:
        for index, (kind, target) in enumerate(self._queue):
            if target is None or target == command:
                del self._queue[index]
                return kind
        return self._dead

    def apply(self, reply: bytes, terminator: bytes, command: str | None = None) -> bytes:
        kind = self.take(command)
        if kind is None:
            return reply
        if kind is FaultKind.EMPTY:
            return b""
        if reply.endswith(terminator):
            return reply[: -len(terminator)]
        return reply
