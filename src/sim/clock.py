"""Virtual clock for the mount simulator.

The whole simulator is detached from wall-clock time: kinematics integrate over
this clock and tests advance it explicitly with ``clock.advance(dt)``. No module
in ``src/sim`` is allowed to call ``time.time`` / ``time.monotonic`` / ``sleep``;
everything time-related flows through a ``Clock`` instance.

This class also implements the production ``clock.Clock`` protocol
(``monotonic()`` + ``sleep()``), so a test can hand the very same instance to
``SerialLine`` and to the motor drivers and virtualize the whole stack.
"""


class Clock:
    """A monotonic virtual clock measured in seconds.

    Starts at ``0.0`` and only moves forward when :meth:`advance` is called.
    Devices read :attr:`now` to integrate position and to schedule response
    readiness; they never observe real time.
    """

    def __init__(self, start_s: float = 0.0) -> None:
        self._now_s = float(start_s)

    @property
    def now(self) -> float:
        return self._now_s

    def advance(self, dt_s: float) -> float:
        if dt_s < 0:
            raise ValueError(f"cannot advance clock backwards: {dt_s}")
        self._now_s += float(dt_s)
        return self._now_s

    def monotonic(self) -> float:
        return self._now_s

    def sleep(self, seconds: float) -> None:
        # Waiting on a virtual clock is exactly "let that much time pass": the
        # devices then integrate over it on their next command, so driver retry
        # loops and timeouts terminate without touching real time.
        self.advance(seconds)
