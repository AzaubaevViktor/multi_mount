"""Wall-clock driver for the simulator: the one place in ``src/sim`` that waits.

``sim/clock.py`` says no module under ``src/sim`` may call ``time.*`` — and for
the tests that is exactly right: they advance a virtual clock and the whole
mount runs in microseconds. But the simulator has a second job that the virtual
clock cannot do.

A **live** run — a real external LX200 client (SkySafari and friends) connected
over TCP to the application, with simulated boards underneath instead of the two
controllers — has to happen in real time, because the client is real: it polls
``GR``/``GD`` several times a second and measures a slew by how long it takes.
On a virtual clock the simulated axis would only move when somebody explicitly
advanced time, so the client would see a mount frozen at a standstill.

This class is that exception, and it is deliberately the only one: it satisfies
the same interface :class:`sim.clock.Clock` does — ``now`` / ``advance`` for the
devices, ``monotonic`` / ``sleep`` for the production ``clock.Clock`` protocol —
but every one of them resolves to wall time. Nothing in ``src/tests`` may use
it; it exists for ``tools.sim_stack`` and for whoever wants to watch a
simulated mount move at the speed the sky does.

``time`` attributes are resolved at call time, exactly as in
``clock.RealClock``, so a test that monkeypatches ``time.monotonic`` globally
still sees through this class.
"""

import time

from sim.clock import Clock


class RealtimeClock(Clock):
    """A :class:`sim.clock.Clock` whose seconds are the wall's.

    Subclassed rather than merely duck-typed so that every place the simulator
    already declares it needs a ``Clock`` — the two device models, the fake
    port, the drivers — accepts this one without a cast or a parallel Protocol.
    The base class's invariant survives: time only ever moves forward. What
    changes is who moves it. On the virtual clock :meth:`advance` *sets* the
    instant; here it can only wait for it, and the inherited ``_now_s`` is
    unused because the wall keeps the count instead.
    """

    @property
    def now(self) -> float:
        return time.monotonic()

    def advance(self, dt_s: float) -> float:
        """Let ``dt_s`` really pass.

        The virtual clock *sets* the time here; this one can only wait for it,
        which is the whole difference between the two runs. ``FakeSerial``
        calls this to burn a read timeout, and burning it for real is what keeps
        a simulated board from answering a client faster than any board could.
        """
        if dt_s < 0:
            raise ValueError(f"cannot advance clock backwards: {dt_s}")
        time.sleep(dt_s)
        return self.now

    def monotonic(self) -> float:
        return time.monotonic()

    def sleep(self, seconds: float) -> None:
        time.sleep(seconds)
