"""The whole application over simulated boards, driven by LX200 command strings.

Every other simulator test hangs one driver off one device model on the virtual
clock. This one builds what ``python -m src --sim`` builds — drivers, axes,
combiner, LX200 handler — and talks to it the way the external client does:
through ``handle("Me")``, the same entry point ``lx200/base_server.py`` calls
with the bytes off the socket.

Why these run on the **wall clock** and cost real seconds
--------------------------------------------------------
``Axis.connect()`` starts a ``_motion_convertor`` thread per axis and
``Combiner.connect()`` starts the polar compensator; all three loop on real
time. Handing that stack a virtual clock does not make it fast — it makes it
hang, because the threads that would advance the work are waiting on a clock
only the test can move, while the test is waiting on them. So the choice here is
not "virtual or wall" but "this seam or none": either the top of the stack is
tested against the boards in real seconds, or the path from an LX200 command to
a turning axis is never tested end to end at all.

The waits are therefore kept to the tenths of a second the axes actually need,
and the assertions are on :attr:`SimStack.ra_sim` / :attr:`SimStack.dec_sim` —
on where the *board* thinks its axis is. Asserting on the driver's own status
would pass just as well if the command never reached the wire, which is the
failure these tests exist to catch.
"""

from collections.abc import Iterator
import time

import pytest

from sim.realtime import RealtimeClock
from tools.sim_stack import SimStack, build_sim_stack

# Long enough for the motion convertor thread to pick the command up and for the
# board to put counts on the axis, short enough to keep the unit suite quick.
# The RA axis moves thousands of counts in this time at the manual rate.
_SETTLE_S = 0.6


@pytest.fixture
def stack() -> Iterator[SimStack]:
    """A connected simulated mount that is always stopped afterwards.

    The teardown is not politeness: the axes' threads are non-daemon, so a test
    that leaves them running hangs the whole suite at exit (FRAME.md §2.5 says
    the same thing about a real axis and a real port).
    """
    built = build_sim_stack(RealtimeClock())
    built.sky_lx200.connect()
    try:
        yield built
    finally:
        built.sky_lx200.stop_all()
        built.sky_lx200.stop()


def test_the_simulated_stack_comes_up_with_both_axes_ready(stack: SimStack) -> None:
    """What ``__main__`` checks before it serves anything.

    Without this the ``--sim`` run falls through to the manual console instead
    of the dashboard, and an external client finds a mount that answers but
    never moves.
    """
    readiness = stack.combiner.motors_readiness()

    assert [axis.is_ready for axis in readiness] == [True, True], [
        axis.describe() for axis in readiness
    ]


def test_a_move_east_command_turns_the_simulated_ra_board(stack: SimStack) -> None:
    """``Me`` -> the RA board's own counter, not the driver's opinion of it."""
    started_at = stack.ra_sim.position

    stack.sky_lx200.handle("Me")
    time.sleep(_SETTLE_S)
    stack.sky_lx200.handle("GR")

    assert stack.ra_sim.position != started_at, (
        "the RA board did not move a step: the command never reached the wire"
    )


def test_halt_east_stops_the_simulated_ra_board(stack: SimStack) -> None:
    """``Qe`` after ``Me``: the board has to come to a stand, not just be told to.

    The stop is a ramp on this controller (``:K1``, RA_PROTOCOL.md §10.3), so the
    axis is given time to run it out before it is asked whether it still moves.
    """
    stack.sky_lx200.handle("Me")
    time.sleep(_SETTLE_S)
    stack.sky_lx200.handle("Qe")
    time.sleep(_SETTLE_S)

    stack.sky_lx200.handle("GR")
    stopped_at = stack.ra_sim.position
    time.sleep(_SETTLE_S)
    stack.sky_lx200.handle("GR")

    assert stack.ra_sim.position == stopped_at


def test_move_north_turns_the_simulated_dec_board(stack: SimStack) -> None:
    started_at = stack.dec_sim.position

    stack.sky_lx200.handle("Mn")
    time.sleep(_SETTLE_S)
    stack.sky_lx200.handle("GD")

    assert stack.dec_sim.position != started_at, "the DEC board did not move a step"


def test_the_stack_never_writes_a_position_to_the_ra_board(stack: SimStack) -> None:
    """FRAME.md §2.1: ``:E`` is destructive for the axes and is never sent.

    ``CM`` SYNC is the command that most looks like it should write one — the
    client is telling the mount where it is pointing. It has to move the
    software offset instead, and the wire has to stay clean.
    """
    sent: list[bytes] = []
    original_feed = stack.ra_sim.feed

    def recording_feed(data: bytes) -> None:
        sent.append(bytes(data))
        original_feed(data)

    stack.ra_sim.feed = recording_feed  # type: ignore[method-assign]

    stack.sky_lx200.handle("Sr12:00:00")
    stack.sky_lx200.handle("Sd+40*00:00")
    stack.sky_lx200.handle("CM")
    time.sleep(_SETTLE_S)
    stack.sky_lx200.handle("GR")

    assert sent, "the vacuity guard: nothing at all reached the RA board"
    assert not any(frame.startswith(b":E") for frame in sent), sent
