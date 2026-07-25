"""The whole application on two simulated boards, assembled in one place.

What this is for
----------------
Until now the simulator could only be reached from inside a test: each test
built its own ``SimSerialLine`` around a device model and hung one driver off
it. The application itself (``src/__main__.py``) went looking for real ports and,
finding none, fell back to :class:`sky.unavailable_motor.UnavailableMotor` — a
stub that answers "axis unavailable" to everything. So there was no way to point
a **real external LX200 client** (SkySafari and the like) at this project
without the hardware on the bench.

This module closes that gap. It builds the same object graph ``__main__`` builds
— two motor drivers, two axes, the combiner, the LX200 handler — with the
bottom of the stack replaced by :class:`sim.fake_serial.SimSerialLine`, and
nothing above the transport can tell the difference. ``python -m src --sim``
serves it over TCP on the usual port; a test imports :func:`build_sim_stack` and
gets the identical graph on a virtual clock.

That "identical" is the point of putting the assembly here rather than writing
it twice: what a test proves about the stack is then a statement about the thing
the external client talks to, not about a lookalike built in a fixture.

Which board is simulated
------------------------
Both device models default to **our** hardware, so a caller that passes nothing
drives the controllers this project actually owns:

- the RA model already defaults to the measured Star Adventurer 2i (CPR
  12 492 146, 16 MHz timer, the 9 000 steps/s ceiling, the 1103 clamp);
- the DEC model is given ``uart_to_driver_dead=True`` here, because that is the
  state of the board on the bench — the UART between the Arduino and the
  TMC2209 is electrically broken (``docs/protocol/DEC_PROTOCOL.md`` §4), and a
  simulator that quietly had a working one would make the driver look healthier
  in simulation than it is in the room.

Two defaults are deliberately *not* the hardware's:

- ``reboot_on_goto_arrival`` is off. The live board dies on the last step before
  a GOTO target (``FRAME.md`` §2.2), and the driver's job is never to let it get
  there; a live simulated session is for watching a client work, not for
  reproducing that crash, and the tests that do reproduce it turn the flag on
  explicitly.
- the clock. :class:`sim.clock.Clock` — virtual, the tests' — is the default;
  a live run passes :class:`sim.realtime.RealtimeClock`, because an external
  client measures a slew by how long it takes.
"""

from dataclasses import dataclass
import logging
from typing import Any

from sim.clock import Clock
from sim.fake_serial import SimSerialLine
from sim.skywatcher_sim import SkyWatcherSim
from sim.tmc_sim import TMC2209Sim
from sky.axis import AxisDEC, AxisRA
from sky.combiner import Combiner
from sky.lx200 import SkyLX200
from skywatcher.motor import SkyWatcherMotor
from tmc2209.motor import TMC2209Motor

# The two ports of `src/__main__.py`, kept to the byte: the RA board is opened
# with a 50 ms timeout and a `\r` terminator, the DEC one with 2 s and `\n`.
# These are not cosmetic — the RA driver's whole retry behaviour is timed
# against that 50 ms, so a simulated line with a different one would exercise a
# driver nobody runs.
RA_TIMEOUT_S = 0.05
DEC_TIMEOUT_S = 2.0



@dataclass(frozen=True)
class SimStack:
    """Every layer of the simulated mount, from the device models up.

    The models (:attr:`ra_sim`, :attr:`dec_sim`) are exposed on purpose: a test
    reaches past the driver to ask the *board* where the axis really is, or to
    make it misbehave through ``.faults``. That is the only way to tell a driver
    that moved the axis from one that merely thinks it did.
    """

    clock: Clock
    ra_sim: SkyWatcherSim
    dec_sim: TMC2209Sim
    ra_line: SimSerialLine
    dec_line: SimSerialLine
    ra_motor: SkyWatcherMotor
    dec_motor: TMC2209Motor
    axis_ra: AxisRA
    axis_dec: AxisDEC
    combiner: Combiner
    sky_lx200: SkyLX200


def build_sim_stack(
    clock: Clock | None = None,
    *,
    ra_config: dict[str, Any] | None = None,
    dec_config: dict[str, Any] | None = None,
) -> SimStack:
    """Assemble the application over simulated boards.

    ``clock`` must satisfy both halves of the simulator's time interface
    (``now``/``advance`` for the device models, ``monotonic``/``sleep`` for the
    transport and the drivers); :class:`sim.clock.Clock` and
    :class:`sim.realtime.RealtimeClock` both do. It defaults to a fresh virtual
    one.

    Nothing is connected here — the caller decides when, because ``connect()``
    talks to the boards and a test may want to watch the handshake.
    """
    clock = Clock() if clock is None else clock

    ra_sim = SkyWatcherSim(clock, **(ra_config or {}))
    ra_line = SimSerialLine(
        ra_sim, clock, port="sim://ra", timeout_s=RA_TIMEOUT_S, name="sw", terminator="\r"
    )
    ra_motor = SkyWatcherMotor(ra_line, clock)

    dec_defaults: dict[str, Any] = {"uart_to_driver_dead": True}
    dec_sim = TMC2209Sim(clock, **{**dec_defaults, **(dec_config or {})})
    dec_line = SimSerialLine(
        dec_sim, clock, port="sim://dec", timeout_s=DEC_TIMEOUT_S, name="tmc", terminator="\n"
    )
    dec_motor = TMC2209Motor(dec_line, clock)

    axis_ra = AxisRA(ra_motor)
    axis_dec = AxisDEC(dec_motor)
    combiner = Combiner(axis_ra, axis_dec)
    sky_lx200 = SkyLX200(combiner)

    return SimStack(
        clock=clock,
        ra_sim=ra_sim,
        dec_sim=dec_sim,
        ra_line=ra_line,
        dec_line=dec_line,
        ra_motor=ra_motor,
        dec_motor=dec_motor,
        axis_ra=axis_ra,
        axis_dec=axis_dec,
        combiner=combiner,
        sky_lx200=sky_lx200,
    )


def build_realtime_sim_stack(**kwargs: Any) -> SimStack:
    """:func:`build_sim_stack` on the wall clock, for a live external client.

    Imported lazily-ish (the realtime clock is the single module in ``src/sim``
    allowed to touch ``time``), and kept as a named function so that the one
    place a wall clock enters the simulator is greppable.
    """
    from sim.realtime import RealtimeClock

    logging.getLogger("sim").info("Simulated mount on the wall clock: boards are fake, seconds are not")
    return build_sim_stack(RealtimeClock(), **kwargs)
