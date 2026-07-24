"""Mount simulator: fake serial ports + device models on a virtual clock.

The simulator substitutes the lowest level of the stack — the pyserial object
inside ``SerialLine`` — so ``SerialLine``, the motor drivers and everything
above run unchanged (PLAN.md §2). Time is fully virtual: tests drive it with
``Clock.advance(dt)``, kinematics integrate over it, and nothing in this
package touches real time.
"""

from sim.clock import Clock
from sim.fake_serial import Device, FakeSerial, SimSerialLine, Transport
from sim.faults import FaultKind, FaultScript
from sim.skywatcher_sim import SkyWatcherSim, decode_revu24, encode_revu24
from sim.tmc_sim import TMC2209Sim

__all__ = [
    "Clock",
    "Device",
    "FakeSerial",
    "FaultKind",
    "FaultScript",
    "SimSerialLine",
    "SkyWatcherSim",
    "TMC2209Sim",
    "Transport",
    "decode_revu24",
    "encode_revu24",
]
