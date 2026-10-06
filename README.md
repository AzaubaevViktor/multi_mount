# Multi-mount astro
Connect a hybrid LX200-like mount where RA is driven by a SkyWatcher SynScan mount and DEC by a DIY Arduino + TMC2209 controller.

## About this project

This repository exposes one LX200-compatible endpoint while coordinating two different physical axes:

- RA is driven by `AxisRA` on top of `SkyWatcherMotor`.
- DEC is driven by `AxisDEC` on top of `TMC2209Motor`.
- `Combiner` joins both axes and feeds `SkyLX200`, so INDI clients can treat the setup as one mount.

The current Python runtime is built from:

- `LX200SimpleServer` for TCP/LX200 framing;
- `LX200Handler` / `SkyLX200` for LX200 command dispatch;
- `Combiner` for two-axis coordination and polar-compensation feedback;
- `AxisRA` and `AxisDEC` for mount-position tracking, motion queuing, and motor compensation;
- `SerialLine` for the low-level serial transport to both controllers.

The DEC firmware lives in `telescope_dec/src/main.cpp`. The Python backend detects
the framed v3 protocol (length, sequence and CRC16), with fallback to the older
line protocol. Both expose the same motion commands. The extended status reports
driver faults, safety state, active microstep resolution and safety events.

The Python app also provides a gravity/magnetic sensor calibration screen and
agent HTTP API at `http://127.0.0.1:8080/`. Run `python -m src --sim` to use the
independent sensor simulator. Calibration points and verification live in Python.
The DEC firmware reads MPU6050 + QMC5883L and remains operational without sensors.
See [sensor workflow and API](docs/POINTING_SENSOR.md) and
[sensor wiring and power levels](docs/DEC_SENSOR_WIRING.md) (A4/A5 I²C;
the mode LED's blue wire moves to A0).

## Scheme
```text
KStars / Ekos / INDI
        |
        v
   LX200 client
        |
        v
 LX200SimpleServer
        |
        v
     SkyLX200
        |
        v
     Combiner
      |     |
      |     `--> AxisDEC --> TMC2209Motor --> Arduino firmware --> TMC2209 --> DEC axis
      |
      `--------> AxisRA  --> SkyWatcherMotor --> SynScan mount
```

## Coordinate system
| Case                           | RA Rate | RA Ticks | RA mount | Dec Rate | Dec Ticks | Dec mount |
|:------------------------------:|:-------:|:--------:|:--------:|:--------:|:---------:|:---------:|
| Mount didn't track, keep still | 0       | const    | ↑        | 0        | const     | const     |
| Mount base track               | 1       | ↑ == T   | const    | 0        | const     | const     |
| **SLEW / GOTO**                |         |          |          |          |           |           |
| East slew                      | -800    | ↓↓       | ↑↑       | 0        | const     | const     |
| West slew                      | 800     | ↑↑       | ↓↓       | 0        | const     | const     |
| North slew                     | 1       | ↑        | const    | > 0      | ↑↑        | ↑↑        |
| South slew                     | 1       | ↑        | const    | < 0      | ↓↓        | ↓↓        |
| **GUIDE**                      |         |          |          |          |           |           |
| East guide                     | 0..1    | ↑ < T    | const    | 0        | const     | const     |
| West guide                     | > 1     | ↑ > T    | const    | 0        | const     | const     |
| North guide                    | 1       | ↑        | const    | > ~0     | ↑         | 0         |
| South guide                    | 1       | ↑        | const    | < ~0     | ↓         | 0         |

RA: `00:00:00` .. `23:59:59`
DEC: `-90*00:00` .. `90*00:00`

Expected runtime behavior:

- the mount starts in tracking mode;
- `SYNC` updates the logical mount coordinates;
- `GOTO` moves both axes toward the target;
- `HALT` returns the affected axis back to tracking.

## Repository layout

- `src/lx200`: LX200 protocol parsing and TCP server.
- `src/sky`: axis state machine, combiner, polar compensation, coordinate math
  and the typed quantities everything above the wire speaks in (`sky/physics.py`:
  `Ha`/`Dec`, `HaPerSecond`/`DecPerSecond`, `Second`, `StepsPerSecond`).
- `src/skywatcher/`: RA backend in four layers — `codec` (pure functions, no I/O),
  `board` (capabilities read from the board at connect), `session` (state and the
  board's quirks), `motor` (what `Axis` sees).
- `src/tmc2209/`: DEC backend — `protocol` (v3 frame: length, opcode, sequence,
  CRC16) and `motor` (dialect autodetection, echo verification).
- `src/sim/`: simulators of both boards plus transport chaos, on a virtual clock.
- `src/pointing/`: raw sensor capability, persistent calibration, agent HTTP API and screen.
- `src/ra_conformance/`: protocol cases taken verbatim from the RA protocol
  document, run by the same code against the simulator and against the live board.
- `src/serial_wrapper/`: shared serial transport and the byte-level session recorder.
- `src/clock.py`: the injectable time source both drivers and the transport take,
  plus the TTL cache they read their slow-changing values through.
- `telescope_dec/src/main.cpp`: AVR firmware for the DEC controller.
- `src/tests/units`: fast tests, no hardware.
- `src/tests/hw`: hardware and end-to-end tests.

## Start here

`FRAME.md` — the goal of the project and the invariants that must not be broken.
Each of them was paid for with either damaged hardware or lost time.
`ARCHITECTURE.md` describes the runtime, `TESTS_PLAN.md` the checks, and
`docs/protocol/DEC_PROTOCOL.md` the DEC protocol and recorded hardware findings.
`PLAN.md` and `OBSERVATORY.md` are local planning documents excluded from Git.

For a hardware-free session with a real LX200 client, use Python 3.14 or newer
with the dependencies declared in `pyproject.toml`, then run:

```sh
python -m src --sim
```

The simulated mount listens on `localhost:7624`. `python -m src` selects the
physical controllers using the serial-device patterns in `src/__main__.py`.

## Current TODO / known gaps

- Pole crossing is still incomplete: when DEC reflection crosses the pole, RA should be mirrored by `+12h` as well.
- RA and DEC backend status contracts are still similar but not fully unified.
- Step counts (`steps`, `delta_steps`, `cpr`, a GOTO target) are still bare `int`.
  The *rate* is typed (`StepsPerSecond`), which already separates it from every
  count; telling a position from a delta from a revolution is a second, smaller
  question and no defect has been traced to it yet.
- **The RA board reboots when a GOTO is allowed to reach its target** — reproduced
  on live hardware. The driver therefore never lets the board arrive on its own.
- DEC UART was repaired; register writes are verified, driver diagnostics and
  current control are available. Target moves use full steps for the distant
  part and return to fine microsteps for the approach.
- **DEC stall detection through UART polling is not reliable on the bench.**
  The latch and fault reporting are implemented, but free and stalled motion
  produced overlapping StallGuard samples. Reliable detection still needs the
  planned DIAG-to-D2 connection and calibration, or an encoder; see
  `telescope_dec/FIRMWARE_PLAN.md`. Keep the bench motor disabled until that
  hardware step is completed.

## Tests

Default `pytest` runs `src/tests/units` only. `src/tests/hw` needs a physically
attached mount and fails at *collection* without one, so it is deliberately kept
out of the default run and takes one explicit argument: `pytest src/tests/hw`.
A more detailed overview is kept in `TESTS_PLAN.md`.
