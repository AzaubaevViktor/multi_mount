# Current Implementation Dump

This file captures the current implementation shape from the live project tree under `src/` and `telescope_dec/`.

It is not a contract by itself. Its role is to help phase-2 regeneration converge toward the real code after the baseline reconstruction is already understandable.

## Runtime wiring

Current runtime entrypoint in the live tree:

- `src/__main__.py` prepends both project root and `src/` to `sys.path`.
- It discovers the RA serial device with `SerialLine.search("PL2303G")`.
- It discovers the DEC serial device with `SerialLine.search("tty.usbserial")`.
- It constructs:
  - `SkyWatcherMotor` over a `SerialLine` configured for `\r`;
  - `AxisRA`;
  - `TMC2209Motor` over a `SerialLine` configured for `\n`;
  - `AxisDEC`;
  - `Combiner`;
  - `SkyLX200`;
  - `LX200SimpleServer`.
- It also starts a monitor HTTP server separately with an empty registry-like payload.

## Axis layer

Current `src/sky/axis.py` is more stateful than the initial reconstruction:

- axis names are explicit: `AxisName.RA` and `AxisName.DEC`;
- public modes are `STOP`, `TRACK`, `SLEW`, `GOTO`;
- command queue items are typed:
  - `SET_POSITION`;
  - `CHANGE_SPEED`;
  - `MOVE`;
  - `GOTO_TO`;
  - `HALT_DIRECTION`;
  - `HALT_ALL`.

Behavioral details visible in the current source:

- Axis owns logical mount position and updates it separately from raw motor position.
- There is a dedicated motion-convertor thread per axis.
- The thread can mark itself failed; public methods guard against using a crashed axis.
- After temporary motions, the axis explicitly resumes tracking if `_sky_speed` is non-zero.
- GOTO compensates for expected sky drift during travel time before programming the motor target.
- Compensation uses `_last_motor_position` and `_last_motor_position_update_s`.

## Combiner and guiding

Current `src/sky/combiner.py` differs from the baseline reconstruction in several important ways:

- guide speed is modeled as interpolation over a fixed `GUIDE_INTERVAL_S = 4s`;
- RA guide profile is centered around sidereal speed;
- DEC guide profile is centered around zero speed;
- guide pulses update axis sky-speed state instead of starting a short timed move;
- a polar-compensator thread waits on guide updates and later replays compensation when external guiding stops;
- position composition is additive:
  - `ra = pos_from_ra.ra + pos_from_dec.ra`;
  - `dec = pos_from_dec.dec + pos_from_ra.dec`;
- there is an explicit TODO for DEC pole-crossing reflection that should mirror RA by `+12h`.

## Polar compensator

Current `src/sky/polar_compensator.py` implements a real polar-alignment model, not just averaging:

- standalone functions:
  - `compute_pole_offset`;
  - `compute_guide_speeds`;
- model inputs:
  - measured RA guide speed;
  - measured DEC guide speed;
  - current HA;
  - current DEC;
- stable-pulse tracking is axis-specific;
- pulse stability is based on tolerance windows, not just raw sample count;
- there are separate notions of:
  - stable external guiding;
  - timeout-based reset;
  - one-axis stop when only the other axis keeps receiving guide pulses;
  - autonomous takeover when external guiding stops.

## LX200 glue

Current `src/sky/lx200.py` uses explicit preset constants:

- guide preset;
- center preset;
- find preset;
- max preset.

Manual motion semantics:

- `move_*` maps to `Combiner.move(...)`;
- `halt_*` maps to axis-specific halts;
- `guide_*` maps to `Combiner.guide(...)`.

Current `src/lx200/base.py` keeps several intentionally simplified behaviors:

- `CM` returns `"OK"`;
- `MS` returns `False`;
- many site/time commands are compatibility shims;
- manual motion directions are tracked explicitly.

## SkyWatcher backend

Current `src/skywatcher/motor.py` includes several details not present in the baseline reconstruction:

- real motion-mode encoding through `_MotionStatus`;
- 24-bit mount encoding through `_Revu24`;
- cached mount position with freshness tracking;
- high-speed and low-speed modes selected by actual speed thresholds;
- GOTO speed chosen from delta size and capped by low/high-speed rules;
- protocol validation that depends on the serial terminator being `\r`;
- `_transact(...)` asks the transport to wait for either success or error prefixes.

## TMC2209 backend

Current `src/tmc2209/motor.py` includes a stronger host contract:

- explicit `ready` handshake with retries on connect;
- `_Response` parser with strict `key=value` enforcement;
- `_Status` parser with explicit phase/mode enums;
- safety checks for operations that are illegal during active GOTO;
- `MotorStopRequire` for direction and microstep changes while moving;
- gear-ratio based conversion from DEC units to step-space;
- retries with buffer draining when a command fails.

## Firmware surface

The current `telescope_dec/src/main.cpp` is not a stub. It contains:

- a concrete pin map for Nano v3 + TMC2209;
- `AltSoftSerial` UART wiring;
- `TMCStepper` setup;
- a `RunnerV2` motion state;
- step generation and pulse scheduling;
- power sensing and LED status logic;
- a ring-buffered serial response path;
- a line parser and command interpreter.

See `context/CURRENT_FIRMWARE_AND_RUNTIME_NOTES.md` for the hardware-facing details that matter most for regeneration.
