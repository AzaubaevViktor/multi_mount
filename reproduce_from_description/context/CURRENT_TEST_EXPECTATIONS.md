# Current Test Expectations Dump

This file summarizes the current live tests that add fidelity beyond the earlier source-free snapshot.

## Unit expectations visible in the live tree

### `src/tests/units/test_combiner_guide_speed.py`

Locks down:

- fixed guide interval behavior;
- linear interpolation from default speed to forward/backward guide speeds;
- sign and routing semantics for north/south and east/west guide pulses.

### `src/tests/units/test_motor_speed_rounding.py`

Locks down:

- SkyWatcher quantizes requested speed through period conversion;
- SkyWatcher switches into high-speed mode when crossing the low-speed boundary;
- SkyWatcher rejects non-positive speed values;
- TMC2209 rounds host-requested speed to an integer;
- TMC2209 rejects negative speed values.

### `src/tests/units/test_polar_compensator.py`

Locks down:

- mathematical round-trip between `compute_pole_offset` and `compute_guide_speeds`;
- degeneracy near `DEC=0`;
- stable pulse counting;
- counter reset after large guide-speed jumps;
- timeout-based reset;
- guide takeover only after stability conditions are met.

### `src/tests/units/test_skywatcher_protocol.py`

Locks down:

- transport waits for a valid response prefix before reading the answer body;
- SkyWatcher backend rejects wrong serial terminators;
- SkyWatcher `_transact` strips the trailing protocol terminator;
- protocol error prefix becomes a command error.

## Hardware expectations visible in the live tree

The current `src/tests/hw/test_7_combiner_hw_v2.py` shows the most complete end-to-end intent:

- sync to a known sky position;
- read both logical mount position and raw motor position;
- validate tracking rate and drift over time;
- exercise slew presets and `MS`;
- exercise manual motion and halts;
- replay guide commands through LX200;
- observe polar-compensation state transitions.

Important implication:

- a faithful regeneration needs more than a green fast suite;
- it also needs enough state exposure and determinism that a hardware harness can inspect polar-compensation state, axis sky-rates, logical coordinates, and raw motor positions.
