# Protocol Baselines

This file summarizes the contracts that the rebuilt code should preserve.

## LX200 Surface

Transport:
- command frame: `:<cmd>#`
- alignment query: single byte `0x06`
- string responses are typically terminated by `#`
- boolean responses are encoded as `0` or `1`

Important commands in active use:
- coordinates:
  - `GR`, `GD`, `Sr`, `Sd`
- motion:
  - `CM`, `MS`, `D`
  - `Me`, `Mw`, `Mn`, `Ms`
  - `Q`, `Qe`, `Qw`, `Qn`, `Qs`
- slew rate presets:
  - `RG`, `RC`, `RM`, `RS`
- guide pulses:
  - `Mg<dir><ms>`
- site and clock compatibility layer:
  - `Gc`, `GM`, `GT`, `Gt`, `Gg`, `GG`, `GL`, `GC`
  - `Sg`, `St`, `SG`, `SL`, `SC`
  - `Sh`, `So`

Behavioral notes:
- `Sr` and `Sd` store target coordinates.
- `CM` syncs the mount to the stored target coordinates.
- `MS` initiates slew/goto to the stored target coordinates.
- manual motion and guide commands are routed through the two-axis model.

## SkyWatcher Controller Contract

Wire format:
- command prefix: `:`
- command terminator: `\r`
- response success prefix: `=`
- response error prefix: `!`
- response terminator: `\r`

Axis encoding:
- RA uses axis id `"1"`

Commands actively used by the Python backend:
- initialize: `F`
- inquire grid per revolution: `a`
- inquire timer frequency: `b`
- inquire status: `f`
- inquire highspeed ratio: `g`
- inquire position: `j`
- set step period: `I`
- set goto target increment: `H`
- set break point increment: `M`
- set axis position: `E`
- set motion mode: `G`
- start motion: `J`
- stop motion: `K`

Behavioral notes:
- driver keeps low-speed and high-speed modes.
- goto is delta-based.
- status is partly queried from controller and partly tracked locally by the backend.
- position is represented in a 24-bit mount-specific format.

## TMC2209 Host <-> Firmware Contract

Wire format:
- line-based ASCII
- command terminator: `\n`
- response starts with `1;` on success or `0;` on error
- response fields are `key=value;`

Core commands:
- `status`
- `position <steps>`
- `speed <steps_per_second>`
- `acceleration <steps_per_second_square>`
- `direction <0|1>`
- `delta <steps>`
- `run`
- `stop`
- `mode <target|free_ride>`
- `set key=value`
- optional diagnostics:
  - `get <name>`
  - `full_status`
  - `driver_status`
  - `profile`

Status keys expected by the host:
- `initialised`
- `enabled`
- `mode`
- `position`
- `phase`
- `target`
- `target_set`
- `speed`
- `actual_speed`
- `accel_per_s`

Motion semantics:
- `target` mode performs delta-based motion to a target.
- `free_ride` mode acts like continuous run mode.
- the firmware owns actual step generation and motion phase transitions.
- the Python backend converts between step-space and typed DEC coordinates/speeds.

## Shared Motor Contract

The two motor backends should present one host-side abstraction:

- `connect()`, `disconnect()`
- `status()`
- `set_steps()`
- `set_speed()`
- `set_acceleration()`
- `set_direction()`
- `set_delta()`
- `get_speed_sps_by_delta()`
- `get_speed_by_speed_sps()`
- `set_motion_mode()`
- `set_microsteps()`
- `convert_position_to_steps()`
- `convert_steps_to_position()`
- `convert_speed_to_steps_per_second()`
- `run()`, `stop()`, `wait_till_stop()`, `reset()`

This abstraction is consumed by `Axis`, so compatibility at this boundary is critical for reconstruction.
