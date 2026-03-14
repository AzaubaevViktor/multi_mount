# Current Firmware And Runtime Notes

This file extracts the hardware-facing details from the live `telescope_dec/` and runtime wiring.

## Current DEC firmware hardware map

Live `telescope_dec/src/main.cpp` is built around:

- board shape: Nano v3 style AVR target;
- driver library: `TMCStepper`;
- UART transport: `AltSoftSerial`;
- TMC pins:
  - `STEP_PIN = 7`
  - `DIR_PIN = 4`
  - `EN_PIN = 12`
  - `TMC_RX_PIN = 8`
  - `TMC_TX_PIN = 9`
- LEDs and sensing:
  - power LED on `11`
  - power sense on `A1`
  - RGB step LEDs on `6`, `5`, `3`
  - mode LEDs on `A3`, `A2`, `A5`

## Firmware runtime model

The live firmware is organized around `RunnerV2`:

- enable/disable state;
- direction;
- running/stop-request flags;
- target-mode vs free-ride mode;
- desired speed and actual speed;
- acceleration in steps per microsecond;
- step scheduling timestamps;
- pulse width and next-step timing.

The firmware is also responsible for:

- maintaining an absolute step-position counter;
- exposing motion phases such as idle, hold, acceleration, running, deceleration;
- streaming structured ASCII responses;
- power-voltage sampling and blink behavior;
- LED color updates tied to step position and mode.

## Host-side expectations derived from the live firmware

The live Python backend expects:

- a `ready` line after reset;
- strict machine-parseable `1;key=value;...;` or `0;error=...;` replies;
- `status` to include:
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
- motion commands to be rejected or deferred when the controller is in target mode and still moving.

## Runtime details still likely to vary per installation

The live project hardcodes some discovery and hardware assumptions:

- RA serial discovery currently targets `"PL2303G"`;
- DEC serial discovery currently targets `"tty.usbserial"`;
- baud rate differs between the two physical links;
- the exact TMC2209 board, current limit, and sense resistor are embedded in firmware values.

If a future regeneration is meant for a different physical rig, use `prompts/13_user_info_needed_for_hardware_variants.md` before freezing those values.
