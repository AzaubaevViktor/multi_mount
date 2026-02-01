# Multi-mount Astro Architecture

## Overview
This repository implements a hybrid LX200-compatible mount controller (the "FrankenMount")
that combines a SkyWatcher RA axis with a DIY Arduino/TMC2209 DEC axis. The Python code
exposes an LX200-style command surface, routes commands to axis-specific backends, and
translates high-level pointing/tracking requests into device-specific serial protocols.

High-level data flow:
```
INDI / LX200 client
        |
        v
  lx200.server + plugins
        |
        v
  lx200_combine.splitter  ----->  skywatcher_lx200 (RA axis via SkyWatcher MC)
        |                       /
        |                      /
        +---------------------> tmc2209_lx200 (DEC axis via Arduino/TMC2209)
```

## Core layers
### 1) LX200 protocol layer (`src/lx200`)
- `protocol.py` defines the LX200 command set, parsing, and framing rules.
- `models.py` defines validated dataclasses for RA/DEC/time/date/UTC offset/site values.
- `server.py` implements a plugin-driven command dispatcher.
- `plugins/` provide command-to-backend adapters for:
  - pointing (RA/DEC, goto, sync, manual moves),
  - tracking (slew rate),
  - time/site/object metadata.

This layer is hardware-agnostic and maps LX200 commands to backend interfaces.

### 2) Backend adapters
#### SkyWatcher backend (`src/skywatcher_lx200`)
- `mount.py` owns SkyWatcher axis state, initialization, goto/sync, and motion control.
- `pointing.py` and `tracking.py` bridge LX200 plugins to `SkyWatcherMount`.
- `time.py`, `site.py`, `object.py` provide stateful LX200 responses not tied to hardware.
- `common.py` holds constants, dataclasses, and config validation.

#### TMC2209 backend (`src/tmc2209_lx200`)
- `mount.py` controls the DEC axis via `TMC2209ArduinoProxy` and maintains zero offsets.
  RA commands are explicitly ignored because this backend is DEC-only.
- `pointing.py` and `tracking.py` bridge LX200 plugins to `TMC2209Mount`.
- `common.py` defines axis configs, direction mapping, and validation.

#### Combined routing (`src/lx200_combine`)
- `splitter.py` routes LX200 commands to RA or DEC handlers based on command type.
- Supports a "primary" axis for time/site commands and combines site-name responses.

### 3) Device/protocol drivers (`src/lib` and `src/tmc2209`)
- `lib/serial_prims.py` implements a line-oriented serial transport with timeouts.
- `lib/skywatcher.py` implements the SkyWatcher low-level protocol and motion primitives.
- `tmc2209/proxy.py` implements the Arduino serial command protocol for TMC2209 control.
- `lib/logging_setup.py` standardizes logging output for the Python services.

## Executables and tools
- `src/dummy_server.py` runs a TCP LX200 server with either:
  - an in-memory dummy backend (for development), or
  - a SkyWatcher backend (for hardware integration).
- `src/pa/__main__.py` is a polar-alignment helper that parses sync lines and computes
  alignment geometry.

## Firmware and hardware
- `telescope_dec/` contains the Arduino firmware for the DEC axis. It exposes a simple
  line-based serial command set (`help`, `move`, `run`, `pos`, etc.) used by
  `tmc2209/proxy.py`.
- SkyWatcher hardware is driven through its motor controller protocol implemented in
  `lib/skywatcher.py`.
- `LX200CommandSet.txt` and `LX200CommandSet.pdf` are protocol references.
- `skywatcher.cpp/h` are reference sources from the INDI driver ecosystem.

## Testing
- `src/tests/` validates core routing and device protocol behavior:
  - `test_lx200_splitter.py` ensures correct command routing/combining.
  - `test_tmc2209_proxy.py` exercises the Arduino proxy (requires hardware/serial).
  - `test_tmc2209_lx200.py` and `test_skywatcher_serial.py` cover mount integrations.

## Key architectural decisions
- Plugin-based LX200 server keeps protocol handling independent from hardware backends.
- Axis-specific backends allow mixing heterogeneous hardware (SkyWatcher RA + TMC2209 DEC).
- The splitter enforces explicit routing rules and validates complete command coverage.
- Hardware state is encapsulated in mount classes with validated config dataclasses.
