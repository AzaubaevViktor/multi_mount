# SkyWatcher LX200 integration architecture

## Overview
The `skywatcher_lx200` package adapts SkyWatcher motor controllers to the LX200 plugin protocols.
The core responsibility is split into a shared mount controller and per-protocol backends.

## Modules
- `common.py`: shared constants, exceptions, config dataclasses, axis mapping/state.
- `mount.py`: `SkyWatcherMount` controlling axes, conversions, goto/sync, and initialization.
- `pointing.py`: LX200 pointing backend (RA/DEC, goto, sync, manual moves).
- `tracking.py`: LX200 tracking backend (slew rate selection).
- `site.py`: site backend (lat/lon and name storage).
- `time.py`: time backend with local clock initialization.
- `object.py`: object metadata backend (distance/size).

## Data flow
1) LX200 server routes commands to plugin backends.
2) Backends delegate to `SkyWatcherMount` for motion-related actions.
3) `SkyWatcherMount` translates RA/DEC to SkyWatcher ticks, configures motion modes, and calls `SkyWatcherMC`.
4) Initialization is performed by `SkyWatcherMount.initialize`, which verifies and resets axis baselines.

## Initialization model
- `SkyWatcherMount` optionally auto-initializes on construction.
- Initialization uses `SkyWatcherMC.do_initialize` for RA/DEC axes with configurable timeout and polling.
- After initialization, axis states are refreshed to establish zero offsets.
