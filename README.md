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

The DEC firmware lives in `telescope_dec/src/main.cpp`. It exposes a small line-based protocol (`status`, `position`, `speed`, `acceleration`, `direction`, `delta`, `run`, `stop`, `mode`, `set`) that the Python `TMC2209Motor` backend uses directly.

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
- `src/sky`: axis state machine, combiner, polar compensation, coordinate math.
- `src/skywatcher/motor.py`: SkyWatcher/SynScan RA backend.
- `src/tmc2209/motor.py`: Python backend for the Arduino DEC controller.
- `src/web_control`: standalone web monitor infrastructure.
- `telescope_dec/src/main.cpp`: AVR firmware for the DEC controller.
- `src/tests/hw`: active hardware and end-to-end tests.
- `src/tests/units`: active fast tests.
- `src/tests/old`: archived tests that are kept for reference and are not collected by default.

## Current TODO / known gaps

- Pole crossing is still incomplete: when DEC reflection crosses the pole, RA should be mirrored by `+12h` as well.
- `LX200SimpleServer` still allows multiple concurrent clients against the same handler and has no connection-level disconnect hook.
- `MS` still returns a simplified boolean status instead of a full LX200 slew result code.
- Step-based typed units such as `steps/s` are not modeled explicitly in `sky.physics` yet.
- RA and DEC backend status contracts are still similar but not fully unified.
- The web monitor server is started separately and currently uses an empty registry in `src/__main__.py`.

## Tests

Default `pytest` discovery uses `src/tests/hw` and `src/tests/units`. Hardware suites expect real devices and serial ports; fast checks live under `src/tests/units`. A more detailed overview is kept in `TESTS_PLAN.md`.
