# Multi-mount astro
Connect DIY lx200-like servo mount based on arduino and SynScan RA-only mount (SkyWatcher Star Adventurer 2i)

## About this project

This repository implements a hybrid telescope control stack that exposes one LX200-compatible endpoint while driving two different physical axes. Right Ascension (RA) is delegated to a SynScan-compatible SkyWatcher mount, and Declination (DEC) is delegated to a DIY Arduino + TMC2209 motor controller. The `LX200Splitter` composes both axis handlers behind one command surface so INDI clients can treat the setup as a single mount.

The Python runtime is organized around a socket server (`LX200SimpleServer`) and protocol/domain layers for LX200 command parsing, coordinate containers (`LX200Ha`, `LX200Dec`), and axis-specific control loops. RA behavior is implemented in `SkyWatcherMount` and `SkyWatcherLX200` with tracking, slewing, guiding, and GOTO supervision. DEC behavior is implemented in `TMC2209Adapter` and `TMC2209LX200` with a serial command protocol, motion profiles, and synchronized telescope/motor position updates.

The project includes broad automated coverage for parsing, coordinate math, motion-status conversions, rate control, and serial error handling, plus hardware-oriented test suites for SkyWatcher, TMC2209, and end-to-end splitter behavior (`SYNC`, `SLEW`, `GOTO`, `HALT`, guiding in all directions). Firmware for the DEC controller lives in `telescope_dec/src/main.cpp`, where a lightweight line protocol (`status/get/set/position/delta/run/stop/mode`) powers the Python adapter and keeps command latency low on embedded hardware.

## Scheme
```
kstars
v
ekos
v
INDI --LX200--> MultiMount
                 `--RA--> SkyWatcherAdapter --SynScan--> SkyWatcher 2i
                 `--DEC--> TMC2209Adapter --LX200--> Arduino --> TMC2209 --> Motor --> Mount dec throught gears
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
| North slew                     | 1       | ↑        | const    | > ~0     | ↑         | 0         |
| South slew                     | 1       | ↑        | const    | < ~0     | ↓         | 0         |

RA: `00:00:00` .. `23:59:59`
DEC: `-90*00:00` .. `90*00:00`

Mount shoud be in tracking mode by default
SYNC command should update mount coordinate
GOTO command should move mount to target coordinates
HALT command must resume mount to tracking mode from SLEW / GOTO / GUIDE (?)

## Already here
- ✅ Remove AI-generated slop
- INDI
    - ✅ Connect with LX200
- Implement commands
    - Base
        - ✅ Mocks: Site, Long/Lat, Time/Date, etc
        - ✅ Moving
        - ✅ Stop
        - ✅ Tracking
        - ✅ Guiding
- SkyWatcher
    - ✅ Connect with mount
    - ✅ Move
    - ✅ Read position
    - ✅ Slew to position
    - ✅  Sideral tracking
    - ✅ Guiding
    - ✅ Tracking model
- Arduino-based dec mount
    - ✅ Connect Arduino to motor
    - ✅ Control motor
    - ✅ Control with LX200
- Combine
    - ✅ Tests for LX200 protocol
    - ✅ Send RA to SkyWatcher
    - ✅ Send DEC to Arduino
- Check with INDI
    - ✅ Sync
    - ✅ Slew
    - ✅ GOTO
    - ✅ Guide


## TODO
Upper - more priority

- Place on mount
    - ✅ Board scheme
    - ✅ Optimise traces
    - Board
        - ✅ Solder
        - ✅* Status lights
        - ✅ Reset button
        - ✅ Voltage measure
    - 💪 3d printed case for board
- Motor-mount connection
    - Reprint gears with better axis and free rotation
    - Reprint middle-plate with more fixation and hole for polarscope
- Fixes
    - Find correct ratio for DEC
- Edge cases
    - DEC more 90 -> RA + 12
- Long tests
    - Sync -> GOTO -> (halt?) -> Check
    - Sync -> Slew -> (halt?) -> Check
    - More tests for cases combination
- Security
    - Watchdog
    - Stop after signals
- Fixes
    - RA motor stop sometimes
    - Slow reaction to manual slew
- Status interface
    - LX200: raw + commands + answers
        - coordinates get/sync/slew
        - aux
        - guiding
    - Splitter?
    - RA/DEC:
        - mount position, speed, tracking rate, moving rate
        - motor position, speed, tracking rate, moving rate
            - RA: some parameters
            - DEC: speed, actual speed, accel, ...
        - voltages
        - correction thread: delta, expected, real motor
        - guiding thread: last X timings, movavg
    - Guiding
        - real polar delta
        - guiding delta
- Rewrite with more clean architecture
    - Sec/Arcsec + per secons + per second square (types)
    - Motor controller 
        - send queryies
        - check current status
        - return errors
        - control speed, accel, microsteps limits
        - run and stop motor
    - Mount controller
        - calculate current mount position
        - store guiding rates
        - converts steps to sec/arcsec
        - control guiding
        - stop motor when its need to be stopped (motor say so)
    - Sky controller
        - get/set sky position
        - change base tracking (sky/lunar/solar/...)
        - guide
            - (*) calculate real polar position and autoguide based on external guiding
        - moving
        - set moving rate
        - goto
        - check goto finished
        - halt, halt_all
        - thread-cycle with:
            - motor to mount position based on status and tracking rate
            - control moving
            - control goto
            - control guide
            - control halt
    - lx200 controller
        - handle lx200 command
        - return correct answers
