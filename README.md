# Multi-mount astro
Connect DIY lx200-like servo mount based on arduino and SynScan RA-only mount (SkyWatcher Star Adventurer 2i)

## Scheme
INDI --LX200--> FrankenMount 

FM --RA--> SkyWatcherAdapter --SynScan--> SkyWatcher 2i
by https://github.com/indilib/indi/blob/master/drivers/telescope/skywatcherAPIMount.cpp

FM --DEC--> LX200Adapter --LX200--> Arduino --> TMC2209 --> Motor --> Mount dec throught gears

FM Controls Current position and movements

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
        - 💪 Guiding
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
    - 💪 Guide


## TODO
- Place on mount
    - Board scheme
    - Board
        - Status lights
        - Reset button
        - Voltage measure
    - 
- Fixes
    - Find correct ratio for DEC
- Long tests
    - Sync -> GOTO -> (halt?) -> Check
    - Sync -> Slew -> (halt?) -> Check
- Edge cases
    - DEC more 90 -> RA + 12
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
