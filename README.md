# Multi-mount astro
Connect DIY lx200-like servo mount based on arduino and SynScan RA-only mount (SkyWatcher Star Adventurer 2i)

## Scheme
INDI --LX200--> FrankenMount 
FM --RA--> SkyWatcherAdapter --SynScan?--> SkyWatcher 2i
by https://github.com/indilib/indi/blob/master/drivers/telescope/skywatcherAPIMount.cpp
FM --DEC--> LX200Adapter --LX200--> Arduino --> TMC2209 --> Motor --> Mount dec
FM Controls Current position

## TODO
- Remove AI-generated slop
- INDI
    - Connect with LX200
- SkyWatcher
    - Connect with mount
    - Move
    - Read position
    - Slew to position
    - Correct sideral tracking
        - Hardware tests: use `pytest -q` with `SKYWATCHER_PORT` env var set
            - Example: `SKYWATCHER_PORT=/dev/tty.PL2303G-USBtoUART1 pytest tests/test_skywatcher_serial.py -q`
- Arduino-based dec mount
    - Connect Arduino to motor
    - Control motor
    - Control with LX200
- Combine
    - Send RA to SkyWatcher
    - Send DEC to arduino
    