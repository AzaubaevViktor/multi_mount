# Multi-mount astro
Connect DIY lx200-like servo mount based on arduino and SynScan RA-only mount (SkyWatcher Star Adventurer 2i)

## Scheme
INDI --LX200--> FrankenMount 
FM --RA--> SkyWatcherAdapter --SynScan?--> SkyWatcher 2i
by https://github.com/indilib/indi/blob/master/drivers/telescope/skywatcherAPIMount.cpp
FM --DEC--> LX200Adapter --LX200--> Arduino --> TMC2209 --> Motor --> Mount dec
FM Controls Current position

## TODO
- ✅ Remove AI-generated slop
- INDI
    - Connect with LX200
- SkyWatcher
    - ✅ Connect with mount
    - ✅ Move
    - ✅ Read position
    - ✅ Slew to position
    - ✅ Sideral tracking
    - Correction sideral tracking
- Arduino-based dec mount
    - ✅ Connect Arduino to motor
    - ✅ Control motor
    - Control with LX200
- Combine
    - Tests for LX200 protocol
    - Send RA to SkyWatcher
    - Send DEC to Arduino
    