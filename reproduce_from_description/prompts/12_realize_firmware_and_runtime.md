# Prompt 12: Realize Firmware Surface and Runtime Wiring

Goal:
- upgrade the reconstructed firmware stub and runtime wiring toward the live implementation notes.

Inputs:
- `context/CURRENT_IMPLEMENTATION_DUMP.md`
- `context/CURRENT_FIRMWARE_AND_RUNTIME_NOTES.md`
- `context/CURRENT_TEST_EXPECTATIONS.md`
- `context/TARGET_TREE.md`
- `prompts/00_global_rules.md`

Primary targets:
- `telescope_dec/src/main.cpp`
- `src/__main__.py`
- `src/web_control/web.py`
- `src/tests/hw/`

Tasks:
- replace firmware-level simulation with a more realistic motion-service and command model;
- align runtime discovery defaults with the live tree while keeping overrides explicit;
- expose enough runtime state that hardware harnesses can observe mount logic and polar-compensation state;
- keep monitor infrastructure optional and separate from the LX200 core.

Deliverables:
- upgraded runtime and firmware code;
- updated hardware harness descriptions or implementations;
- a note describing which firmware details are still board-specific.
