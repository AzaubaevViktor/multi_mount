# Prompt 11: Realize Serial Transport and Motor Backends

Goal:
- upgrade the reconstructed transport, SkyWatcher backend, and TMC2209 backend toward the current live implementation.

Inputs:
- `context/PROTOCOL_BASELINES.md`
- `context/CURRENT_IMPLEMENTATION_DUMP.md`
- `context/CURRENT_TEST_EXPECTATIONS.md`
- `context/CURRENT_FIRMWARE_AND_RUNTIME_NOTES.md`
- `prompts/00_global_rules.md`

Primary targets:
- `src/serial_wrapper/wrapper.py`
- `src/skywatcher/protocol.py`
- `src/skywatcher/motor.py`
- `src/tmc2209/motor.py`
- related unit tests

Tasks:
- align transport behavior with prefix-aware response reads and buffer-drain helpers;
- align SkyWatcher status/motion encoding with the live command model;
- align TMC2209 host behavior with ready handshake, strict status parsing, and stop-before-reconfigure rules;
- keep the shared Motor contract coherent for the Axis layer.

Deliverables:
- updated transport and backend code;
- tests that lock the improved host-side assumptions;
- a list of backend details that still need real hardware validation.
