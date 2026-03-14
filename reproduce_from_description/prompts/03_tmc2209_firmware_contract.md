# Prompt 03: Rebuild TMC2209 Host and Firmware Contract

Goal:
- reconstruct `src/tmc2209/motor.py`;
- reconstruct a compatible `telescope_dec/src/main.cpp` firmware surface.

Allowed inputs:
- `context/ARCHITECTURE.md`
- `context/TEST_PLAN.md`
- `context/PROTOCOL_BASELINES.md`
- `prompts/00_global_rules.md`

Do not read:
- the existing implementation under `src/tmc2209/`;
- the existing implementation under `telescope_dec/src/`.

Target behavior:
- define one line-based ASCII protocol shared by the firmware and the Python host;
- support status, position, speed, acceleration, direction, delta, run, stop, mode, and parameter-setting commands;
- expose motion phases and target-state information so the host can implement the Motor abstraction cleanly;
- convert between step-space and typed DEC units on the Python side;
- keep the firmware non-blocking enough for motion servicing.

Deliverables:
- `src/tmc2209/motor.py`
- `telescope_dec/src/main.cpp`
- protocol or compatibility tests that exercise host assumptions about firmware responses.

Important guarantees to preserve:
- the firmware is the authority on actual step generation and motion phase;
- the host backend should not guess hidden firmware state without an explicit contract;
- success and error responses must be machine-parseable and stable;
- host-side retries, readiness handshake, and stop semantics must be compatible with the firmware contract.
