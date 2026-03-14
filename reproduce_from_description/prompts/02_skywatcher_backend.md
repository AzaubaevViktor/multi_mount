# Prompt 02: Rebuild SkyWatcher RA Backend

Goal:
- reconstruct `src/skywatcher/protocol.py` and `src/skywatcher/motor.py`.

Allowed inputs:
- `context/ARCHITECTURE.md`
- `context/TEST_PLAN.md`
- `context/PROTOCOL_BASELINES.md`
- `prompts/00_global_rules.md`
- optional external references if available:
  - `references/skywatcher.h`
  - `references/skywatcher.cpp`
  - `references/skywatcher_motor_controller_command_set.pdf`

Do not read:
- the existing implementation under `src/skywatcher/`.

Target behavior:
- implement the RA motor backend over the SkyWatcher serial controller protocol;
- preserve low-speed and high-speed motion behavior;
- support status, position, tracking-like run mode, and delta-based goto mode;
- convert between typed `Ha` or `HaPerSecond` values and controller step-space;
- expose the common Motor abstraction expected by the Axis layer.

Deliverables:
- `src/skywatcher/protocol.py`
- `src/skywatcher/motor.py`
- tests covering protocol framing, speed rounding, status parsing, and at least one goto-related contract.

Important guarantees to preserve:
- protocol framing and response validation must be isolated from higher-level Axis logic;
- local cached state and queried controller state must have a clear contract;
- conversions between SkyWatcher 24-bit position format and typed units must be deterministic;
- if a command table or schema can reduce drift against the reference protocol, prefer that shape.
