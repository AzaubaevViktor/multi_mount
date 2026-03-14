# Prompt 05: Rebuild LX200 Boundary

Goal:
- reconstruct:
  - `src/lx200/protocol.py`
  - `src/lx200/base.py`
  - `src/lx200/base_server.py`
  - `src/sky/lx200.py`

Allowed inputs:
- `context/ARCHITECTURE.md`
- `context/TEST_PLAN.md`
- `context/PROTOCOL_BASELINES.md`
- `prompts/00_global_rules.md`
- optional external reference if available:
  - `references/LX200CommandSet.txt`

Do not read:
- the existing implementation under `src/lx200/`;
- the existing implementation of `src/sky/lx200.py`.

Target behavior:
- expose one LX200-compatible TCP surface;
- parse LX200 commands and route them to domain methods;
- keep track of stored target coordinates for `Sr`, `Sd`, `CM`, and `MS`;
- route manual movement, halts, and guide pulses into the two-axis Combiner;
- handle alignment query and command stream framing at the socket layer.

Deliverables:
- the files listed above;
- tests for command parsing, limit commands, at least one move or guide path, and basic server framing.

Important guarantees to preserve:
- LX200 framing is isolated from mount mechanics;
- client-visible API stays LX200-like even if internal implementation changes;
- manual motion bookkeeping and target-coordinate flow are explicit;
- server-side response serialization follows the protocol baseline.
