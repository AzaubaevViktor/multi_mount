# Prompt 04: Rebuild Axis, Combiner, Physics, and Polar Compensation

Goal:
- reconstruct:
  - `src/sky/physics.py`
  - `src/sky/motor.py`
  - `src/sky/axis.py`
  - `src/sky/combiner.py`
  - `src/sky/polar_compensator.py`
  - `src/sky/constants.py`

Allowed inputs:
- `context/ARCHITECTURE.md`
- `context/TEST_PLAN.md`
- `context/PROTOCOL_BASELINES.md`
- `prompts/00_global_rules.md`

Do not read:
- the existing implementation under `src/sky/`.

Target behavior:
- define typed coordinate and speed primitives for HA and DEC;
- define one Motor abstraction consumed by the Axis layer;
- implement per-axis state management with command queueing and motion compensation;
- combine RA and DEC into one logical mount surface;
- support guide-speed routing and polar-compensation takeover;
- preserve the distinction between RA tracking semantics and DEC zero-base-rate semantics.

Deliverables:
- the files listed above;
- fast tests for typed arithmetic, guide speed calculation, and polar-compensator math;
- a documented set of invariants for Axis modes, GOTO behavior, and compensation behavior.

Important guarantees to preserve:
- Axis owns logical mount coordinates, not raw motor positions;
- the Motor boundary is explicit and backend-agnostic;
- compensation logic must distinguish expected tracking motion from actual encoder deltas;
- guide and goto flows must compose correctly with tracking and halts.
