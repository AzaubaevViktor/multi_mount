# Prompt 08: Synthesize the Rebuilt Codebase

Goal:
- merge the outputs of prompts `01` through `07` into one coherent repository.

Inputs:
- all files under `context/`
- all outputs from `prompts/01_...md` through `prompts/07_...md`
- `prompts/00_global_rules.md`

Tasks:
- resolve interface mismatches between the generated subsystems;
- ensure the Motor abstraction is implemented consistently by SkyWatcher and TMC2209 backends;
- ensure the Axis layer, Combiner, and LX200 surface compose without hidden assumptions;
- ensure runtime wiring creates one functional mount surface from the two physical axes;
- ensure the test tree matches the resulting module layout.

Deliverables:
- one integrated codebase laid out like `context/TARGET_TREE.md`;
- a short integration report listing:
  - any assumptions added during synthesis;
  - any intentionally deferred behaviors;
  - which test expectations are covered immediately and which still require hardware validation.

Final acceptance check:
- the repository should be understandable to an engineer who only has this `reproduce_from_description/` package;
- subsystem boundaries should be explicit enough that later regeneration does not require reverse-engineering the original source.
