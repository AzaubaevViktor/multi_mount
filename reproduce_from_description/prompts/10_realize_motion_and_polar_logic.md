# Prompt 10: Realize Axis, Combiner, and Polar Logic

Goal:
- upgrade the reconstructed `sky/` layer from baseline behavior to the motion and guiding behavior described in the live implementation dump.

Inputs:
- `context/ARCHITECTURE.md`
- `context/PROTOCOL_BASELINES.md`
- `context/CURRENT_IMPLEMENTATION_DUMP.md`
- `context/CURRENT_TEST_EXPECTATIONS.md`
- `prompts/00_global_rules.md`

Primary targets:
- `src/sky/axis.py`
- `src/sky/combiner.py`
- `src/sky/polar_compensator.py`
- `src/sky/constants.py`
- matching unit tests under `src/tests/units/`

Tasks:
- move guide behavior toward fixed-interval sky-speed interpolation instead of timed one-shot movement;
- implement the mathematical polar-offset model and forward guide-speed model;
- preserve the logical-position ownership of the Axis layer;
- preserve the invariant that tracking resumes correctly after halts and GOTO completion;
- document any remaining pole-crossing or reflection gaps explicitly.

Deliverables:
- updated `sky/` implementation;
- unit tests that lock the new guide math and polar math;
- short notes on any still-deferred motion behavior.
