# Prompt 07: Rebuild Tests and Acceptance Harness

Goal:
- reconstruct:
  - `pytest.ini`
  - `TESTS_PLAN.md`
  - fast unit tests under `src/tests/units/`
  - hardware and integration harness under `src/tests/hw/`
  - any minimal shared fixtures needed under `src/tests/base/` or `src/tests/`.

Allowed inputs:
- `context/TEST_PLAN.md`
- `context/ARCHITECTURE.md`
- `context/PROTOCOL_BASELINES.md`
- `prompts/00_global_rules.md`

Do not read:
- the existing tests under `src/tests/`.

Target behavior:
- recreate a two-level test strategy:
  - fast tests for math, parsing, and contract logic;
  - hardware or integration tests for live devices and end-to-end motion behavior;
- preserve the acceptance themes described in the test plan;
- make the test tree reflect the reconstructed module boundaries.

Deliverables:
- `pytest.ini`
- refreshed `TESTS_PLAN.md`
- enough unit tests to validate the non-hardware contract surface;
- placeholder or full hardware tests that describe the acceptance behavior for real devices.

Important guarantees to preserve:
- fast tests must validate the most important non-hardware contracts;
- hardware suites must describe actual runtime expectations, not just low-level API calls;
- test structure should mirror the architecture so failures localize by layer.
