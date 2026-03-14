# Prompt 09: Align With The Current Live Implementation

Goal:
- use the new context dumps to raise fidelity from the source-free reconstruction toward the current real codebase.

Inputs:
- all files under `context/`
- especially:
  - `context/CURRENT_IMPLEMENTATION_DUMP.md`
  - `context/CURRENT_TEST_EXPECTATIONS.md`
  - `context/CURRENT_FIRMWARE_AND_RUNTIME_NOTES.md`
- `prompts/00_global_rules.md`

Do not read:
- the live implementation under `src/` or `telescope_dec/` directly if these dump files are already present and sufficient.

Tasks:
- compare the current reconstructed code against the live implementation dump;
- identify behaviors that are still only placeholders, simplifications, or compatibility shims;
- produce a short fidelity-gap report grouped into:
  - runtime wiring;
  - axis/combiner/polar logic;
  - hardware backends;
  - firmware;
  - tests.

Deliverables:
- an updated reconstruction plan that explicitly lists which gaps can be closed without new user input;
- any new context notes needed to support the follow-up prompts.
