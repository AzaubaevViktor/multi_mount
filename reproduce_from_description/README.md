# Reproduce From Description

This directory is a reconstruction package for the current project state.

Goal:
- allow a future agent to rebuild a functional copy of the codebase from descriptions and contracts;
- avoid depending on direct reads of the current implementation under `src/`;
- preserve subsystem boundaries, protocol contracts, runtime wiring, and acceptance criteria.

How to use this directory:
1. Read `context/ARCHITECTURE.md`.
2. Read `context/TEST_PLAN.md`.
3. Read `context/TARGET_TREE.md`.
4. Read `context/PROTOCOL_BASELINES.md`.
5. Use `prompts/00_global_rules.md`.
6. Run `prompts/01_...md` through `prompts/07_...md` in parallel.
7. Run `prompts/08_synthesis.md` to merge the parallel outputs into one codebase.

Definition of success:
- the rebuilt repository exposes the same major runtime surfaces:
  - LX200 TCP endpoint;
  - hybrid RA + DEC mount composition;
  - SkyWatcher RA backend;
  - TMC2209 DEC backend and compatible firmware contract;
  - monitor web server;
  - test layout and acceptance profile.
- the rebuilt code passes the fast test profile described in `context/TEST_PLAN.md`;
- the rebuilt code is compatible with the hardware and integration expectations described there.

Important reconstruction bias:
- treat descriptions in `context/` as the primary source of truth;
- treat protocol summaries in `context/PROTOCOL_BASELINES.md` as the contract to preserve;
- when available, external reference materials may be used to refine protocol details, but the package should remain usable without the current source tree.

Directory layout:
- `context/ARCHITECTURE.md`: architecture and module responsibilities snapshot.
- `context/TEST_PLAN.md`: acceptance and coverage snapshot.
- `context/TARGET_TREE.md`: expected repository layout to reconstruct.
- `context/PROTOCOL_BASELINES.md`: summarized protocol contracts.
- `prompts/`: subsystem-specific reconstruction prompts plus one synthesis prompt.
