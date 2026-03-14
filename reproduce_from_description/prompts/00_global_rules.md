# Global Rules

You are reconstructing a functional codebase from descriptions.

Rules:
- do not read the current implementation under `src/` as source material;
- use only the files inside `reproduce_from_description/context/` plus any explicitly allowed external protocol references;
- rebuild behavior and contracts, not line-by-line source copies;
- prefer clear boundaries and explicit guarantees over cleverness;
- when a protocol can be made table-driven or spec-driven, prefer that design;
- preserve the public behavior described in architecture and tests even if the internal design improves.

Output expectations for each subsystem prompt:
- list the target files to create;
- state the invariants and guarantees that the implementation must preserve;
- produce code or a sufficiently concrete implementation blueprint;
- list the unit or integration tests needed to lock the behavior.

Global acceptance criteria:
- the fast non-hardware test profile should be reproducible;
- the hardware-facing interfaces should match the documented protocol baselines;
- runtime wiring must expose one LX200-compatible mount surface over two different physical motor backends.
