# Prompt 06: Rebuild Runtime, Monitor, and Logging

Goal:
- reconstruct:
  - `src/__main__.py`
  - `src/logging_setup.py`
  - `src/web_control/web.py`
  - `src/web_control/static/index.html`
  - `src/web_control/static/app.js`
  - `src/web_control/static/app.css`

Allowed inputs:
- `context/ARCHITECTURE.md`
- `context/TEST_PLAN.md`
- `context/TARGET_TREE.md`
- `prompts/00_global_rules.md`

Do not read:
- the existing implementation under `src/web_control/`;
- the existing implementation of `src/__main__.py` and `src/logging_setup.py`.

Target behavior:
- wire up the runtime from serial discovery through motors, axes, combiner, LX200 surface, and monitor server;
- provide structured logging suitable for debugging hardware/runtime interaction;
- expose a standalone monitor server with SSE updates and a simple static UI;
- keep monitor infrastructure separate from the LX200 mount core so it can exist as optional runtime wiring.

Deliverables:
- the files listed above;
- at least one fast test for monitor behavior or structure generation;
- startup notes documenting how the runtime composes the physical axes into one logical mount.

Important guarantees to preserve:
- runtime wiring must remain easy to follow and deterministic;
- monitor infrastructure must not own core mount logic;
- logging must help debug serial and motion layers without changing their contracts.
