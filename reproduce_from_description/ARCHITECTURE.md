# Reconstructed Architecture Index

This package now contains two architecture views:

- `context/ARCHITECTURE.md` keeps the source-free snapshot used for baseline regeneration.
- `context/CURRENT_IMPLEMENTATION_DUMP.md` captures the current implementation shape from the live project tree.

Recommended reading order:

1. `context/ARCHITECTURE.md`
2. `context/PROTOCOL_BASELINES.md`
3. `context/CURRENT_IMPLEMENTATION_DUMP.md`
4. `context/CURRENT_TEST_EXPECTATIONS.md`
5. `context/CURRENT_FIRMWARE_AND_RUNTIME_NOTES.md`

Use the original prompts `00` through `08` for source-free reconstruction.

Use the new prompts `09` through `13` when the goal is to raise reconstruction fidelity toward the current real implementation, or when you need to ask the user for hardware-specific overrides.
