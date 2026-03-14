# Prompt 01: Rebuild Serial Transport

Goal:
- reconstruct `src/serial_wrapper/wrapper.py`.

Allowed inputs:
- `context/ARCHITECTURE.md`
- `context/TEST_PLAN.md`
- `context/PROTOCOL_BASELINES.md`
- `prompts/00_global_rules.md`

Do not read:
- the existing implementation under `src/serial_wrapper/`.

Target behavior:
- provide generic serial transport reusable by both SkyWatcher and TMC2209 backends;
- support device search by regex pattern;
- support connect, reset, line query, prefix-aware reads, bulk buffer drain, and close;
- make timeout, terminator, and encoding configurable;
- provide a clear failure model for broken serial state and reconnect scenarios;
- be safe for use from threaded callers.

Deliverables:
- `src/serial_wrapper/wrapper.py`
- fast tests that lock search behavior and at least one prefix-aware read scenario.

Important guarantees to preserve:
- the transport must not contain backend-specific command semantics;
- callers should not need to manually manage read buffering quirks on every call;
- close/reconnect behavior should be explicit and recoverable;
- both binary-ish SkyWatcher replies and line-oriented TMC replies must be supported by configuration, not by forked codepaths.
