# Prompt 14: Controlled Source Mining From The Live Codebase

Use this prompt only when the existing dumps under `context/` are no longer sufficient and you need to inspect the live implementation under `src/` or `telescope_dec/`.

Goal:
- extract high-value behavioral context from the current source tree without turning the reconstruction into a line-by-line copy;
- explicitly separate what should be preserved from what should be ignored as incidental implementation detail.

When this prompt is allowed:
- a phase-2 or phase-3 regeneration is blocked because the current `context/*.md` files are missing important details;
- you need to confirm a real invariant, conversion constant, pin map, handshake rule, or test expectation that is not yet dumped;
- you need to distinguish production behavior from outdated stubs or archived experiments.

Default bias:
- prefer the already dumped `context/` files first;
- read the live source only to fill a concrete gap;
- if one file is enough, read one file;
- avoid broad codebase sweeps unless the gap truly spans multiple layers.

Scope you may inspect:
- `src/`
- `telescope_dec/`
- active tests under `src/tests/hw`, `src/tests/units`, `src/tests/base`

Do not rely on:
- `__pycache__/`
- generated build artifacts
- dead or archived experiments unless you explicitly say they are historical only
- accidental naming or formatting details that do not affect contracts

What you may take from the live source:

1. Real contracts and invariants:
   - protocol framing;
   - response parsing rules;
   - command/state-machine invariants;
   - typed-unit conversions;
   - pin maps and board-level constants;
   - acceptance behavior visible in tests.

2. Real tuning values when they materially affect behavior:
   - timeouts;
   - guide intervals;
   - speed thresholds;
   - tolerance windows;
   - gear ratios;
   - microstep defaults.

3. Real boundaries between modules:
   - which layer owns logical position;
   - which layer owns raw hardware state;
   - which layer performs compensation or retries.

4. Real user- or hardware-facing assumptions:
   - device-search patterns;
   - firmware readiness handshake;
   - runtime wiring expectations;
   - monitoring surfaces that acceptance tests depend on.

What you must not take directly:

1. Line-by-line code structure:
   - do not copy function bodies verbatim;
   - do not mirror private helper layout just because it exists;
   - do not reproduce incidental naming unless it helps preserve a contract.

2. Clearly incidental or stale implementation detail:
   - debugging leftovers;
   - commented-out logic;
   - pycache artifacts;
   - one-off hacks without test or runtime evidence.

3. Source quirks that conflict with the cleaner reconstruction:
   - preserve behavior, not historical messiness;
   - if the live code is noisy but the invariant is clear, restate the invariant and reimplement it cleanly.

Required output format from this prompt:

- list the exact live files inspected;
- for each inspected file, extract only:
  - preserved facts;
  - useful constants;
  - behavior to emulate;
  - behavior to ignore.
- update or create context files under `reproduce_from_description/context/` instead of leaving the findings implicit;
- if the mined information changes implementation work, point to the follow-up prompt that should consume it.

Preferred phrasing for the extracted summary:

> After analyzing the current source, here is what can be taken:
> - ...
>
> Here is what should not be taken directly:
> - ...

Recommended follow-up flow:

1. Run this prompt to mine only the missing facts.
2. Write those facts into `context/` as a durable dump.
3. Continue with one of:
   - `prompts/10_realize_motion_and_polar_logic.md`
   - `prompts/11_realize_transport_and_backends.md`
   - `prompts/12_realize_firmware_and_runtime.md`
   - `prompts/13_user_info_needed_for_hardware_variants.md`

Success criteria:
- the live source may be inspected, but the resulting reconstruction remains description-driven;
- future agents can use the new `context/` dump without reopening the original source unless another genuine gap appears.
