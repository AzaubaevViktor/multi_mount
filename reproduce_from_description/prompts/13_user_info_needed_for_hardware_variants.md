# Prompt 13: Request Hardware Variant Information From The User

Use this prompt only if the target regeneration is not supposed to match the currently dumped physical rig exactly.

Goal:
- ask the user for the minimum hardware-specific data needed to avoid freezing the wrong values into runtime discovery or firmware.

Ask for:

1. The actual USB-serial identifiers or regexes to use for RA and DEC device discovery.
2. The exact Arduino or MCU board variant intended for `telescope_dec/`.
3. The TMC2209 board details that affect `R_SENSE`, current limits, and UART wiring.
4. Any changed pinout for `STEP`, `DIR`, `EN`, LEDs, or power sensing.
5. Any changed gear ratio, microstep default, or mechanical conversion constant for DEC.

Output expectations:
- present the questions as a short checklist;
- explain which files will depend on the answers;
- after the user replies, update:
  - `context/CURRENT_FIRMWARE_AND_RUNTIME_NOTES.md`
  - the relevant reconstruction prompts
  - any affected runtime or firmware files.
