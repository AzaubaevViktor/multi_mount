# AGENTS

## Arduino: What you SHOULDN'T do
- Don't use F() or PSTR macros in global/namespace-scope initializers; keep them inside functions or use PROGMEM arrays + FPSTR.
- Don't define static const integral members twice (in-class + out-of-class); pick one (prefer constexpr).
- Don't use dynamic allocation or heavy STL on AVR unless necessary.
- Don't block in loops when timing-sensitive tasks need servicing; avoid long delays.
- Don't use PROGMEM