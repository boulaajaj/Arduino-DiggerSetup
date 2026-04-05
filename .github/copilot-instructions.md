# Copilot Code Review Instructions — Arduino Digger Controller

## Project Context
Arduino Nano R4 (Renesas RA4M1, 32-bit ARM Cortex-M4, 48MHz) controlling a ride-on excavator.
Safety-critical: code errors can cause a 50lb machine with a child riding it to behave unexpectedly.

## Critical Checks (Flag as errors)
- **Blocking calls in loop()**: Flag any use of `delay()`, `pulseIn()`, or blocking `while` loops in production sketches. The control loop must be non-blocking at ~20kHz.
- **Missing constrain() on servo output**: Every value written to `writeMicroseconds()` MUST be constrained to 1000-2000.
- **Integer overflow**: Watch for arithmetic on `int` (16-bit on some platforms, 32-bit on ARM) that could overflow. Flag multiplication of large values without casting to `long` first.
- **Float without f suffix**: Flag `1.0` (promotes to double, software-emulated on some ARM cores). Should be `1.0f`.
- **Magic numbers**: Flag raw numeric constants outside the [CONFIG] section. All tunables must be named constants.
- **Missing failsafe**: Any new RC input path must have a timeout/failsafe that returns to neutral (SVC = 1500) when signal is lost.
- **Global state mutation**: Flag functions that modify global state without clear documentation. Prefer passing state explicitly.

## Code Quality Checks (Flag as suggestions)
- **Cyclomatic complexity**: Flag functions with more than 3 levels of nesting or more than 10 branches.
- **Single Responsibility**: Each [MODULE] section should do one thing. Flag functions that mix input reading with output writing.
- **Inefficient patterns**: Flag nested loops, repeated calculations that could be cached, or string operations in the hot loop.
- **Consistent naming**: Constants = UPPER_SNAKE_CASE, structs = PascalCase, functions = camelCase, pins = PIN_NAME.

## Arduino-Specific
- Servo PWM range: 1000-2000us, center 1500us
- ADC: 14-bit (0-16383), center 8192
- SBUS range: 172-1811, center ~992
- Board FQBN: arduino:renesas_uno:nanor4
