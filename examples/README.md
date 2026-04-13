# Examples

This directory contains minimal integration examples for common PMW3360 module
use cases.

The examples are intentionally small. They are meant to answer one question:
"What is the minimum I need to copy into my own ZMK config to get this module
working?"

## Included examples

- `minimal/` — minimal PMW3360 integration with one input listener and direct
  runtime tuning behaviors

## How to use these examples

Do not copy the example files verbatim into a production config without reading
them first. They contain placeholder GPIO and SPI pin assignments.

Instead:

1. Use the files as a reference structure.
2. Replace the SPI pinctrl and GPIO assignments with your board values.
3. Keep the PMW3360 node properties and behavior nodes that match your build.
4. Add the bindings you actually want in your real keymap.
