# Minimal Integration Example

This example shows one self-contained PMW3360 integration for a typical ZMK
keyboard or trackball build.

It demonstrates:

- one PMW3360 node on SPI
- one input listener
- runtime CPI tuning
- runtime burst accumulation tuning

## Files

- `pmw3360.conf` — minimum Kconfig options
- `pmw3360.overlay` — PMW3360 devicetree node and input listener
- `pmw3360.keymap` — behavior declarations and example bindings

## Important

The GPIO and SPI pin assignments are placeholders. You must replace them with
values that match your own board.
