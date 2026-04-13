# ZMK PMW3360 Driver Module

A standalone ZMK module for the PixArt PMW3360 optical motion sensor.

The PMW3360 is a high-performance relative pointing sensor that appears in many
custom mice and trackball builds. This module focuses on making the sensor
usable in ZMK-based builds, including wireless BLE builds where a raw
one-burst-per-report strategy can create visible pointer backlog.

## Why this module exists

The PMW3360 was designed for high-rate relative motion tracking, not
specifically for low-power BLE keyboards. That mismatch matters in practice:
fast physical motion can produce sensor updates faster than a BLE pointing path
can always carry them one-by-one.

If every PMW3360 motion burst is forwarded as its own HID report, the host link
can build up report backlog. On a BLE build this often feels like pointer lag,
rubber-banding, or delayed motion that catches up in chunks.

This module addresses that problem with **burst accumulation**.

## Burst accumulation

Burst accumulation combines multiple PMW3360 motion bursts into one larger HID
report before the report is sent to the host.

This changes the **report rate**, not the **physical motion total**:

- Without burst accumulation, the driver tends to emit one HID report for every
  sensor burst.
- With burst accumulation enabled, the driver sums several sensor deltas and
  emits a single HID report containing the combined movement.

In practical terms this helps BLE pointing devices because it reduces report
pressure on the wireless link.

### Trade-off

Burst accumulation is a throughput and pacing tool. It does **not** intentionally
throw away pointer motion. The main trade-off is time granularity:

- Lower values behave more like a raw sensor stream.
- Higher values reduce report storms but make host-visible motion slightly less
  fine-grained in time.

A value of `1` disables burst accumulation entirely.

### Feedback wanted

The burst accumulation feature is intentionally exposed as a first-class tuning
parameter because its optimal value depends on the complete build:

- BLE vs USB
- mouse vs trackball
- IRQ vs polling
- host Bluetooth quality
- pointer sensitivity / CPI

Good feedback includes the device type, transport mode, CPI, whether IRQ is
used, and values that felt better or worse. A useful first set of values to try
is:

- `1`
- `4`
- `6`
- `12`
- `24`
- `48`

## Features

- PMW3360 SPI driver for ZMK / Zephyr input
- Interrupt-driven mode via MOTION pin
- Polling fallback when no motion pin is wired
- Optional external power gating via `power-gpios`
- Runtime CPI control
- Runtime burst accumulation control
- Directly usable ZMK CPI cycle behavior
- Directly usable ZMK burst accumulation cycle behavior
- Configurable orientation and axis inversion
- Angle tuning support
- Optional 3 mm lift-off configuration
- Optional `force-awake` mode to disable the PMW3360 internal rest mode
- Optional dedicated work queue for better isolation from unrelated system work

## Repository layout

- `drivers/input/input_pmw3360.c` — driver implementation
- `drivers/input/input_pmw3360.h` — private driver internals
- `include/zmk_driver_pmw3360.h` — public runtime control API
- `include/zmk_pmw3360_behaviors.h` — public helper API for runtime behaviors
- `behaviors/` — optional directly usable ZMK runtime tuning behaviors
- `dts/bindings/behaviors/` — devicetree bindings for the runtime behaviors
- `dts/bindings/input/pixart,pmw3360.yaml` — devicetree binding for the sensor
- `examples/` — minimal copy-adapt examples for common integration scenarios
- `snippets/runtime-tuning.dtsi` — minimal behavior declarations for CPI and
  burst accumulation
- `RELEASE_CHECKLIST.md` — publish checklist for validating a release
- `drivers/input/Kconfig` — driver Kconfig options
- `zephyr/module.yml` — Zephyr/ZMK module metadata

## Installing the module in a ZMK config

This repository is intended to be used as a Zephyr module in a ZMK workspace.
A typical setup uses `west.yml` in your ZMK config repository.

Add the module under `manifest.projects`, for example:

```yaml
manifest:
  remotes:
    - name: zmkfirmware
      url-base: https://github.com/zmkfirmware
    - name: your-account
      url-base: https://github.com/your-account

  projects:
    - name: zmk
      remote: zmkfirmware
      revision: main
      import: app/west.yml

    - name: zmk-driver-pmw3360
      remote: your-account
      revision: main

  self:
    path: config
```

Then update the workspace:

```sh
west update
```

If your workspace already exists and you add the module later, run:

```sh
west zephyr-export
```

## Compatibility and scope

- Validated against current ZMK `main` in a Zephyr 3.5 workspace.
- The runtime tuning behaviors currently assume one enabled PMW3360 device per
  build.
- The driver itself does not require that limitation; it applies only to the
  convenience behaviors shipped in `behaviors/`.

## Enabling the driver

In your keyboard configuration, enable Zephyr input and the PMW3360 driver if
those options are not already enabled by your board configuration:

```conf
CONFIG_ZMK_POINTING=y
CONFIG_INPUT=y
CONFIG_INPUT_PIXART_PMW3360=y
```

Optional tuning for the driver work queue:

```conf
CONFIG_INPUT_PIXART_PMW3360_USE_OWN_THREAD=y
CONFIG_INPUT_PIXART_PMW3360_THREAD_PRIORITY=3
CONFIG_INPUT_PIXART_PMW3360_THREAD_STACK_SIZE=768
```

Optional runtime tuning behaviors:

```conf
CONFIG_ZMK_BEHAVIOR_PMW3360_CPI_CYCLE=y
CONFIG_ZMK_BEHAVIOR_PMW3360_BURST_ACCUMULATION_CYCLE=y
```

These are enabled by default when the driver is enabled.

## Devicetree setup

### 1. Configure the SPI pins

The PMW3360 is an SPI device. Your board must configure SPI clock, MOSI, MISO,
and chip select using the normal Zephyr pinctrl and SPI mechanisms.

Example:

```dts
&pinctrl {
    spi0_default: spi0_default {
        group1 {
            pinmux = <SPI0_CSN_P21>, <SPI0_SCK_P22>, <SPI0_TX_P23>;
        };
        group2 {
            pinmux = <SPI0_RX_P20>;
            input-enable;
        };
    };
};
```

### 2. Add the PMW3360 device node

```dts
&spi0 {
    status = "okay";
    cs-gpios = <&gpio0 21 GPIO_ACTIVE_LOW>;

    mouse: pmw3360@0 {
        compatible = "pixart,pmw3360";
        status = "okay";
        reg = <0>;
        spi-max-frequency = <2000000>;

        irq-gpios = <&gpio0 27 (GPIO_ACTIVE_LOW | GPIO_PULL_UP)>;
        power-gpios = <&gpio0 28 GPIO_ACTIVE_HIGH>;

        cpi = <600>;
        burst-accumulation-max-samples = <12>;
        force-awake;
        rotate-90;
    };
};
```

### 3. Attach an input listener

```dts
/ {
    mouse_listener {
        compatible = "zmk,input-listener";
        device = <&mouse>;
    };
};
```

### 4. Add runtime tuning behaviors to the keymap

The module ships optional ready-to-use ZMK behaviors for runtime CPI and burst
accumulation tuning.

These behaviors currently assume a build contains one enabled PMW3360 device.
That matches the most common keyboard or trackball design and avoids extra
boilerplate in the keymap.

Example:

```dts
/ {
    behaviors {
        pmw_cpi_next: pmw_cpi_next {
            compatible = "zmk,behavior-pmw3360-cpi-cycle";
            #binding-cells = <0>;
            next;
            default-cpi = <300>;
        };

        pmw_cpi_prev: pmw_cpi_prev {
            compatible = "zmk,behavior-pmw3360-cpi-cycle";
            #binding-cells = <0>;
            default-cpi = <300>;
        };

        pmw_burst_next: pmw_burst_next {
            compatible = "zmk,behavior-pmw3360-burst-accumulation-cycle";
            #binding-cells = <0>;
            next;
            default-max-samples = <12>;
        };

        pmw_burst_prev: pmw_burst_prev {
            compatible = "zmk,behavior-pmw3360-burst-accumulation-cycle";
            #binding-cells = <0>;
            default-max-samples = <12>;
        };
    };
};
```

These nodes can then be used like any other ZMK behavior in the keymap.

Common pairings are:

- dedicated CPI up/down buttons
- `mod-morph` switching between CPI and burst accumulation on a held modifier
- temporary maintenance layers for runtime BLE tuning

If you prefer copy-paste over handwritten behavior nodes, a minimal snippet is
included here:

- `snippets/runtime-tuning.dtsi`

If you want a more complete starting point, also look at:

- `examples/minimal/pmw3360.conf`
- `examples/minimal/pmw3360.overlay`
- `examples/minimal/pmw3360.keymap`

## Configuration reference

### `power-gpios`

Optional external power gate for the sensor.

Use this when the PMW3360 is behind a transistor or load switch and should only
be powered while a specific layer or mode is active.

### `irq-gpios`

Optional motion interrupt pin connected to the PMW3360 MOTION output.

When present, the driver runs in interrupt-driven mode. This is the recommended
mode for low-latency pointing builds.

When absent, the driver polls the sensor at `polling-interval`.

### `cpi`

Pointer sensitivity in counts per inch.

Higher CPI values move the host cursor farther for the same physical movement.
Lower values feel slower and often more controllable.

The PMW3360 stores CPI in 100-CPI steps, so values are effectively rounded down
to the nearest multiple of 100.

### `burst-accumulation-max-samples`

Maximum number of PMW3360 motion bursts that may be summed into one HID report.

This is the main control for balancing raw report frequency against BLE report
backlog risk.

## Runtime behavior reference

### CPI cycle behavior

The CPI cycle behavior walks through this built-in preset list:

- `100`
- `200`
- `300`
- `400`
- `500`
- `600`
- `800`
- `1000`

`next;` moves forward through the list, while omitting `next;` creates the
matching reverse-step behavior.

### Burst accumulation cycle behavior

The burst accumulation cycle behavior walks through this built-in preset list:

- `1`
- `4`
- `6`
- `12`
- `24`
- `48`

This preset list intentionally covers:

- raw or near-raw behavior (`1`, `4`)
- moderate BLE backlog mitigation (`6`, `12`)
- aggressive BLE backlog mitigation (`24`, `48`)

For many BLE builds, `12` and `24` are good first candidates.

## Optional integration hook

Boards may override the weak hook
`zmk_pmw3360_runtime_setting_changed()` to react whenever the runtime behaviors
change CPI or burst accumulation.

Typical uses:

- trigger a host-facing GATT/OSD status update
- blink a local LED
- log the new runtime tuning value for debugging

The default implementation is a no-op, so no extra integration is required if
your board does not need custom feedback.

## CI smoke build

This repository ships a tiny self-contained smoke consumer under:

- `ci/smoke-config`

The GitHub Actions workflow builds that consumer against upstream ZMK and the
module itself. The smoke build is intentionally small: it proves the module can
be fetched as an external ZMK module, that the PMW3360 devicetree binding is
valid, and that the directly usable runtime behaviors instantiate cleanly.

## Preparing a public release

Before publishing the repository or tagging a new release, run through
`RELEASE_CHECKLIST.md`.

### `rotate-90`, `rotate-180`, `rotate-270`

Sensor orientation helpers for rotated PCB or sensor placement.

Use exactly one when the physical sensor is mounted rotated relative to the
expected pointer orientation.

### `angle-tune`

Fine correction for small mounting skew.

This is separate from the 90/180/270 degree rotation flags and is useful when
movement feels slightly diagonal even after the gross rotation is correct.

### `lift-height-3mm`

Switches the PMW3360 to a higher lift-off threshold.

Useful when tracking stops too early or when your lens/surface combination wants
a slightly more tolerant lift-off distance.

### `force-awake`

Disables the PMW3360 internal rest mode.

This is most useful for designs that already use external power gating and want
the sensor to feel immediately responsive whenever it is powered.

### `polling-interval`

Polling interval in microseconds for builds without a motion interrupt pin.

Lower values improve responsiveness at the cost of more SPI and CPU activity.
Higher values reduce overhead but can feel less immediate.

### `invert-x`, `invert-y`

Apply axis inversion after any rotation has already been configured.

Use these when the physical motion direction is still mirrored after the sensor
orientation is otherwise correct.

## Runtime API

The public runtime API lives in `include/zmk_driver_pmw3360.h`.

### `zmk_driver_pmw3360_set_enabled()`

Enable or disable the sensor at runtime.

Typical use cases:

- only power the sensor while a pointing layer is active
- disable the sensor while typing to avoid accidental motion
- explicitly re-run the initialization sequence after powering up

### `zmk_driver_pmw3360_set_cpi()`

Change pointer sensitivity at runtime.

This can be called from custom behaviors or other ZMK glue code to implement CPI
presets or CPI cycling.

### `zmk_driver_pmw3360_set_burst_accumulation_max_samples()`

Change the burst accumulation limit at runtime.

This allows experiments such as switching between a low-latency profile and a
BLE-friendly high-throughput profile without rebuilding the firmware.

## Recommended starting points

### Wireless BLE trackball or mouse

- Use `irq-gpios` if the MOTION pin is wired.
- Start with `burst-accumulation-max-samples = <12>`.
- Try `force-awake` when an external power gate already handles idle power.

### USB-only or wired-first build

- Start with `burst-accumulation-max-samples = <1>` or a low value.
- Raise it only if you observe actual report backlog or host-side stutter.

### No MOTION pin available

- Use polling mode.
- Start with `polling-interval = <1000>` and adjust from there.

## Limitations

- The PMW3360 is not a low-power sensor. It can work in wireless builds, but it
  benefits from careful power management and realistic expectations.
- Burst accumulation is a transport mitigation, not a substitute for poor RF or
  host Bluetooth performance.
- Optimal values vary across boards, hosts, and pointing hardware.

## Building a firmware that uses this module

The exact build command depends on your ZMK configuration repository. A typical
workflow looks like this:

```sh
west build -s zmk/app -b nice_nano_v2 -- -DSHIELD=your_keyboard_right
```

The important part is that the module is present in the workspace and imported
via `west.yml` before building.
