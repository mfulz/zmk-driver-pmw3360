/*
 * Copyright (c) 2026 Matthias Fulz
 *
 * Public runtime control helpers for the ZMK PMW3360 driver module.
 *
 * The functions declared in this header are intended for higher-level ZMK code
 * such as custom behaviors, layer logic, or board-specific power management.
 * They deliberately avoid exposing any of the driver's internal structs so the
 * runtime API can remain stable even if the implementation evolves.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Enable or disable a PMW3360 sensor instance at runtime.
 *
 * Enabling the sensor powers it up when an optional external power gate is
 * present, schedules the PMW3360 initialization sequence, and starts motion
 * reporting once initialization has completed successfully.
 *
 * Disabling the sensor stops motion reporting immediately, cancels pending
 * driver work items, disables the motion interrupt when one is configured, and
 * powers the sensor down when `power-gpios` is present in devicetree.
 *
 * This function exists so ZMK keymap logic can make the sensor available only
 * on the layers that need it, which is especially useful for wireless builds
 * where the PMW3360's idle power draw would otherwise remain visible even when
 * the pointing device is not in active use.
 *
 * @param dev
 *     PMW3360 device instance obtained from devicetree.
 * @param enabled
 *     `true` to power up and initialize the sensor, `false` to stop reporting
 *     motion and power it down when possible.
 *
 * @retval 0 Sensor state change was accepted.
 * @retval -EINVAL `dev` was `NULL`.
 * @retval negative errno Another lower-level power or scheduling operation
 *         failed.
 */
int zmk_driver_pmw3360_set_enabled(const struct device *dev, bool enabled);

/**
 * Change the PMW3360 CPI (counts per inch) value at runtime.
 *
 * CPI controls pointer sensitivity. Higher CPI values produce larger cursor
 * movement for the same physical mouse or trackball movement, while lower CPI
 * values make the pointer move more slowly and often feel more precise for
 * fine-grained work.
 *
 * The PMW3360 register format stores CPI in 100-CPI steps. The driver therefore
 * normalizes any requested value to the next lower 100-CPI step and clamps it
 * into the driver's supported range. For example, requesting `650` results in a
 * programmed value of `600`.
 *
 * If the device is currently disabled or still initializing, the normalized CPI
 * is stored and applied during the next successful initialization sequence.
 *
 * @param dev
 *     PMW3360 device instance obtained from devicetree.
 * @param cpi
 *     Desired CPI value. In practical terms this is pointer sensitivity. Common
 *     values are `100`, `200`, `400`, `600`, `800`, and `1000`.
 *
 * @retval 0 The request was accepted. This may mean the value was applied
 *         immediately or stored for the next initialization.
 * @retval -EINVAL `dev` was `NULL`.
 * @retval negative errno The live sensor rejected the register write or the
 *         read-back verification failed.
 */
int zmk_driver_pmw3360_set_cpi(const struct device *dev, uint16_t cpi);

/**
 * Change the maximum number of PMW3360 motion bursts accumulated into a single
 * HID report.
 *
 * The PMW3360 can produce motion updates faster than a BLE host link can always
 * carry them when the sensor is used in interrupt-driven mode. If the driver
 * emits one HID report for every individual sensor burst, the host-facing report
 * queue can build up backlog and become visible as delayed or rubber-banded
 * pointer movement.
 *
 * Burst accumulation mitigates that problem by summing several PMW3360 motion
 * bursts into one larger HID report. A value of `1` disables accumulation and
 * makes the driver emit a report for every burst. Higher values reduce report
 * pressure on the BLE path at the cost of coarser time granularity.
 *
 * This setting does not change the physical movement that reaches the host; it
 * changes how many sensor deltas may be combined before they are reported.
 *
 * @param dev
 *     PMW3360 device instance obtained from devicetree.
 * @param max_samples
 *     Maximum number of motion bursts to accumulate into one HID report. The
 *     driver clamps this value into a conservative runtime range.
 *
 * @retval 0 The request was accepted.
 * @retval -EINVAL `dev` was `NULL`.
 */
int zmk_driver_pmw3360_set_burst_accumulation_max_samples(const struct device *dev,
                                                          uint8_t max_samples);

#ifdef __cplusplus
}
#endif
