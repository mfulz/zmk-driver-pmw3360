/*
 * Copyright (c) 2026 Matthias Fulz
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <stdint.h>

#include <zephyr/device.h>

/**
 * @file zmk_pmw3360_behaviors.h
 * @brief Public helpers for optional PMW3360 runtime tuning behaviors.
 *
 * The PMW3360 driver itself exposes low-level runtime setters such as
 * `zmk_driver_pmw3360_set_cpi()`. The optional runtime behaviors built by this
 * module add ready-to-use ZMK keymap behaviors that cycle through predefined
 * CPI and burst accumulation presets.
 *
 * The helper functions declared here serve two purposes:
 *
 * 1. Host-facing status code can query the most recently selected runtime
 *    values without performing a fresh sensor SPI transaction.
 * 2. Board-level integration code can hook into runtime changes to trigger
 *    custom feedback such as OSD refreshes or LEDs.
 *
 * The runtime behaviors currently assume a single enabled PMW3360 device in a
 * build. This matches the most common keyboard/trackball use case.
 */

/**
 * Identifies which PMW3360 runtime tuning parameter changed.
 */
enum zmk_pmw3360_runtime_setting {
    /**
     * PMW3360 pointer sensitivity changed.
     *
     * The value passed to the runtime hook is CPI in counts per inch.
     */
    ZMK_PMW3360_RUNTIME_SETTING_CPI = 0,

    /**
     * PMW3360 burst accumulation limit changed.
     *
     * The value passed to the runtime hook is the new maximum number of motion
     * bursts the driver may accumulate into one HID report.
     */
    ZMK_PMW3360_RUNTIME_SETTING_BURST_ACCUMULATION = 1,
};

/**
 * Return the cached CPI selected by the PMW3360 CPI cycle behavior.
 *
 * The CPI cycle behavior stores the currently selected CPI in RAM so host-side
 * or board-side code can inspect it cheaply. This avoids an extra sensor
 * transaction when a status layer merely needs the current user-selected value.
 *
 * @param pmw PMW3360 device whose cached CPI should be returned.
 * @param fallback_cpi Value returned if no runtime behavior state exists yet
 *                     for the device, for example before the first CPI change.
 *
 * @return Cached CPI in counts per inch, or `fallback_cpi` if the behavior has
 *         not created runtime state for the device yet.
 */
uint16_t zmk_behavior_pmw3360_get_cpi_for_device(const struct device *pmw, uint16_t fallback_cpi);

/**
 * Return the cached burst accumulation limit selected by the corresponding
 * PMW3360 runtime behavior.
 *
 * @param pmw PMW3360 device whose cached burst accumulation value should be
 *            returned.
 * @param fallback_max_samples Value returned if no runtime behavior state
 *                             exists yet for the device.
 *
 * @return Cached maximum number of PMW3360 motion bursts accumulated into one
 *         HID report, or `fallback_max_samples` if the behavior has not
 *         created runtime state for the device yet.
 */
uint8_t zmk_behavior_pmw3360_get_burst_accumulation_max_samples_for_device(
    const struct device *pmw, uint8_t fallback_max_samples);

/**
 * Optional board-level runtime tuning hook.
 *
 * The PMW3360 runtime tuning behaviors call this hook after a setting change
 * has been applied successfully. The module ships a weak no-op implementation
 * so consumers do not need to provide it.
 *
 * Boards may override the hook with a strong symbol to trigger local feedback
 * such as:
 * - host status notifications
 * - LEDs or displays
 * - debug logging or telemetry
 *
 * @param pmw PMW3360 device whose runtime setting changed.
 * @param setting Which logical tuning parameter changed.
 * @param value New runtime value. For CPI this is counts per inch. For burst
 *              accumulation this is the new maximum number of accumulated
 *              sensor bursts per HID report.
 */
void zmk_pmw3360_runtime_setting_changed(const struct device *pmw,
                                         enum zmk_pmw3360_runtime_setting setting,
                                         uint16_t value);
