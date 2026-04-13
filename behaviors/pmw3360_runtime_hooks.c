/*
 * Copyright (c) 2026 Matthias Fulz
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/toolchain.h>
#include <zephyr/sys/util.h>

#include <zmk_pmw3360_behaviors.h>

/**
 * Default weak runtime hook implementation.
 *
 * Boards that want local feedback may provide a strong definition with the
 * same signature. Leaving the default in place is valid and simply means that
 * runtime tuning changes do not trigger any extra side effects beyond the
 * updated sensor setting itself.
 */
__weak void zmk_pmw3360_runtime_setting_changed(const struct device *pmw,
                                                enum zmk_pmw3360_runtime_setting setting,
                                                uint16_t value) {
    ARG_UNUSED(pmw);
    ARG_UNUSED(setting);
    ARG_UNUSED(value);
}
