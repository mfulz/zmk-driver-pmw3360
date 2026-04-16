/*
 * Copyright (c) 2026 Matthias Fulz
 * SPDX-License-Identifier: MIT
 */

/**
 * @file behavior_pmw3360_save_settings.c
 * @brief Persist the current PMW3360 runtime tuning values on explicit key press.
 *
 * This behavior is intended for users who want to tune CPI and burst
 * accumulation live, then explicitly commit the current values once they are
 * satisfied with the result. It avoids implicit flash writes on every cycle
 * step and makes persistence a conscious action in the keymap.
 */

#define DT_DRV_COMPAT zmk_behavior_pmw3360_save_settings

#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/logging/log.h>

#include <drivers/behavior.h>

#include <zmk/behavior.h>

#include <zmk_pmw3360_behaviors.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

/**
 * Resolve the PMW3360 device targeted by the convenience behaviors.
 *
 * @return The first enabled PMW3360 device in devicetree, or `NULL`.
 */
static const struct device *behavior_pmw3360_save_settings_resolve_device(void) {
#if DT_HAS_COMPAT_STATUS_OKAY(pixart_pmw3360)
    return DEVICE_DT_GET(DT_COMPAT_GET_ANY_STATUS_OKAY(pixart_pmw3360));
#else
    return NULL;
#endif
}

/**
 * Handle one explicit "save PMW runtime settings" key press.
 *
 * @param binding Active behavior binding.
 * @param event ZMK key event metadata.
 *
 * @retval ZMK_BEHAVIOR_OPAQUE The event was consumed successfully.
 * @retval -ENODEV No ready PMW3360 device was available.
 * @retval negative errno Persisting the runtime settings failed.
 */
static int on_behavior_pmw3360_save_settings_pressed(struct zmk_behavior_binding *binding,
                                                     struct zmk_behavior_binding_event event) {
    ARG_UNUSED(binding);
    ARG_UNUSED(event);

    const struct device *pmw = behavior_pmw3360_save_settings_resolve_device();
    if (!pmw || !device_is_ready(pmw)) {
        LOG_WRN("PMW3360 device not ready");
        return -ENODEV;
    }

    const int rc = zmk_pmw3360_save_runtime_settings(pmw);
    if (rc < 0) {
        return rc;
    }

    LOG_INF("Saved PMW3360 runtime settings");
    return ZMK_BEHAVIOR_OPAQUE;
}

/**
 * Save behavior acts on key press only.
 *
 * @retval ZMK_BEHAVIOR_OPAQUE Always consumes the release event.
 */
static int on_behavior_pmw3360_save_settings_released(struct zmk_behavior_binding *binding,
                                                      struct zmk_behavior_binding_event event) {
    ARG_UNUSED(binding);
    ARG_UNUSED(event);
    return ZMK_BEHAVIOR_OPAQUE;
}

static const struct behavior_driver_api behavior_pmw3360_save_settings_driver_api = {
    .binding_pressed = on_behavior_pmw3360_save_settings_pressed,
    .binding_released = on_behavior_pmw3360_save_settings_released,
#if IS_ENABLED(CONFIG_ZMK_BEHAVIOR_METADATA)
    .get_parameter_metadata = zmk_behavior_get_empty_param_metadata,
#endif
};

BEHAVIOR_DT_INST_DEFINE(0, NULL, NULL, NULL, NULL, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
                        &behavior_pmw3360_save_settings_driver_api);

#endif /* DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT) */
