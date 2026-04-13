/*
 * Copyright (c) 2026 Matthias Fulz
 * SPDX-License-Identifier: MIT
 */

/**
 * @file behavior_pmw3360_cpi_cycle.c
 * @brief Ready-to-use runtime CPI cycle behavior for the PMW3360.
 *
 * This behavior provides a directly consumable ZMK behavior for "CPI up/down"
 * style key bindings. It intentionally sits one layer above the PMW3360 driver:
 *
 * - the driver exposes the low-level runtime setter
 * - this behavior adds preset cycling, shared runtime state, and serialization
 *   of repeated key presses
 *
 * The implementation assumes a single enabled PMW3360 device in the build and
 * resolves that device automatically, so boards can use the behavior directly
 * from their keymap without extra boilerplate.
 */

#define DT_DRV_COMPAT zmk_behavior_pmw3360_cpi_cycle

#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include <drivers/behavior.h>

#include <zmk/behavior.h>

#include <zmk_driver_pmw3360.h>
#include <zmk_pmw3360_behaviors.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

static const uint16_t cpi_steps[] = {100, 200, 300, 400, 500, 600, 800, 1000};

struct behavior_pmw3360_cpi_cycle_config {
    bool next;
    uint16_t default_cpi;
};

struct pmw3360_cpi_state_entry {
    const struct device *pmw;
    struct k_work apply_work;
    uint16_t default_cpi;
    uint16_t cpi;
    int pending_delta;
    bool work_inited;
};

K_MUTEX_DEFINE(pmw3360_cpi_state_mutex);
static struct pmw3360_cpi_state_entry pmw3360_cpi_states[4];

/**
 * Resolve the first enabled PMW3360 device from devicetree.
 *
 * This behavior is meant to be directly usable on the common "one PMW3360 per
 * board" design. Avoiding an explicit devicetree phandle keeps the keymap side
 * simple and prevents init-order problems when the behavior is wrapped by
 * higher-level behaviors such as `mod-morph`.
 *
 * @return PMW3360 device instance when present in the build, otherwise `NULL`.
 */
static const struct device *behavior_pmw3360_cpi_cycle_resolve_device(void) {
#if DT_HAS_COMPAT_STATUS_OKAY(pixart_pmw3360)
    return DEVICE_DT_GET(DT_COMPAT_GET_ANY_STATUS_OKAY(pixart_pmw3360));
#else
    return NULL;
#endif
}

/**
 * Find or allocate the shared CPI state entry for one PMW3360 device.
 *
 * NEXT and PREV behavior instances must operate on one shared runtime value.
 * Otherwise both keys would maintain their own independent copy of the current
 * CPI and stepping would feel inconsistent.
 *
 * @param pmw PMW3360 device whose shared runtime state is requested.
 * @param default_cpi Initial CPI used for newly created state slots.
 *
 * @return Existing or newly allocated state slot, or `NULL` if the fixed-size
 *         state table is exhausted.
 */
static struct pmw3360_cpi_state_entry *pmw3360_cpi_state_get_or_create(const struct device *pmw,
                                                                        uint16_t default_cpi) {
    struct pmw3360_cpi_state_entry *free_entry = NULL;

    for (size_t i = 0; i < ARRAY_SIZE(pmw3360_cpi_states); i++) {
        if (pmw3360_cpi_states[i].pmw == pmw) {
            return &pmw3360_cpi_states[i];
        }
        if (!pmw3360_cpi_states[i].pmw && !free_entry) {
            free_entry = &pmw3360_cpi_states[i];
        }
    }

    if (!free_entry) {
        return NULL;
    }

    free_entry->pmw = pmw;
    free_entry->default_cpi = default_cpi;
    free_entry->cpi = default_cpi;
    return free_entry;
}

uint16_t zmk_behavior_pmw3360_get_cpi_for_device(const struct device *pmw, uint16_t fallback_cpi) {
    if (!pmw) {
        return fallback_cpi;
    }

    uint16_t cpi = fallback_cpi;
    k_mutex_lock(&pmw3360_cpi_state_mutex, K_FOREVER);
    for (size_t i = 0; i < ARRAY_SIZE(pmw3360_cpi_states); i++) {
        if (pmw3360_cpi_states[i].pmw == pmw) {
            cpi = pmw3360_cpi_states[i].cpi;
            break;
        }
    }
    k_mutex_unlock(&pmw3360_cpi_state_mutex);
    return cpi;
}

/**
 * Return the preset index for one CPI value.
 *
 * @param cpi CPI value in counts per inch.
 *
 * @return Zero-based preset index, or `-1` if the value is not part of the
 *         built-in cycle list.
 */
static int pmw3360_cpi_find_step_index(uint16_t cpi) {
    for (size_t i = 0; i < ARRAY_SIZE(cpi_steps); i++) {
        if (cpi_steps[i] == cpi) {
            return (int)i;
        }
    }

    return -1;
}

/**
 * Compute the next CPI preset in the circular cycle list.
 *
 * @param current_cpi Current runtime CPI.
 * @param next `true` to move to the next preset, `false` to move backward.
 * @param default_cpi Default CPI used when the current runtime value is not one
 *                    of the known presets.
 *
 * @return The next preset CPI value.
 */
static uint16_t pmw3360_cpi_cycle_value(uint16_t current_cpi, bool next, uint16_t default_cpi) {
    int idx = pmw3360_cpi_find_step_index(current_cpi);
    if (idx < 0) {
        idx = pmw3360_cpi_find_step_index(default_cpi);
        if (idx < 0) {
            idx = 0;
        }

        return cpi_steps[idx];
    }

    if (next) {
        idx = (idx + 1) % (int)ARRAY_SIZE(cpi_steps);
    } else {
        idx = (idx + (int)ARRAY_SIZE(cpi_steps) - 1) % (int)ARRAY_SIZE(cpi_steps);
    }

    return cpi_steps[idx];
}

/**
 * Serialize pending CPI step requests and apply them to the PMW3360.
 *
 * The behavior does not program the sensor directly in the key handler. That
 * keeps fast repeated taps deterministic: instead of multiple overlapping SPI
 * writes, they collapse into one serialized queue on a per-device worker.
 *
 * @param work Worker embedded in one PMW3360 runtime state slot.
 */
static void pmw3360_cpi_apply_work_handler(struct k_work *work) {
    struct pmw3360_cpi_state_entry *state =
        CONTAINER_OF(work, struct pmw3360_cpi_state_entry, apply_work);

    for (;;) {
        bool next = false;
        uint16_t current_cpi = 0;
        uint16_t new_cpi = 0;

        k_mutex_lock(&pmw3360_cpi_state_mutex, K_FOREVER);

        if (state->pending_delta == 0) {
            k_mutex_unlock(&pmw3360_cpi_state_mutex);
            return;
        }

        next = state->pending_delta > 0;
        current_cpi = state->cpi;
        new_cpi = pmw3360_cpi_cycle_value(current_cpi, next, state->default_cpi);

        const int rc = zmk_driver_pmw3360_set_cpi(state->pmw, new_cpi);
        if (rc < 0) {
            k_mutex_unlock(&pmw3360_cpi_state_mutex);
            LOG_ERR("Failed to set PMW3360 CPI=%u: %d", new_cpi, rc);
            return;
        }

        state->cpi = new_cpi;
        state->pending_delta += next ? -1 : 1;
        k_mutex_unlock(&pmw3360_cpi_state_mutex);

        LOG_INF("PMW3360 CPI %s: %u -> %u", next ? "NEXT" : "PREV", current_cpi, new_cpi);
        zmk_pmw3360_runtime_setting_changed(state->pmw, ZMK_PMW3360_RUNTIME_SETTING_CPI, new_cpi);
    }
}

/**
 * Initialize one CPI cycle behavior instance.
 *
 * @param dev Behavior device instance generated from devicetree.
 *
 * @retval 0 Initialization completed successfully.
 */
static int behavior_pmw3360_cpi_cycle_init(const struct device *dev) {
    const struct behavior_pmw3360_cpi_cycle_config *cfg = dev->config;
    const struct device *pmw = behavior_pmw3360_cpi_cycle_resolve_device();

    if (!pmw) {
        return 0;
    }

    k_mutex_lock(&pmw3360_cpi_state_mutex, K_FOREVER);
    struct pmw3360_cpi_state_entry *state = pmw3360_cpi_state_get_or_create(pmw, cfg->default_cpi);
    if (state && !state->work_inited) {
        k_work_init(&state->apply_work, pmw3360_cpi_apply_work_handler);
        state->work_inited = true;
    }
    k_mutex_unlock(&pmw3360_cpi_state_mutex);

    return 0;
}

/**
 * Handle one CPI cycle key press.
 *
 * @param binding Active behavior binding.
 * @param event ZMK binding event metadata.
 *
 * @retval ZMK_BEHAVIOR_OPAQUE The event was consumed.
 * @retval -ENODEV No ready PMW3360 device was found.
 * @retval -ENOMEM No free runtime state slot was available.
 */
static int on_behavior_pmw3360_cpi_cycle_pressed(struct zmk_behavior_binding *binding,
                                                 struct zmk_behavior_binding_event event) {
    ARG_UNUSED(event);

    const struct device *dev = zmk_behavior_get_binding(binding->behavior_dev);
    const struct behavior_pmw3360_cpi_cycle_config *cfg = dev->config;
    const struct device *pmw = behavior_pmw3360_cpi_cycle_resolve_device();

    if (!pmw || !device_is_ready(pmw)) {
        LOG_WRN("PMW3360 device not ready");
        return -ENODEV;
    }

    k_mutex_lock(&pmw3360_cpi_state_mutex, K_FOREVER);
    struct pmw3360_cpi_state_entry *state = pmw3360_cpi_state_get_or_create(pmw, cfg->default_cpi);
    if (!state) {
        k_mutex_unlock(&pmw3360_cpi_state_mutex);
        LOG_ERR("No PMW3360 CPI state slots available");
        return -ENOMEM;
    }

    if (!state->work_inited) {
        k_work_init(&state->apply_work, pmw3360_cpi_apply_work_handler);
        state->work_inited = true;
    }

    state->pending_delta += cfg->next ? 1 : -1;
    struct k_work *apply_work = &state->apply_work;
    k_mutex_unlock(&pmw3360_cpi_state_mutex);

    (void)k_work_submit(apply_work);
    return ZMK_BEHAVIOR_OPAQUE;
}

/**
 * CPI cycle acts on key press only.
 *
 * @retval ZMK_BEHAVIOR_OPAQUE Always consumes the release event.
 */
static int on_behavior_pmw3360_cpi_cycle_released(struct zmk_behavior_binding *binding,
                                                  struct zmk_behavior_binding_event event) {
    ARG_UNUSED(binding);
    ARG_UNUSED(event);
    return ZMK_BEHAVIOR_OPAQUE;
}

static const struct behavior_driver_api behavior_pmw3360_cpi_cycle_driver_api = {
    .binding_pressed = on_behavior_pmw3360_cpi_cycle_pressed,
    .binding_released = on_behavior_pmw3360_cpi_cycle_released,
#if IS_ENABLED(CONFIG_ZMK_BEHAVIOR_METADATA)
    .get_parameter_metadata = zmk_behavior_get_empty_param_metadata,
#endif
};

#define PMW3360_CPI_CYCLE_INST(n)                                                                  \
    static const struct behavior_pmw3360_cpi_cycle_config behavior_pmw3360_cpi_cycle_config_##n = { \
        .next = DT_INST_PROP(n, next),                                                             \
        .default_cpi = DT_INST_PROP(n, default_cpi),                                               \
    };                                                                                             \
    BEHAVIOR_DT_INST_DEFINE(n, behavior_pmw3360_cpi_cycle_init, NULL, NULL,                       \
                            &behavior_pmw3360_cpi_cycle_config_##n, POST_KERNEL,                  \
                            CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,                                   \
                            &behavior_pmw3360_cpi_cycle_driver_api);

DT_INST_FOREACH_STATUS_OKAY(PMW3360_CPI_CYCLE_INST)

#endif /* DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT) */
