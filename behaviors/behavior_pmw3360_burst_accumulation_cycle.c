/*
 * Copyright (c) 2026 Matthias Fulz
 * SPDX-License-Identifier: MIT
 */

/**
 * @file behavior_pmw3360_burst_accumulation_cycle.c
 * @brief Ready-to-use runtime burst accumulation cycle behavior for the PMW3360.
 *
 * Burst accumulation controls how many PMW3360 motion bursts the driver may
 * combine into one HID report. On BLE builds this can be an important runtime
 * tuning parameter because it directly affects host-facing report pressure.
 */

#define DT_DRV_COMPAT zmk_behavior_pmw3360_burst_accumulation_cycle

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

static const uint8_t burst_accumulation_steps[] = {1, 4, 6, 12, 24, 48};

struct behavior_pmw3360_burst_accumulation_cycle_config {
    bool next;
    uint8_t default_max_samples;
};

struct pmw3360_burst_accumulation_state_entry {
    const struct device *pmw;
    struct k_work apply_work;
    uint8_t default_max_samples;
    uint8_t max_samples;
    int pending_delta;
    bool work_inited;
};

K_MUTEX_DEFINE(pmw3360_burst_accumulation_state_mutex);
static struct pmw3360_burst_accumulation_state_entry pmw3360_burst_accumulation_states[4];

/**
 * Resolve the first enabled PMW3360 device from devicetree.
 *
 * @return PMW3360 device instance when present in the build, otherwise `NULL`.
 */
static const struct device *behavior_pmw3360_burst_accumulation_cycle_resolve_device(void) {
#if DT_HAS_COMPAT_STATUS_OKAY(pixart_pmw3360)
    return DEVICE_DT_GET(DT_COMPAT_GET_ANY_STATUS_OKAY(pixart_pmw3360));
#else
    return NULL;
#endif
}

/**
 * Find or allocate the shared runtime state slot for one PMW3360 device.
 *
 * @param pmw PMW3360 device instance.
 * @param default_max_samples Initial burst accumulation value for new state.
 *
 * @return Existing or newly allocated state slot, or `NULL` if the fixed-size
 *         state table is exhausted.
 */
static struct pmw3360_burst_accumulation_state_entry *
pmw3360_burst_accumulation_state_get_or_create(const struct device *pmw,
                                               uint8_t default_max_samples) {
    struct pmw3360_burst_accumulation_state_entry *free_entry = NULL;

    for (size_t i = 0; i < ARRAY_SIZE(pmw3360_burst_accumulation_states); i++) {
        if (pmw3360_burst_accumulation_states[i].pmw == pmw) {
            return &pmw3360_burst_accumulation_states[i];
        }
        if (!pmw3360_burst_accumulation_states[i].pmw && !free_entry) {
            free_entry = &pmw3360_burst_accumulation_states[i];
        }
    }

    if (!free_entry) {
        return NULL;
    }

    free_entry->pmw = pmw;
    free_entry->default_max_samples = default_max_samples;
    free_entry->max_samples = default_max_samples;
    return free_entry;
}

uint8_t zmk_behavior_pmw3360_get_burst_accumulation_max_samples_for_device(
    const struct device *pmw, uint8_t fallback_max_samples) {
    if (!pmw) {
        return fallback_max_samples;
    }

    uint8_t max_samples = fallback_max_samples;
    k_mutex_lock(&pmw3360_burst_accumulation_state_mutex, K_FOREVER);
    for (size_t i = 0; i < ARRAY_SIZE(pmw3360_burst_accumulation_states); i++) {
        if (pmw3360_burst_accumulation_states[i].pmw == pmw) {
            max_samples = pmw3360_burst_accumulation_states[i].max_samples;
            break;
        }
    }
    k_mutex_unlock(&pmw3360_burst_accumulation_state_mutex);
    return max_samples;
}

/**
 * Find the preset index for one burst accumulation value.
 *
 * @param max_samples Burst accumulation limit to search for.
 *
 * @return Zero-based preset index, or `-1` if the value is not present in the
 *         built-in preset list.
 */
static int pmw3360_burst_accumulation_find_step_index(uint8_t max_samples) {
    for (size_t i = 0; i < ARRAY_SIZE(burst_accumulation_steps); i++) {
        if (burst_accumulation_steps[i] == max_samples) {
            return (int)i;
        }
    }

    return -1;
}

/**
 * Compute the next burst accumulation preset in the circular preset list.
 *
 * @param current_max_samples Current runtime value.
 * @param next `true` to move forward, `false` to move backward.
 * @param default_max_samples Default value used if the current runtime value is
 *                            not one of the built-in presets.
 *
 * @return The next preset value.
 */
static uint8_t pmw3360_burst_accumulation_cycle_value(uint8_t current_max_samples, bool next,
                                                      uint8_t default_max_samples) {
    int idx = pmw3360_burst_accumulation_find_step_index(current_max_samples);
    if (idx < 0) {
        idx = pmw3360_burst_accumulation_find_step_index(default_max_samples);
        if (idx < 0) {
            idx = 0;
        }

        return burst_accumulation_steps[idx];
    }

    if (next) {
        idx = (idx + 1) % (int)ARRAY_SIZE(burst_accumulation_steps);
    } else {
        idx = (idx + (int)ARRAY_SIZE(burst_accumulation_steps) - 1)
              % (int)ARRAY_SIZE(burst_accumulation_steps);
    }

    return burst_accumulation_steps[idx];
}

/**
 * Serialize pending burst accumulation changes and apply them to the PMW3360.
 *
 * @param work Worker embedded in one per-device state entry.
 */
static void pmw3360_burst_accumulation_apply_work_handler(struct k_work *work) {
    struct pmw3360_burst_accumulation_state_entry *state =
        CONTAINER_OF(work, struct pmw3360_burst_accumulation_state_entry, apply_work);

    for (;;) {
        bool next = false;
        uint8_t current_max_samples = 0;
        uint8_t new_max_samples = 0;

        k_mutex_lock(&pmw3360_burst_accumulation_state_mutex, K_FOREVER);

        if (state->pending_delta == 0) {
            k_mutex_unlock(&pmw3360_burst_accumulation_state_mutex);
            return;
        }

        next = state->pending_delta > 0;
        current_max_samples = state->max_samples;
        new_max_samples = pmw3360_burst_accumulation_cycle_value(
            current_max_samples, next, state->default_max_samples);

        const int rc =
            zmk_driver_pmw3360_set_burst_accumulation_max_samples(state->pmw, new_max_samples);
        if (rc < 0) {
            k_mutex_unlock(&pmw3360_burst_accumulation_state_mutex);
            LOG_ERR("Failed to set PMW3360 burst accumulation=%u: %d", new_max_samples, rc);
            return;
        }

        state->max_samples = new_max_samples;
        state->pending_delta += next ? -1 : 1;
        k_mutex_unlock(&pmw3360_burst_accumulation_state_mutex);

        LOG_INF("PMW3360 burst accumulation %s: %u -> %u", next ? "NEXT" : "PREV",
                current_max_samples, new_max_samples);
        zmk_pmw3360_runtime_setting_changed(state->pmw,
                                            ZMK_PMW3360_RUNTIME_SETTING_BURST_ACCUMULATION,
                                            new_max_samples);
    }
}

/**
 * Initialize one burst accumulation cycle behavior instance.
 *
 * @param dev Behavior device instance.
 *
 * @retval 0 Initialization completed successfully.
 */
static int behavior_pmw3360_burst_accumulation_cycle_init(const struct device *dev) {
    const struct behavior_pmw3360_burst_accumulation_cycle_config *cfg = dev->config;
    const struct device *pmw = behavior_pmw3360_burst_accumulation_cycle_resolve_device();

    if (!pmw) {
        return 0;
    }

    k_mutex_lock(&pmw3360_burst_accumulation_state_mutex, K_FOREVER);
    struct pmw3360_burst_accumulation_state_entry *state =
        pmw3360_burst_accumulation_state_get_or_create(pmw, cfg->default_max_samples);
    if (state && !state->work_inited) {
        k_work_init(&state->apply_work, pmw3360_burst_accumulation_apply_work_handler);
        state->work_inited = true;
    }
    k_mutex_unlock(&pmw3360_burst_accumulation_state_mutex);

    return 0;
}

/**
 * Handle one burst accumulation cycle key press.
 *
 * @param binding Active behavior binding.
 * @param event ZMK binding event metadata.
 *
 * @retval ZMK_BEHAVIOR_OPAQUE The event was consumed.
 * @retval -ENODEV No ready PMW3360 device was found.
 * @retval -ENOMEM No free runtime state slot was available.
 */
static int on_behavior_pmw3360_burst_accumulation_cycle_pressed(
    struct zmk_behavior_binding *binding, struct zmk_behavior_binding_event event) {
    ARG_UNUSED(event);

    const struct device *dev = zmk_behavior_get_binding(binding->behavior_dev);
    const struct behavior_pmw3360_burst_accumulation_cycle_config *cfg = dev->config;
    const struct device *pmw = behavior_pmw3360_burst_accumulation_cycle_resolve_device();

    if (!pmw || !device_is_ready(pmw)) {
        LOG_WRN("PMW3360 device not ready");
        return -ENODEV;
    }

    k_mutex_lock(&pmw3360_burst_accumulation_state_mutex, K_FOREVER);
    struct pmw3360_burst_accumulation_state_entry *state =
        pmw3360_burst_accumulation_state_get_or_create(pmw, cfg->default_max_samples);
    if (!state) {
        k_mutex_unlock(&pmw3360_burst_accumulation_state_mutex);
        LOG_ERR("No PMW3360 burst accumulation state slots available");
        return -ENOMEM;
    }

    if (!state->work_inited) {
        k_work_init(&state->apply_work, pmw3360_burst_accumulation_apply_work_handler);
        state->work_inited = true;
    }

    state->pending_delta += cfg->next ? 1 : -1;
    struct k_work *apply_work = &state->apply_work;
    k_mutex_unlock(&pmw3360_burst_accumulation_state_mutex);

    (void)k_work_submit(apply_work);
    return ZMK_BEHAVIOR_OPAQUE;
}

/**
 * Burst accumulation cycle acts on key press only.
 *
 * @retval ZMK_BEHAVIOR_OPAQUE Always consumes the release event.
 */
static int on_behavior_pmw3360_burst_accumulation_cycle_released(
    struct zmk_behavior_binding *binding, struct zmk_behavior_binding_event event) {
    ARG_UNUSED(binding);
    ARG_UNUSED(event);
    return ZMK_BEHAVIOR_OPAQUE;
}

static const struct behavior_driver_api behavior_pmw3360_burst_accumulation_cycle_driver_api = {
    .binding_pressed = on_behavior_pmw3360_burst_accumulation_cycle_pressed,
    .binding_released = on_behavior_pmw3360_burst_accumulation_cycle_released,
#if IS_ENABLED(CONFIG_ZMK_BEHAVIOR_METADATA)
    .get_parameter_metadata = zmk_behavior_get_empty_param_metadata,
#endif
};

#define PMW3360_BURST_ACCUMULATION_CYCLE_INST(n)                                                   \
    static const struct behavior_pmw3360_burst_accumulation_cycle_config                           \
        behavior_pmw3360_burst_accumulation_cycle_config_##n = {                                   \
            .next = DT_INST_PROP(n, next),                                                         \
            .default_max_samples = DT_INST_PROP(n, default_max_samples),                           \
        };                                                                                         \
    BEHAVIOR_DT_INST_DEFINE(n, behavior_pmw3360_burst_accumulation_cycle_init, NULL, NULL,        \
                            &behavior_pmw3360_burst_accumulation_cycle_config_##n, POST_KERNEL,   \
                            CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,                                   \
                            &behavior_pmw3360_burst_accumulation_cycle_driver_api);

DT_INST_FOREACH_STATUS_OKAY(PMW3360_BURST_ACCUMULATION_CYCLE_INST)

#endif /* DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT) */
