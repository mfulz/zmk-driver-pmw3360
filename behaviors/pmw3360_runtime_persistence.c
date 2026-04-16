/*
 * Copyright (c) 2026 Matthias Fulz
 * SPDX-License-Identifier: MIT
 */

/**
 * @file pmw3360_runtime_persistence.c
 * @brief Persist and restore PMW3360 runtime tuning values via Zephyr settings.
 *
 * The PMW3360 cycle behaviors intentionally keep their live state in RAM so
 * repeated tuning does not hammer flash storage. This module adds the explicit
 * persistence path on top:
 *
 * - a keymap can invoke the save behavior once the current CPI and burst
 *   accumulation values are satisfactory
 * - the persisted values are restored after reboot, power loss, or firmware
 *   reflash as long as the keyboard settings storage is preserved
 *
 * Persistence is scoped to the convenience-behavior model used by this module:
 * one enabled PMW3360 device per build.
 */

#include <errno.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>

#include <zmk_pmw3360_behaviors.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#if IS_ENABLED(CONFIG_ZMK_BEHAVIOR_PMW3360_RUNTIME_PERSISTENCE)

#define PMW3360_RUNTIME_SETTINGS_VERSION 1
#define PMW3360_RUNTIME_SETTINGS_KEY "pmw3360/runtime"

struct pmw3360_runtime_settings_blob {
    uint8_t version;
    uint8_t burst_accumulation_max_samples;
    uint16_t cpi;
};

static struct pmw3360_runtime_settings_blob pending_runtime_settings;
static bool pending_runtime_settings_valid;

/**
 * Resolve the single PMW3360 device targeted by the convenience behaviors.
 *
 * @return Ready-to-use PMW3360 device handle when the build contains one,
 *         otherwise `NULL`.
 */
static const struct device *pmw3360_runtime_persistence_resolve_device(void) {
#if DT_HAS_COMPAT_STATUS_OKAY(pixart_pmw3360)
    return DEVICE_DT_GET(DT_COMPAT_GET_ANY_STATUS_OKAY(pixart_pmw3360));
#else
    return NULL;
#endif
}

/**
 * Return the devicetree default CPI for the single enabled PMW3360 device.
 *
 * @return Sensor default CPI, or `300` if no PMW3360 devicetree node exists.
 */
static uint16_t pmw3360_runtime_persistence_default_cpi(void) {
#if DT_HAS_COMPAT_STATUS_OKAY(pixart_pmw3360) && DT_NODE_HAS_PROP(DT_COMPAT_GET_ANY_STATUS_OKAY(pixart_pmw3360), cpi)
    return DT_PROP(DT_COMPAT_GET_ANY_STATUS_OKAY(pixart_pmw3360), cpi);
#else
    return 300;
#endif
}

/**
 * Return the devicetree default burst accumulation limit for the PMW3360.
 *
 * @return Configured burst accumulation limit, or `12` when the property is
 *         absent.
 */
static uint8_t pmw3360_runtime_persistence_default_burst_accumulation(void) {
#if DT_HAS_COMPAT_STATUS_OKAY(pixart_pmw3360) && DT_NODE_HAS_PROP(DT_COMPAT_GET_ANY_STATUS_OKAY(pixart_pmw3360), burst_accumulation_max_samples)
    return DT_PROP(DT_COMPAT_GET_ANY_STATUS_OKAY(pixart_pmw3360), burst_accumulation_max_samples);
#else
    return 12;
#endif
}

int zmk_pmw3360_save_runtime_settings(const struct device *pmw) {
    if (!pmw || !device_is_ready(pmw)) {
        return -ENODEV;
    }

    const uint16_t default_cpi = pmw3360_runtime_persistence_default_cpi();
    const uint8_t default_burst = pmw3360_runtime_persistence_default_burst_accumulation();

    const struct pmw3360_runtime_settings_blob blob = {
        .version = PMW3360_RUNTIME_SETTINGS_VERSION,
        .burst_accumulation_max_samples =
            zmk_behavior_pmw3360_get_burst_accumulation_max_samples_for_device(pmw, default_burst),
        .cpi = zmk_behavior_pmw3360_get_cpi_for_device(pmw, default_cpi),
    };

    const int rc = settings_save_one(PMW3360_RUNTIME_SETTINGS_KEY, &blob, sizeof(blob));
    if (rc < 0) {
        LOG_ERR("Failed to persist PMW3360 runtime settings (err %d)", rc);
        return rc;
    }

    LOG_INF("Persisted PMW3360 runtime settings: CPI=%u BA=%u", blob.cpi,
            blob.burst_accumulation_max_samples);
    return 0;
}

/**
 * Handle one persisted PMW3360 runtime settings blob from Zephyr settings.
 *
 * @param name Setting name relative to the handler root.
 * @param len Serialized blob size.
 * @param read_cb Settings framework read callback.
 * @param cb_arg Callback context passed through from Zephyr settings.
 *
 * @retval 0 The settings item was either accepted or ignored.
 * @retval -EINVAL The stored payload had an unexpected shape or version.
 * @retval negative errno Reading from settings failed.
 */
static int pmw3360_runtime_settings_handle_set(const char *name, size_t len, settings_read_cb read_cb,
                                               void *cb_arg) {
    const char *next;

    if (!settings_name_steq(name, "runtime", &next) || next) {
        return 0;
    }

    if (len != sizeof(struct pmw3360_runtime_settings_blob)) {
        LOG_ERR("Invalid PMW3360 runtime settings size (got %u expected %u)", (unsigned int)len,
                (unsigned int)sizeof(struct pmw3360_runtime_settings_blob));
        return -EINVAL;
    }

    struct pmw3360_runtime_settings_blob loaded = {0};
    const int err = read_cb(cb_arg, &loaded, sizeof(loaded));
    if (err <= 0) {
        LOG_ERR("Failed to load PMW3360 runtime settings (err %d)", err);
        return err;
    }

    if (loaded.version != PMW3360_RUNTIME_SETTINGS_VERSION) {
        LOG_WRN("Ignoring PMW3360 runtime settings version %u", loaded.version);
        return -EINVAL;
    }

    pending_runtime_settings = loaded;
    pending_runtime_settings_valid = true;
    LOG_INF("Loaded persisted PMW3360 runtime settings: CPI=%u BA=%u", loaded.cpi,
            loaded.burst_accumulation_max_samples);
    return 0;
}

/**
 * Apply persisted PMW3360 runtime settings after the Zephyr settings subsystem
 * has finished loading all values from storage.
 *
 * @retval 0 Nothing needed restoration or the persisted values were restored.
 * @retval negative errno One of the runtime setters rejected the persisted
 *         values.
 */
static int pmw3360_runtime_settings_commit(void) {
    if (!pending_runtime_settings_valid) {
        return 0;
    }

    const struct device *pmw = pmw3360_runtime_persistence_resolve_device();
    if (!pmw || !device_is_ready(pmw)) {
        LOG_WRN("Skipping PMW3360 runtime settings restore because the device is not ready");
        return 0;
    }

    const uint16_t default_cpi = pmw3360_runtime_persistence_default_cpi();
    const uint8_t default_burst = pmw3360_runtime_persistence_default_burst_accumulation();

    int rc = zmk_behavior_pmw3360_set_cpi_for_device(pmw, pending_runtime_settings.cpi, default_cpi);
    if (rc < 0) {
        LOG_ERR("Failed to restore persisted PMW3360 CPI (err %d)", rc);
        return rc;
    }

    rc = zmk_behavior_pmw3360_set_burst_accumulation_max_samples_for_device(
        pmw, pending_runtime_settings.burst_accumulation_max_samples, default_burst);
    if (rc < 0) {
        LOG_ERR("Failed to restore persisted PMW3360 burst accumulation (err %d)", rc);
        return rc;
    }

    LOG_INF("Restored persisted PMW3360 runtime settings");
    return 0;
}

static struct settings_handler pmw3360_runtime_settings_handler = {
    .name = "pmw3360",
    .h_set = pmw3360_runtime_settings_handle_set,
    .h_commit = pmw3360_runtime_settings_commit,
};

/**
 * Register the PMW3360 runtime settings handler during application startup.
 *
 * @retval 0 The settings handler was registered successfully.
 * @retval negative errno Settings handler registration failed.
 */
static int pmw3360_runtime_settings_init(void) {
    return settings_register(&pmw3360_runtime_settings_handler);
}

SYS_INIT(pmw3360_runtime_settings_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

#endif /* CONFIG_ZMK_BEHAVIOR_PMW3360_RUNTIME_PERSISTENCE */
