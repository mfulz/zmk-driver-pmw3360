/*
 * Copyright (c) 2025 George Norton
 * Copyright (c) 2026 Matthias Fulz
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT pixart_pmw3360

#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/input/input.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include "input_pmw3360.h"

#include <zmk_driver_pmw3360.h>

LOG_MODULE_REGISTER(pmw3360, CONFIG_INPUT_LOG_LEVEL);

#if defined(CONFIG_INPUT_PIXART_PMW3360_USE_OWN_THREAD)
    K_THREAD_STACK_DEFINE(pmw3360_stack, CONFIG_INPUT_PIXART_PMW3360_THREAD_STACK_SIZE);
#endif

#define PMW3360_CPI_WRITE_RETRIES 3
#define PMW3360_INIT_RETRIES 3
#define PMW3360_SPI_PORT_RESET_DELAY_US 40
#define PMW3360_POWER_CYCLE_DELAY_MS 20
#define PMW3360_BURST_ACCUMULATION_MIN_SAMPLES 1
#define PMW3360_BURST_ACCUMULATION_MAX_SAMPLES 64

/**
 * Enable or disable the PMW3360 motion interrupt line.
 *
 * This helper is only used when the sensor has a wired MOTION pin. In polling
 * mode the function is never called because there is no interrupt source to
 * configure.
 *
 * @param dev PMW3360 device instance.
 * @param en `true` to enable level-triggered motion interrupts, `false` to
 *           disable them.
 *
 * @retval 0 Interrupt state was changed successfully.
 * @retval negative errno GPIO interrupt configuration failed.
 */
static int pmw3360_set_interrupt(const struct device *dev, const bool en) {
    const struct pmw3360_config *config = dev->config;
    int err = gpio_pin_interrupt_configure_dt(&config->irq_gpio,
                                                en ? GPIO_INT_LEVEL_ACTIVE : GPIO_INT_DISABLE);
    if (err < 0) {
        LOG_ERR("Can't set interrupt");
    }
    return err;
}

/**
 * Toggle external power to the PMW3360 sensor.
 *
 * This helper only acts when `power-gpios` is configured in devicetree. Boards
 * without an external power gate simply return success so the rest of the
 * driver can use one code path for both powered and always-on designs.
 *
 * @param dev PMW3360 device instance.
 * @param en `true` to request sensor power on, `false` to request power off.
 *
 * @retval 0 Power was toggled successfully or no power gate exists.
 * @retval -ENODEV The configured GPIO controller is not ready.
 * @retval negative errno GPIO configuration or set operation failed.
 */
static int pmw3360_set_power(const struct device *dev, const bool en) {
    const struct pmw3360_config *config = dev->config;
    if (!config->power_gpio.port) {
        return 0;
    }

    if (!device_is_ready(config->power_gpio.port)) {
        LOG_ERR("Power GPIO device not ready");
        return -ENODEV;
    }

    /* Ensure pin is an output. */
    int err = gpio_pin_configure_dt(&config->power_gpio, GPIO_OUTPUT_INACTIVE);
    if (err < 0) {
        LOG_ERR("Can't configure power gpio");
        return err;
    }

    err = gpio_pin_set_dt(&config->power_gpio, en ? 1 : 0);
    if (err < 0) {
        LOG_ERR("Can't set power gpio");
    }
    return err;
}

/**
 * Reset the PMW3360 SPI state machine by driving NCS high and then low.
 *
 * The datasheet describes this in physical line levels, not GPIO logical
 * active/inactive semantics. The chip select on this board is active-low, so
 * using `gpio_pin_set_dt(..., 0/1)` would be inverted. We therefore drive the
 * raw GPIO line directly.
 *
 * @param dev The PMW3360 device.
 *
 * @retval 0 if the pulse sequence was applied successfully.
 * @retval negative errno if the chip-select line could not be driven.
 */
static int pmw3360_reset_spi_port(const struct device *dev) {
    const struct pmw3360_config *config = dev->config;

    if (!config->cs_gpio.port) {
        return -ENODEV;
    }

    if (!device_is_ready(config->cs_gpio.port)) {
        LOG_ERR("PMW3360 CS GPIO device not ready");
        return -ENODEV;
    }

    int err = gpio_pin_set_raw(config->cs_gpio.port, config->cs_gpio.pin, 1);
    if (err < 0) {
        LOG_ERR("Failed to drive PMW3360 CS high: %d", err);
        return err;
    }

    k_usleep(PMW3360_SPI_PORT_RESET_DELAY_US);

    err = gpio_pin_set_raw(config->cs_gpio.port, config->cs_gpio.pin, 0);
    if (err < 0) {
        LOG_ERR("Failed to drive PMW3360 CS low: %d", err);
        return err;
    }

    k_usleep(PMW3360_SPI_PORT_RESET_DELAY_US);
    return 0;
}

/**
 * Read one or more bytes from a PMW3360 register address over SPI.
 *
 * The PMW3360 read protocol requires a write of the register address, a
 * datasheet-defined wait period, and then a separate read transaction while the
 * chip select remains asserted.
 *
 * @param dev PMW3360 device instance.
 * @param addr Register address to read from.
 * @param buf Destination buffer for the read payload.
 * @param len Number of bytes to read into `buf`.
 * @param address_wait Datasheet-mandated delay between sending the register
 *                     address and clocking out the response payload.
 *
 * @retval 0 SPI read completed successfully.
 * @retval negative errno An SPI write or read transaction failed.
 */
static int pmw3360_spi_read(const struct device *dev, const uint8_t addr, uint8_t *buf, const uint8_t len, const int32_t address_wait) {
    const struct pmw3360_config *config = dev->config;
    uint8_t tx_buffer[1] = { PMW3360_SPI_READ | addr };

    /* Send the register address that should be read next. */
    const struct spi_buf tx_buf[1] = {
        {
        .buf = tx_buffer,
        .len = 1,
        }
    };
    const struct spi_buf_set tx = {
        .buffers = tx_buf,
        .count = 1,
    };
    
    int err = spi_write_dt(&config->spi, &tx);
    if (err < 0) {
        LOG_ERR("Error writing the SPI read-address: %d", err);
        spi_release_dt(&config->spi);
        return err;
    }

    /* Wait the datasheet-specified delay before clocking out the payload. */
    k_usleep(address_wait);

    /* Read the response payload while chip select remains asserted. */
    struct spi_buf rx_buf[1] = {
        {
        .buf = buf,
        .len = len,
        },
    };
    const struct spi_buf_set rx = {
        .buffers = rx_buf,
        .count = 1,
    };
    err = spi_read_dt(&config->spi, &rx);
    if (err != 0) {
        LOG_ERR("Error reading the SPI payload: %d", err);
        spi_release_dt(&config->spi);
        return err;
    }

    /* Respect the NCS release timing before ending the transaction. */
    k_usleep(PMW3360_T_SCLK_NCS_READ);

    spi_release_dt(&config->spi);

    return err;
}

/**
 * Write a single PMW3360 register over SPI.
 *
 * Register writes invalidate motion burst mode, so the driver marks the burst
 * state as inactive before performing the transaction.
 *
 * @param dev PMW3360 device instance.
 * @param addr Register address to write.
 * @param val Value to store in the target register.
 *
 * @retval 0 Register write succeeded.
 * @retval negative errno The SPI transceive operation failed.
 */
static int pmw3360_spi_write_reg(const struct device *dev, const uint8_t addr, const uint8_t val) {
    struct pmw3360_data *data = dev->data;
    const struct pmw3360_config *config = dev->config;

    data->motion_burst_active = false;

    /* Write address and payload while simultaneously draining dummy RX bytes. */
    uint8_t tx_buffer[2] = {PMW3360_SPI_WRITE | addr, val};
    uint8_t rx_buffer[2] = {};

    const struct spi_buf tx_buf = {
        .buf = tx_buffer,
        .len = 2,
    };
    const struct spi_buf_set tx = {
        .buffers = &tx_buf,
        .count = 1,
    };

    const struct spi_buf rx_buf = {
        .buf = rx_buffer,
        .len = 2,
    };
    const struct spi_buf_set rx = {
        .buffers = &rx_buf,
        .count = 1,
    };

    const int err = spi_transceive_dt(&config->spi, &tx, &rx);
    if (err < 0) {
        LOG_ERR("spi err: %d", err);
    }

    /* Respect the NCS release timing before ending the transaction. */
    k_usleep(PMW3360_T_SCLK_NCS_WRITE);

    spi_release_dt(&config->spi);

    /* Respect the inter-transaction delay required by the datasheet. */
    k_usleep(PMW3360_T_SWR);

    return err;
}

/**
 * Read a single PMW3360 register and exit motion burst mode if necessary.
 *
 * @param dev PMW3360 device instance.
 * @param addr Register address to read.
 * @param val Destination for the register value.
 *
 * @retval 0 Register read succeeded.
 * @retval negative errno The SPI read sequence failed.
 */
static int pmw3360_spi_read_reg(const struct device *dev, const uint8_t addr, uint8_t *val) {
    int err = 0;
    struct pmw3360_data *data = dev->data;

    data->motion_burst_active = false;

    err = pmw3360_spi_read(dev, addr, val, 1, PMW3360_T_SRAD);
    /* Respect the inter-transaction delay required by the datasheet. */
    k_usleep(PMW3360_T_SRR);
    return err;
}

/**
 * Read one PMW3360 motion burst payload.
 *
 * The PMW3360 keeps a stateful burst mode between reads. The first read after a
 * configuration change must re-arm burst mode by writing to the motion burst
 * register before the actual payload read can occur.
 *
 * @param dev PMW3360 device instance.
 * @param val Destination buffer for the motion burst payload.
 * @param len Size of the motion burst payload to read.
 *
 * @retval 0 Motion burst read succeeded.
 * @retval negative errno SPI access failed.
 */
static int pmw3360_spi_read_motion_burst(const struct device *dev, uint8_t *val, const uint8_t len) {
    int err = 0;
    struct pmw3360_data *data = dev->data;

    if (!data->motion_burst_active) {
        /* Any write re-arms PMW3360 burst mode before the next burst read. */
        pmw3360_spi_write_reg(dev, PMW3360_REG_MOTION_BURST, 0);
        data->motion_burst_active = true;
    }
    err = pmw3360_spi_read(dev, PMW3360_REG_MOTION_BURST, val, len, PMW3360_T_SRAD_MOTBR);
    /* The datasheet asks for 500 ns; 1 us is the closest practical delay here. */
    k_usleep(1);
    return err;
}

/**
 * Convert a CPI value expressed in user-facing units into the PMW3360 register
 * encoding used by CONFIG_1.
 *
 * @param cpi CPI value already normalized to the driver's supported 100-step
 *            scale.
 *
 * @return Encoded CONFIG_1 register value for the requested CPI.
 */
static uint8_t pmw3360_config_1_reg_from_cpi(const uint16_t cpi) {
    return (uint8_t)(((uint32_t)cpi / 100u) - 1u);
}

/**
 * Convert the PMW3360 CONFIG_1 register encoding back into a CPI value.
 *
 * @param reg Raw CONFIG_1 register value.
 *
 * @return CPI expressed in user-facing counts per inch.
 */
static uint16_t pmw3360_cpi_from_config_1_reg(const uint8_t reg) {
    return (uint16_t)(((uint16_t)reg + 1u) * 100u);
}

/**
 * Clamp a runtime burst accumulation request into the driver's supported range.
 *
 * The lower bound of `1` disables burst accumulation and emits one HID report
 * per PMW3360 motion burst. Higher values allow more bursts to be summed into a
 * single host-visible report.
 *
 * @param samples Requested burst accumulation limit.
 *
 * @return Normalized burst accumulation limit within the driver's supported
 *         runtime range.
 */
static uint8_t pmw3360_normalize_burst_accumulation_max_samples(uint32_t samples) {
    samples = CLAMP(samples, PMW3360_BURST_ACCUMULATION_MIN_SAMPLES,
                    PMW3360_BURST_ACCUMULATION_MAX_SAMPLES);
    return (uint8_t)samples;
}

/*
 * CPI changes must be serialized against motion burst reads.
 *
 * The PMW3360 keeps stateful burst mode between reads. When CPI writes race
 * with active motion traffic, the host-side state/OSD can advance while the
 * sensor keeps the previous CONFIG_1 value. To make CPI switching truthful, we
 * write and then immediately read back CONFIG_1 under the driver's mutex.
 */
static int pmw3360_apply_cpi_locked(const struct device *dev, const uint16_t cpi, uint16_t *applied_cpi) {
    const uint8_t expected_reg = pmw3360_config_1_reg_from_cpi(cpi);

    for (int attempt = 1; attempt <= PMW3360_CPI_WRITE_RETRIES; attempt++) {
        int err = pmw3360_spi_write_reg(dev, PMW3360_REG_CONFIG_1, expected_reg);
        if (err < 0) {
            LOG_WRN("Failed to write CPI=%u on attempt %d: %d", cpi, attempt, err);
            continue;
        }

        uint8_t readback = 0;
        err = pmw3360_spi_read_reg(dev, PMW3360_REG_CONFIG_1, &readback);
        if (err < 0) {
            LOG_WRN("Failed to read back CPI=%u on attempt %d: %d", cpi, attempt, err);
            continue;
        }

        if (readback == expected_reg) {
            if (applied_cpi) {
                *applied_cpi = pmw3360_cpi_from_config_1_reg(readback);
            }
            return 0;
        }

        LOG_WRN("CPI write mismatch on attempt %d: wanted %u (reg=0x%02x), read reg=0x%02x",
                attempt, cpi, expected_reg, readback);
    }

    return -EIO;
}

/**
 * GPIO interrupt callback for the PMW3360 MOTION pin.
 *
 * The callback disables the interrupt line immediately and schedules a work item
 * that will drain one or more sensor motion bursts. Interrupts are re-enabled
 * later by the work handler after the queued motion has been processed.
 *
 * @param gpiob GPIO controller that observed the interrupt.
 * @param cb Driver-owned GPIO callback descriptor.
 * @param pins Bit mask of interrupting GPIO lines.
 */
static void pmw3360_gpio_callback(const struct device *gpiob, struct gpio_callback *cb, uint32_t pins) {
    ARG_UNUSED(gpiob);
    ARG_UNUSED(pins);
    struct pmw3360_data *data = CONTAINER_OF(cb, struct pmw3360_data, irq_gpio_cb);
    const struct device *dev = data->dev;
    if (!data->enabled) {
        pmw3360_set_interrupt(dev, false);
        return;
    }
    pmw3360_set_interrupt(dev, false);
#if defined(CONFIG_INPUT_PIXART_PMW3360_USE_OWN_THREAD)
    k_work_reschedule_for_queue(&data->driver_work_queue, &data->motion_work, K_NO_WAIT);
#else
    k_work_reschedule(&data->motion_work, K_NO_WAIT);
#endif
}

/**
 * Configure the PMW3360 motion interrupt pin and register the callback.
 *
 * When no IRQ pin is defined in devicetree, the function returns `-ENODEV` so
 * the caller can switch into polling mode instead.
 *
 * @param dev PMW3360 device instance.
 *
 * @retval 0 IRQ pin and callback were configured successfully.
 * @retval -ENODEV No IRQ GPIO is available or the GPIO controller is not ready.
 * @retval negative errno GPIO configuration or callback registration failed.
 */
static int pmw3360_init_irq(const struct device *dev) {
    int err = 0;
    struct pmw3360_data *data = dev->data;
    const struct pmw3360_config *config = dev->config;

    if (!config->irq_gpio.port) {
        return -ENODEV;
    }

    /* Ensure the GPIO controller behind the MOTION line is available. */
    if (!device_is_ready(config->irq_gpio.port)) {
        LOG_ERR("IRQ GPIO device not ready");
        return -ENODEV;
    }

    /* Configure the PMW3360 MOTION line as input before adding the callback. */
    err = gpio_pin_configure_dt(&config->irq_gpio, GPIO_INPUT);
    if (err) {
        LOG_ERR("Cannot configure IRQ GPIO");
        return err;
    }

    /* Register the callback that schedules motion processing work. */
    gpio_init_callback(&data->irq_gpio_cb, pmw3360_gpio_callback, BIT(config->irq_gpio.pin));
    err = gpio_add_callback(config->irq_gpio.port, &data->irq_gpio_cb);
    if (err) {
        LOG_ERR("Cannot add IRQ GPIO callback");
    }

    return err;
}

/**
 * Drain PMW3360 motion bursts and emit a single HID report.
 *
 * The PMW3360 can generate motion bursts faster than a BLE pointing path can
 * always forward them one-by-one. To prevent host-visible backlog, the driver
 * sums several PMW3360 motion bursts into one HID report. The accumulation
 * window is capped by `burst_accumulation_max_samples`.
 *
 * @param dev PMW3360 device instance.
 */
static void pmw3360_read_motion_report(const struct device *dev) {
    const struct pmw3360_config *config = dev->config;
    struct pmw3360_data *data = dev->data;
    int32_t dx_total = 0;
    int32_t dy_total = 0;
    int samples = 0;
    const uint8_t max_samples =
        data->burst_accumulation_max_samples ? data->burst_accumulation_max_samples : 1;

    /*
     * In IRQ mode, a fast trackball can easily generate more motion interrupts
     * than the BLE mouse report path can drain one-by-one. If we emit a full
     * HID report for every single burst read, backlog accumulates and later
     * gets flushed as visible lag. Drain and accumulate a bounded number of
     * PMW motion bursts per work run so one IRQ work item maps to one HID
     * report instead of a report storm.
     */
    for (uint8_t attempt = 0; attempt < max_samples; attempt++) {
        struct motion_burst motion_report = {};
        int err =
            pmw3360_spi_read_motion_burst(dev, (uint8_t *)&motion_report, sizeof(motion_report));
        if ((err != 0) || (motion_report.motion == 0xff)) {
            /* If burst mode became deactivated, reactivate once and stop. */
            data->motion_burst_active = false;
            break;
        }

        if (!(motion_report.motion & PMW3360_MOTION_MOT)) {
            break;
        }

        int16_t dx = (motion_report.delta_x_h << 8) | motion_report.delta_x_l;
        int16_t dy = (motion_report.delta_y_h << 8) | motion_report.delta_y_l;

        if (config->invert_x) {
            dx = -dx;
        }
        if (config->invert_y) {
            dy = -dy;
        }

        dx_total += dx;
        dy_total += dy;
        samples++;
    }

    if (samples > 0) {
        dx_total = CLAMP(dx_total, INT16_MIN, INT16_MAX);
        dy_total = CLAMP(dy_total, INT16_MIN, INT16_MAX);
        input_report_rel(dev, INPUT_REL_X, (int16_t)dx_total, false, K_FOREVER);
        input_report_rel(dev, INPUT_REL_Y, (int16_t)dy_total, true, K_FOREVER);

        if (samples > 1) {
            LOG_DBG("Accumulated %d PMW burst samples into one report (%d, %d)", samples,
                    dx_total, dy_total);
        }
    }
}

/**
 * Workqueue callback that handles PMW3360 motion processing.
 *
 * In interrupt-driven mode the work item drains pending motion and then
 * re-enables the MOTION interrupt. In polling mode it additionally reschedules
 * itself according to the configured polling interval.
 *
 * @param work Delayable work item associated with PMW3360 motion processing.
 */
static void pmw3360_work_callback(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct pmw3360_data *data = CONTAINER_OF(dwork, struct pmw3360_data, motion_work);
    const struct device *dev = data->dev;

    /*
     * Safety: The motion work can be in-flight while the sensor is being power
     * gated or re-initialized (mouse layer toggles). If we read/reschedule when
     * disabled/not-ready, we can end up hammering SPI while the chip is off and
     * keep re-queuing work forever.
     */
    bool enabled = false;
    bool ready = false;
    bool polling_mode = false;
    uint32_t polling_interval = 0;
    k_mutex_lock(&data->mutex, K_FOREVER);
    enabled = data->enabled;
    ready = data->ready;
    if (enabled && ready) {
        polling_mode = data->polling_mode;
        if (polling_mode) {
            const struct pmw3360_config *config = dev->config;
            polling_interval = config->polling_interval;
        }
        pmw3360_read_motion_report(dev);
    }
    k_mutex_unlock(&data->mutex);

    if (!enabled || !ready) {
        return;
    }

    if (polling_mode) {
#if defined(CONFIG_INPUT_PIXART_PMW3360_USE_OWN_THREAD)
        k_work_reschedule_for_queue(&data->driver_work_queue, dwork, K_USEC(polling_interval));
#else
        k_work_reschedule(dwork, K_USEC(polling_interval));
#endif
    }
    else {
        if (enabled) {
            pmw3360_set_interrupt(dev, true);
        }
    }
}

/**
 * Read the enabled flag while holding the driver's mutex.
 *
 * @param dev PMW3360 device instance.
 *
 * @return `true` when the sensor is expected to be powered and active.
 */
static bool pmw3360_is_enabled(const struct device *dev) {
    struct pmw3360_data *data = dev->data;
    bool enabled = false;
    k_mutex_lock(&data->mutex, K_FOREVER);
    enabled = data->enabled;
    k_mutex_unlock(&data->mutex);
    return enabled;
}

/**
 * Run the PMW3360 initialization sequence asynchronously.
 *
 * This routine is intentionally executed from a work item after the sensor has
 * been powered on. That gives the external power rail and PMW3360 oscillator a
 * short settling period before the SPI reset and register programming sequence
 * begins.
 *
 * @param work Delayable work item associated with deferred sensor
 *             initialization.
 */
static void pmw3360_async_init(struct k_work *work) {
    struct k_work_delayable *work_delayable = (struct k_work_delayable *)work;
    struct pmw3360_data *data = CONTAINER_OF(work_delayable, struct pmw3360_data, init_work);
    const struct device *dev = data->dev;
    const struct pmw3360_config *config = dev->config;

    if (!pmw3360_is_enabled(dev)) {
        return;
    }

    for (int attempt = 1; attempt <= PMW3360_INIT_RETRIES; attempt++) {
        int err = pmw3360_reset_spi_port(dev);
        if (err < 0) {
            LOG_WRN("PMW3360 init attempt %d/%d: SPI port reset failed: %d",
                    attempt, PMW3360_INIT_RETRIES, err);
            goto retry;
        }

        err = pmw3360_spi_write_reg(dev, PMW3360_REG_POWER_UP, 0x5A);
        if (err < 0) {
            LOG_WRN("PMW3360 init attempt %d/%d: power-up reset write failed: %d",
                    attempt, PMW3360_INIT_RETRIES, err);
            goto retry;
        }

        k_msleep(50);
        if (!pmw3360_is_enabled(dev)) {
            return;
        }

        for (int reg = PMW3360_REG_MOTION; reg <= PMW3360_REG_DELTA_Y_H; reg++) {
            uint8_t value = 0;
            err = pmw3360_spi_read_reg(dev, reg, &value);
            if (err < 0) {
                LOG_WRN("PMW3360 init attempt %d/%d: warm-up read reg 0x%02x failed: %d",
                        attempt, PMW3360_INIT_RETRIES, reg, err);
                goto retry;
            }
        }

        uint8_t product_id = 0;
        err = pmw3360_spi_read_reg(dev, PMW3360_REG_PRODUCT_ID, &product_id);
        if (err < 0) {
            LOG_WRN("PMW3360 init attempt %d/%d: product-id read failed: %d",
                    attempt, PMW3360_INIT_RETRIES, err);
            goto retry;
        }

        uint8_t revision_id = 0;
        err = pmw3360_spi_read_reg(dev, PMW3360_REG_REVISION_ID, &revision_id);
        if (err < 0) {
            LOG_WRN("PMW3360 init attempt %d/%d: revision-id read failed: %d",
                    attempt, PMW3360_INIT_RETRIES, err);
            goto retry;
        }

        LOG_INF("PMW3360 init attempt %d/%d: product=0x%02x revision=0x%02x",
                attempt, PMW3360_INIT_RETRIES, product_id, revision_id);

        uint8_t control = 0;
        if (config->rotate_90) {
            if (config->rotate_180 || config->rotate_270) {
                LOG_ERR("Multiple rotations specified, configuring 90 degrees");
            }
            control = PMW3360_CONTROL_ROTATE_90;
        } else if (config->rotate_180) {
            if (config->rotate_270) {
                LOG_ERR("Multiple rotations specified, configuring 180 degrees");
            }
            control = PMW3360_CONTROL_ROTATE_180;
        } else if (config->rotate_270) {
            control = PMW3360_CONTROL_ROTATE_270;
        }

        if (control != 0) {
            err = pmw3360_spi_write_reg(dev, PMW3360_REG_CONTROL, control);
            if (err < 0) {
                LOG_WRN("PMW3360 init attempt %d/%d: control write failed: %d",
                        attempt, PMW3360_INIT_RETRIES, err);
                goto retry;
            }
        }

        k_mutex_lock(&data->mutex, K_FOREVER);
        if (!data->enabled) {
            k_mutex_unlock(&data->mutex);
            return;
        }

        uint16_t cpi = data->cpi ? data->cpi : config->cpi;
        err = pmw3360_apply_cpi_locked(dev, cpi, NULL);
        k_mutex_unlock(&data->mutex);
        if (err < 0) {
            LOG_WRN("PMW3360 init attempt %d/%d: failed to apply CPI=%u: %d",
                    attempt, PMW3360_INIT_RETRIES, cpi, err);
            goto retry;
        }

        if (!config->force_awake) {
            err = pmw3360_spi_write_reg(dev, PMW3360_REG_CONFIG_2, PMW3360_CONFIG_2_REST_EN);
            if (err < 0) {
                LOG_WRN("PMW3360 init attempt %d/%d: rest-mode write failed: %d",
                        attempt, PMW3360_INIT_RETRIES, err);
                goto retry;
            }
        } else {
            /*
             * `force-awake` must actively clear CONFIG_2, not merely skip the
             * REST_EN write. Otherwise the chip keeps whatever power-up default
             * or previous state it had, which defeats the purpose of using the
             * external power gate as the only sleep mechanism.
             */
            err = pmw3360_spi_write_reg(dev, PMW3360_REG_CONFIG_2, 0x00);
            if (err < 0) {
                LOG_WRN("PMW3360 init attempt %d/%d: force-awake config write failed: %d",
                        attempt, PMW3360_INIT_RETRIES, err);
                goto retry;
            }
            LOG_INF("PMW3360 init attempt %d/%d: force-awake enabled; CONFIG_2 cleared",
                    attempt, PMW3360_INIT_RETRIES);
        }

        err = pmw3360_spi_write_reg(dev, PMW3360_REG_ANGLE_TUNE, config->angle_tune);
        if (err < 0) {
            LOG_WRN("PMW3360 init attempt %d/%d: angle-tune write failed: %d",
                    attempt, PMW3360_INIT_RETRIES, err);
            goto retry;
        }

        if (config->lift_height_3mm) {
            err = pmw3360_spi_write_reg(dev, PMW3360_REG_LIFT_CONFIG, PMW3360_LIFT_CONFIG_3MM);
            if (err < 0) {
                LOG_WRN("PMW3360 init attempt %d/%d: lift-config write failed: %d",
                        attempt, PMW3360_INIT_RETRIES, err);
                goto retry;
            }
        }

        k_mutex_lock(&data->mutex, K_FOREVER);
        data->ready = true;
        k_mutex_unlock(&data->mutex);

        goto init_complete;

retry:
        k_mutex_lock(&data->mutex, K_FOREVER);
        data->ready = false;
        data->motion_burst_active = false;
        k_mutex_unlock(&data->mutex);

        if (!pmw3360_is_enabled(dev) || attempt >= PMW3360_INIT_RETRIES) {
            break;
        }

        if (config->irq_gpio.port) {
            (void)pmw3360_set_interrupt(dev, false);
        }

        (void)pmw3360_set_power(dev, false);
        k_msleep(PMW3360_POWER_CYCLE_DELAY_MS);
        if (!pmw3360_is_enabled(dev)) {
            return;
        }

        err = pmw3360_set_power(dev, true);
        if (err < 0) {
            LOG_WRN("PMW3360 init retry %d/%d: power-on failed: %d",
                    attempt + 1, PMW3360_INIT_RETRIES, err);
            break;
        }

        k_msleep(100);
    }

    LOG_ERR("PMW3360 init failed after %d attempts; sensor stays not-ready", PMW3360_INIT_RETRIES);
    return;

init_complete:
    if (data->polling_mode) {
        struct k_work_delayable *dwork = &data->motion_work;
#if defined(CONFIG_INPUT_PIXART_PMW3360_USE_OWN_THREAD)
        k_work_reschedule_for_queue(&data->driver_work_queue, dwork, K_USEC(config->polling_interval));
#else
        k_work_reschedule(dwork, K_USEC(config->polling_interval));
#endif
    }
    else {
        pmw3360_set_interrupt(dev, true);
    }

}

/**
 * Initialize one PMW3360 Zephyr device instance.
 *
 * The driver performs one-time initialization of mutexes, work items, the
 * optional dedicated work queue, and the optional MOTION interrupt path. The
 * sensor itself remains disabled until a caller explicitly enables it at
 * runtime via `zmk_driver_pmw3360_set_enabled()`.
 *
 * @param dev PMW3360 device instance.
 *
 * @retval 0 Driver-side initialization completed successfully.
 */
static int pmw3360_init(const struct device *dev) {
    struct pmw3360_data *data = dev->data;
    const struct pmw3360_config *config = dev->config;
    data->dev = dev;
    k_mutex_init(&data->mutex);
    data->ready = false;
    data->enabled = false;
    data->cpi = config->cpi;
    data->burst_accumulation_max_samples =
        pmw3360_normalize_burst_accumulation_max_samples(config->burst_accumulation_max_samples);
    data->work_queue_started = false;
    data->work_items_inited = false;
    data->irq_configured = false;

    /*
     * Kernel object init MUST happen only once:
     * - Starting the dedicated work queue thread multiple times can lock up.
     * - Adding the GPIO IRQ callback multiple times can cause duplicate events.
     */
#if defined(CONFIG_INPUT_PIXART_PMW3360_USE_OWN_THREAD)
    k_work_queue_init(&data->driver_work_queue);
    k_work_queue_start(&data->driver_work_queue, pmw3360_stack,
                       K_THREAD_STACK_SIZEOF(pmw3360_stack),
                       CONFIG_INPUT_PIXART_PMW3360_THREAD_PRIORITY,
                       NULL);
    data->work_queue_started = true;
#endif

    k_work_init_delayable(&data->motion_work, pmw3360_work_callback);
    k_work_init_delayable(&data->init_work, pmw3360_async_init);
    data->work_items_inited = true;

    if (pmw3360_init_irq(dev) < 0) {
        LOG_INF("Starting in polling mode.");
        data->polling_mode = true;
    }
    data->irq_configured = true;

    return 0;
}

/**
 * Public runtime API that powers the PMW3360 on or off.
 *
 * @param dev PMW3360 device instance.
 * @param enabled Requested runtime state.
 *
 * @retval 0 The state transition was accepted or the sensor was already in the
 *         requested state.
 * @retval -EINVAL `dev` was `NULL`.
 * @retval negative errno External power control failed while enabling.
 */
int zmk_driver_pmw3360_set_enabled(const struct device *dev, bool enabled) {
    if (!dev) {
        return -EINVAL;
    }

    struct pmw3360_data *data = dev->data;
    const struct pmw3360_config *config = dev->config;

    k_mutex_lock(&data->mutex, K_FOREVER);

    if (data->enabled == enabled) {
        k_mutex_unlock(&data->mutex);
        return 0;
    }

    data->enabled = enabled;
    data->ready = false;
    data->motion_burst_active = false;

    k_mutex_unlock(&data->mutex);

    if (!enabled) {
        /* Stop IRQs first to avoid queued work re-enabling itself. */
        if (config->irq_gpio.port) {
            (void)pmw3360_set_interrupt(dev, false);
        }
        (void)k_work_cancel_delayable(&data->motion_work);
        (void)k_work_cancel_delayable(&data->init_work);
        /* Cut sensor power if we have a gate. */
        (void)pmw3360_set_power(dev, false);
        return 0;
    }

    /* Power on, then run init sequence shortly after. */
    int err = pmw3360_set_power(dev, true);
    if (err < 0) {
        return err;
    }

    /* Allow rails/oscillator to settle a bit before SPI init. */
#if defined(CONFIG_INPUT_PIXART_PMW3360_USE_OWN_THREAD)
    k_work_schedule_for_queue(&data->driver_work_queue, &data->init_work, K_MSEC(100));
#else
    k_work_schedule(&data->init_work, K_MSEC(100));
#endif
    return 0;
}

/**
 * Normalize a user-facing CPI value into the PMW3360 register scale.
 *
 * @param cpi Requested CPI value.
 *
 * @return CPI rounded down to the nearest 100-CPI step and clamped into the
 *         driver's supported runtime range.
 */
static uint16_t pmw3360_normalize_cpi(uint16_t cpi) {
    if (cpi < 100) {
        cpi = 100;
    }
    /* The PMW3360 config register uses 100-CPI steps. */
    cpi = (uint16_t)((cpi / 100u) * 100u);
    /* Conservative clamp: PMW3360 nominally supports up to 12000 CPI. */
    if (cpi > 12000) {
        cpi = 12000;
    }
    return cpi;
}

/**
 * Public runtime API that updates PMW3360 pointer sensitivity.
 *
 * @param dev PMW3360 device instance.
 * @param cpi Requested CPI value in user-facing counts per inch.
 *
 * @retval 0 The new value was accepted.
 * @retval -EINVAL `dev` was `NULL`.
 * @retval negative errno A live register write failed verification.
 */
int zmk_driver_pmw3360_set_cpi(const struct device *dev, uint16_t cpi) {
    if (!dev) {
        return -EINVAL;
    }

    struct pmw3360_data *data = dev->data;
    cpi = pmw3360_normalize_cpi(cpi);

    k_mutex_lock(&data->mutex, K_FOREVER);
    const uint16_t previous_cpi = data->cpi;
    data->cpi = cpi;
    const bool can_write_now = data->enabled && data->ready;

    if (!can_write_now) {
        k_mutex_unlock(&data->mutex);
        return 0;
    }

    uint16_t applied_cpi = previous_cpi;
    const int rc = pmw3360_apply_cpi_locked(dev, cpi, &applied_cpi);
    if (rc < 0) {
        data->cpi = previous_cpi;
    } else {
        data->cpi = applied_cpi;
    }
    k_mutex_unlock(&data->mutex);
    return rc;
}

/**
 * Public runtime API that updates the burst accumulation limit.
 *
 * @param dev PMW3360 device instance.
 * @param max_samples Requested maximum number of PMW3360 motion bursts to merge
 *                    into one HID report.
 *
 * @retval 0 The new burst accumulation limit was accepted.
 * @retval -EINVAL `dev` was `NULL`.
 */
int zmk_driver_pmw3360_set_burst_accumulation_max_samples(const struct device *dev,
                                                          uint8_t max_samples) {
    if (!dev) {
        return -EINVAL;
    }

    struct pmw3360_data *data = dev->data;
    const uint8_t normalized =
        pmw3360_normalize_burst_accumulation_max_samples(max_samples);

    k_mutex_lock(&data->mutex, K_FOREVER);
    data->burst_accumulation_max_samples = normalized;
    k_mutex_unlock(&data->mutex);

    return 0;
}

/**
 * Bridge the Zephyr sensor attribute API onto the PMW3360 runtime controls.
 *
 * This allows callers that prefer the generic Zephyr sensor API to reach the
 * same CPI and burst accumulation controls as the explicit PMW3360 public API.
 *
 * @param dev PMW3360 device instance.
 * @param chan Target sensor channel. Only `SENSOR_CHAN_ALL` is supported.
 * @param attr Driver-specific attribute selector.
 * @param val Requested attribute value.
 *
 * @retval 0 Attribute update was accepted.
 * @retval -ENOTSUP Unsupported channel or attribute was requested.
 * @retval -EINVAL `val` was `NULL`.
 * @retval negative errno Forwarded runtime setter failed.
 */
static int pmw3360_attr_set(const struct device *dev, enum sensor_channel chan, enum sensor_attribute attr, const struct sensor_value *val) {
    if (unlikely(chan != SENSOR_CHAN_ALL)) {
        return -ENOTSUP;
    }

    if (unlikely(val == NULL)) {
        return -EINVAL;
    }

    switch((int32_t) attr) {
        case PMW3360_ATTR_CPI:
            return zmk_driver_pmw3360_set_cpi(dev, (uint16_t)val->val1);
        case PMW3360_ATTR_BURST_ACCUMULATION_MAX_SAMPLES:
            return zmk_driver_pmw3360_set_burst_accumulation_max_samples(
                dev, (uint8_t)MAX(val->val1, 0));
        default:
            LOG_ERR("Unknown attribute");
            return -ENOTSUP;
    }
}

static const struct sensor_driver_api pmw3360_driver_api = {
    .attr_set = pmw3360_attr_set,
};

/** SPI mode required by the PMW3360 datasheet and Zephyr SPI layer. */
#define PMW3360_SPI_MODE (SPI_OP_MODE_MASTER | SPI_WORD_SET(8) | SPI_MODE_CPOL | SPI_MODE_CPHA | SPI_TRANSFER_MSB | SPI_HOLD_ON_CS | SPI_LOCK_ON)

/**
 * Instantiate one PMW3360 device from devicetree.
 *
 * The devicetree binding populates the immutable configuration while the driver
 * state lives in a separate mutable `pmw3360_data` structure.
 */
#define PMW3360_DEFINE(n)                                                                          \
    static struct pmw3360_data data##n = {};                                                       \
    static const struct pmw3360_config config##n = {                                               \
        .spi = SPI_DT_SPEC_INST_GET(n, PMW3360_SPI_MODE, 0),                                       \
        .cs_gpio = SPI_CS_GPIOS_DT_SPEC_GET(DT_DRV_INST(n)),                                       \
        .irq_gpio = GPIO_DT_SPEC_GET_OR(DT_DRV_INST(n), irq_gpios, {}),                            \
        .power_gpio = GPIO_DT_SPEC_GET_OR(DT_DRV_INST(n), power_gpios, {}),                        \
        .cpi = DT_PROP(DT_DRV_INST(n), cpi),                                                       \
        .burst_accumulation_max_samples = DT_PROP(DT_DRV_INST(n), burst_accumulation_max_samples), \
        .rotate_90 = DT_PROP(DT_DRV_INST(n), rotate_90),                                           \
        .rotate_180 = DT_PROP(DT_DRV_INST(n), rotate_180),                                         \
        .rotate_270 = DT_PROP(DT_DRV_INST(n), rotate_270),                                         \
        .angle_tune = DT_PROP(DT_DRV_INST(n), angle_tune),                                         \
        .lift_height_3mm = DT_PROP(DT_DRV_INST(n), lift_height_3mm),                               \
        .force_awake = DT_PROP(DT_DRV_INST(n), force_awake),                                       \
        .polling_interval = DT_PROP(DT_DRV_INST(n), polling_interval),                             \
        .invert_x = DT_PROP(DT_DRV_INST(n), invert_x),                                             \
        .invert_y = DT_PROP(DT_DRV_INST(n), invert_y),                                             \
    };                                                                                             \
    DEVICE_DT_INST_DEFINE(n, pmw3360_init, NULL, &data##n, &config##n, POST_KERNEL,                \
        CONFIG_INPUT_INIT_PRIORITY, &pmw3360_driver_api);

DT_INST_FOREACH_STATUS_OKAY(PMW3360_DEFINE)
