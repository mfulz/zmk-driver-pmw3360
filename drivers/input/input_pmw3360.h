/*
 * Copyright (c) 2025 George Norton
 * Copyright (c) 2026 Matthias Fulz
 *
 * Private definitions for the ZMK PMW3360 driver module.
 *
 * This header intentionally contains implementation details that are only meant
 * for the driver itself. Public runtime control entry points live in
 * `include/zmk_driver_pmw3360.h`.
 */

#pragma once

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/spi.h>

#ifdef __cplusplus
extern "C" {
#endif

/** PMW3360 SPI register read marker. */
#define PMW3360_SPI_READ 0
/** PMW3360 SPI register write marker. */
#define PMW3360_SPI_WRITE 0x80

/* Timing values from the PMW3360 datasheet, expressed in microseconds. */
#define PMW3360_T_SRAD 160
#define PMW3360_T_SCLK_NCS_READ 120
#define PMW3360_T_SCLK_NCS_WRITE 35
#define PMW3360_T_SRR 20
#define PMW3360_T_SWR 180
#define PMW3360_T_SRAD_MOTBR 35

/* PMW3360 register addresses and selected register bit definitions. */
#define PMW3360_REG_PRODUCT_ID 0x00
#define PMW3360_REG_REVISION_ID 0x01
#define PMW3360_REG_MOTION 0x02
#define PMW3360_MOTION_MOT 0x80
#define PMW3360_REG_DELTA_X_L 0x03
#define PMW3360_REG_DELTA_X_H 0x04
#define PMW3360_REG_DELTA_Y_L 0x05
#define PMW3360_REG_DELTA_Y_H 0x06
#define PMW3360_REG_CONTROL 0x0D
#define PMW3360_CONTROL_ROTATE_90 0xC0
#define PMW3360_CONTROL_ROTATE_180 0x60
#define PMW3360_CONTROL_ROTATE_270 0xA0
#define PMW3360_REG_CONFIG_1 0x0F
#define PMW3360_REG_CONFIG_2 0x10
#define PMW3360_CONFIG_2_RTP_MOD 0x04
#define PMW3360_CONFIG_2_REST_EN 0x20
#define PMW3360_REG_ANGLE_TUNE 0x11
#define PMW3360_REG_POWER_UP 0x3A
#define PMW3360_REG_MOTION_BURST 0x50
#define PMW3360_REG_LIFT_CONFIG 0x63
#define PMW3360_LIFT_CONFIG_2MM 0x02
#define PMW3360_LIFT_CONFIG_3MM 0x03

/**
 * Runtime-configurable PMW3360 sensor attributes exposed through Zephyr's
 * generic `sensor_attr_set()` entry point.
 */
enum pmw3360_attributes {
    /** Runtime CPI update. */
    PMW3360_ATTR_CPI,
    /** Runtime burst accumulation limit update. */
    PMW3360_ATTR_BURST_ACCUMULATION_MAX_SAMPLES,
};

/**
 * Minimal PMW3360 motion burst payload used by this driver.
 *
 * The PMW3360 can provide additional raw image and quality data, but ZMK only
 * needs the motion bit and the relative X/Y deltas for pointer reporting.
 */
struct motion_burst {
    uint8_t motion;
    uint8_t observation;
    uint8_t delta_x_l;
    uint8_t delta_x_h;
    uint8_t delta_y_l;
    uint8_t delta_y_h;
} __packed;

/**
 * Mutable driver state for one PMW3360 instance.
 */
struct pmw3360_data {
    /** Owning Zephyr device. */
    const struct device *dev;
    /** Serializes runtime updates against initialization and motion reads. */
    struct k_mutex mutex;
    /** True once the sensor completed initialization successfully. */
    bool ready;
    /** True while the sensor should be powered and producing motion. */
    bool enabled;
    /** Last requested or applied CPI value. */
    uint16_t cpi;
    /** Current burst accumulation limit used at runtime. */
    uint8_t burst_accumulation_max_samples;
    /** True while PMW3360 burst mode remains active between reads. */
    bool motion_burst_active;
    /** True when the driver had to fall back to polling mode. */
    bool polling_mode;
    /** Last poll timestamp in hardware cycles, reserved for future diagnostics. */
    uint32_t last_poll_cycles;
#if defined(CONFIG_INPUT_PIXART_PMW3360_USE_OWN_THREAD)
    /** Optional dedicated work queue used for init and motion processing. */
    struct k_work_q driver_work_queue;
#endif
    /** Work item that drains motion and emits host-visible relative reports. */
    struct k_work_delayable motion_work;
    /** Work item that performs the sensor initialization sequence. */
    struct k_work_delayable init_work;
    /** GPIO callback tied to the PMW3360 MOTION pin. */
    struct gpio_callback irq_gpio_cb;
    /** True once the optional dedicated work queue has been started. */
    bool work_queue_started;
    /** True once delayable work items have been initialized. */
    bool work_items_inited;
    /** True once the IRQ callback and pin setup have been attempted. */
    bool irq_configured;
};

/**
 * Immutable configuration assembled from devicetree.
 */
struct pmw3360_config {
    /** SPI bus configuration for the sensor. */
    struct spi_dt_spec spi;
    /** Chip-select GPIO derived from the SPI node. */
    struct gpio_dt_spec cs_gpio;
    /** Optional MOTION interrupt GPIO. */
    struct gpio_dt_spec irq_gpio;
    /** Optional external power gate control GPIO. */
    struct gpio_dt_spec power_gpio;
    /** Default CPI value to apply during initialization. */
    uint16_t cpi;
    /** Default burst accumulation limit to apply during initialization. */
    uint8_t burst_accumulation_max_samples;
    /** Rotate the sensor coordinate system by 90 degrees. */
    bool rotate_90;
    /** Rotate the sensor coordinate system by 180 degrees. */
    bool rotate_180;
    /** Rotate the sensor coordinate system by 270 degrees. */
    bool rotate_270;
    /** Fine-grained angle correction for slightly skewed mounting. */
    int8_t angle_tune;
    /** Use the PMW3360 3 mm lift-off setting instead of the default 2 mm. */
    bool lift_height_3mm;
    /** Keep the PMW3360 out of its internal rest mode. */
    bool force_awake;
    /** Polling interval in microseconds when no IRQ pin is available. */
    uint32_t polling_interval;
    /** Invert X movement after orientation correction. */
    bool invert_x;
    /** Invert Y movement after orientation correction. */
    bool invert_y;
};

#ifdef __cplusplus
}
#endif
