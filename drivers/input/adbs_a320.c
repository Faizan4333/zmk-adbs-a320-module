/*
 * ADBS-A320 Optical Finger Navigation sensor driver
 * For BlackBerry-style trackpads over I2C
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT avago_adbs_a320

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/device.h>

LOG_MODULE_REGISTER(adbs_a320, CONFIG_INPUT_LOG_LEVEL);

/* ADBS-A320 Register addresses */
#define REG_PRODUCT_ID    0x00
#define REG_MOTION        0x02
#define REG_DELTA_X       0x03
#define REG_DELTA_Y       0x04

#define EXPECTED_PRODUCT_ID 0x83
#define MOTION_FLAG         0x80

struct adbs_a320_config {
    struct i2c_dt_spec i2c;
    struct gpio_dt_spec rst_gpio;
    struct gpio_dt_spec motion_gpio;
    struct gpio_dt_spec led_gpio;
    struct gpio_dt_spec btn_gpio;
    struct gpio_dt_spec btn_drive_gpio;
    bool swap_xy;
    bool invert_x;
    bool invert_y;
};

struct adbs_a320_data {
    const struct device *dev;
    struct k_timer poll_timer;
    struct k_work poll_work;
    bool btn_pressed;
};

static int read_reg(const struct device *dev, uint8_t reg, uint8_t *val)
{
    const struct adbs_a320_config *cfg = dev->config;
    return i2c_reg_read_byte_dt(&cfg->i2c, reg, val);
}

static void poll_work_handler(struct k_work *work)
{
    struct adbs_a320_data *data = CONTAINER_OF(work, struct adbs_a320_data, poll_work);
    const struct device *dev = data->dev;
    const struct adbs_a320_config *cfg = dev->config;
    uint8_t motion_reg;
    int ret;

    /* Read motion status register */
    ret = read_reg(dev, REG_MOTION, &motion_reg);
    if (ret < 0) {
        return;
    }

    /* If motion was detected, read delta X and Y */
    if (motion_reg & MOTION_FLAG) {
        uint8_t raw_dx, raw_dy;

        read_reg(dev, REG_DELTA_X, &raw_dx);
        read_reg(dev, REG_DELTA_Y, &raw_dy);

        int8_t dx = (int8_t)raw_dx;
        int8_t dy = (int8_t)raw_dy;

        /* Apply axis transformations */
        if (cfg->swap_xy) {
            int8_t tmp = dx;
            dx = dy;
            dy = tmp;
        }
        if (cfg->invert_x) {
            dx = -dx;
        }
        if (cfg->invert_y) {
            dy = -dy;
        }

        /* Report relative motion to the input subsystem */
        if (dx != 0 || dy != 0) {
            input_report_rel(dev, INPUT_REL_X, (int16_t)dx, false, K_FOREVER);
            input_report_rel(dev, INPUT_REL_Y, (int16_t)dy, true, K_FOREVER);
        }
    }

    /* Check dome button (Dome A driven LOW, read Dome B) */
    if (cfg->btn_gpio.port != NULL) {
        bool pressed = (gpio_pin_get_dt(&cfg->btn_gpio) > 0);
        if (pressed != data->btn_pressed) {
            data->btn_pressed = pressed;
            input_report_key(dev, INPUT_BTN_LEFT, pressed ? 1 : 0, true, K_FOREVER);
        }
    }
}

static void poll_timer_handler(struct k_timer *timer)
{
    struct adbs_a320_data *data = CONTAINER_OF(timer, struct adbs_a320_data, poll_timer);
    k_work_submit(&data->poll_work);
}

static int adbs_a320_init(const struct device *dev)
{
    const struct adbs_a320_config *cfg = dev->config;
    struct adbs_a320_data *data = dev->data;
    uint8_t product_id;
    int ret;

    data->dev = dev;
    data->btn_pressed = false;

    /* Verify I2C bus is ready */
    if (!i2c_is_ready_dt(&cfg->i2c)) {
        LOG_ERR("I2C bus not ready");
        return -ENODEV;
    }

    /* Reset the sensor: assert reset (active low), wait, then release */
    if (cfg->rst_gpio.port != NULL) {
        ret = gpio_pin_configure_dt(&cfg->rst_gpio, GPIO_OUTPUT_ACTIVE);
        if (ret < 0) {
            LOG_ERR("Failed to configure reset GPIO");
            return ret;
        }
        k_msleep(10);
        gpio_pin_set_dt(&cfg->rst_gpio, 0); /* deassert reset = pin goes HIGH */
        k_msleep(150);
    }

    /* Turn on the optical sensor LED */
    if (cfg->led_gpio.port != NULL) {
        gpio_pin_configure_dt(&cfg->led_gpio, GPIO_OUTPUT_ACTIVE);
    }

    /* Configure dome switch: drive Dome A low, read Dome B */
    if (cfg->btn_drive_gpio.port != NULL) {
        gpio_pin_configure_dt(&cfg->btn_drive_gpio, GPIO_OUTPUT_ACTIVE);
    }
    if (cfg->btn_gpio.port != NULL) {
        gpio_pin_configure_dt(&cfg->btn_gpio, GPIO_INPUT);
    }

    /* Read and verify product ID */
    ret = read_reg(dev, REG_PRODUCT_ID, &product_id);
    if (ret < 0) {
        LOG_ERR("Failed to read product ID (err %d)", ret);
        return ret;
    }
    if (product_id != EXPECTED_PRODUCT_ID) {
        LOG_WRN("Unexpected product ID: 0x%02x (expected 0x%02x)",
                product_id, EXPECTED_PRODUCT_ID);
    } else {
        LOG_INF("ADBS-A320 trackpad initialized (ID: 0x%02x)", product_id);
    }

    /* Start polling timer */
    k_work_init(&data->poll_work, poll_work_handler);
    k_timer_init(&data->poll_timer, poll_timer_handler, NULL);
    k_timer_start(&data->poll_timer,
                  K_MSEC(CONFIG_ADBS_A320_POLL_INTERVAL_MS),
                  K_MSEC(CONFIG_ADBS_A320_POLL_INTERVAL_MS));

    return 0;
}

/*
 * PM suspend/resume: stop/start the polling timer, and
 * turn the optical LED on/off to save power.
 */
static int adbs_a320_pm_action(const struct device *dev,
                               enum pm_device_action action)
{
    struct adbs_a320_data *data = dev->data;
    const struct adbs_a320_config *cfg = dev->config;

    switch (action) {
    case PM_DEVICE_ACTION_SUSPEND:
        k_timer_stop(&data->poll_timer);
        if (cfg->led_gpio.port != NULL) {
            gpio_pin_set_dt(&cfg->led_gpio, 0); // 0 = Inactive
        }
        LOG_INF("ADBS-A320 suspended");
        return 0;

    case PM_DEVICE_ACTION_RESUME:
        if (cfg->led_gpio.port != NULL) {
            gpio_pin_set_dt(&cfg->led_gpio, 1); // 1 = Active
        }
        k_timer_start(&data->poll_timer,
                      K_MSEC(CONFIG_ADBS_A320_POLL_INTERVAL_MS),
                      K_MSEC(CONFIG_ADBS_A320_POLL_INTERVAL_MS));
        LOG_INF("ADBS-A320 resumed");
        return 0;

    default:
        return -ENOTSUP;
    }
}

/* Instantiation macro for each device tree instance */
#define ADBS_A320_INST(n)                                                       \
    static struct adbs_a320_data adbs_a320_data_##n;                            \
    static const struct adbs_a320_config adbs_a320_config_##n = {               \
        .i2c = I2C_DT_SPEC_INST_GET(n),                                        \
        .rst_gpio = GPIO_DT_SPEC_INST_GET_OR(n, rst_gpios, {0}),               \
        .motion_gpio = GPIO_DT_SPEC_INST_GET_OR(n, motion_gpios, {0}),         \
        .led_gpio = GPIO_DT_SPEC_INST_GET_OR(n, led_gpios, {0}),               \
        .btn_gpio = GPIO_DT_SPEC_INST_GET_OR(n, btn_gpios, {0}),               \
        .btn_drive_gpio = GPIO_DT_SPEC_INST_GET_OR(n, btn_drive_gpios, {0}),   \
        .swap_xy = DT_INST_PROP(n, swap_xy),                                   \
        .invert_x = DT_INST_PROP(n, invert_x),                                 \
        .invert_y = DT_INST_PROP(n, invert_y),                                 \
    };                                                                          \
    PM_DEVICE_DT_INST_DEFINE(n, adbs_a320_pm_action);                          \
    DEVICE_DT_INST_DEFINE(n, adbs_a320_init, PM_DEVICE_DT_INST_GET(n),         \
                          &adbs_a320_data_##n, &adbs_a320_config_##n,           \
                          POST_KERNEL, CONFIG_INPUT_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(ADBS_A320_INST)
