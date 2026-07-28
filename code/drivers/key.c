#include "key.h"
#include "number.h"
#include "app_config.h"
#include "board_defs.h"
#include "Delay.h"
#include "ti_msp_dl_config.h"

#include <stdbool.h>
#include <stdint.h>

/* SysConfig：key_1=PB24, key_2=PA27, key_3=PA25；低电平为按下 */
static const GpioPin KEY1 = {KEY_key_1_PORT, KEY_key_1_PIN};
static const GpioPin KEY2 = {KEY_key_2_PORT, KEY_key_2_PIN};
static const GpioPin KEY3 = {KEY_key_3_PORT, KEY_key_3_PIN};

/* 软件补开内部上拉（SysConfig 若未勾选 PULL_UP 也能用） */
static void key_enable_pullups(void)
{
    DL_GPIO_initDigitalInputFeatures(KEY_key_1_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(KEY_key_2_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(KEY_key_3_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
}

static bool key_pin_pressed(const GpioPin *p)
{
    bool raw = pin_read_raw(p);
    return USER_KEY_ACTIVE_LOW ? (!raw) : raw;
}

static bool key_any_pressed(void)
{
    return key_pin_pressed(&KEY1) || key_pin_pressed(&KEY2)
        || key_pin_pressed(&KEY3);
}

static void key_wait_all_release(void)
{
    while (key_any_pressed()) {
        Delay_ms(5U);
    }
    Delay_ms(50U);
}

static uint8_t key_read_level(void)
{
    if (key_pin_pressed(&KEY1)) {
        return 1U;
    }
    if (key_pin_pressed(&KEY2)) {
        return 2U;
    }
    if (key_pin_pressed(&KEY3)) {
        return 3U;
    }
    return 0U;
}

static void key_show_then_hold(uint8_t level)
{
    number_show_digit(level);
    while (1) {
        Delay_ms(500U);
    }
}

void key1_run(void)
{
    key_show_then_hold(1U);
}

void key2_run(void)
{
    key_show_then_hold(2U);
}

void key3_run(void)
{
    key_show_then_hold(3U);
}

void key_control(void)
{
    uint8_t level;

    key_enable_pullups();
    number_show_digit(0U);
    key_wait_all_release();

    for (;;) {
        level = key_read_level();
        if (level != 0U) {
            Delay_ms((uint32_t)USER_KEY_DEBOUNCE_MS);
            level = key_read_level();
            if (level != 0U) {
                key_wait_all_release();
                switch (level) {
                case 1U:
                    key1_run();
                    break;
                case 2U:
                    key2_run();
                    break;
                case 3U:
                    key3_run();
                    break;
                default:
                    break;
                }
            }
        }
        Delay_ms(5U);
    }
}
