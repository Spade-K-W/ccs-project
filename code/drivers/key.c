#include "key.h"
#include "app_config.h"
#include "board_defs.h"
#include "ti_msp_dl_config.h"

#include <stdbool.h>
#include <stdint.h>

/* SysConfig：KEY1=PB24，KEY2=PA27，KEY3=PA25；低电平按下。 */
static const GpioPin KEY1 = {KEY_key_1_PORT, KEY_key_1_PIN};
static const GpioPin KEY2 = {KEY_key_2_PORT, KEY_key_2_PIN};
static const GpioPin KEY3 = {KEY_key_3_PORT, KEY_key_3_PIN};

#define KEY_DEBOUNCE_TICKS \
    ((USER_KEY_DEBOUNCE_MS + LOOP_PERIOD_MS - 1U) / LOOP_PERIOD_MS)

static uint8_t g_key_candidate = 0U;
static uint8_t g_key_stable    = 0U;
static uint8_t g_key_same_count = 0U;

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

void key_init(void)
{
    key_enable_pullups();
    g_key_candidate = key_read_level();
    g_key_stable = g_key_candidate;
    g_key_same_count = (uint8_t)KEY_DEBOUNCE_TICKS;
}

uint8_t key_scan(void)
{
    uint8_t raw = key_read_level();

    if (raw != g_key_candidate) {
        g_key_candidate = raw;
        g_key_same_count = 0U;
        return 0U;
    }

    if (g_key_same_count < (uint8_t)KEY_DEBOUNCE_TICKS) {
        g_key_same_count++;
        if (g_key_same_count < (uint8_t)KEY_DEBOUNCE_TICKS) {
            return 0U;
        }
    }

    if (g_key_stable == g_key_candidate) {
        return 0U;
    }

    g_key_stable = g_key_candidate;

    /* 松开只更新稳定状态；按下时才产生一次 1/2/3 事件。 */
    if (g_key_stable != 0U) {
        return g_key_stable;
    }

    return 0U;
}
