#include "number.h"
#include "board_defs.h"
#include "USER_BUTTON.h"
#include "Delay.h"
#include "app_config.h"
#include "ti_msp_dl_config.h"

#include <stdbool.h>
#include <stdint.h>

/* 共阳段码：低电平点亮，bit7 为小数点。 */
static const uint8_t SEG_CODE[10] = {
    0xC0, 0xF9, 0xA4, 0xB0, 0x99,
    0x92, 0x82, 0xF8, 0x80, 0x90
};

#define NUMBER_DIGITS       (4U)
#define NUMBER_BLANK        (0xFFU)
#define NUMBER_DP_MASK      (0x80U)
#define NUMBER_MAX_TENTHS   (999U)

/* SysConfig 组 number：SCLK=PB27 RCLK=PB25 DIO=PB26 */
static const GpioPin NUM_SCLK = {number_PORT, number_SCLK_PIN};
static const GpioPin NUM_RCLK = {number_PORT, number_RCLK_PIN};
static const GpioPin NUM_DIO  = {number_PORT, number_DIO_PIN};

static volatile uint8_t g_number_digits[NUMBER_DIGITS] = {
    NUMBER_BLANK, NUMBER_BLANK, NUMBER_BLANK, NUMBER_BLANK
};
static volatile uint8_t g_number_dp_mask = 0U;
static volatile uint8_t g_number_scan_index = 0U;

static uint32_t number_lock(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static void number_unlock(uint32_t primask)
{
    if (primask == 0U) {
        __enable_irq();
    }
}

static void number_shift_out(uint8_t data)
{
    uint8_t i;

    for (i = 0U; i < 8U; i++) {
        if ((data & 0x80U) != 0U) {
            pin_high(&NUM_DIO);
        } else {
            pin_low(&NUM_DIO);
        }
        data <<= 1;

        pin_low(&NUM_SCLK);
        pin_high(&NUM_SCLK);
    }
}

static void number_latch(void)
{
    pin_low(&NUM_RCLK);
    pin_high(&NUM_RCLK);
}

void number_init(void)
{
    pin_low(&NUM_SCLK);
    pin_low(&NUM_RCLK);
    pin_low(&NUM_DIO);
    g_number_scan_index = 0U;
    number_show_time_ms(0U);
}

void number_show_digit(uint8_t digit)
{
    uint8_t i;
    uint32_t primask;

    if (digit > 9U) {
        digit = 9U;
    }

    primask = number_lock();
    g_number_digits[0] = digit;
    for (i = 1U; i < NUMBER_DIGITS; i++) {
        g_number_digits[i] = NUMBER_BLANK;
    }
    g_number_dp_mask = 0U;
    number_unlock(primask);
}

void number_show_time_ms(uint32_t elapsed_ms)
{
    uint32_t tenths = elapsed_ms / 100U;
    uint32_t seconds;
    uint32_t primask;

    if (tenths > NUMBER_MAX_TENTHS) {
        tenths = NUMBER_MAX_TENTHS;
    }
    seconds = tenths / 10U;

    primask = number_lock();
    g_number_digits[3] = (uint8_t)(seconds / 10U);
    g_number_digits[2] = (uint8_t)(seconds % 10U);
    g_number_digits[1] = (uint8_t)(tenths % 10U);
    g_number_digits[0] = NUMBER_BLANK;
    g_number_dp_mask = (uint8_t)(1U << 2);
    number_unlock(primask);
}

void number_refresh_1ms_isr(void)
{
    uint8_t index = g_number_scan_index;
    uint8_t digit = g_number_digits[index];
    uint8_t seg = NUMBER_BLANK;
    uint8_t select = (uint8_t)(1U << index);

    if (digit <= 9U) {
        seg = SEG_CODE[digit];
        if ((g_number_dp_mask & (uint8_t)(1U << index)) != 0U) {
            seg &= (uint8_t)(~NUMBER_DP_MASK);
        }
    }

    number_shift_out(seg);
    number_shift_out(select);
    number_latch();

    index++;
    if (index >= NUMBER_DIGITS) {
        index = 0U;
    }
    g_number_scan_index = index;
}

void number_user_key_count_demo(void)
{
    uint8_t digit = 0U;
    bool wasPressed = false;
    bool nowPressed;

    number_show_digit(digit);

    for (;;) {
        nowPressed = USER_BUTTON_IsPressed();

        if (nowPressed && !wasPressed) {
            Delay_ms((uint32_t)USER_KEY_DEBOUNCE_MS);
            if (USER_BUTTON_IsPressed()) {
                if (digit < 9U) {
                    digit++;
                    number_show_digit(digit);
                }
                while (USER_BUTTON_IsPressed()) {
                    Delay_ms(5U);
                }
                wasPressed = false;
                continue;
            }
        }

        wasPressed = nowPressed;
        Delay_ms(5U);
    }
}
