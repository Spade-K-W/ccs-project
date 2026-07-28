#include "number.h"
#include "board_defs.h"
#include "USER_BUTTON.h"
#include "Delay.h"
#include "app_config.h"
#include "ti_msp_dl_config.h"

#include <stdint.h>
#include <stdbool.h>

/* 共阳段码：低电平点亮，顺序 0~9 */
static const uint8_t SEG_CODE[10] = {
    0xC0, 0xF9, 0xA4, 0xB0, 0x99,
    0x92, 0x82, 0xF8, 0x80, 0x90
};

#define DIGIT1_SEL (0x01U)  /* 第 1 位选通 */

/* SysConfig 组 number：SCLK=PB27 RCLK=PB25 DIO=PB26 */
static const GpioPin NUM_SCLK = {number_PORT, number_SCLK_PIN};
static const GpioPin NUM_RCLK = {number_PORT, number_RCLK_PIN};
static const GpioPin NUM_DIO  = {number_PORT, number_DIO_PIN};

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
    Delay_us(1U);
    pin_high(&NUM_RCLK);
}

void number_init(void)
{
    pin_low(&NUM_SCLK);
    pin_low(&NUM_RCLK);
    pin_low(&NUM_DIO);
    number_show_digit(0U);
}

void number_show_digit(uint8_t digit)
{
    uint8_t seg;

    if (digit > 9U) {
        digit = 9U;
    }
    seg = SEG_CODE[digit];

    number_shift_out(seg);
    number_shift_out(DIGIT1_SEL);
    number_latch();
}

void number_user_key_count_demo(void)
{
    uint8_t digit = 0U;
    bool    wasPressed = false;
    bool    nowPressed;

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
