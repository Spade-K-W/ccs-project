#include "line_sensor.h"
#include "board_defs.h"
#include "app_config.h"

/* 八路巡线模块引脚（74HC4051 多路复用）
 * AD0 -> PB15, AD1 -> PB16, AD2 -> PA17, OUT -> PA28
 */

static bool sensor_on_line(const GpioPin *p)
{
    bool raw = pin_read_raw(p);
    return (LINE_SENSOR_ACTIVE_LOW != 0U) ? (!raw) : raw;
}

static uint8_t bit_count_u8(uint8_t x)
{
    uint8_t c = 0;

    while (x) {
        c += (x & 1U);
        x >>= 1;
    }

    return c;
}

static void line_mux_settle(void)
{
    volatile uint8_t d;

    for (d = 0; d < 100U; d++) {
    }
}

/* 标准接线：AD0=S0, AD1=S1, AD2=S2 */
static void line_select_channel(uint8_t ch)
{
    if ((ch & 0x01U) != 0U) {
        pin_high(&LINE_AD0);
    } else {
        pin_low(&LINE_AD0);
    }

    if ((ch & 0x02U) != 0U) {
        pin_high(&LINE_AD1);
    } else {
        pin_low(&LINE_AD1);
    }

    if ((ch & 0x04U) != 0U) {
        pin_high(&LINE_AD2);
    } else {
        pin_low(&LINE_AD2);
    }

    line_mux_settle();
}

uint8_t line_read_pattern(void)
{
    uint8_t pattern = 0U;
    uint8_t ch;

    for (ch = 0; ch < LINE_SENSOR_CHANNEL_COUNT; ch++) {
        line_select_channel(ch);

        if (sensor_on_line(&LINE_OUT)) {
            pattern |= (1U << ch);
        }
    }

    return pattern;
}

static bool line_calc_error_common(uint8_t pattern, int16_t *sum, uint8_t *cnt)
{
    static const int8_t weights[LINE_SENSOR_CHANNEL_COUNT] = {
        -4, -3, -2, -1, 1, 2, 3, 4
    };
    int16_t acc = 0;
    uint8_t n = 0;
    uint8_t i;

    for (i = 0; i < LINE_SENSOR_CHANNEL_COUNT; i++) {
        if (pattern & (1U << i)) {
            acc += weights[i];
            n++;
        }
    }

    if (n == 0U) {
        return false;
    }

    *sum = acc;
    *cnt = n;
    return true;
}

bool line_calc_error(uint8_t pattern, int16_t *error)
{
    int16_t sum;
    uint8_t cnt;

    if (!line_calc_error_common(pattern, &sum, &cnt)) {
        return false;
    }

    *error = sum / (int16_t)cnt;
    return true;
}

bool line_calc_error_f(uint8_t pattern, float *error)
{
    int16_t sum;
    uint8_t cnt;

    if (!line_calc_error_common(pattern, &sum, &cnt)) {
        return false;
    }
//算出了error
    *error = (float)sum / (float)cnt;
    return true;
}

bool is_vertex_like_pattern(uint8_t pattern)
{
    uint8_t cnt = bit_count_u8(pattern);
    bool center_on = ((pattern & ((1U << 3) | (1U << 4))) != 0U);

    return (center_on && (cnt >= 4U));
}

/* 通道编号按模块 1~8：bit0=CH1 ... bit7=CH8 */
#define LINE_MASK_CH123   (0x07U)  /* CH1|CH2|CH3 */
#define LINE_MASK_CH678   (0xE0U)  /* CH6|CH7|CH8 */

bool line_ch123_all_on(uint8_t pattern)
{
    return ((pattern & LINE_MASK_CH123) == LINE_MASK_CH123);
}

bool line_ch678_all_on(uint8_t pattern)
{
    return ((pattern & LINE_MASK_CH678) == LINE_MASK_CH678);
}

bool line_all_on(uint8_t pattern)
{
    return (pattern == 0xFFU);
}
