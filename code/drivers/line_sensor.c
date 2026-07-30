#include "line_sensor.h"
#include "board_defs.h"
#include "app_config.h"

/*
 * 按当前小车安装方向：X1（最左）...X8（最右）。
 * 对外保持bit0=X1、bit7=X8。
 */
static const GpioPin *const line_pins[LINE_SENSOR_CHANNEL_COUNT] = {
    &LINE_X1, &LINE_X2, &LINE_X3, &LINE_X4,
    &LINE_X5, &LINE_X6, &LINE_X7, &LINE_X8
};

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

uint8_t line_read_pattern(void)
{
    uint8_t pattern = 0U;
    uint8_t ch;

    for (ch = 0U; ch < LINE_SENSOR_CHANNEL_COUNT; ch++) {
        if (sensor_on_line(line_pins[ch])) {
            pattern |= (uint8_t)(1U << ch);
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
    *error = (float)sum / (float)cnt;
    return true;
}

bool line_calc_error_arc_f(uint8_t pattern, float *error)
{
    /* CH4/CH5 权重 0：对准 4、5 时 error≈0 */
    static const int8_t weights[LINE_SENSOR_CHANNEL_COUNT] = {
        (int8_t)LINE_WEIGHT_ARC_CH1,
        (int8_t)LINE_WEIGHT_ARC_CH2,
        (int8_t)LINE_WEIGHT_ARC_CH3,
        (int8_t)LINE_WEIGHT_ARC_CH4,
        (int8_t)LINE_WEIGHT_ARC_CH5,
        (int8_t)LINE_WEIGHT_ARC_CH6,
        (int8_t)LINE_WEIGHT_ARC_CH7,
        (int8_t)LINE_WEIGHT_ARC_CH8
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

    *error = (float)acc / (float)n;
    return true;
}

bool is_vertex_like_pattern(uint8_t pattern)
{
    uint8_t cnt = bit_count_u8(pattern);
    bool center_on = ((pattern & ((1U << 3) | (1U << 4))) != 0U);

    return (center_on && (cnt >= 4U));
}

/* 车辆物理通道从左到右：bit0=CH1 ... bit7=CH8 */
#define LINE_MASK_CH234   (0x0EU)  /* CH2|CH3|CH4 */
#define LINE_MASK_CH678   (0xE0U)  /* CH6|CH7|CH8 */

bool line_ch234_all_on(uint8_t pattern)
{
    return ((pattern & LINE_MASK_CH234) == LINE_MASK_CH234);
}

bool line_ch678_all_on(uint8_t pattern)
{
    return ((pattern & LINE_MASK_CH678) == LINE_MASK_CH678);
}

bool line_all_on(uint8_t pattern)
{
    return (pattern == 0xFFU);
}
