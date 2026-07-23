#include "app_utils.h"
#include "Delay.h"

/* =========================================================
 * 限幅函数
 * 如果 x 小于最小值，返回最小值
 * 如果 x 大于最大值，返回最大值
 * 否则返回 x 本身
 * ========================================================= */
int16_t clamp_i16(int16_t x, int16_t minVal, int16_t maxVal)
{
    if (x < minVal) return minVal;
    if (x > maxVal) return maxVal;
    return x;
}

/* 求绝对值 */
int16_t abs_i16(int16_t x)
{
    return (x >= 0) ? x : (int16_t)(-x);
}

/* 兼容旧接口：转发到 Delay 库 */
void delay_ms(uint32_t ms)
{
    Delay_ms(ms);
}