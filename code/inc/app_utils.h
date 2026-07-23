#ifndef APP_UTILS_H
#define APP_UTILS_H

#include <stdint.h>

/* 将 x 限制在 [minVal, maxVal] 范围内 */
int16_t clamp_i16(int16_t x, int16_t minVal, int16_t maxVal);
/* 求 int16_t 类型的绝对值 */
int16_t abs_i16(int16_t x);
/* 粗略毫秒延时 */
void delay_ms(uint32_t ms);

#endif