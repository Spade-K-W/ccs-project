#ifndef DELAY_H
#define DELAY_H

#include <stdint.h>

/* 按 CPU 周期粗略阻塞延时（内部空操作消耗时间） */
void Delay_cycles(volatile uint32_t n);

/* 粗略微秒延时（阻塞，非高精度） */
void Delay_us(uint32_t us);

/* 粗略毫秒延时（阻塞，非高精度） */
void Delay_ms(uint32_t ms);

#endif /* DELAY_H */
