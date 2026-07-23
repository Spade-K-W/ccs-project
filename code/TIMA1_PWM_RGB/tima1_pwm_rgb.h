#ifndef TIMA1_PWM_RGB_H
#define TIMA1_PWM_RGB_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 你当前配置对应：
 * CPU 时钟      = 80MHz      ← 改成 80MHz
 * PWM 频率      = 800kHz
 * 单 bit 周期   = 1.25us
 * 周期 tick 数  = 80MHz / 800kHz = 100
 */
#define TIMA1_PWM_RGB_PERIOD_TICKS     (100U)
#define TIMA1_PWM_RGB_CCR_IDLE         (100U)
#define TIMA1_PWM_RGB_CCR_0            (67U)
#define TIMA1_PWM_RGB_CCR_1            (35U)

/* SK6812 reset/latch 低电平时间，通常 > 80us 即可 */
#define TIMA1_PWM_RGB_RESET_US         (100U)

void tima1_pwm_rgb_init(void);
bool tima1_pwm_rgb_start_frame(const uint16_t *compare_buf, uint32_t compare_count);
bool tima1_pwm_rgb_is_busy(void);
void tima1_pwm_rgb_wait_done(void);

#ifdef __cplusplus
}
#endif

#endif