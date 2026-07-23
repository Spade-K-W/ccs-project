#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    int32_t leftCount;
    int32_t rightCount;
} EncoderCounts;

typedef struct {
    float leftSpeed;
    float rightSpeed;
} EncoderSpeeds;

/* 物理速度（cm/s），由脉冲 + 车轮半径 + 拟合系数解算 */
typedef struct {
    float leftSpeedCms;
    float rightSpeedCms;
    float vehicleSpeedCms;  /* 左右平均，供视觉模块 */
} EncoderPhysicalSpeeds;

void encoder_init(void);
void encoder_on_gpio_isr(void);
/* 标定轮询：主循环调用，手转轮时不依赖中断也能计数 */
void encoder_poll(void);
void encoder_update(void);
void encoder_reset(void);

/*
 * 转弯预延时结束、进入转弯时调用：
 * 丢掉延时期间积压的脉冲，从当前计数重新开始测速
 */
void encoder_speed_restart(void);

void encoder_get_counts(EncoderCounts *out);
/* 四路电机原始 A 相边沿累计脉冲：out[0..3] = M1..M4 */
void encoder_get_motor_counts(int32_t out[4]);
void encoder_get_speeds(EncoderSpeeds *out);

/*
 * 脉冲 ↔ 距离：
 *   mm_per_pulse = π * WHEEL_DIAMETER_MM / ENC_PULSES_PER_REV
 *   distance_mm  = pulses * mm_per_pulse
 */
float encoder_mm_per_pulse(void);
float encoder_pulses_to_mm(float pulses);
float encoder_pulses_to_cm(float pulses);

/* 测速 = 本周期距离 / 时间 → cm/s */
float encoder_calc_speed_cm_s(float dPulse);

/* 左右平均车速 cm/s（需先 encoder_update） */
void encoder_get_physical_speeds(EncoderPhysicalSpeeds *out);
float encoder_get_vehicle_speed_cm_s(void);

/* 累计脉冲（调试用，非速度） */
int32_t encoder_get_vehicle_pos_pulses(void);

/*
 * OLED：页0 状态，页1 Cmd，页2 Out，页3 四路 A 相脉冲计数（标定用）
 * 串口：仅 turn 时按 UART_DEBUG_PRINT_INTERVAL_MS 输出
 */
void encoder_display_oled_uart(bool isStraight,
                               int16_t cmdLeft, int16_t cmdRight,
                               int16_t outLeft, int16_t outRight);

#endif /* ENCODER_H */
