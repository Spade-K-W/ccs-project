#ifndef PID_H
#define PID_H

#include <stdint.h>

void pid_line_reset(void);

/* 直线循迹 PD，返回差速，限幅 ±STRAIGHT_DIFF_MAX */
int16_t pid_line_calc_diff(float error);

void pid_accel_reset(void);

/*
 * 直线共模速度内环：左右加同一 corr，保留外环差速。
 * 调用前需先 encoder_update()。
 */
void pid_accel_apply(int16_t *leftPwm, int16_t *rightPwm);

/*
 * 转弯左右独立速度内环：每侧 err = cmd - mea，各自 PID。
 * 用于原地转 Cmd 异号（如 -30/+30）。调用前需先 encoder_update()。
 */
void pid_accel_apply_turn(int16_t *leftPwm, int16_t *rightPwm);

#endif /* PID_H */
