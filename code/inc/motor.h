#ifndef MOTOR_H
#define MOTOR_H

#include "board_defs.h"
#include <stdint.h>
#include <stdbool.h>

/* 初始化电机模块：安全停止、初始化软件 PWM、拉高 STBY */
void motor_init(void);

/* 初始化软件 PWM（基于 SysTick） */
void motor_pwm_init(void);

/* TB6612FNG 使能/待机控制 */
void motor_driver_enable(void);
void motor_driver_disable(void);
bool motor_driver_is_enabled(void);

/* 设置单个电机速度
 * speed 范围建议：-100 ~ 100
 */
void motor_set_speed(Motor *m, int16_t speed);

/* 设置底盘左右轮速度（-100~100，负值为倒转） */
void chassis_set(int16_t leftSpeed, int16_t rightSpeed);

/* 停止底盘 */
void chassis_stop(void);

#endif