#ifndef MPU6050_H
#define MPU6050_H

#include <stdbool.h>
#include <stdint.h>

/* 初始化 MPU6050 */
bool mpu6050_init(void);

/* 静止状态下校准陀螺仪 Z 轴零偏 */
bool mpu6050_calibrate_bias(void);

/* 上电启动：提示静止 → 初始化 → 开中断 → 校准零偏 → 角度清零；失败则提示并停住 */
void mpu6050_startup(void);

/* 读取 Z 轴原始值 */
bool mpu6050_read_gyro_z_raw(int16_t *gz);

/* 获取当前缓存的 Z 轴角速度，单位：dps */
float mpu6050_get_gyro_z_dps(void);

/* 获取当前累计角度，单位：deg */
float mpu6050_get_z_angle_deg(void);

/* 角度清零 */
void mpu6050_reset_z_angle(void);

/* 进入转弯时调用：记录当前 Z 角作为基准 */
void mpu6050_turn_mark_start(void);

/* 相对转弯起点是否已转过 target_deg（含 ±180° 回绕，取绝对值） */
bool mpu6050_turn_yaw_reached(float target_deg);

/* 相对转弯起点是否已转过 90°（TURN_TARGET_DEG） */
bool mpu6050_turn_yaw_reached_90(void);

/* 在 GPIO 外部中断里调用 */
void mpu6050_on_data_ready_isr(void);

#endif