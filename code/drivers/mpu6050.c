#include "mpu6050.h"
#include "mpu6050_i2c.h"
#include "app_config.h"
#include "app_utils.h"
#include "oled.h"
#include "uart_debug.h"
#include "ti_msp_dl_config.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#define MPU6050_INT_DT_SEC        (1.0f / MPU6050_DATA_RATE_HZ)
#define MPU6050_ACCEL_XOUT_H_REG  (0x3BU)
#define MPU_RAD_TO_DEG            (57.2957795f)

static volatile float g_gyro_z_bias = 0.0f;
static volatile float g_gyro_z_dps  = 0.0f;
static volatile float g_z_angle_deg = 0.0f;
static volatile float g_roll_deg   = 0.0f;
static volatile float g_pitch_deg  = 0.0f;
static volatile bool  g_mpu_run_enable = false;
static volatile float g_turn_start_deg = 0.0f;

bool mpu6050_read_gyro_z_raw(int16_t *gz)
{
    uint8_t buf[2];

    if (gz == 0) {
        return false;
    }

    if (!mpu_i2c_read_bytes(MPU6050_ADDR, MPU6050_GYRO_ZOUT_H_REG, buf, 2)) {
        return false;
    }

    *gz = (int16_t)(((uint16_t)buf[0] << 8) | buf[1]);
    return true;
}

bool mpu6050_init(void)
{
    uint8_t who = 0;
    uint8_t dummy = 0;

    g_mpu_run_enable = false;

    if (!mpu_i2c_read_byte(MPU6050_ADDR, MPU6050_WHO_AM_I_REG, &who)) {
        mpu6050_i2c_sda_unlock();
        if (!mpu_i2c_read_byte(MPU6050_ADDR, MPU6050_WHO_AM_I_REG, &who)) {
            return false;
        }
    }

    if ((who != 0x68U) && (who != 0x70U)) {
        return false;
    }

    if (!mpu_i2c_write_byte(MPU6050_ADDR, MPU6050_PWR_MGMT_1_REG, 0x00U)) return false;
    delay_ms(50);

    if (!mpu_i2c_write_byte(MPU6050_ADDR, MPU6050_SMPLRT_DIV_REG, 0x07U)) return false;
    if (!mpu_i2c_write_byte(MPU6050_ADDR, MPU6050_CONFIG_REG, 0x03U)) return false;
    if (!mpu_i2c_write_byte(MPU6050_ADDR, MPU6050_GYRO_CONFIG_REG, 0x08U)) return false;
    if (!mpu_i2c_write_byte(MPU6050_ADDR, MPU6050_ACCEL_CONFIG_REG, 0x00U)) return false;

    /* 调试阶段建议锁存中断，便于观察 */
    if (!mpu_i2c_write_byte(MPU6050_ADDR, MPU6050_INT_PIN_CFG_REG, 0x00U)) return false;
    if (!mpu_i2c_write_byte(MPU6050_ADDR, MPU6050_INT_ENABLE_REG, 0x01U)) return false;

    (void)mpu_i2c_read_byte(MPU6050_ADDR, MPU6050_INT_STATUS_REG, &dummy);

    g_gyro_z_bias = 0.0f;
    g_gyro_z_dps  = 0.0f;
    g_z_angle_deg = 0.0f;
    g_roll_deg    = 0.0f;
    g_pitch_deg   = 0.0f;

    return true;
}

bool mpu6050_calibrate_bias(void)
{
    uint32_t i;
    int16_t raw = 0;

    float sum = 0.0f;
    float max_v = -10000.0f;
    float min_v = 10000.0f;

    g_mpu_run_enable = false;

    for (i = 0; i < 1200; i++)
    {
        if (!mpu6050_read_gyro_z_raw(&raw)) {
            return false;
        }

        float v = (float)raw / MPU_GYRO_SENS_500DPS;

        sum += v;

        if (v > max_v) max_v = v;
        if (v < min_v) min_v = v;

        delay_ms(3);
    }

    /* 去掉极值（抗抖动） */
    sum = sum - max_v - min_v;
    g_gyro_z_bias = sum / (1200.0f - 2.0f);

    g_gyro_z_dps  = 0.0f;
    g_z_angle_deg = 0.0f;

    g_mpu_run_enable = true;
    return true;
}

void mpu6050_on_data_ready_isr(void)
{
    uint8_t intStatus = 0;
    int16_t raw = 0;

    if (!g_mpu_run_enable) {
        return;
    }

    if (!mpu_i2c_read_byte(MPU6050_ADDR, MPU6050_INT_STATUS_REG, &intStatus)) {
        return;
    }

    if ((intStatus & 0x01U) == 0U) {
        return;
    }

    if (!mpu6050_read_gyro_z_raw(&raw)) {
        return;
    }

    float gyro = ((float)raw / MPU_GYRO_SENS_500DPS - g_gyro_z_bias) * GYRO_Z_SIGN;

    if (gyro > -0.8f && gyro < 0.8f) {
        gyro = 0.0f;
    }

    g_gyro_z_dps = gyro;

    if (gyro != 0.0f) {
        float dYaw = gyro * MPU6050_INT_DT_SEC;

        g_z_angle_deg += dYaw;

        if (g_z_angle_deg > 180.0f) {
            g_z_angle_deg -= 360.0f;
        } else if (g_z_angle_deg < -180.0f) {
            g_z_angle_deg += 360.0f;
        }
    }
}

float mpu6050_get_gyro_z_dps(void)
{
    return (float)g_gyro_z_dps;
}

float mpu6050_get_z_angle_deg(void)
{
    return (float)g_z_angle_deg;
}

bool mpu6050_update_angles(void)
{
    uint8_t buf[14];
    uint8_t intStatus = 0;
    int16_t ax;
    int16_t ay;
    int16_t az;
    int16_t gz;
    float fax;
    float fay;
    float faz;
    float gyro;

    /*
     * 关 MPU 所在 GPIOA 中断，避免与 ISR 抢 I2C，
     * 也避免 M0+ 软浮点在主循环与 ISR 中重入导致跑飞/卡死。
     */
    NVIC_DisableIRQ(GPIO_MULTIPLE_GPIOA_INT_IRQN);

    /* 一次读完 Accel(6) + Temp(2) + Gyro(6) */
    if (!mpu_i2c_read_bytes(MPU6050_ADDR, MPU6050_ACCEL_XOUT_H_REG, buf, 14U)) {
        return false;
    }

    /* 清 data-ready 标志，防止 INT 一直挂起 */
    (void)mpu_i2c_read_byte(MPU6050_ADDR, MPU6050_INT_STATUS_REG, &intStatus);

    ax = (int16_t)(((uint16_t)buf[0] << 8) | buf[1]);
    ay = (int16_t)(((uint16_t)buf[2] << 8) | buf[3]);
    az = (int16_t)(((uint16_t)buf[4] << 8) | buf[5]);
    gz = (int16_t)(((uint16_t)buf[12] << 8) | buf[13]);

    fax = (float)ax;
    fay = (float)ay;
    faz = (float)az;

    g_roll_deg  = atan2f(fay, faz) * MPU_RAD_TO_DEG;
    g_pitch_deg = atan2f(-fax, sqrtf(fay * fay + faz * faz)) * MPU_RAD_TO_DEG;

    /* 轮询周期与 main 中 Delay_ms(100) 对齐 */
    gyro = ((float)gz / MPU_GYRO_SENS_500DPS - g_gyro_z_bias) * GYRO_Z_SIGN;
    if ((gyro > -0.8f) && (gyro < 0.8f)) {
        gyro = 0.0f;
    }
    g_gyro_z_dps = gyro;
    if (gyro != 0.0f) {
        g_z_angle_deg += gyro * 0.1f;
        if (g_z_angle_deg > 180.0f) {
            g_z_angle_deg -= 360.0f;
        } else if (g_z_angle_deg < -180.0f) {
            g_z_angle_deg += 360.0f;
        }
    }

    /* 本测试走轮询，保持中断关闭，防止再次与主循环冲突 */
    return true;
}

float mpu6050_get_roll(void)
{
    return (float)g_roll_deg;
}

float mpu6050_get_pitch(void)
{
    return (float)g_pitch_deg;
}

float mpu6050_get_yaw(void)
{
    return (float)g_z_angle_deg;
}

void mpu6050_reset_z_angle(void)
{
    g_z_angle_deg = 0.0f;
}

static float mpu6050_angle_diff_deg(float now, float base)
{
    float d = now - base;
    if (d >  180.0f) {
        d -= 360.0f;
    }
    if (d < -180.0f) {
        d += 360.0f;
    }
    return d;
}

void mpu6050_turn_mark_start(void)
{
    g_turn_start_deg = g_z_angle_deg;
}

bool mpu6050_turn_yaw_reached(float target_deg)
{
    float d = mpu6050_angle_diff_deg(g_z_angle_deg, g_turn_start_deg);
    if (d < 0.0f) {
        d = -d;
    }
    return (d >= target_deg);
}

bool mpu6050_turn_yaw_reached_90(void)
{
    return mpu6050_turn_yaw_reached(TURN_TARGET_DEG);
}

static void mpu6050_halt_fail(const char *oled_msg, void (*uart_fail)(void))
{
    oled_display_string(2, 0, oled_msg);
    uart_fail();
    while (1) {
        delay_ms(500U);
    }
}

void mpu6050_startup(void)
{
    oled_display_string(0, 0, "MPU Calibrating");
    oled_display_string(1, 0, "Keep still...    ");

    if (!mpu6050_init()) {
        mpu6050_halt_fail("MPU init FAIL   ", uart_debug_print_mpu_init_fail);
    }

    NVIC_EnableIRQ(GPIO_MULTIPLE_GPIOA_INT_IRQN);

    if (!mpu6050_calibrate_bias()) {
        mpu6050_halt_fail("MPU calib FAIL  ", uart_debug_print_mpu_calib_fail);
    }

    mpu6050_reset_z_angle();
}
