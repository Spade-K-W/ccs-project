#include "mpu6050.h"
#include "mpu6050_i2c.h"
#include "app_config.h"
#include "app_utils.h"
#include "oled.h"
#include "uart_debug.h"
#include "ti_msp_dl_config.h"

#include <stdbool.h>
#include <stdint.h>

#define MPU6050_INT_DT_SEC   (1.0f / MPU6050_DATA_RATE_HZ)

static volatile float g_gyro_z_bias = 0.0f;
static volatile float g_gyro_z_dps  = 0.0f;
static volatile float g_z_angle_deg = 0.0f;
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

    for (i = 0; i < 1200; i++)   // ↑ 提高采样数量
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

    // 去掉极值（抗抖动）
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

    // 转换 + 去零偏
    float gyro = ((float)raw / MPU_GYRO_SENS_500DPS - g_gyro_z_bias) * GYRO_Z_SIGN;

    // =========================
    //  死区（核心）
    // =========================
    if (gyro > -0.8f && gyro < 0.8f)
    {
        gyro = 0.0f;
    }

    g_gyro_z_dps = gyro;

    // =========================
    // 只有有效运动才积分
    // =========================
    if (gyro != 0.0f)
    {
        float dYaw = gyro * MPU6050_INT_DT_SEC;

        g_z_angle_deg += dYaw;

        // 限制角度范围
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
