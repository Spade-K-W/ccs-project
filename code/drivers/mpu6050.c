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

/* 与寄存器配置一致：1kHz/(1+SMPLRT_DIV=7) = 125Hz */
#define MPU6050_INT_DT_SEC        (1.0f / MPU6050_DATA_RATE_HZ)
#define MPU6050_ACCEL_XOUT_H_REG  (0x3BU)
#define MPU_RAD_TO_DEG            (57.2957795f)
#define MPU6050_PR_FILTER_ALPHA   (0.20f)

static volatile float g_gyro_z_bias = 0.0f;
static volatile float g_gyro_z_dps  = 0.0f;
static volatile float g_gyro_z_filt = 0.0f;
static volatile bool  g_gyro_filt_ready = false;
static volatile float g_z_angle_deg = 0.0f;
static volatile float g_roll_deg   = 0.0f;
static volatile float g_pitch_deg  = 0.0f;
static volatile bool  g_mpu_run_enable = false;
static volatile float g_turn_start_deg = 0.0f;
static volatile float g_roll_offset_deg = 0.0f;
static volatile float g_pitch_offset_deg = 0.0f;
static volatile bool  g_pr_filter_ready = false;

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

    /* SMPLRT_DIV=7 + DLPF(CONFIG=3) -> 125Hz DATA_READY */
    if (!mpu_i2c_write_byte(MPU6050_ADDR, MPU6050_SMPLRT_DIV_REG, 0x07U)) return false;
    if (!mpu_i2c_write_byte(MPU6050_ADDR, MPU6050_CONFIG_REG, 0x03U)) return false;
    if (!mpu_i2c_write_byte(MPU6050_ADDR, MPU6050_GYRO_CONFIG_REG, 0x08U)) return false;
    if (!mpu_i2c_write_byte(MPU6050_ADDR, MPU6050_ACCEL_CONFIG_REG, 0x00U)) return false;

    if (!mpu_i2c_write_byte(MPU6050_ADDR, MPU6050_INT_PIN_CFG_REG, 0x00U)) return false;
    if (!mpu_i2c_write_byte(MPU6050_ADDR, MPU6050_INT_ENABLE_REG, 0x01U)) return false;

    (void)mpu_i2c_read_byte(MPU6050_ADDR, MPU6050_INT_STATUS_REG, &dummy);

    g_gyro_z_bias     = 0.0f;
    g_gyro_z_dps      = 0.0f;
    g_gyro_z_filt     = 0.0f;
    g_gyro_filt_ready = false;
    g_z_angle_deg     = 0.0f;
    g_roll_deg        = 0.0f;
    g_pitch_deg       = 0.0f;
    g_pr_filter_ready = false;

    return true;
}

static float gyro_z_lpf_integrate(float gyro_dps)
{
    float filt;
    float dYaw;
    float maxD;

    if (!g_gyro_filt_ready) {
        g_gyro_z_filt     = gyro_dps;
        g_gyro_filt_ready = true;
    } else {
        g_gyro_z_filt = (GYRO_Z_LPF_ALPHA * gyro_dps)
                      + ((1.0f - GYRO_Z_LPF_ALPHA) * g_gyro_z_filt);
    }
    filt = g_gyro_z_filt;

    if ((filt > -(float)GYRO_Z_DEADZONE_DPS)
        && (filt < (float)GYRO_Z_DEADZONE_DPS)) {
        filt = 0.0f;
    }

    g_gyro_z_dps = filt;

    if (filt != 0.0f) {
        dYaw = filt * MPU6050_INT_DT_SEC;
        maxD = (float)GYRO_Z_DYAW_MAX_DEG;
        if (dYaw > maxD) {
            dYaw = maxD;
        } else if (dYaw < -maxD) {
            dYaw = -maxD;
        }

        g_z_angle_deg += dYaw;
        if (g_z_angle_deg > 180.0f) {
            g_z_angle_deg -= 360.0f;
        } else if (g_z_angle_deg < -180.0f) {
            g_z_angle_deg += 360.0f;
        }
    }

    return filt;
}

static void gyro_calib_sort_asc(float *a, uint32_t n)
{
    uint32_t i;
    uint32_t j;
    float t;

    for (i = 0U; i + 1U < n; i++) {
        for (j = i + 1U; j < n; j++) {
            if (a[j] < a[i]) {
                t    = a[i];
                a[i] = a[j];
                a[j] = t;
            }
        }
    }
}

bool mpu6050_calibrate_bias(void)
{
    static float s_calibBuf[MPU_CALIB_SAMPLES];
    uint32_t i;
    uint32_t n;
    uint32_t trim;
    uint32_t lo;
    uint32_t hi;
    int16_t  raw = 0;
    float    sum;

    g_mpu_run_enable  = false;
    g_gyro_filt_ready = false;

    n = (uint32_t)MPU_CALIB_SAMPLES;
    if (n > (uint32_t)(sizeof(s_calibBuf) / sizeof(s_calibBuf[0]))) {
        n = (uint32_t)(sizeof(s_calibBuf) / sizeof(s_calibBuf[0]));
    }

    for (i = 0U; i < n; i++) {
        if (!mpu6050_read_gyro_z_raw(&raw)) {
            return false;
        }
        s_calibBuf[i] = (float)raw / MPU_GYRO_SENS_500DPS;
        delay_ms(8U);
    }

    gyro_calib_sort_asc(s_calibBuf, n);

    trim = (uint32_t)MPU_CALIB_TRIM_EACH;
    if ((trim * 2U) >= n) {
        trim = (n / 10U);
    }
    lo  = trim;
    hi  = n - trim;
    sum = 0.0f;
    for (i = lo; i < hi; i++) {
        sum += s_calibBuf[i];
    }
    g_gyro_z_bias = sum / (float)(hi - lo);

    g_gyro_z_dps      = 0.0f;
    g_gyro_z_filt     = 0.0f;
    g_gyro_filt_ready = false;
    g_z_angle_deg     = 0.0f;

    g_mpu_run_enable = true;
    return true;
}

bool mpu6050_calibrate_level_offsets(uint16_t samples)
{
    uint32_t i;
    uint8_t buf[6];
    int16_t ax;
    int16_t ay;
    int16_t az;
    float fax;
    float fay;
    float faz;
    float roll;
    float pitch;
    float roll_sum = 0.0f;
    float pitch_sum = 0.0f;
    bool old_run_enable;

    if (samples < 20U) {
        samples = 20U;
    }

    old_run_enable = g_mpu_run_enable;
    g_mpu_run_enable = false;
    NVIC_DisableIRQ(GPIO_MULTIPLE_GPIOA_INT_IRQN);

    for (i = 0U; i < (uint32_t)samples; i++) {
        if (!mpu_i2c_read_bytes(MPU6050_ADDR, MPU6050_ACCEL_XOUT_H_REG, buf, 6U)) {
            g_mpu_run_enable = old_run_enable;
            if (old_run_enable) {
                NVIC_EnableIRQ(GPIO_MULTIPLE_GPIOA_INT_IRQN);
            }
            return false;
        }

        ax = (int16_t)(((uint16_t)buf[0] << 8) | buf[1]);
        ay = (int16_t)(((uint16_t)buf[2] << 8) | buf[3]);
        az = (int16_t)(((uint16_t)buf[4] << 8) | buf[5]);

        fax = (float)ax;
        fay = (float)ay;
        faz = (float)az;

        roll  = atan2f(fay, faz) * MPU_RAD_TO_DEG;
        pitch = atan2f(-fax, sqrtf(fay * fay + faz * faz)) * MPU_RAD_TO_DEG;

        roll_sum += roll;
        pitch_sum += pitch;

        delay_ms(5U);
    }

    g_roll_offset_deg = roll_sum / (float)samples;
    g_pitch_offset_deg = pitch_sum / (float)samples;

    g_roll_deg = 0.0f;
    g_pitch_deg = 0.0f;
    g_pr_filter_ready = false;

    g_mpu_run_enable = old_run_enable;
    if (old_run_enable) {
        NVIC_EnableIRQ(GPIO_MULTIPLE_GPIOA_INT_IRQN);
    }

    return true;
}

void mpu6050_on_data_ready_isr(void)
{
    uint8_t intStatus = 0;
    int16_t raw = 0;
    float   gyro;

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

    gyro = ((float)raw / MPU_GYRO_SENS_500DPS - g_gyro_z_bias) * GYRO_Z_SIGN;
    (void)gyro_z_lpf_integrate(gyro);
}

float mpu6050_get_gyro_z_dps(void)
{
    return (float)g_gyro_z_dps;
}

float mpu6050_get_z_angle_deg(void)
{
    return (float)g_z_angle_deg;
}

static void mpu6050_apply_pitch_roll_filter(float roll_raw, float pitch_raw)
{
    if (!g_pr_filter_ready) {
        g_roll_deg = roll_raw;
        g_pitch_deg = pitch_raw;
        g_pr_filter_ready = true;
    } else {
        g_roll_deg += MPU6050_PR_FILTER_ALPHA * (roll_raw - g_roll_deg);
        g_pitch_deg += MPU6050_PR_FILTER_ALPHA * (pitch_raw - g_pitch_deg);
    }
}

bool mpu6050_update_pitch_roll(void)
{
    uint8_t buf[6];
    int16_t ax;
    int16_t ay;
    int16_t az;
    float fax;
    float fay;
    float faz;
    float roll_raw;
    float pitch_raw;
    bool old_run_enable;
    bool ok;

    old_run_enable = g_mpu_run_enable;
    g_mpu_run_enable = false;
    NVIC_DisableIRQ(GPIO_MULTIPLE_GPIOA_INT_IRQN);

    ok = mpu_i2c_read_bytes(MPU6050_ADDR, MPU6050_ACCEL_XOUT_H_REG, buf, 6U);

    g_mpu_run_enable = old_run_enable;
    if (old_run_enable) {
        NVIC_EnableIRQ(GPIO_MULTIPLE_GPIOA_INT_IRQN);
    }

    if (!ok) {
        return false;
    }

    ax = (int16_t)(((uint16_t)buf[0] << 8) | buf[1]);
    ay = (int16_t)(((uint16_t)buf[2] << 8) | buf[3]);
    az = (int16_t)(((uint16_t)buf[4] << 8) | buf[5]);

    fax = (float)ax;
    fay = (float)ay;
    faz = (float)az;

    roll_raw  = atan2f(fay, faz) * MPU_RAD_TO_DEG - g_roll_offset_deg;
    pitch_raw = atan2f(-fax, sqrtf(fay * fay + faz * faz)) * MPU_RAD_TO_DEG - g_pitch_offset_deg;

    mpu6050_apply_pitch_roll_filter(roll_raw, pitch_raw);
    return true;
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
    float roll_raw;
    float pitch_raw;

    NVIC_DisableIRQ(GPIO_MULTIPLE_GPIOA_INT_IRQN);

    if (!mpu_i2c_read_bytes(MPU6050_ADDR, MPU6050_ACCEL_XOUT_H_REG, buf, 14U)) {
        return false;
    }

    (void)mpu_i2c_read_byte(MPU6050_ADDR, MPU6050_INT_STATUS_REG, &intStatus);

    ax = (int16_t)(((uint16_t)buf[0] << 8) | buf[1]);
    ay = (int16_t)(((uint16_t)buf[2] << 8) | buf[3]);
    az = (int16_t)(((uint16_t)buf[4] << 8) | buf[5]);
    gz = (int16_t)(((uint16_t)buf[12] << 8) | buf[13]);

    fax = (float)ax;
    fay = (float)ay;
    faz = (float)az;

    roll_raw  = atan2f(fay, faz) * MPU_RAD_TO_DEG - g_roll_offset_deg;
    pitch_raw = atan2f(-fax, sqrtf(fay * fay + faz * faz)) * MPU_RAD_TO_DEG - g_pitch_offset_deg;
    mpu6050_apply_pitch_roll_filter(roll_raw, pitch_raw);

    /*
     * 轮询路径仅更新姿态显示用的 roll/pitch；
     * Z 角积分只由 DATA_READY ISR 按 125Hz*INT_DT 做。
     */
    gyro = ((float)gz / MPU_GYRO_SENS_500DPS - g_gyro_z_bias) * GYRO_Z_SIGN;
    if ((gyro > -(float)GYRO_Z_DEADZONE_DPS) && (gyro < (float)GYRO_Z_DEADZONE_DPS)) {
        gyro = 0.0f;
    }
    g_gyro_z_dps = gyro;

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
    g_z_angle_deg     = 0.0f;
    g_gyro_z_filt     = 0.0f;
    g_gyro_filt_ready = false;
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

    if (!mpu6050_calibrate_level_offsets(200U)) {
        mpu6050_halt_fail("MPU level FAIL  ", uart_debug_print_mpu_calib_fail);
    }

    mpu6050_reset_z_angle();
}