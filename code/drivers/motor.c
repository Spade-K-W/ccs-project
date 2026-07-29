#include "motor.h"
#include "app_config.h"
#include "app_utils.h"
#include "number.h"
#include <stdbool.h>

/* =========================================================
 * 软件 PWM 计数器
 * 取值范围 0 ~ SOFT_PWM_PERIOD-1
 * ========================================================= */
static volatile uint8_t g_pwmCounter = 0U;
static bool g_motorDriverEnabled = false;
static volatile uint32_t g_systemMs = 0U;
static volatile uint16_t g_msDivider = 0U;

/* =========================================================
 * 根据电机当前 dir 设置方向引脚
 * dir > 0 : 正转
 * dir < 0 : 反转
 * dir = 0 : 停止
 * ========================================================= */
static void motor_apply_dir(Motor *m)
{
    if (m->dir > 0) {
        pin_high(&m->in1);
        pin_low(&m->in2);
    } else if (m->dir < 0) {
        pin_low(&m->in1);
        pin_high(&m->in2);
    } else {
        pin_low(&m->in1);
        pin_low(&m->in2);
        pin_low(&m->pwm);
    }
}

/* =========================================================
 * 设置单个电机速度
 * 1. 先把速度限制在 -100 ~ 100
 * 2. 如果该电机标记为反向，则速度取反
 * 3. 根据正负决定方向 dir
 * 4. duty 取速度绝对值
 * ========================================================= */
void motor_set_speed(Motor *m, int16_t speed)
{
    speed = clamp_i16(speed, -100, 100);

    /* 电机方向修正 */
    if (m->reversed) {
        speed = -speed;
    }

    if (speed > 0) {
        m->dir = 1;
        m->duty = (uint8_t)speed;
    } else if (speed < 0) {
        m->dir = -1;
        m->duty = (uint8_t)(-speed);
    } else {
        m->dir = 0;
        m->duty = 0U;
    }

    motor_apply_dir(m);
}

/* =========================================================
 * 设置底盘左右轮速度（物理左右）
 * 速度范围：-100 ~ 100（负值为倒转）
 * 物理左：M3 左前 + M4 左后（同速）
 * 物理右：M1 右后 + M2 右前（同速）
 * ========================================================= */
void chassis_set(int16_t leftSpeed, int16_t rightSpeed)
{
    leftSpeed  = clamp_i16(leftSpeed,  -100, 100);
    rightSpeed = clamp_i16(rightSpeed, -100, 100);

    motor_set_speed(&motor1, rightSpeed); /* M1 右后 */
    motor_set_speed(&motor2, rightSpeed); /* M2 右前 */
    motor_set_speed(&motor3, leftSpeed);  /* M3 左前 */
    motor_set_speed(&motor4, leftSpeed);  /* M4 左后 */
}

void chassis_set4(int16_t speedA, int16_t speedB, int16_t speedC, int16_t speedD)
{
    motor_set_speed(&motor1, clamp_i16(speedA, -100, 100));
    motor_set_speed(&motor2, clamp_i16(speedB, -100, 100));
    motor_set_speed(&motor3, clamp_i16(speedC, -100, 100));
    motor_set_speed(&motor4, clamp_i16(speedD, -100, 100));
}

/* 底盘停车 */
void chassis_stop(void)
{
    chassis_set(0, 0);
}

/* =========================================================
 * 初始化 SysTick，用作软件 PWM 定时中断，启动一个 20 kHz（50us） 的定时中断
 * ========================================================= */
void motor_pwm_init(void)
{   
    /* 拉高 TB6612 STBY，退出待机 */
    pin_high(&MOTOR_STBY);
    SysTick->CTRL = 0U;
    SysTick->LOAD = (CPU_CLOCK_HZ / SOFT_PWM_TICK_HZ) - 1U;
    SysTick->VAL  = 0U;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |
                    SysTick_CTRL_TICKINT_Msk   |
                    SysTick_CTRL_ENABLE_Msk;
    
    __enable_irq();
}

/* =========================================================
 * SysTick 中断服务函数
 * 作用：生成四个电机的“软件 PWM”
 * 原理：
 * - 计数器不断累加
 * - 如果 duty > 当前计数值，则 PWM 输出高
 * - 否则输出低
 * ========================================================= */
void SysTick_Handler(void)
{
    uint8_t i;
    /*
 * SOFT_PWM_TICK_HZ 当前为 20000 Hz，
 * 因此每 20 次中断累计 1 ms。
 */
    g_msDivider++;
    if (g_msDivider >= (uint16_t)(SOFT_PWM_TICK_HZ / 1000U)) {
        g_msDivider = 0U;
        g_systemMs++;
        number_refresh_1ms_isr();
    }
    g_pwmCounter++;

    /* 一个 PWM 周期结束后归零 */
    if (g_pwmCounter >= SOFT_PWM_PERIOD) {
        g_pwmCounter = 0U;
    }

    /* 依次刷新四个电机的 PWM 输出 */
    for (i = 0; i < 4; i++) {
        Motor *m = motors[i];

        if ((m->dir != 0) && (m->duty > g_pwmCounter)) {
            pin_high(&m->pwm);
        } else {
            pin_low(&m->pwm);
        }
    }
}

void motor_driver_enable(void)
{
    pin_high(&MOTOR_STBY);
    g_motorDriverEnabled = true;
}

void motor_driver_disable(void)
{
    chassis_stop();
    pin_low(&MOTOR_STBY);
    g_motorDriverEnabled = false;
}

bool motor_driver_is_enabled(void)
{
    return g_motorDriverEnabled;
}

void motor_init(void)
{
    /* 先保持 TB6612 待机 */
    pin_low(&MOTOR_STBY);
    g_motorDriverEnabled = false;

    /* 所有电机输出置安全状态 */
    chassis_stop();

    /* 初始化软件 PWM */
    motor_pwm_init();

    /* 最后拉高 STBY，退出待机 */
    motor_driver_enable();
}
uint32_t motor_millis(void)
{
    /*
     * MSPM0 为 32 位 MCU，对齐的 uint32_t 读取是原子的。
     * 使用无符号时间差还能正确处理约 49 天后的溢出。
     */
    return g_systemMs;
}
