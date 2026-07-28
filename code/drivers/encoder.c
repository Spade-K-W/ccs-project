#include "encoder.h"
#include "app_config.h"
#include "oled.h"
#include "ti_msp_dl_config.h"

#include <stdio.h>

/*
 * 左右各一路编码器（省 4 脚）：
 *   左 M1: A=PB19  B=PA31
 *   右 M4: A=PA3   B=PA4
 * 引脚由 SysConfig GPIO_GRP_ENCODER 配置
 */
#define ENC_A_IRQ_GPIOA  (GPIO_GRP_ENCODER_M4_ENCODER_A_PIN)
#define ENC_A_IRQ_GPIOB  (GPIO_GRP_ENCODER_M1_ENCODER_A_PIN)

/* 避免依赖 math.h 的 M_PI */
#define ENC_PI  (3.14159265358979323846f)

/* [0]=左 M1，[1]=右 M4 */
static volatile int32_t s_cnt[2] = {0, 0};
static int32_t s_cntPrev[2]      = {0, 0};
static float   s_leftSpeed       = 0.0f;  /* 脉冲增量/周期，供速度环 */
static float   s_rightSpeed      = 0.0f;
static float   s_leftSpeedCms    = 0.0f;  /* 物理速度 cm/s */
static float   s_rightSpeedCms   = 0.0f;
static float   s_vehicleSpeedCms = 0.0f;

/*
 * 脉冲 ↔ 距离换算（核心）
 *
 * 每脉冲路程(mm) = π × 直径 / 一圈脉冲数
 * 距离(mm)       = 脉冲数 × 每脉冲路程
 */
float encoder_mm_per_pulse(void)
{
    if (ENC_PULSES_PER_REV <= 0.0f) {
        return 0.0f;
    }
    return (ENC_PI * WHEEL_DIAMETER_MM) / ENC_PULSES_PER_REV;
}

float encoder_pulses_to_mm(float pulses)
{
    return pulses * encoder_mm_per_pulse();
}

float encoder_pulses_to_cm(float pulses)
{
    return encoder_pulses_to_mm(pulses) / 10.0f;
}

/*
 * 测速 = 本周期走过的距离 / 时间
 *   v(cm/s) = pulses_to_cm(dPulse) / dt
 */
float encoder_calc_speed_cm_s(float dPulse)
{
    float dtSec;
    float vCms;

    dtSec = (float)LOOP_PERIOD_MS / 1000.0f;
    if (dtSec <= 0.0f) {
        return 0.0f;
    }

    vCms = encoder_pulses_to_cm(dPulse) / dtSec;
    return (ENC_SPEED_FIT_K * vCms) + ENC_SPEED_FIT_B;
}

static float enc_pulses_to_cms(float dPulse)
{
    return encoder_calc_speed_cm_s(dPulse);
}

static bool enc_pin_high(GPIO_Regs *port, uint32_t pin)
{
    return (DL_GPIO_readPins(port, pin) != 0U);
}

static void enc_on_a_edge(volatile int32_t *count,
                          GPIO_Regs *aPort, uint32_t aPin,
                          GPIO_Regs *bPort, uint32_t bPin)
{
    bool a = enc_pin_high(aPort, aPin);
    bool b = enc_pin_high(bPort, bPin);

    if (a == b) {
        (*count)++;
    } else {
        (*count)--;
    }
}

void encoder_on_gpio_isr(void)
{
    uint32_t stA;
    uint32_t stB;

    stA = DL_GPIO_getEnabledInterruptStatus(GPIOA, ENC_A_IRQ_GPIOA);
    if (stA != 0U) {
        if ((stA & GPIO_GRP_ENCODER_M4_ENCODER_A_PIN) != 0U) {
            enc_on_a_edge(&s_cnt[1],
                GPIO_GRP_ENCODER_M4_ENCODER_A_PORT, GPIO_GRP_ENCODER_M4_ENCODER_A_PIN,
                GPIO_GRP_ENCODER_M4_ENCODER_B_PORT, GPIO_GRP_ENCODER_M4_ENCODER_B_PIN);
        }
        DL_GPIO_clearInterruptStatus(GPIOA, stA);
    }

    stB = DL_GPIO_getEnabledInterruptStatus(GPIOB, ENC_A_IRQ_GPIOB);
    if (stB != 0U) {
        if ((stB & GPIO_GRP_ENCODER_M1_ENCODER_A_PIN) != 0U) {
            enc_on_a_edge(&s_cnt[0],
                GPIO_GRP_ENCODER_M1_ENCODER_A_PORT, GPIO_GRP_ENCODER_M1_ENCODER_A_PIN,
                GPIO_GRP_ENCODER_M1_ENCODER_B_PORT, GPIO_GRP_ENCODER_M1_ENCODER_B_PIN);
        }
        DL_GPIO_clearInterruptStatus(GPIOB, stB);
    }
}

void encoder_init(void)
{
#if ENC_CALIB_POLL_MODE
    /*
     * 标定模式：关掉编码器 GPIO 中断，改由 encoder_poll() 数边沿，
     * 避免中断异常时 OLED 一直为 0；手转足够慢，10ms 轮询够用。
     * MPU6050 仍用 GPIOA 中断（mpu6050_startup 已 EnableIRQ）。
     */
    DL_GPIO_disableInterrupt(GPIOA, GPIO_GRP_ENCODER_M4_ENCODER_A_PIN);
    DL_GPIO_disableInterrupt(GPIOB, GPIO_GRP_ENCODER_M1_ENCODER_A_PIN);
#else
    NVIC_EnableIRQ(GPIO_MULTIPLE_GPIOA_INT_IRQN);
    NVIC_EnableIRQ(GPIO_GRP_ENCODER_GPIOB_INT_IRQN);
#endif

    encoder_reset();
}

/*
 * 主循环轮询左右 A 相边沿（仅 ENC_CALIB_POLL_MODE=1 时需要）
 */
void encoder_poll(void)
{
#if ENC_CALIB_POLL_MODE
    static bool s_prevA[2];
    static bool s_ready = false;
    bool nowA[2];
    uint8_t i;

    nowA[0] = enc_pin_high(GPIO_GRP_ENCODER_M1_ENCODER_A_PORT,
                           GPIO_GRP_ENCODER_M1_ENCODER_A_PIN);
    nowA[1] = enc_pin_high(GPIO_GRP_ENCODER_M4_ENCODER_A_PORT,
                           GPIO_GRP_ENCODER_M4_ENCODER_A_PIN);

    if (!s_ready) {
        for (i = 0U; i < 2U; i++) {
            s_prevA[i] = nowA[i];
        }
        s_ready = true;
        return;
    }

    if (nowA[0] != s_prevA[0]) {
        enc_on_a_edge(&s_cnt[0],
            GPIO_GRP_ENCODER_M1_ENCODER_A_PORT, GPIO_GRP_ENCODER_M1_ENCODER_A_PIN,
            GPIO_GRP_ENCODER_M1_ENCODER_B_PORT, GPIO_GRP_ENCODER_M1_ENCODER_B_PIN);
    }
    if (nowA[1] != s_prevA[1]) {
        enc_on_a_edge(&s_cnt[1],
            GPIO_GRP_ENCODER_M4_ENCODER_A_PORT, GPIO_GRP_ENCODER_M4_ENCODER_A_PIN,
            GPIO_GRP_ENCODER_M4_ENCODER_B_PORT, GPIO_GRP_ENCODER_M4_ENCODER_B_PIN);
    }

    for (i = 0U; i < 2U; i++) {
        s_prevA[i] = nowA[i];
    }
#else
    /* 运行模式靠中断计数，此处为空 */
#endif
}

void encoder_reset(void)
{
    uint8_t i;

    __disable_irq();
    for (i = 0U; i < 2U; i++) {
        s_cnt[i] = 0;
        s_cntPrev[i] = 0;
    }
    __enable_irq();

    s_leftSpeed       = 0.0f;
    s_rightSpeed      = 0.0f;
    s_leftSpeedCms    = 0.0f;
    s_rightSpeedCms   = 0.0f;
    s_vehicleSpeedCms = 0.0f;
}

void encoder_speed_restart(void)
{
    uint8_t i;
    int32_t now[2];

    /* 以当前脉冲为新基准，丢弃延时期间未采样的增量 */
    __disable_irq();
    for (i = 0U; i < 2U; i++) {
        now[i] = s_cnt[i];
        s_cntPrev[i] = now[i];
    }
    __enable_irq();

    s_leftSpeed       = 0.0f;
    s_rightSpeed      = 0.0f;
    s_leftSpeedCms    = 0.0f;
    s_rightSpeedCms   = 0.0f;
    s_vehicleSpeedCms = 0.0f;
}

void encoder_update(void)
{
    int32_t now[2];
    int32_t dL;
    int32_t dR;
    uint8_t i;
    float alpha = ENC_SPEED_FILTER_ALPHA;
    float rawLCms;
    float rawRCms;

    __disable_irq();
    for (i = 0U; i < 2U; i++) {
        now[i] = s_cnt[i];
    }
    __enable_irq();

    dL = now[1] - s_cntPrev[1]; /* M4 左后 → 物理左 */
    dR = now[0] - s_cntPrev[0]; /* M1 右后 → 物理右 */
    for (i = 0U; i < 2U; i++) {
        s_cntPrev[i] = now[i];
    }

    if (ENC_LEFT_REVERSE) {
        dL = -dL;
    }
    if (ENC_RIGHT_REVERSE) {
        dR = -dR;
    }

    /*
     * 速度 = 本周期脉冲增量 / 时间，不是累计位置。
     * 本周期无脉冲时直接清零，避免低通滤波拖尾让停下后 Speed 仍非 0。
     * 正转再反转到原位：看累计脉冲 Pos/OLED 计数应回 ~0，Speed 停下应为 0。
     */
    if ((dL == 0) && (dR == 0)) {
        s_leftSpeed       = 0.0f;
        s_rightSpeed      = 0.0f;
        s_leftSpeedCms    = 0.0f;
        s_rightSpeedCms   = 0.0f;
        s_vehicleSpeedCms = 0.0f;
    } else {
        s_leftSpeed  = (alpha * (float)dL) + ((1.0f - alpha) * s_leftSpeed);
        s_rightSpeed = (alpha * (float)dR) + ((1.0f - alpha) * s_rightSpeed);

        rawLCms = enc_pulses_to_cms((float)dL);
        rawRCms = enc_pulses_to_cms((float)dR);
        s_leftSpeedCms  = (alpha * rawLCms) + ((1.0f - alpha) * s_leftSpeedCms);
        s_rightSpeedCms = (alpha * rawRCms) + ((1.0f - alpha) * s_rightSpeedCms);
        s_vehicleSpeedCms = 0.5f * (s_leftSpeedCms + s_rightSpeedCms);
    }
}

void encoder_get_counts(EncoderCounts *out)
{
    if (out == NULL) {
        return;
    }
    __disable_irq();
    out->leftCount  = s_cnt[1]; /* M4 左后 */
    out->rightCount = s_cnt[0]; /* M1 右后 */
    __enable_irq();
}

void encoder_get_motor_counts(int32_t out[2])
{
    if (out == NULL) {
        return;
    }
    __disable_irq();
    out[0] = s_cnt[0];  /* M1 右后 */
    out[1] = s_cnt[1];  /* M4 左后 */
    __enable_irq();
}

void encoder_get_speeds(EncoderSpeeds *out)
{
    if (out == NULL) {
        return;
    }
    out->leftSpeed  = s_leftSpeed;
    out->rightSpeed = s_rightSpeed;
}

void encoder_get_physical_speeds(EncoderPhysicalSpeeds *out)
{
    if (out == NULL) {
        return;
    }
    out->leftSpeedCms    = s_leftSpeedCms;
    out->rightSpeedCms   = s_rightSpeedCms;
    out->vehicleSpeedCms = s_vehicleSpeedCms;
}

float encoder_get_vehicle_speed_cm_s(void)
{
    return s_vehicleSpeedCms;
}

/*
 * 左右平均累计脉冲（带方向）。正转再反转回原位时，该值应回到约 0。
 */
int32_t encoder_get_vehicle_pos_pulses(void)
{
    int32_t left;
    int32_t right;

    __disable_irq();
    left  = s_cnt[1]; /* M4 物理左 */
    right = s_cnt[0]; /* M1 物理右 */
    __enable_irq();

    if (ENC_LEFT_REVERSE) {
        left = -left;
    }
    if (ENC_RIGHT_REVERSE) {
        right = -right;
    }
    return (left + right) / 2;
}

void encoder_display_oled_uart(bool isStraight,
                               int16_t cmdLeft, int16_t cmdRight,
                               int16_t outLeft, int16_t outRight)
{
    char lineMode[24];
    char lineCmd[24];
    char lineOut[24];
    char linePulse[24];
    int32_t motorCnt[2];

    snprintf(lineMode, sizeof(lineMode), "%-16s",
             isStraight ? "straight" : "turn");
    snprintf(lineCmd, sizeof(lineCmd),
             "Cmd L:%3d R:%3d", (int)cmdLeft, (int)cmdRight);
    snprintf(lineOut, sizeof(lineOut),
             "Out L:%3d R:%3d", (int)outLeft, (int)outRight);

    /* 第 4 行：左右 A 相累计脉冲（手转 1 圈读数 → ENC_PULSES_PER_REV） */
    encoder_poll();
    encoder_get_motor_counts(motorCnt);
    snprintf(linePulse, sizeof(linePulse),
             "L:%5ld R:%5ld",
             (long)motorCnt[0], (long)motorCnt[1]);

    oled_display_string(0, 0, lineMode);
    oled_display_string(1, 0, lineCmd);
    oled_display_string(2, 0, lineOut);
    oled_display_string(3, 0, linePulse);
}
