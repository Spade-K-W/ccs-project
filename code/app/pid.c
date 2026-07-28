#include "pid.h"
#include "app_config.h"
#include "app_utils.h"
#include "encoder.h"
#include "mpu6050.h"

#include <stdbool.h>
#include <stddef.h>

/* ============================================================
 * 直线循迹 PD（红外误差）外环
 * ============================================================ */

static float s_lineErrFiltered = 0.0f;
static float s_lineErrPrev     = 0.0f;
static bool  s_dReady          = false;

/* 弧线循迹滤波 / D 状态（与直线分开，避免串扰） */
static float s_arcErrFiltered = 0.0f;
static float s_arcErrPrev     = 0.0f;
static bool  s_arcDReady      = false;

typedef struct {
    float errPrev;
    float integral;
    bool  ready;
} SpeedLoopState;

/* 直线：共模一路；转弯：左右各一路 */
static SpeedLoopState s_spdAvg = {0};
static SpeedLoopState s_spdL   = {0};
static SpeedLoopState s_spdR   = {0};
static bool           s_turnLoopActive = false;

static int16_t pid_round_i16(float x)
{
    if (x >= 0.0f) {
        return (int16_t)(x + 0.5f);
    }
    return (int16_t)(x - 0.5f);
}

static void speed_state_reset(SpeedLoopState *st)
{
    st->errPrev  = 0.0f;
    st->integral = 0.0f;
    st->ready    = false;
}

static int16_t pid_clamp_diff(int16_t diff)
{
    if (diff > (int16_t)STRAIGHT_DIFF_MAX) {
        return (int16_t)STRAIGHT_DIFF_MAX;
    }
    if (diff < -(int16_t)STRAIGHT_DIFF_MAX) {
        return -(int16_t)STRAIGHT_DIFF_MAX;
    }
    return diff;
}

static int16_t pid_clamp_diff_arc(int16_t diff)
{
    if (diff > (int16_t)ARC_DIFF_MAX) {
        return (int16_t)ARC_DIFF_MAX;
    }
    if (diff < -(int16_t)ARC_DIFF_MAX) {
        return -(int16_t)ARC_DIFF_MAX;
    }
    return diff;
}

static float pid_filter_error(float error)
{
    if ((error > -LINE_ERROR_DEADZONE) && (error < LINE_ERROR_DEADZONE)) {
        error = 0.0f;
    }

    s_lineErrFiltered = (LINE_ERROR_FILTER_ALPHA * error)
                      + ((1.0f - LINE_ERROR_FILTER_ALPHA) * s_lineErrFiltered);
    return s_lineErrFiltered;
}

static float pid_p_term(float filteredErr)
{
    return KP_LINE_STRAIGHT * filteredErr;
}

static float pid_d_term_error_rate(float filteredErr)
{
    float dErr;
    float dtSec;

    if (!s_dReady) {
        s_lineErrPrev = filteredErr;
        s_dReady      = true;
        return 0.0f;
    }

    dtSec = (float)LOOP_PERIOD_MS / 1000.0f;
    if (dtSec <= 0.0f) {
        return 0.0f;
    }

    dErr  = (filteredErr - s_lineErrPrev) / dtSec;
    s_lineErrPrev = filteredErr;

    return (float)KD_LINE_STRAIGHT * dErr;
}

void pid_line_reset(void)
{
    s_lineErrFiltered = 0.0f;
    s_lineErrPrev     = 0.0f;
    s_dReady          = false;

    s_arcErrFiltered = 0.0f;
    s_arcErrPrev     = 0.0f;
    s_arcDReady      = false;
}

int16_t pid_line_calc_diff(float error)
{
    float filteredErr = pid_filter_error(error);
    float pTerm       = pid_p_term(filteredErr);
    float dTerm       = pid_d_term_error_rate(filteredErr);

    return pid_clamp_diff((int16_t)(pTerm + dTerm));
}

int16_t pid_line_arc_calc_diff(float error, float mirrorDir)
{
    float filteredErr;
    float pTerm;
    float dTerm;
    float gyroTerm;
    float diffF;
    float dtSec;
    float dErr;
    float gyroZ;

    (void)mirrorDir; /* 弯道方向由 line_follow_arc 前馈处理，此处只做贴线 PD */

    /* 死区 + 低通 */
    if ((error > -(float)LINE_ERROR_DEADZONE_ARC)
        && (error < (float)LINE_ERROR_DEADZONE_ARC)) {
        error = 0.0f;
    }
    filteredErr = (LINE_ERROR_FILTER_ALPHA_ARC * error)
                + ((1.0f - LINE_ERROR_FILTER_ALPHA_ARC) * s_arcErrFiltered);
    s_arcErrFiltered = filteredErr;

    /* P：error>0（线偏右/CH678）→ 正差速 → 左快右慢 → 右转回线 */
    pTerm = (float)KP_LINE_ARC * filteredErr;

    /* D：抑制抖动 */
    if (!s_arcDReady) {
        s_arcErrPrev = filteredErr;
        s_arcDReady  = true;
        dTerm        = 0.0f;
    } else {
        dtSec = (float)LOOP_PERIOD_MS / 1000.0f;
        if (dtSec > 0.0f) {
            dErr  = (filteredErr - s_arcErrPrev) / dtSec;
            dTerm = (float)KD_LINE_ARC * dErr;
        } else {
            dTerm = 0.0f;
        }
        s_arcErrPrev = filteredErr;
    }

    /* 陀螺角速度阻尼（可选） */
    gyroZ    = mpu6050_get_gyro_z_dps();
    gyroTerm = -(float)KD_GYRO_ARC * (float)KD_GYRO_ARC_SIGN * gyroZ;

    diffF = pTerm + dTerm + gyroTerm;
    return pid_clamp_diff_arc(pid_round_i16(diffF));
}

void pid_accel_reset(void)
{
    speed_state_reset(&s_spdAvg);
    speed_state_reset(&s_spdL);
    speed_state_reset(&s_spdR);
    s_turnLoopActive = false;
}

/*
 * 通用单路速度 PID：corr = PID(cmd - mea)，带积分抗饱和
 */
static int16_t pid_speed_one(SpeedLoopState *st, float cmd, float mea,
                             float kp, float ki, float kd,
                             float iMax, int16_t corrMax)
{
    float dtSec;
    float err;
    float dErr;
    float corr;
    int16_t corrI16;
    bool saturated;

    dtSec = (float)LOOP_PERIOD_MS / 1000.0f;
    if (dtSec <= 0.0f) {
        return 0;
    }

    err = cmd - mea;

    if (!st->ready) {
        st->errPrev  = err;
        st->integral = 0.0f;
        st->ready    = true;
        return 0;
    }

    dErr = (err - st->errPrev) / dtSec;
    st->errPrev = err;

    corr = (kp * err) + (ki * st->integral) + (kd * dErr);
    corrI16 = clamp_i16(pid_round_i16(corr), -corrMax, corrMax);

    saturated = ((corrI16 >= corrMax) && (err > 0.0f))
             || ((corrI16 <= -corrMax) && (err < 0.0f));
    if (!saturated) {
        st->integral += err * dtSec;
        if (st->integral > iMax) {
            st->integral = iMax;
        } else if (st->integral < -iMax) {
            st->integral = -iMax;
        }
    }

    corr = (kp * err) + (ki * st->integral) + (kd * dErr);
    return clamp_i16(pid_round_i16(corr), -corrMax, corrMax);
}

/*
 * 直线共模：err = cmdAvg - meaAvg，左右同加 corr
 */
static int16_t pid_speed_common_corr(int16_t cmdL, int16_t cmdR,
                                     float spdL, float spdR)
{
    float cmdAvg;
    float meaAvg;

    if (PWM_TO_SPEED_GAIN <= 0.0f) {
        return 0;
    }

    /* 异号命令交给转弯环，直线环不处理 */
    if (((cmdL > 0) && (cmdR < 0)) || ((cmdL < 0) && (cmdR > 0))) {
        return 0;
    }

    cmdAvg = 0.5f * ((float)cmdL + (float)cmdR);
    if ((cmdAvg > -1.0f) && (cmdAvg < 1.0f)) {
        return 0;
    }

    /* 命令同向但实测左右异号对消：不可信 */
    if ((spdL * spdR) < 0.0f) {
        float absL = (spdL >= 0.0f) ? spdL : -spdL;
        float absR = (spdR >= 0.0f) ? spdR : -spdR;
        if ((absL > 2.0f) && (absR > 2.0f)) {
            return 0;
        }
    }

    meaAvg = 0.5f * (spdL + spdR) / PWM_TO_SPEED_GAIN;
    return pid_speed_one(&s_spdAvg, cmdAvg, meaAvg,
                         KP_ACCEL_STRAIGHT, KI_ACCEL_STRAIGHT, KD_ACCEL_STRAIGHT,
                         ACCEL_INTEGRAL_MAX_STRAIGHT,
                         (int16_t)ACCEL_PWM_CORR_MAX_STRAIGHT);
}

void pid_accel_apply(int16_t *leftPwm, int16_t *rightPwm)
{
#if !ACCEL_LOOP_ENABLE
    (void)leftPwm;
    (void)rightPwm;
#else
    EncoderSpeeds spd;
    int16_t corr;
    int16_t cmdL;
    int16_t cmdR;

    if ((leftPwm == NULL) || (rightPwm == NULL)) {
        return;
    }

    /* 转弯→直线：清积分一次 */
    if (s_turnLoopActive) {
        speed_state_reset(&s_spdL);
        speed_state_reset(&s_spdR);
        s_turnLoopActive = false;
    }

    cmdL = *leftPwm;
    cmdR = *rightPwm;
    encoder_get_speeds(&spd);
    corr = pid_speed_common_corr(cmdL, cmdR, spd.leftSpeed, spd.rightSpeed);

    *leftPwm  = clamp_i16((int16_t)(cmdL + corr), -100, 100);
    *rightPwm = clamp_i16((int16_t)(cmdR + corr), -100, 100);
#endif
}

void pid_accel_apply_turn(int16_t *leftPwm, int16_t *rightPwm)
{
#if !ACCEL_LOOP_ENABLE
    (void)leftPwm;
    (void)rightPwm;
#else
    EncoderSpeeds spd;
    float meaL;
    float meaR;
    int16_t corrL;
    int16_t corrR;
    int16_t cmdL;
    int16_t cmdR;
    int16_t ffL;
    int16_t ffR;

    if ((leftPwm == NULL) || (rightPwm == NULL)) {
        return;
    }

    if (PWM_TO_SPEED_GAIN <= 0.0f) {
        return;
    }

    /* 直线→转弯：清状态一次；转弯中保持积分连续 */
    if (!s_turnLoopActive) {
        speed_state_reset(&s_spdAvg);
        speed_state_reset(&s_spdL);
        speed_state_reset(&s_spdR);
        s_turnLoopActive = true;
    }

    cmdL = *leftPwm;
    cmdR = *rightPwm;
    encoder_get_speeds(&spd);
    meaL = spd.leftSpeed  / PWM_TO_SPEED_GAIN;
    meaR = spd.rightSpeed / PWM_TO_SPEED_GAIN;

    /* 开环预抬：原地摩擦大，先把 |Out| 抬到 |Cmd|+FF */
    ffL = (cmdL > 0) ? (int16_t)ACCEL_TURN_FF_BOOST
        : (cmdL < 0) ? -(int16_t)ACCEL_TURN_FF_BOOST : 0;
    ffR = (cmdR > 0) ? (int16_t)ACCEL_TURN_FF_BOOST
        : (cmdR < 0) ? -(int16_t)ACCEL_TURN_FF_BOOST : 0;

    corrL = pid_speed_one(&s_spdL, (float)cmdL, meaL,
                          KP_ACCEL_TURN, KI_ACCEL_TURN, KD_ACCEL_TURN,
                          ACCEL_INTEGRAL_MAX_TURN,
                          (int16_t)ACCEL_PWM_CORR_MAX_TURN);
    corrR = pid_speed_one(&s_spdR, (float)cmdR, meaR,
                          KP_ACCEL_TURN, KI_ACCEL_TURN, KD_ACCEL_TURN,
                          ACCEL_INTEGRAL_MAX_TURN,
                          (int16_t)ACCEL_PWM_CORR_MAX_TURN);

    *leftPwm  = clamp_i16((int16_t)(cmdL + ffL + corrL), -100, 100);
    *rightPwm = clamp_i16((int16_t)(cmdR + ffR + corrR), -100, 100);
#endif
}
