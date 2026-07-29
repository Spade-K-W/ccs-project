/* ============================================================
 * app_task.c
 * 当前仅保留循迹调试用的对外接口；比赛状态机见下方 #if 0
 * ============================================================ */

#include "app_task.h"
#include "app_config.h"
#include "app_utils.h"
#include "motor.h"
#include "pid.h"
#include "encoder.h"
#include "line_sensor.h"
#include "mpu6050.h"
#include "oled.h"
#include "uart_debug.h"
#include "uart_vision.h"
#include "bluetooth.h"
#include "Delay.h"

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <math.h>

/* 状态一：直线循迹；状态二：左转；状态三：右转 */
typedef enum {
    LINE_STATE_STRAIGHT = 1,
    LINE_STATE_LEFT_TURN,
    LINE_STATE_RIGHT_TURN
} LineDriveState;

static LineDriveState g_line_state = LINE_STATE_STRAIGHT;
static bool g_oledDebugEnabled = true;

/* CH123/CH678 路口累计：每 CORNERS_PER_LAP 次记 1 圈 */
static uint32_t g_corner_count = 0U;
static uint32_t g_lap_count    = 0U;

#if 0 /* 以下为整库未调用的比赛代码，暂禁用 */
#include "app_utils.h"
#include "signal.h"
#include "line_sensor.h"
#include "mpu6050.h"
#include "board_defs.h"
#include "oled.h"

/* ============================================================
 * 模式枚举（仅在本文件使用，不放头文件）
 * ============================================================ */
typedef enum {
    TEST_MODE_1_STOP_AT_B = 1,
    TEST_MODE_2_ONE_LAP   = 2,
    TEST_MODE_3_ONE_LAP   = 3,
    TEST_MODE_4_FOUR_LAPS = 4
} TestMode;

/* ============================================================
 * 模块全局变量（比赛状态机）
 * ============================================================ */
static TestMode  g_mode              = TEST_MODE_1_STOP_AT_B;
static RunPhase  g_phase             = PHASE_START;

static uint32_t  g_phaseTimeMs       = 0U;
static uint32_t  g_curveEntryCnt     = 0U;
static float     g_arcAccumDeg       = 0.0f;
static float     g_phaseStartAngle   = 0.0f;
static uint8_t   g_lapCount          = 0U;
static uint32_t  g_totalTimeMs       = 0U;

static int16_t   g_stepError         = 0;
static bool      g_stepVertexLike    = false;
static bool      g_stepLineValid     = false;

static float     g_straightHeading   = 0.0f;
static float     g_initHeading       = 0.0f;
#endif

/* 循迹调试仍使用 */
static int16_t   g_lastError         = 0;
static int16_t   g_cmdLeft           = 0;
static int16_t   g_cmdRight          = 0;
static int16_t   g_outLeft           = 0;  /* 速度环修正后实际下发 */
static int16_t   g_outRight          = 0;
static uint8_t   g_driveSpeedPercent = 100U;

/* Straight(锁航向) ↔ Arc(循迹) 演示状态机 */
typedef enum {
    DEMO_STATE_STRAIGHT = 0,
    DEMO_STATE_ARC
} DemoDriveState;

static DemoDriveState g_demo_state     = DEMO_STATE_STRAIGHT;
static float          g_holdHeadingDeg = 0.0f;
static float          g_demoArcMirror  = -1.0f;
static uint8_t        g_lineSeeCnt     = 0U;
static uint8_t        g_lineLostCnt    = 0U;
static float          g_arcYawAccumDeg = 0.0f; /* 入弧后累计偏航（可过 ±180） */
static float          g_arcYawLastDeg  = 0.0f;

static int16_t drive_scale_speed(int16_t speed)
{
    return (int16_t)(((int32_t)speed * (int32_t)g_driveSpeedPercent) / 100);
}

void app_task_set_speed_percent(uint8_t percent)
{
    if (percent < 1U) {
        percent = 1U;
    } else if (percent > 100U) {
        percent = 100U;
    }
    g_driveSpeedPercent = percent;
}

void app_task_set_oled_debug_enabled(bool enabled)
{
    g_oledDebugEnabled = enabled;
}

/*
 * OLED / 串口六行（与 uart_debug 对齐）：
 *  0 Straight / Arc
 *  1 巡线结果 00011000（CH1..CH8）
 *  2 Cmd 速度
 *  3 编码器实测 Mea
 *  4 速度环补偿 Corr = Out - Cmd
 *  5 陀螺仪 Z 角
 */
static void line_follow_refresh_ui(uint8_t pattern, float error, bool lineValid,
                                  const char *mode)
{
    char lineState[24];
    char linePat[24];
    char lineCmd[24];
    char lineMea[24];
    char lineCorr[24];
    char lineAngle[24];
    char bits[9];
    uint8_t i;
    EncoderSpeeds spd;
    float meaL;
    float meaR;
    int16_t corrL;
    int16_t corrR;
    UartDebugStatus dbg;

    (void)error;
    (void)lineValid;

    encoder_get_speeds(&spd);
    if (PWM_TO_SPEED_GAIN > 0.0f) {
        meaL = spd.leftSpeed  / PWM_TO_SPEED_GAIN;
        meaR = spd.rightSpeed / PWM_TO_SPEED_GAIN;
    } else {
        meaL = spd.leftSpeed;
        meaR = spd.rightSpeed;
    }

    corrL = (int16_t)(g_outLeft  - g_cmdLeft);
    corrR = (int16_t)(g_outRight - g_cmdRight);

    for (i = 0U; i < 8U; i++) {
        bits[i] = ((pattern & (1U << i)) != 0U) ? '1' : '0';
    }
    bits[8] = '\0';

    snprintf(lineState, sizeof(lineState), "%-16s",
             (mode != NULL) ? mode : "Straight");
    snprintf(linePat, sizeof(linePat), "%s        ", bits);
    snprintf(lineCmd, sizeof(lineCmd),
             "Cmd L:%3d R:%3d", (int)g_cmdLeft, (int)g_cmdRight);
    snprintf(lineMea, sizeof(lineMea),
             "Mea L:%4.1f R:%4.1f", meaL, meaR);
    snprintf(lineCorr, sizeof(lineCorr),
             "CorrL:%+3d R:%+3d", (int)corrL, (int)corrR);
    /* Arc 时显示累计偏航（出弯判据），Straight 显示绝对 Z */
    if (g_demo_state == DEMO_STATE_ARC) {
        snprintf(lineAngle, sizeof(lineAngle), "A:%+7.1f deg   ",
                 g_arcYawAccumDeg);
    } else {
        snprintf(lineAngle, sizeof(lineAngle), "Z:%+7.1f deg   ",
                 mpu6050_get_z_angle_deg());
    }

    if (g_oledDebugEnabled) {
        oled_display_string(0, 0, lineState);
        oled_display_string(1, 0, linePat);
        oled_display_string(2, 0, lineCmd);
        oled_display_string(3, 0, lineMea);
        oled_display_string(4, 0, lineCorr);
        oled_display_string(5, 0, lineAngle);
    }

    uart_debug_build_status6(&dbg, mode, pattern,
                             g_cmdLeft, g_cmdRight,
                             meaL, meaR, corrL, corrR,
                             mpu6050_get_z_angle_deg());
    uart_debug_print_status(&dbg);
#if BLUETOOTH_ENABLE
    bluetooth_cache_oled_status(&dbg);
#endif
}

/* 外环目标 PWM → 直线共模速度环 → 下发 */
static void chassis_set_with_accel(int16_t left, int16_t right)
{
    int16_t outL = left;
    int16_t outR = right;

    encoder_update();
    pid_accel_apply(&outL, &outR);

    g_cmdLeft  = left;
    g_cmdRight = right;
    g_outLeft  = outL;
    g_outRight = outR;
    chassis_set(outL, outR);
}

/* 外环目标 PWM → 转弯左右独立速度环 → 下发 */
static void chassis_set_with_accel_turn(int16_t left, int16_t right)
{
    int16_t outL = left;
    int16_t outR = right;

    encoder_update();
    pid_accel_apply_turn(&outL, &outR);

    g_cmdLeft  = left;
    g_cmdRight = right;
    g_outLeft  = outL;
    g_outRight = outR;
    chassis_set(outL, outR);
}

/* 开环下发（不走速度环），用于全亮/十字等测速不可靠场景 */
static void chassis_set_openloop(int16_t left, int16_t right)
{
    encoder_update();
    pid_accel_reset();
    g_cmdLeft  = left;
    g_cmdRight = right;
    g_outLeft  = left;
    g_outRight = right;
    chassis_set(left, right);
}

/* 八路全亮或左右侧同时亮：停车或开环直行，避免速度环误补导致飞转 */
static void line_follow_all_on_or_cross(uint8_t pattern, float error, bool allOn)
{
#if LINE_ALL_ON_STOP
    if (allOn) {
        chassis_set_openloop(0, 0);
        line_follow_refresh_ui(pattern, error, true, "all-on stop     ");
        return;
    }
#else
    (void)allOn;
#endif
    chassis_set_openloop((int16_t)BASE_SPEED_STRAIGHT,
                         (int16_t)BASE_SPEED_STRAIGHT);
    line_follow_refresh_ui(pattern, error, true, "cross straight  ");
}

/* ============================================================
 * line_follow_drive — 直线循迹驱动入口
 *
 * 根据红外误差决定左右轮速度并立即下发电机。
 * - lineValid=true : 走 PD（pid.c：红外 P + d(error)/dt 的 D），
 *   得到差速 diff，再合成 L=BASE+diff、R=BASE-diff。
 * - lineValid=false: 丢线，按上次误差方向用慢速搜索占空比
 *   （SEARCH_SPEED_LOW/HIGH）转弯找线，不走 PD。
 *
 * 同时把本次命令速度存到 g_cmdLeft/g_cmdRight，并刷新 OLED/串口。
 * ============================================================ */
void line_follow_drive(uint8_t pattern, float error, bool lineValid)
{
    int16_t diff;
    int16_t left;
    int16_t right;

    if (!lineValid) {
        if (g_lastError <= 0) {
            left  = (int16_t)SEARCH_SPEED_LOW;
            right = (int16_t)SEARCH_SPEED_HIGH;
        } else {
            left  = (int16_t)SEARCH_SPEED_HIGH;
            right = (int16_t)SEARCH_SPEED_LOW;
        }
        if (!pid_is_key3_profile()) {
            left  = drive_scale_speed(left);
            right = drive_scale_speed(right);
        }
        chassis_set_with_accel(left, right);
        line_follow_refresh_ui(pattern, error, lineValid, "go straight     ");
        return;
    }

    g_lastError = (int16_t)error;

    diff  = pid_line_calc_diff(error);
    left  = (int16_t)BASE_SPEED_STRAIGHT + diff;
    right = (int16_t)BASE_SPEED_STRAIGHT - diff;
    if (left < 0) {
        left = 0;
    }
    if (right < 0) {
        right = 0;
    }
    left  = drive_scale_speed(left);
    right = drive_scale_speed(right);
    chassis_set_with_accel(left, right);
    line_follow_refresh_ui(pattern, error, lineValid, "go straight     ");
}

/* 弧线循迹：入弧辅助计时与方向 */
static uint32_t g_arcPhaseMs    = 0U;
static uint32_t g_arcStartMs    = 0U;
static uint32_t g_arcCoastMs    = 0U; /* 末端 coast 已持续时长 */
static float    g_arcMirrorDir  = 1.0f;
static int16_t  g_arcCoastLeft  = 0; /* 贴线最后一拍 Cmd，弧末端线灭后沿用 */
static int16_t  g_arcCoastRight = 0;
static bool     g_arcRecovering = false; /* 异常走出线，原地找回中 */
static bool     g_arcCoasting   = false; /* 弧线自然结束，保持差速滑行中 */

void app_task_arc_prepare(float mirrorDir)
{
    int16_t bias;

    g_arcPhaseMs     = 0U;
    g_arcStartMs     = motor_millis();
    g_arcCoastMs     = 0U;
    g_arcMirrorDir   = (mirrorDir >= 0.0f) ? 1.0f : -1.0f;
    g_arcYawLastDeg  = mpu6050_get_z_angle_deg();
    g_arcYawAccumDeg = 0.0f;
    g_arcRecovering  = false;
    g_arcCoasting    = false;

    bias = (int16_t)ARC_CURVE_BIAS;
    if (g_arcMirrorDir >= 0.0f) {
        g_arcCoastLeft  = (int16_t)BASE_SPEED_ARC - bias;
        g_arcCoastRight = (int16_t)BASE_SPEED_ARC + bias;
    } else {
        g_arcCoastLeft  = (int16_t)BASE_SPEED_ARC + bias;
        g_arcCoastRight = (int16_t)BASE_SPEED_ARC - bias;
    }
    if (g_arcCoastLeft < 0) {
        g_arcCoastLeft = 0;
    }
    if (g_arcCoastRight < 0) {
        g_arcCoastRight = 0;
    }
    g_arcCoastLeft  = drive_scale_speed(g_arcCoastLeft);
    g_arcCoastRight = drive_scale_speed(g_arcCoastRight);

    pid_line_reset();
    pid_accel_reset();
    encoder_reset();
}

static void arc_apply_coast(uint8_t pattern, float errF, bool angleDone)
{
    g_arcCoasting   = true;
    g_arcRecovering = false;
    g_arcCoastMs   += (uint32_t)LOOP_PERIOD_MS;
    chassis_set_openloop(g_arcCoastLeft, g_arcCoastRight);
    line_follow_refresh_ui(pattern, errF, false,
                           angleDone ? "Arc end wait    " : "Arc coast       ");
}

/*
 * line_follow_arc — 弧线循迹
 *
 * 有线：弯道前馈 + 红外 PD
 * 线灭：
 *   |累计角| ≥ ARC_END_COAST_DEG → 弧线自然结束，沿用上一拍轮速（必须 coast）
 *   否则 → 异常走出线，原地差速找回
 * 切 Straight 由 demo_step：angleOk + 已 coast 够久 + 线灭，不得线灭瞬间直切
 */
void line_follow_arc(uint8_t pattern, float error, bool lineValid)
{
    int16_t pd;
    int16_t feedforward;
    int16_t steer;
    int16_t spdL;
    int16_t spdR;
    int16_t keepMin;
    int16_t enterAssistBias;
    float   errF;
    float   absYaw;
    bool    valid;
    bool    leftArc;
    bool    angleDone;
    bool    nearArcEnd;

    g_arcPhaseMs = motor_millis() - g_arcStartMs;
    /*
     * 弧线准备段：进入弧线状态后的前 ARC_LINE_ONLY_MS，
     * 仍使用普通红外巡线，不施加任何强制左/右转前馈。
     */
    if (g_arcPhaseMs < (uint32_t)ARC_LINE_ONLY_MS) {
        float straightError = 0.0f;
        bool straightValid = line_calc_error_f(pattern, &straightError);

        line_follow_drive(pattern, straightError, straightValid);
        return;
    }

    leftArc    = (g_arcMirrorDir >= 0.0f);
    keepMin    = (int16_t)ARC_STEER_KEEP_MIN;
    absYaw     = fabsf(g_arcYawAccumDeg);
    angleDone  = (absYaw >= (float)ARC_TARGET_DEG);
    nearArcEnd = (absYaw >= (float)ARC_END_COAST_DEG);

    valid = line_calc_error_arc_f(pattern, &errF);
    (void)error;
    (void)lineValid;

    /* 已在末端 coast：只要仍无线，继续保持上一拍差速（哪怕角已满） */
    if (g_arcCoasting && !valid) {
        arc_apply_coast(pattern, errF, angleDone);
        return;
    }
    if (g_arcCoasting && valid) {
        /* 末端又摸到线：退出 coast，恢复贴线 */
        g_arcCoasting = false;
        g_arcCoastMs  = 0U;
    }

    if (g_arcRecovering) {
        if (valid) {
            g_arcRecovering = false;
        } else if (nearArcEnd || angleDone) {
            arc_apply_coast(pattern, errF, angleDone);
            return;
        } else {
            if (g_lastError < 0) {
                spdL = (int16_t)ARC_LOST_SPEED_L;
                spdR = (int16_t)ARC_LOST_SPEED_R;
            } else {
                spdL = (int16_t)ARC_LOST_SPEED_R;
                spdR = (int16_t)ARC_LOST_SPEED_L;
            }
            spdL = drive_scale_speed(spdL);
            spdR = drive_scale_speed(spdR);
            chassis_set_openloop(spdL, spdR);
            line_follow_refresh_ui(pattern, errF, false, "Arc recover     ");
            return;
        }
    }

    if (!valid) {
        if (nearArcEnd || angleDone) {
            arc_apply_coast(pattern, errF, angleDone);
            return;
        }

        g_arcRecovering = true;
        if (g_lastError < 0) {
            spdL = (int16_t)ARC_LOST_SPEED_L;
            spdR = (int16_t)ARC_LOST_SPEED_R;
        } else {
            spdL = (int16_t)ARC_LOST_SPEED_R;
            spdR = (int16_t)ARC_LOST_SPEED_L;
        }
        spdL = drive_scale_speed(spdL);
        spdR = drive_scale_speed(spdR);
        chassis_set_openloop(spdL, spdR);
        line_follow_refresh_ui(pattern, errF, false, "Arc recover     ");
        return;
    }

    g_lastError = (int16_t)errF;
    pd = pid_line_arc_calc_diff(errF, 1.0f);
    enterAssistBias = pid_is_key3_profile()
        ? (int16_t)ARC_ENTER_TURN_BIAS_KEY3
        : (int16_t)ARC_ENTER_TURN_BIAS;

    if (leftArc) {
        feedforward = -(int16_t)ARC_CURVE_BIAS;
        if (g_arcPhaseMs <=
            ((uint32_t)ARC_LINE_ONLY_MS + (uint32_t)ARC_ENTER_ASSIST_MS)) {
            feedforward -= enterAssistBias;
        }
    } else {
        feedforward = (int16_t)ARC_CURVE_BIAS;
        if (g_arcPhaseMs <=
            ((uint32_t)ARC_LINE_ONLY_MS + (uint32_t)ARC_ENTER_ASSIST_MS)) {
            feedforward += enterAssistBias;
        }
    }

    steer = feedforward + pd;
    if (steer > (int16_t)ARC_DIFF_MAX) {
        steer = (int16_t)ARC_DIFF_MAX;
    } else if (steer < -(int16_t)ARC_DIFF_MAX) {
        steer = -(int16_t)ARC_DIFF_MAX;
    }

    if (leftArc) {
        if (steer > -keepMin) {
            steer = -keepMin;
        }
    } else {
        if (steer < keepMin) {
            steer = keepMin;
        }
    }

    spdL = (int16_t)BASE_SPEED_ARC + steer;
    spdR = (int16_t)BASE_SPEED_ARC - steer;
    if (spdL < 0) {
        spdL = 0;
    }
    if (spdR < 0) {
        spdR = 0;
    }
    spdL = drive_scale_speed(spdL);
    spdR = drive_scale_speed(spdR);

    g_arcCoastLeft  = spdL;
    g_arcCoastRight = spdR;

    chassis_set_with_accel(spdL, spdR);
    line_follow_refresh_ui(pattern, errF, true, "Arc             ");
}

/* ============================================================
 * line_follow_get_wheels — 读取最近一次轮速命令
 *
 * 不改变电机状态，只把最近下发的左/右轮占空比通过指针返回。
 * left/right 可为 NULL（对应一侧不读）。
 * ============================================================ */
void line_follow_get_wheels(int16_t *left, int16_t *right)
{
    if (left != NULL) {
        *left = g_cmdLeft;
    }
    if (right != NULL) {
        *right = g_cmdRight;
    }
}

void line_follow_get_out_wheels(int16_t *left, int16_t *right)
{
    if (left != NULL) {
        *left = g_outLeft;
    }
    if (right != NULL) {
        *right = g_outRight;
    }
}

void line_follow_left_turn(uint8_t pattern, float error, bool lineValid)
{
    (void)lineValid; /* 转弯中不检测、不执行丢线保护 */

    /* 左转：左内侧慢倒、右外侧快进（|R| > |L|） */
    chassis_set_with_accel_turn((int16_t)(-(int16_t)TURN_INNER_SPEED),
                                (int16_t)TURN_OUTER_SPEED);
    /* UI 强制按有线显示，避免转弯过程刷 LOST */
    line_follow_refresh_ui(pattern, error, true, "turn left       ");
}

void line_follow_right_turn(uint8_t pattern, float error, bool lineValid)
{
    (void)lineValid; /* 转弯中不检测、不执行丢线保护 */

    /* 右转：左外侧快进、右内侧慢倒（|L| > |R|） */
    chassis_set_with_accel_turn((int16_t)TURN_OUTER_SPEED,
                                (int16_t)(-(int16_t)TURN_INNER_SPEED));
    /* UI 强制按有线显示，避免转弯过程刷 LOST */
    line_follow_refresh_ui(pattern, error, true, "turn right      ");
}

/* 检测到路口侧灯后：先按当前循迹轮速直行预延时，再进入转弯 */
static void app_task_on_corner_detected(void)
{
    g_corner_count++;
    if (g_corner_count >= (uint32_t)CORNERS_PER_LAP) {
        g_corner_count = 0U;
        g_lap_count++;
    }
}

static void line_turn_after_detect_delay(uint8_t pattern, float error, bool lineValid,
                                         bool turn_left)
{
    uint32_t elapsedMs = 0U;
    float    holdSpeed;

    /* 每次 CH123 或 CH678 触发转弯，计一次路口 */
    app_task_on_corner_detected();

    /* 先下发行车速度（内部会 encoder_update），记下此刻车速 */
    line_follow_drive(pattern, error, lineValid);
    holdSpeed = encoder_get_vehicle_speed_cm_s();
    uart_vision_hold_speed(holdSpeed);

    /*
     * 预延时：切片 Delay，不卡死整段。
     * 视觉帧由 TIMG0 中断继续发；hold_speed 冻结速度，角度仍实时。
     * 不调用 encoder_update，避免用错误 dt 测速。
     */
    while (elapsedMs < (uint32_t)TURN_DETECT_DELAY_MS) {
        Delay_ms((uint32_t)LOOP_PERIOD_MS);
        elapsedMs += (uint32_t)LOOP_PERIOD_MS;
    }

    /* 延时结束 → 进入转弯：丢掉延时期间脉冲，立刻重新测速 */
    encoder_speed_restart();
    uart_vision_release_speed();

    mpu6050_turn_mark_start();
    if (turn_left) {
        g_line_state = LINE_STATE_LEFT_TURN;
        line_follow_left_turn(pattern, error, lineValid);
    } else {
        g_line_state = LINE_STATE_RIGHT_TURN;
        line_follow_right_turn(pattern, error, lineValid);
    }
}

static void app_task_on_turn_finished(void)
{
    g_line_state = LINE_STATE_STRAIGHT;
    app_task_straight_prepare(0.0f);
}

void app_task_line_step(uint8_t pattern, float error, bool lineValid)
{
    bool ch234;
    bool ch678;
    bool allOn;

    switch (g_line_state) {
        case LINE_STATE_STRAIGHT:
            ch234 = line_ch234_all_on(pattern);
            ch678 = line_ch678_all_on(pattern);
            allOn = line_all_on(pattern);

            /* 八路全亮优先：停车/开环，避免速度环把空转当停转猛补 */
            if (allOn || (ch234 && ch678)) {
                line_follow_all_on_or_cross(pattern, error, allOn);
            } else if (ch234 && !ch678) {
                line_turn_after_detect_delay(pattern, error, lineValid, true);
            } else if (ch678 && !ch234) {
                line_turn_after_detect_delay(pattern, error, lineValid, false);
            } else {
                line_follow_drive(pattern, error, lineValid);
            }
            break;

        case LINE_STATE_LEFT_TURN:
            /* 转弯中又全亮：多半压在大黑块上原地打转，退出转弯 */
            if (line_all_on(pattern)) {
                app_task_on_turn_finished();
                line_follow_all_on_or_cross(pattern, error, true);
                break;
            }
            line_follow_left_turn(pattern, error, lineValid);
            if (mpu6050_turn_yaw_reached_90()) {
                app_task_on_turn_finished();
            }
            break;

        case LINE_STATE_RIGHT_TURN:
            if (line_all_on(pattern)) {
                app_task_on_turn_finished();
                line_follow_all_on_or_cross(pattern, error, true);
                break;
            }
            line_follow_right_turn(pattern, error, lineValid);
            if (mpu6050_turn_yaw_reached_90()) {
                app_task_on_turn_finished();
            }
            break;

        default:
            g_line_state = LINE_STATE_STRAIGHT;
            line_follow_drive(pattern, error, lineValid);
            break;
    }
}

bool app_task_line_is_straight(void)
{
    return (g_line_state == LINE_STATE_STRAIGHT);
}

/* ============================================================
 * 对外接口：走直前设置目标航向（mpu6050 校准后通常传 0，或任意值表示“锁当前角”）
 * ============================================================ */
void app_task_straight_prepare(float targetHeadingDeg)
{
    g_lastError  = 0;
    g_line_state = LINE_STATE_STRAIGHT;
    /* targetHeadingDeg 未用绝对值：演示里以当前角为基准 */
    (void)targetHeadingDeg;
    g_holdHeadingDeg = mpu6050_get_z_angle_deg();
    g_demo_state     = DEMO_STATE_STRAIGHT;
    g_lineSeeCnt     = 0U;
    g_lineLostCnt    = 0U;
    pid_line_reset();
    pid_accel_reset();
    encoder_reset();
}

static float angle_diff_deg_live(float now, float base)
{
    float d = now - base;
    if (d > 180.0f) {
        d -= 360.0f;
    }
    if (d < -180.0f) {
        d += 360.0f;
    }
    return d;
}

static void app_task_arc_update_yaw(void)
{
    float yawNow = mpu6050_get_z_angle_deg();

    g_arcYawAccumDeg += angle_diff_deg_live(yawNow, g_arcYawLastDeg);
    g_arcYawLastDeg   = yawNow;
}

void app_task_arc_step(uint8_t pattern, float error, bool lineValid)
{
    app_task_arc_update_yaw();
    line_follow_arc(pattern, error, lineValid);
}

/* 不循线：陀螺仪锁航向直行 + 速度内环 */
static void gyro_heading_straight_drive(uint8_t pattern)
{
    float   headingErr;
    int16_t diff;
    int16_t left;
    int16_t right;

    headingErr = angle_diff_deg_live(mpu6050_get_z_angle_deg(), g_holdHeadingDeg);
    if ((headingErr > -HEADING_ERROR_DEADZONE_DEG)
        && (headingErr < HEADING_ERROR_DEADZONE_DEG)) {
        headingErr = 0.0f;
    }

    diff = (int16_t)(K_HEADING_STRAIGHT * headingErr);
    if (diff > (int16_t)STRAIGHT_DIFF_MAX) {
        diff = (int16_t)STRAIGHT_DIFF_MAX;
    } else if (diff < -(int16_t)STRAIGHT_DIFF_MAX) {
        diff = -(int16_t)STRAIGHT_DIFF_MAX;
    }

    left  = (int16_t)BASE_SPEED_STRAIGHT + diff;
    right = (int16_t)BASE_SPEED_STRAIGHT - diff;
    if (left < 0) {
        left = 0;
    }
    if (right < 0) {
        right = 0;
    }

    left  = drive_scale_speed(left);
    right = drive_scale_speed(right);
    chassis_set_with_accel(left, right);
    line_follow_refresh_ui(pattern, 0.0f, false, "Straight        ");
}

void app_task_demo_prepare(float arcMirrorDir)
{
    g_demoArcMirror  = (arcMirrorDir >= 0.0f) ? 1.0f : -1.0f;
    g_demo_state     = DEMO_STATE_STRAIGHT;
    g_holdHeadingDeg = mpu6050_get_z_angle_deg();
    g_lineSeeCnt     = 0U;
    g_lineLostCnt    = 0U;
    g_lastError      = 0;
    g_line_state     = LINE_STATE_STRAIGHT;
    pid_line_reset();
    pid_accel_reset();
    encoder_reset();
}

void app_task_demo_step(void)
{
    uint8_t pattern;
    float   error;
    bool    lineValid;
    bool    anyLine;
    bool    angleOk;
    bool    lostOk;
    bool    timeOk;
    bool    coastOk;

    pattern   = line_read_pattern();
    anyLine   = (pattern != 0U);
    lineValid = line_calc_error_arc_f(pattern, &error);

    switch (g_demo_state) {
    case DEMO_STATE_STRAIGHT:
        if (anyLine) {
            g_lineSeeCnt++;
            g_lineLostCnt = 0U;
        } else {
            g_lineSeeCnt = 0U;
        }

        if (g_lineSeeCnt >= (uint8_t)LINE_DETECT_HOLD_CNT) {
            g_demo_state  = DEMO_STATE_ARC;
            g_lineSeeCnt  = 0U;
            g_lineLostCnt = 0U;
            app_task_arc_prepare(g_demoArcMirror);
            line_follow_arc(pattern, error, lineValid);
        } else {
            gyro_heading_straight_drive(pattern);
        }
        break;

    case DEMO_STATE_ARC:
        /* 累计入弧后偏航（逐步积分，可超过 ±180） */
        app_task_arc_update_yaw();

        /*
         * 仅在接近弯末（|A|≥ARC_END_COAST_DEG）才累计线灭：
         * 避免弯中途闪断把 lostOk 攒满，积分一到就直切 Straight。
         */
        if (!anyLine
            && (fabsf(g_arcYawAccumDeg) >= (float)ARC_END_COAST_DEG)) {
            g_lineLostCnt++;
            g_lineSeeCnt = 0U;
        } else if (anyLine) {
            g_lineLostCnt = 0U;
        }

        angleOk = (fabsf(g_arcYawAccumDeg) >= (float)ARC_TARGET_DEG);
        lostOk  = (g_lineLostCnt >= (uint8_t)LINE_LOST_HOLD_CNT);
        timeOk  = (g_arcPhaseMs >= (uint32_t)ARC_MIN_TIME_MS);
        /* 必须先经历 Arc coast，且滑行够 ARC_COAST_MIN_MS */
        coastOk = g_arcCoasting
               && (g_arcCoastMs >= (uint32_t)ARC_COAST_MIN_MS);

        /*
         * Arc→Straight：角满 + 末端线灭 + 最短弧时 + 已 coast 够久。
         * 线灭瞬间即使角已满，也会先 Arc coast 保持差速，不会马上 Straight。
         */
        if (angleOk && lostOk && timeOk && coastOk) {
            g_demo_state     = DEMO_STATE_STRAIGHT;
            g_lineLostCnt    = 0U;
            g_lineSeeCnt     = 0U;
            g_arcCoasting    = false;
            g_arcCoastMs     = 0U;
            g_holdHeadingDeg = mpu6050_get_z_angle_deg();
            pid_line_reset();
            pid_accel_reset();
            gyro_heading_straight_drive(pattern);
        } else {
            line_follow_arc(pattern, error, lineValid);
        }
        break;

    default:
        g_demo_state = DEMO_STATE_STRAIGHT;
        gyro_heading_straight_drive(pattern);
        break;
    }
}

uint32_t app_task_get_lap_count(void)
{
    return g_lap_count;
}

void app_task_reset_lap_count(void)
{
    g_corner_count = 0U;
    g_lap_count    = 0U;
}

#if 0 /* 以下为整库未调用的比赛代码，暂禁用 */
/* ============================================================
 * 对外接口：查询是否完成
 * ============================================================ */
bool app_task_is_finished(void)
{
    return (g_phase == PHASE_FINISH);
}

/* ============================================================
 * 对外接口：主循环单步（每 LOOP_PERIOD_MS 调用一次）
 * ============================================================ */
void app_task_run_once(void)
{
    if (g_phase == PHASE_FINISH) {
        chassis_stop();
        return;
    }

    g_totalTimeMs += (uint32_t)LOOP_PERIOD_MS;
    uint32_t timeoutMs = 0U;
    switch (g_mode) {
        case TEST_MODE_1_STOP_AT_B:  timeoutMs =  15000U; break;
        case TEST_MODE_2_ONE_LAP:    timeoutMs =  30000U; break;
        case TEST_MODE_3_ONE_LAP:    timeoutMs =  40000U; break;
        case TEST_MODE_4_FOUR_LAPS:  timeoutMs = 160000U; break;
        default:                     timeoutMs =  40000U; break;
    }
    if (g_totalTimeMs >= timeoutMs) {
        phase_enter(PHASE_FINISH, 0.0f);
        return;
    }

    switch (g_mode) {
        case TEST_MODE_1_STOP_AT_B:
            task_mode1_stop_at_b();       break;
        case TEST_MODE_2_ONE_LAP:
            task_mode2_one_lap_abcda();   break;
        case TEST_MODE_3_ONE_LAP:
            task_mode3_one_lap_figure8(); break;
        case TEST_MODE_4_FOUR_LAPS:
            task_mode4_four_laps();       break;
        default:
            phase_enter(PHASE_FINISH, 0.0f); break;
    }
}
#endif
