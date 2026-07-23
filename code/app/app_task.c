/* ============================================================
 * app_task.c
 * 当前仅保留循迹调试用的对外接口；比赛状态机见下方 #if 0
 * ============================================================ */

#include "app_task.h"
#include "app_config.h"
#include "motor.h"
#include "pid.h"
#include "encoder.h"
#include "line_sensor.h"
#include "mpu6050.h"
#include "oled.h"
#include "uart_debug.h"
#include "uart_vision.h"
#include "Delay.h"

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

/* 状态一：直线循迹；状态二：左转；状态三：右转 */
typedef enum {
    LINE_STATE_STRAIGHT = 1,
    LINE_STATE_LEFT_TURN,
    LINE_STATE_RIGHT_TURN
} LineDriveState;

static LineDriveState g_line_state = LINE_STATE_STRAIGHT;

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

#if 0 /* 以下为整库未调用的比赛代码，暂禁用 */
/* ============================================================
 * 工具：角度差，处理 ±180° 回绕
 * ============================================================ */
static float angle_diff_deg(float now, float base)
{
    float d = now - base;
    if (d >  180.0f) d -= 360.0f;
    if (d < -180.0f) d += 360.0f;
    return d;
}

/* ============================================================
 * 进入新阶段
 * straightHeadingOffset：相对初始航向的偏移角（弧线段传 0.0f）
 * ============================================================ */
static void phase_enter(RunPhase next, float straightHeadingOffset)
{
    g_phase           = next;
    g_phaseTimeMs     = 0U;
    g_curveEntryCnt   = 0U;
    g_arcAccumDeg     = 0.0f;
    g_phaseStartAngle = mpu6050_get_z_angle_deg();

    g_straightHeading = g_initHeading + straightHeadingOffset;

    if (next == PHASE_FINISH) {
        chassis_stop();
        motor_driver_disable();
        signal_finish_async();
    }
}

/* ============================================================
 * USER 按键：3s 窗口期选模式
 * ============================================================ */
static bool user_key_is_pressed(void)
{
    bool raw = pin_read_raw(&USER_KEY);
    return USER_KEY_ACTIVE_LOW ? (!raw) : raw;
}

static TestMode select_mode_from_user_key(void)
{
    const uint32_t windowMs   = 3000U;
    const uint32_t debounceMs = (uint32_t)USER_KEY_DEBOUNCE_MS;
    const uint32_t stepMs     = (uint32_t)LOOP_PERIOD_MS;

    TestMode mode        = TEST_MODE_1_STOP_AT_B;
    uint32_t elapsed     = 0U;
    bool     lastPressed = false;

    oled_clear();
    oled_display_string(0, 0, "Select Mode     ");
    oled_display_string(1, 0, "Default: Mode 1 ");
    oled_display_string(2, 0, "1: A->B Stop    ");
    oled_display_string(3, 0, "2: ABCDA CW     ");
    oled_display_string(4, 0, "3: ACBDA Fig8   ");
    oled_display_string(5, 0, "4: Fig8 x4 Laps ");
    oled_display_string(6, 0, "Press KEY...    ");

    while (elapsed < windowMs) {
        bool pressed = user_key_is_pressed();
        if (pressed && !lastPressed) {
            delay_ms(debounceMs);
            if (user_key_is_pressed()) {
                mode = (TestMode)(((uint32_t)mode % 4U) + 1U);
                oled_display_string(1, 0, "Mode:           ");
                oled_display_num(1, 36, (uint8_t)mode);
                switch (mode) {
                    case TEST_MODE_1_STOP_AT_B:
                        oled_display_string(2, 0, "A->B Stop       "); break;
                    case TEST_MODE_2_ONE_LAP:
                        oled_display_string(2, 0, "A-B-C-D-A CW    "); break;
                    case TEST_MODE_3_ONE_LAP:
                        oled_display_string(2, 0, "A-C-B-D-A Fig8  "); break;
                    case TEST_MODE_4_FOUR_LAPS:
                        oled_display_string(2, 0, "Fig8 x4 Laps    "); break;
                    default: break;
                }
                signal_blink(60U, 50U, (uint8_t)mode);
                while (user_key_is_pressed()) delay_ms(20U);
            }
        }
        lastPressed = pressed;
        delay_ms(stepMs);
        elapsed += stepMs;
    }
    return mode;
}
#endif

/*
 * OLED 五行列：
 *  0 直行/转弯
 *  1 陀螺仪 Z 角
 *  2 cmd 速度
 *  3 速度环调整后速度
 *  4 编码器测速(Mea)
 */
static void line_follow_refresh_ui(uint8_t pattern, float error, bool lineValid,
                                  const char *mode)
{
    char lineMode[24];
    char lineGyro[24];
    char lineCmd[24];
    char lineOut[24];
    char lineMea[24];
    EncoderSpeeds spd;
    float meaL;
    float meaR;
    UartDebugStatus dbg;
    bool isStraight = app_task_line_is_straight();

    encoder_get_speeds(&spd);
    if (PWM_TO_SPEED_GAIN > 0.0f) {
        meaL = spd.leftSpeed  / PWM_TO_SPEED_GAIN;
        meaR = spd.rightSpeed / PWM_TO_SPEED_GAIN;
    } else {
        meaL = spd.leftSpeed;
        meaR = spd.rightSpeed;
    }

    snprintf(lineMode, sizeof(lineMode), "%-16s",
             isStraight ? "straight" : "turn");
    snprintf(lineGyro, sizeof(lineGyro), "Z:%+7.1f deg   ",
             mpu6050_get_z_angle_deg());
    snprintf(lineCmd, sizeof(lineCmd),
             "Cmd L:%3d R:%3d", (int)g_cmdLeft, (int)g_cmdRight);
    snprintf(lineOut, sizeof(lineOut),
             "Out L:%3d R:%3d", (int)g_outLeft, (int)g_outRight);
    snprintf(lineMea, sizeof(lineMea),
             "Mea L:%4.1f R:%4.1f", meaL, meaR);

    oled_display_string(0, 0, lineMode);
    oled_display_string(1, 0, lineGyro);
    oled_display_string(2, 0, lineCmd);
    oled_display_string(3, 0, lineOut);
    oled_display_string(4, 0, lineMea);

    uart_debug_build_status(&dbg, error, lineValid, pattern,
                            g_cmdLeft, g_cmdRight,
                            mpu6050_get_z_angle_deg(),
                            app_task_get_lap_count(), mode);
    uart_debug_print_status(&dbg);
    (void)mode;
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
    chassis_set_with_accel(left, right);
    line_follow_refresh_ui(pattern, error, lineValid, "go straight     ");
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
    bool ch123;
    bool ch678;
    bool allOn;

    switch (g_line_state) {
        case LINE_STATE_STRAIGHT:
            ch123 = line_ch123_all_on(pattern);
            ch678 = line_ch678_all_on(pattern);
            allOn = line_all_on(pattern);

            /* 八路全亮优先：停车/开环，避免速度环把空转当停转猛补 */
            if (allOn || (ch123 && ch678)) {
                line_follow_all_on_or_cross(pattern, error, allOn);
            } else if (ch123 && !ch678) {
                line_turn_after_detect_delay(pattern, error, lineValid, true);
            } else if (ch678 && !ch123) {
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

#if 0 /* 以下为整库未调用的比赛代码，暂禁用 */
/* ============================================================
 * 底层A：直线段红外循迹单步
 * ============================================================ */
void do_straight_drive_step(void)
{
    uint8_t pattern    = line_read_pattern();
    int16_t error      = 0;
    bool    lineValid  = line_calc_error(pattern, &error);
    bool    vertexLike = is_vertex_like_pattern(pattern);

    g_stepLineValid  = lineValid;
    g_stepVertexLike = vertexLike;
    g_phaseTimeMs     += (uint32_t)LOOP_PERIOD_MS;

    if (lineValid) {
        g_stepError = error;
    } else {
        error       = g_lastError;
        g_stepError = g_lastError;
    }

    line_follow_drive(pattern, (float)error, lineValid);

    bool curveLikely = (abs_i16(error) >= (int16_t)CURVE_ENTRY_ERROR_TH) || vertexLike;
    if (curveLikely) {
        g_curveEntryCnt++;
    } else {
        g_curveEntryCnt = 0U;
    }
}

/* ============================================================
 * 底层B：有线弧线循迹单步
 * mirrorDir    : +1.0f 正向，-1.0f 反向
 * arcEnterBias : 入弧偏置
 * ============================================================ */
static void do_arc_follow_step(float mirrorDir, int16_t arcEnterBias)
{
    uint8_t pattern    = line_read_pattern();
    int16_t error      = 0;
    bool    lineValid  = line_calc_error(pattern, &error);
    bool    vertexLike = is_vertex_like_pattern(pattern);

    g_stepLineValid  = lineValid;
    g_stepVertexLike = vertexLike;

    if (lineValid) {
        g_lastError = error;
        g_stepError = error;
    } else {
        error       = g_lastError;
        g_stepError = g_lastError;
    }

    float zAngle   = mpu6050_get_z_angle_deg();
    g_arcAccumDeg  = angle_diff_deg(zAngle, g_phaseStartAngle);
    g_phaseTimeMs += (uint32_t)LOOP_PERIOD_MS;

    /* 丢线保护 */
    if (!lineValid) {
        if (g_lastError <= 0) chassis_set(SEARCH_SPEED_LOW,  SEARCH_SPEED_HIGH);
        else                   chassis_set(SEARCH_SPEED_HIGH, SEARCH_SPEED_LOW);
        return;
    }

    int16_t diff = (int16_t)((float)KP_LINE_ARC * (float)error * mirrorDir);

    if (g_phaseTimeMs <= (uint32_t)ARC_ENTER_ASSIST_MS) {
        diff += arcEnterBias;
    }

    chassis_set((int16_t)BASE_SPEED_ARC + diff,
                (int16_t)BASE_SPEED_ARC - diff);

    bool curveLikely = (abs_i16(error) >= (int16_t)CURVE_ENTRY_ERROR_TH) || vertexLike;
    if (curveLikely) g_curveEntryCnt++;
    else             g_curveEntryCnt = 0U;
}

/* ============================================================
 * 判断1：直行到达终点
 * 条件：超过最短保护时间 + 传感器检测到顶点特征
 * ============================================================ */
static bool check_straight_arrived(void)
{
    return (g_phaseTimeMs >= (uint32_t)STRAIGHT_MIN_TIME_MS)
        && g_stepVertexLike;
}

/* ============================================================
 * 判断2：弧线完成 180°
 * arcSign: +1.0f 逆时针，-1.0f 顺时针
 * ============================================================ */
static bool check_arc_finished(float arcSign)
{
    return (g_phaseTimeMs >= (uint32_t)ARC_MIN_TIME_MS)
        && ((arcSign * g_arcAccumDeg) >= (float)ARC_TARGET_DEG)
        && ((abs_i16(g_stepError) <= (int16_t)ARC_EXIT_ERR_BAND)
            || g_stepVertexLike);
}

/* ============================================================
 * 题目(1)：A→B 直行到 B 停车
 * ============================================================ */
static void task_mode1_stop_at_b(void)
{
    do_straight_drive_step();
    if (check_straight_arrived()) {
        phase_enter(PHASE_FINISH, 0.0f);
    }
}

/* ============================================================
 * 题目(2)：A→B→C→D→A 顺时针一圈
 * ============================================================ */
static void task_mode2_one_lap_abcda(void)
{
    switch (g_phase) {

        case PH2_AB_STRAIGHT:
            do_straight_drive_step();
            if (check_straight_arrived()) {
                signal_pass_point_async();
                phase_enter(PH2_BC_ARC, 0.0f);
            }
            break;

        case PH2_BC_ARC:
            do_arc_follow_step(+1.0f, +(int16_t)ARC_ENTER_TURN_BIAS);
            if (check_arc_finished(-1.0f)) {
                signal_pass_point_async();
                phase_enter(PH2_CD_STRAIGHT, HEADING_OFFSET_CD);
            }
            break;

        case PH2_CD_STRAIGHT:
            do_straight_drive_step();
            if (check_straight_arrived()) {
                signal_pass_point_async();
                phase_enter(PH2_DA_ARC, 0.0f);
            }
            break;

        case PH2_DA_ARC:
            do_arc_follow_step(-1.0f, -(int16_t)ARC_ENTER_TURN_BIAS);
            if (check_arc_finished(-1.0f)) {
                phase_enter(PHASE_FINISH, 0.0f);
            }
            break;

        default: break;
    }
}

/* ============================================================
 * 题目(3)：A→C→B→D→A 八字形一圈
 * ============================================================ */
static void task_mode3_one_lap_figure8(void)
{
    switch (g_phase) {

        case PH34_AC_STRAIGHT:
            do_straight_drive_step();
            if (check_straight_arrived()) {
                signal_pass_point_async();
                phase_enter(PH34_CB_ARC, 0.0f);
            }
            break;

        case PH34_CB_ARC:
            do_arc_follow_step(+1.0f, -(int16_t)ARC_ENTER_TURN_BIAS);
            if (check_arc_finished(+1.0f)) {
                signal_pass_point_async();
                phase_enter(PH34_BD_STRAIGHT, HEADING_OFFSET_BD);
            }
            break;

        case PH34_BD_STRAIGHT:
            do_straight_drive_step();
            if (check_straight_arrived()) {
                signal_pass_point_async();
                phase_enter(PH34_DA_ARC, 0.0f);
            }
            break;

        case PH34_DA_ARC:
            do_arc_follow_step(-1.0f, +(int16_t)ARC_ENTER_TURN_BIAS);
            if (check_arc_finished(+1.0f)) {
                phase_enter(PHASE_FINISH, 0.0f);
            }
            break;

        default: break;
    }
}

/* ============================================================
 * 题目(4)：八字形 ×4 圈
 * ============================================================ */
static void task_mode4_four_laps(void)
{
    switch (g_phase) {

        case PH34_AC_STRAIGHT:
            do_straight_drive_step();
            if (check_straight_arrived()) {
                signal_pass_point_async();
                phase_enter(PH34_CB_ARC, 0.0f);
            }
            break;

        case PH34_CB_ARC:
            do_arc_follow_step(+1.0f, -(int16_t)ARC_ENTER_TURN_BIAS);
            if (check_arc_finished(+1.0f)) {
                signal_pass_point_async();
                phase_enter(PH34_BD_STRAIGHT, HEADING_OFFSET_BD);
            }
            break;

        case PH34_BD_STRAIGHT:
            do_straight_drive_step();
            if (check_straight_arrived()) {
                signal_pass_point_async();
                phase_enter(PH34_DA_ARC, 0.0f);
            }
            break;

        case PH34_DA_ARC:
            do_arc_follow_step(-1.0f, +(int16_t)ARC_ENTER_TURN_BIAS);
            if (check_arc_finished(+1.0f)) {
                g_lapCount++;
                if (g_lapCount >= (uint8_t)TARGET_LAPS) {
                    phase_enter(PHASE_FINISH, 0.0f);
                } else {
                    signal_pass_point_async();
                    phase_enter(PH34_AC_STRAIGHT, HEADING_OFFSET_AC);
                }
            }
            break;

        default: break;
    }
}

/* ============================================================
 * 对外接口：初始化
 * ============================================================ */
void app_task_init(void)
{
    g_initHeading      = mpu6050_get_z_angle_deg();

    g_lastError        = 0;
    g_phaseTimeMs      = 0U;
    g_curveEntryCnt    = 0U;
    g_arcAccumDeg      = 0.0f;
    g_lapCount         = 0U;
    g_totalTimeMs      = 0U;
    g_phaseStartAngle  = g_initHeading;
    g_stepError        = 0;
    g_stepVertexLike   = false;
    g_stepLineValid    = false;
    g_straightHeading  = g_initHeading;

    g_mode = select_mode_from_user_key();
    signal_start_async();

    switch (g_mode) {
        case TEST_MODE_1_STOP_AT_B:
            phase_enter(PH1_AB_STRAIGHT,  HEADING_OFFSET_AB); break;
        case TEST_MODE_2_ONE_LAP:
            phase_enter(PH2_AB_STRAIGHT,  HEADING_OFFSET_AB); break;
        case TEST_MODE_3_ONE_LAP:
        case TEST_MODE_4_FOUR_LAPS:
            phase_enter(PH34_AC_STRAIGHT, HEADING_OFFSET_AC); break;
        default:
            phase_enter(PH1_AB_STRAIGHT,  HEADING_OFFSET_AB); break;
    }
}
#endif

/* ============================================================
 * 对外接口：走直前设置目标航向（mpu6050_reset_z_angle 后通常传 0）
 * ============================================================ */
void app_task_straight_prepare(float targetHeadingDeg)
{
    (void)targetHeadingDeg;

    g_lastError  = 0;
    g_line_state = LINE_STATE_STRAIGHT;
    pid_line_reset();
    pid_accel_reset();
    encoder_reset();
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
