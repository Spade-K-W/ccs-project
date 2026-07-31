#include "task.h"
#include "app_config.h"
#include "bluetooth.h"
#include "key.h"
#include "app_task.h"
#include "encoder.h"
#include "line_sensor.h"
#include "motor.h"
#include "mpu6050.h"
#include "number.h"
#include "oled.h"
#include "pid.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

/*
 * 改进的路线状态机：红外传感器作为主要判断，编码器仅作保护
 *   A→B：红外循迹，检测到右转特征→进入弧线
 *   B→C：红外弧线，陀螺仪判断180°完成
 *   C→D：红外循迹，检测到右转特征→进入弧线
 *   D→A：红外弧线，陀螺仪判断180°完成
 *
 * 编码器作为保护：防止红外失效时卡死
 */

typedef enum {
    TASK_IDLE = 0,
    TASK_KEY1_ONE_LAP,
    TASK_KEY2_A_TO_B,
    TASK_KEY3_SLOW_LAP,
    TASK_FINISHED
} TaskState;

typedef enum {
    ROUTE_AB_STRAIGHT = 0,
    ROUTE_BC_ARC,
    ROUTE_CD_STRAIGHT,
    ROUTE_DA_ARC,
    ROUTE_LEFT_WHEEL_WAIT,
    ROUTE_KEY1_REAR_WHEELS_BACKWARD,
    ROUTE_KEY3_LEFT_COAST
} RoutePhase;

typedef enum {
    FINISH_SUCCESS = 0,
    FINISH_TIMEOUT,
    FINISH_CANCELLED
} FinishReason;

/* 状态变量 */
static TaskState  g_task_state = TASK_IDLE;
static RoutePhase g_route_phase = ROUTE_AB_STRAIGHT;

static uint32_t g_task_start_ms = 0U;
static uint32_t g_task_elapsed_ms = 0U;
static uint32_t g_display_last_ms = 0U;
static uint32_t g_uart_notice_start_ms = 0U;
static uint8_t g_uart_notice_key = 0U;

/* 陀螺仪相关 */
static float g_yaw_last_deg = 0.0f;
static float g_yaw_accum_deg = 0.0f;
static float g_phase_start_yaw_deg = 0.0f;

/* 编码器相关 */
static float g_distance_base_cm = 0.0f;

/* 红外特征检测相关 */
static uint8_t g_turn_pattern_frame_0 = 0U;        /* 当前帧目标通道 */
static uint8_t g_turn_pattern_frame_1 = 0U;        /* 前1帧目标通道 */
static uint8_t g_turn_pattern_frame_2 = 0U;        /* 前2帧目标通道 */
static uint8_t g_turn_pattern_frame_3 = 0U;        /* 前3帧目标通道 */
static uint8_t g_turn_pattern_frame_4 = 0U;        /* 前4帧目标通道 */
static uint8_t g_turn_pattern_frame_5 = 0U;        /* 前5帧目标通道 */
static uint8_t g_turn_pattern_frame_6 = 0U;        /* 前6帧目标通道 */
static uint8_t g_line_lost_cnt = 0U;                /* 丢线连续计数 */
static uint8_t g_ab_bc_switch_source = 0U;          /* 1=红外，2=编码器保护 */
static uint8_t g_cd_da_switch_source = 0U;          /* 1=红外，2=编码器保护 */
static uint8_t g_key2_stop_source = 0U;             /* 3=8秒计时结束 */
static float g_cd_last_distance_cm = 0.0f;          /* CD当前/切换瞬间距离，供OLED诊断 */
static bool g_key3_start_ramp_done = true;
static uint32_t g_left_extra_wait_start_ms = 0U;
static int32_t g_left_extra_start_count = 0;
static int32_t g_right_extra_start_count = 0;
static uint32_t g_key3_left_coast_start_ms = 0U;
static int16_t g_key3_left_coast_speed = 0;

/* 辅助函数 */
#if LINE_SENSOR_STATIC_TEST_ENABLE
static void task_static_line_sensor_test(void)
{
    static uint8_t last_pattern = 0U;
    static bool first_refresh = true;
    uint8_t pattern = line_read_pattern();
    uint8_t i;
    char bit_line[] = "B:0 0 0 0 0 0 0 0";
    char hex_line[22];

    chassis_stop();
    motor_driver_disable();

    if ((!first_refresh) && (pattern == last_pattern)) {
        return;
    }
    first_refresh = false;
    last_pattern = pattern;

    for (i = 0U; i < 8U; i++) {
        bit_line[2U + (2U * i)] =
            ((pattern & (uint8_t)(1U << i)) != 0U) ? '1' : '0';
    }

    snprintf(hex_line, sizeof(hex_line), "Pattern:0x%02X     ",
             (unsigned int)pattern);
    oled_display_string(0, 0, "LINE STATIC TEST");
    oled_display_string(1, 0, "X:1 2 3 4 5 6 7 8");
    oled_display_string(2, 0, bit_line);
    oled_display_string(3, 0, hex_line);
    oled_display_string(5, 0, "BLACK=1 WHITE=0");
    oled_display_string(7, 0, "MOTOR DISABLED");
}
#endif

static float abs_f(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static float task_segment_distance_cm(void)
{
    float distance = encoder_pulses_to_cm(
        (float)encoder_get_vehicle_pos_pulses());
    return abs_f(distance);
}

static void task_update_key2_acceleration(void)
{
    uint32_t start_percent;
    uint32_t target_percent;
    uint32_t speed_percent;

    if (g_task_state != TASK_KEY2_A_TO_B) {
        return;
    }

    start_percent = (uint32_t)TASK_KEY2_START_SPEED_PERCENT;
    target_percent = (uint32_t)TASK_KEY2_SPEED_PERCENT;

    if ((TASK_KEY2_START_RAMP_MS == 0U)
        || (start_percent >= target_percent)) {
        app_task_set_speed_percent((uint8_t)target_percent);
        return;
    }

    speed_percent = start_percent
        + (((target_percent - start_percent) * g_task_elapsed_ms)
           / (uint32_t)TASK_KEY2_START_RAMP_MS);

    app_task_set_speed_percent((uint8_t)speed_percent);
}

static void task_update_key3_start_ramp(void)
{
    uint32_t start_percent;
    uint32_t target_percent;
    uint32_t speed_percent;

    if ((g_task_state != TASK_KEY3_SLOW_LAP)
        || (g_route_phase != ROUTE_AB_STRAIGHT)
        || g_key3_start_ramp_done) {
        return;
    }

    start_percent = (uint32_t)TASK_KEY3_START_SPEED_PERCENT;
    target_percent = (uint32_t)TASK_KEY3_AB_TARGET_PERCENT;

    if ((TASK_KEY3_START_RAMP_MS == 0U)
        || (start_percent >= target_percent)
        || (g_task_elapsed_ms >= (uint32_t)TASK_KEY3_START_RAMP_MS)) {
        app_task_set_speed_percent((uint8_t)target_percent);
        g_key3_start_ramp_done = true;
        return;
    }

    speed_percent = start_percent
        + (((target_percent - start_percent) * g_task_elapsed_ms)
           / (uint32_t)TASK_KEY3_START_RAMP_MS);
    app_task_set_speed_percent((uint8_t)speed_percent);
}

static float task_phase_yaw_deg(void)
{
    return abs_f(g_yaw_accum_deg - g_phase_start_yaw_deg);
}

/* 陀螺仪更新 */
static void task_update_yaw(void)
{
    float now = mpu6050_get_z_angle_deg();
    float delta = now - g_yaw_last_deg;

    if (delta > 180.0f) {
        delta -= 360.0f;
    } else if (delta < -180.0f) {
        delta += 360.0f;
    }

    g_yaw_accum_deg += delta;
    g_yaw_last_deg = now;
}

/* ========== 红外特征检测逻辑（核心改进） ========== */

static void reset_right_turn_detect(void)
{
    g_turn_pattern_frame_0 = 0U;
    g_turn_pattern_frame_1 = 0U;
    g_turn_pattern_frame_2 = 0U;
    g_turn_pattern_frame_3 = 0U;
    g_turn_pattern_frame_4 = 0U;
    g_turn_pattern_frame_5 = 0U;
    g_turn_pattern_frame_6 = 0U;
}

/**
 * 检测左转特征：CH6/CH7/CH8 全亮（黑线在右，即将左转）
 */
static bool detect_left_turn_feature(uint8_t pattern)
{
    return line_ch678_all_on(pattern);
}

/**
 * 检测丢线
 */
static bool detect_line_lost(uint8_t pattern)
{
    return (pattern == 0U);
}

/*
 * 最近七帧滚动累计目标通道：
 * AB→BC和CD→DA都要求最近七帧合计覆盖CH7、CH8。
 */
static bool process_right_turn_detect(uint8_t pattern)
{
    const uint8_t required_mask = (uint8_t)TASK_TURN_REQUIRED_MASK;
    uint8_t accumulated;

    g_turn_pattern_frame_6 = g_turn_pattern_frame_5;
    g_turn_pattern_frame_5 = g_turn_pattern_frame_4;
    g_turn_pattern_frame_4 = g_turn_pattern_frame_3;
    g_turn_pattern_frame_3 = g_turn_pattern_frame_2;
    g_turn_pattern_frame_2 = g_turn_pattern_frame_1;
    g_turn_pattern_frame_1 = g_turn_pattern_frame_0;
    g_turn_pattern_frame_0 = (uint8_t)(pattern & required_mask);

    accumulated = (uint8_t)(g_turn_pattern_frame_0
                  | g_turn_pattern_frame_1
                  | g_turn_pattern_frame_2
                  | g_turn_pattern_frame_3
                  | g_turn_pattern_frame_4
                  | g_turn_pattern_frame_5
                  | g_turn_pattern_frame_6);

    if ((accumulated & required_mask) == required_mask) {
        reset_right_turn_detect();
        return true;
    }

    return false;
}

/**
 * 主要判断函数：是否应该进入弧线阶段
 * 优先级：红外特征 > 编码器保护
 */
static bool should_enter_arc_straight_to_arc(uint8_t pattern)
{
    float distance;
    float arm_distance;

    distance = task_segment_distance_cm();
    arm_distance = (g_route_phase == ROUTE_CD_STRAIGHT)
        ? TASK_CD_TURN_ARM_DISTANCE_CM
        : TASK_AB_TURN_ARM_DISTANCE_CM;

    /* 当前直线路段到达各自解锁距离前不累计红外转弯特征。 */
    if (distance < arm_distance) {
        reset_right_turn_detect();
        return false;
    }

    /* 最近七帧累计覆盖CH7、CH8时进入弧线。 */
    if (process_right_turn_detect(pattern)) {
        if (g_route_phase == ROUTE_AB_STRAIGHT) {
            g_ab_bc_switch_source = 1U;
        } else if (g_route_phase == ROUTE_CD_STRAIGHT) {
            g_cd_da_switch_source = 1U;
        }
        return true;
    }

    /* 编码器仅作红外检测失效时的距离保护。 */
    if ((g_route_phase == ROUTE_AB_STRAIGHT)
        && (distance >= TASK_AB_ARC_PREP_DISTANCE_CM)) {
        g_ab_bc_switch_source = 2U;
        return true;
    }

    if ((g_route_phase == ROUTE_CD_STRAIGHT)
        && (distance >= TASK_CD_ARC_PREP_DISTANCE_CM)) {
        g_cd_da_switch_source = 2U;
        return true;
    }

    return false;
}

/* ========== UI显示 ========== */

static void task_show_uart_sent(uint8_t key)
{
    char line[22];

    if ((key < 1U) || (key > 3U)) {
        return;
    }

    snprintf(line, sizeof(line), "SENT:'%u'            ",
             (unsigned int)key);
    oled_display_string(5U, 0U, line);
    g_uart_notice_key = key;
    g_uart_notice_start_ms = motor_millis();
}

static void task_update_uart_sent(void)
{
    if (g_uart_notice_key == 0U) {
        return;
    }

    if ((motor_millis() - g_uart_notice_start_ms)
        >= (uint32_t)UART_KEY_OLED_NOTICE_MS) {
        oled_display_string(5U, 0U, "                     ");
        g_uart_notice_key = 0U;
    }
}

static uint32_t task_timeout_ms(void)
{
    switch (g_task_state) {
    case TASK_KEY1_ONE_LAP:
        return TASK_KEY1_TIMEOUT_MS;
    case TASK_KEY2_A_TO_B:
        return TASK_KEY2_TIMEOUT_MS;
    case TASK_KEY3_SLOW_LAP:
        return TASK_KEY3_TIMEOUT_MS;
    default:
        return 0U;
    }
}

static void task_show_switch_sources(void)
{
    char line[22];
    uint32_t cd_distance_x10;

    snprintf(line, sizeof(line), "AB:%u                 ",
             (unsigned int)g_ab_bc_switch_source);
    oled_display_string(0U, 0U, line);

    snprintf(line, sizeof(line), "CD:%u                 ",
             (unsigned int)g_cd_da_switch_source);
    oled_display_string(1U, 0U, line);

    snprintf(line, sizeof(line), "K2:%u                 ",
             (unsigned int)g_key2_stop_source);
    oled_display_string(3U, 0U, line);

    cd_distance_x10 =
        (uint32_t)((g_cd_last_distance_cm * 10.0f) + 0.5f);
    snprintf(line, sizeof(line), "CDcm:%lu.%lu          ",
             (unsigned long)(cd_distance_x10 / 10U),
             (unsigned long)(cd_distance_x10 % 10U));
    oled_display_string(2U, 0U, line);
}

static void task_show_select(void)
{
    number_show_time_ms(0U);
    oled_clear();
    task_show_switch_sources();
}

static void task_show_running(void)
{
    if ((g_task_elapsed_ms - g_display_last_ms)
        < TASK_DISPLAY_PERIOD_MS) {
        return;
    }
    g_display_last_ms = g_task_elapsed_ms;
    task_show_switch_sources();
}

/* ========== 状态机 ========== */

static void task_send_route_signal(RoutePhase phase)
{
    char signal = '\0';

    switch (phase) {
    case ROUTE_AB_STRAIGHT:
        signal = 'A';
        break;
    case ROUTE_BC_ARC:
        signal = 'B';
        break;
    case ROUTE_CD_STRAIGHT:
        signal = 'C';
        break;
    case ROUTE_DA_ARC:
        signal = 'D';
        break;
    default:
        break;
    }

    if (signal != '\0') {
        bluetooth_putc(signal);
    }
}

static void task_enter_phase(RoutePhase next)
{
    /* 保存刚完成路段的编码器距离 */
    g_distance_base_cm += task_segment_distance_cm();
    g_route_phase = next;
    g_phase_start_yaw_deg = g_yaw_accum_deg;
    pid_set_second_arc_profile(next == ROUTE_DA_ARC);
    app_task_set_second_arc(next == ROUTE_DA_ARC);

    /* KEY3让AB、BC、CD、DA的共模速度均保持在约20.7。 */
    if (g_task_state == TASK_KEY3_SLOW_LAP) {
        if ((next == ROUTE_BC_ARC) || (next == ROUTE_DA_ARC)) {
            g_key3_start_ramp_done = true;
            app_task_set_speed_percent((uint8_t)TASK_SLOW_SPEED_PERCENT);
        } else if (next == ROUTE_CD_STRAIGHT) {
            app_task_set_speed_percent((uint8_t)TASK_KEY3_CD_SPEED_PERCENT);
        }
    }

    task_send_route_signal(next);

    /* 重置红外检测计数 */
    reset_right_turn_detect();
    g_line_lost_cnt = 0U;

    if (next == ROUTE_CD_STRAIGHT) {
        g_cd_last_distance_cm = 0.0f;
    }

    if ((next == ROUTE_AB_STRAIGHT)
        || (next == ROUTE_CD_STRAIGHT)) {
        app_task_straight_prepare(0.0f);
    } else {
        app_task_arc_prepare(TASK_ARC_MIRROR_DIR);
    }
}

static void task_start(TaskState state)
{
    uint8_t speed_percent = TASK_NORMAL_SPEED_PERCENT;

    if (state == TASK_KEY2_A_TO_B) {
        speed_percent = TASK_KEY2_START_SPEED_PERCENT;
    } else if (state == TASK_KEY3_SLOW_LAP) {
        speed_percent = TASK_KEY3_START_SPEED_PERCENT;
    }

    g_task_state = state;
    g_route_phase = ROUTE_AB_STRAIGHT;
    g_task_start_ms = motor_millis();
    g_task_elapsed_ms = 0U;
    g_display_last_ms = 0U;

    chassis_stop();
    motor_driver_enable();

    mpu6050_reset_z_angle();
    g_yaw_last_deg = mpu6050_get_z_angle_deg();
    g_yaw_accum_deg = 0.0f;
    g_phase_start_yaw_deg = 0.0f;
    g_distance_base_cm = 0.0f;

    /* 重置红外检测 */
    reset_right_turn_detect();
    g_line_lost_cnt = 0U;
    g_ab_bc_switch_source = 0U;
    g_cd_da_switch_source = 0U;
    g_key2_stop_source = 0U;
    g_cd_last_distance_cm = 0.0f;
    g_left_extra_wait_start_ms = 0U;
    g_left_extra_start_count = 0;
    g_right_extra_start_count = 0;
    g_key3_left_coast_start_ms = 0U;
    g_key3_left_coast_speed = 0;
    g_key3_start_ramp_done =
        (state != TASK_KEY3_SLOW_LAP)
        || (TASK_KEY3_START_RAMP_MS == 0U)
        || (TASK_KEY3_START_SPEED_PERCENT >= TASK_KEY3_AB_TARGET_PERCENT);

    if ((state == TASK_KEY3_SLOW_LAP) && g_key3_start_ramp_done) {
        speed_percent = TASK_KEY3_AB_TARGET_PERCENT;
    }

    pid_set_key3_profile(state == TASK_KEY3_SLOW_LAP);
    pid_set_second_arc_profile(false);
    app_task_set_second_arc(false);
    app_task_set_speed_percent(speed_percent);
    app_task_straight_prepare(0.0f);
    task_send_route_signal(ROUTE_AB_STRAIGHT);

    oled_clear();
    number_show_time_ms(0U);
    task_show_switch_sources();
}

static void task_finish(FinishReason reason)
{
    (void)reason;

    g_left_extra_wait_start_ms = 0U;
    g_key3_left_coast_start_ms = 0U;
    g_key3_left_coast_speed = 0;
    chassis_stop();
    number_show_time_ms(g_task_elapsed_ms);
    g_task_state = TASK_FINISHED;
    bluetooth_putc('F');

    oled_clear();
    task_show_switch_sources();
}

static void task_begin_key1_rear_wheels_backward(void)
{
    EncoderCounts counts;

    encoder_get_counts(&counts);
    g_left_extra_start_count = counts.leftCount;
    g_right_extra_start_count = counts.rightCount;
    g_route_phase = ROUTE_KEY1_REAR_WHEELS_BACKWARD;

    /* M1右后轮与M4左后轮同时后退，PWM与前一动作相同。 */
    chassis_set4(-(int16_t)TASK_KEY1_REAR_EXTRA_WHEEL_SPEED, 0, 0,
                 -(int16_t)TASK_KEY1_REAR_EXTRA_WHEEL_SPEED);
}

static void task_begin_left_wheel_wait(void)
{
    g_left_extra_wait_start_ms = g_task_elapsed_ms;
    g_route_phase = ROUTE_LEFT_WHEEL_WAIT;
    chassis_stop();
}

static void task_begin_key3_left_coast(void)
{
    line_follow_get_out_wheels(&g_key3_left_coast_speed, NULL);
    g_key3_left_coast_start_ms = g_task_elapsed_ms;
    g_route_phase = ROUTE_KEY3_LEFT_COAST;

    /* 左前轮M3、左后轮M4保持当前实际PWM，右侧两轮停止。 */
    chassis_set(g_key3_left_coast_speed, 0);
}

/* ========== 核心路线执行逻辑 ========== */

static void task_run_route(void)
{
    uint8_t pattern = line_read_pattern();

    switch (g_route_phase) {
    case ROUTE_AB_STRAIGHT:
        /* 直线阶段：红外特征优先，编码器保护 */
        if (g_task_state != TASK_KEY2_A_TO_B) {
            /* KEY1/KEY3：检测到右转特征则进入弧线 */
            if (should_enter_arc_straight_to_arc(pattern)) {
                task_enter_phase(ROUTE_BC_ARC);
                return;
            }
        }
        
        /* 执行直线循迹 */
        {
            float error = 0.0f;
            bool line_valid = line_calc_error_f(pattern, &error);
            line_follow_drive(pattern, error, line_valid);
        }
        break;

    case ROUTE_BC_ARC:
        /* 弧线阶段：用陀螺仪判断转弯完成 */
        if (task_phase_yaw_deg() >= TASK_ARC_YAW_DEG) {
            task_enter_phase(ROUTE_CD_STRAIGHT);
            return;
        }
        
        /* 执行弧线循迹 */
        {
            float error = 0.0f;
            bool line_valid = line_calc_error_arc_f(pattern, &error);
            app_task_arc_step(pattern, error, line_valid);
        }
        break;

    case ROUTE_CD_STRAIGHT:
        /* 直线阶段：红外特征优先，编码器保护 */
        g_cd_last_distance_cm = task_segment_distance_cm();

        if (should_enter_arc_straight_to_arc(pattern)) {
            task_enter_phase(ROUTE_DA_ARC);
            return;
        }
        
        /* 执行直线循迹 */
        {
            float error = 0.0f;
            bool line_valid = line_calc_error_f(pattern, &error);
            line_follow_drive(pattern, error, line_valid);
        }
        break;

    case ROUTE_DA_ARC:
        /* 弧线阶段：用陀螺仪判断转弯完成 */
        if (g_task_state == TASK_KEY1_ONE_LAP) {
            /* KEY1：累计转角达到350度后，左轮再转1/4圈。 */
            if (abs_f(g_yaw_accum_deg) >= TASK_KEY1_TARGET_YAW_DEG) {
                task_begin_left_wheel_wait();
                return;
            }
        } else {
            /* KEY3：累计转角达到目标后，左侧两轮保持当前转速0.2秒。 */
            if (abs_f(g_yaw_accum_deg) >= TASK_KEY3_TARGET_YAW_DEG) {
                task_begin_key3_left_coast();
                return;
            }
        }
        
        /* 执行弧线循迹 */
        {
            float error = 0.0f;
            bool line_valid = line_calc_error_arc_f(pattern, &error);
            app_task_arc_step(pattern, error, line_valid);
        }
        break;

    case ROUTE_LEFT_WHEEL_WAIT:
        chassis_stop();
        if ((g_task_elapsed_ms - g_left_extra_wait_start_ms)
            >= (uint32_t)TASK_LEFT_EXTRA_WAIT_MS) {
            task_begin_key1_rear_wheels_backward();
        }
        break;

    case ROUTE_KEY1_REAR_WHEELS_BACKWARD:
        {
            EncoderCounts counts;
            int32_t leftDelta;
            int32_t rightDelta;
            bool leftDone;
            bool rightDone;

            encoder_get_counts(&counts);
            leftDelta = counts.leftCount - g_left_extra_start_count;
            rightDelta = counts.rightCount - g_right_extra_start_count;
            if (leftDelta < 0) {
                leftDelta = -leftDelta;
            }
            if (rightDelta < 0) {
                rightDelta = -rightDelta;
            }

            leftDone =
                ((uint32_t)leftDelta >= TASK_KEY1_REAR_BACKWARD_WHEEL_PULSES);
            rightDone =
                ((uint32_t)rightDelta >= TASK_KEY1_REAR_BACKWARD_WHEEL_PULSES);

            if (leftDone && rightDone) {
                task_finish(FINISH_SUCCESS);
                return;
            }

            chassis_set4(
                rightDone ? 0
                          : -(int16_t)TASK_KEY1_REAR_EXTRA_WHEEL_SPEED,
                0,
                0,
                leftDone ? 0
                         : -(int16_t)TASK_KEY1_REAR_EXTRA_WHEEL_SPEED);
        }
        break;

    case ROUTE_KEY3_LEFT_COAST:
        if ((g_task_elapsed_ms - g_key3_left_coast_start_ms)
            >= (uint32_t)TASK_KEY3_LEFT_COAST_MS) {
            task_finish(FINISH_SUCCESS);
            return;
        }
        chassis_set(g_key3_left_coast_speed, 0);
        break;

    default:
        task_finish(FINISH_CANCELLED);
        break;
    }
}

/* ========== 主入口函数 ========== */

void task_init(void)
{
    g_task_state = TASK_IDLE;
    g_route_phase = ROUTE_AB_STRAIGHT;
    g_task_start_ms = 0U;
    g_task_elapsed_ms = 0U;
    g_display_last_ms = 0U;
    g_uart_notice_start_ms = 0U;
    g_uart_notice_key = 0U;
    g_yaw_last_deg = 0.0f;
    g_yaw_accum_deg = 0.0f;
    g_phase_start_yaw_deg = 0.0f;
    g_distance_base_cm = 0.0f;
    reset_right_turn_detect();
    g_line_lost_cnt = 0U;
    g_ab_bc_switch_source = 0U;
    g_cd_da_switch_source = 0U;
    g_key2_stop_source = 0U;
    g_cd_last_distance_cm = 0.0f;
    g_key3_start_ramp_done = true;
    g_left_extra_wait_start_ms = 0U;
    g_left_extra_start_count = 0;
    g_right_extra_start_count = 0;
    g_key3_left_coast_start_ms = 0U;
    g_key3_left_coast_speed = 0;
    pid_set_second_arc_profile(false);
    app_task_set_second_arc(false);

    chassis_stop();
    app_task_set_oled_debug_enabled(false);
#if LINE_SENSOR_STATIC_TEST_ENABLE
    motor_driver_disable();
    oled_clear();
    task_static_line_sensor_test();
#else
    task_show_select();
#endif
}

void task_step(void)
{
    uint8_t key;
    uint32_t timeout;

    task_update_uart_sent();

#if LINE_SENSOR_STATIC_TEST_ENABLE
    task_static_line_sensor_test();
    return;
#endif

    if ((g_task_state == TASK_IDLE)
        || (g_task_state == TASK_FINISHED)) {
        key = key_scan();

        /* 每次有效按键只发送一个字符：'1'、'2' 或 '3'。 */
        bluetooth_send_key(key);

        switch (key) {
        case 1U:
            task_start(TASK_KEY1_ONE_LAP);
            break;
        case 2U:
            task_start(TASK_KEY2_A_TO_B);
            break;
        case 3U:
            task_start(TASK_KEY3_SLOW_LAP);
            break;
        default:
            break;
        }
        task_show_uart_sent(key);
        return;
    }

    /* 运行中按键停止 */
    key = key_scan();
    if (key != 0U) {
        bluetooth_send_key(key);
        task_finish(FINISH_CANCELLED);
        task_show_uart_sent(key);
        return;
    }

    g_task_elapsed_ms = motor_millis() - g_task_start_ms;
    number_show_time_ms(g_task_elapsed_ms);
    task_update_yaw();
    task_update_key2_acceleration();
    task_update_key3_start_ramp();

    /* 超时保护 */
    timeout = task_timeout_ms();
    if ((timeout != 0U) && (g_task_elapsed_ms >= timeout)) {
        if (g_task_state == TASK_KEY2_A_TO_B) {
            g_key2_stop_source = 3U;
        }
        task_finish(FINISH_TIMEOUT);
        return;
    }

    /* 执行路线 */
    task_run_route();
    if (g_task_state != TASK_FINISHED) {
        task_show_running();
    }
}
