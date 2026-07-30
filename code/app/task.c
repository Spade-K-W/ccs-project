#include "task.h"
#include "app_config.h"
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

/* 红外传感器判断参数 */
#define LINE_LOST_CONFIRM_CNT            (5U)      /* 丢线确认帧数 */

/* 陀螺仪参数 */
#define TASK_ARC_YAW_DEG                 (180.0f)  /* 弧线转弯180度 */
#define TASK_KEY1_TARGET_YAW_DEG         (350.0f)  /* KEY1一圈的目标角度 */
#define TASK_KEY3_TARGET_YAW_DEG         (350.0f)  /* KEY3一圈的目标角度 */

/* 编码器保护参数 */
#define TASK_STRAIGHT_DISTANCE_MAX_CM    (220.0f)  /* 直线段编码器保护上限 */
#define TASK_AB_TURN_ARM_DISTANCE_CM     (90.0f)   /* AB前90cm禁止累计转弯红外特征 */
#define TASK_CD_TURN_ARM_DISTANCE_CM     (60.0f)   /* CD前60cm禁止累计转弯红外特征 */
#define TASK_AB_ARC_PREP_DISTANCE_CM     (220.0f)  /* AB→BC编码器保护阈值 */
#define TASK_CD_SLOWDOWN_DISTANCE_CM     (125.0f)  /* CD末段开始降速 */
#define TASK_CD_ARC_PREP_DISTANCE_CM     (170.0f)  /* CD→DA编码器保护阈值 */

/* 速度参数 */
#define TASK_NORMAL_SPEED_PERCENT        (100U)
#define TASK_SLOW_SPEED_PERCENT          (70U)
#define TASK_CD_SPEED_REDUCTION          (5U)

/* 显示参数 */
#define TASK_DISPLAY_PERIOD_MS           (100U)

/* app_task.c 约定：+1 为左弧，-1 为右弧；顺时针使用右弧。 */
#define TASK_ARC_MIRROR_DIR              (-1.0f)

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
    ROUTE_DA_ARC
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
static bool g_cd_slowdown_applied = false;
static uint8_t g_ab_bc_switch_source = 0U;          /* 1=红外，2=编码器保护 */
static uint8_t g_cd_da_switch_source = 0U;          /* 1=红外，2=编码器保护 */

/* 辅助函数 */
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

static uint8_t task_normal_speed_percent(void)
{
    return (g_task_state == TASK_KEY3_SLOW_LAP)
        ? TASK_SLOW_SPEED_PERCENT
        : TASK_NORMAL_SPEED_PERCENT;
}

static uint8_t task_cd_slow_speed_percent(void)
{
    uint32_t normal_percent = task_normal_speed_percent();
    uint32_t normal_speed =
        ((uint32_t)BASE_SPEED_STRAIGHT * normal_percent) / 100U;
    uint32_t target_speed;
    uint32_t target_percent;

    if (normal_speed > TASK_CD_SPEED_REDUCTION) {
        target_speed = normal_speed - TASK_CD_SPEED_REDUCTION;
    } else {
        target_speed = 1U;
    }

    target_percent =
        ((target_speed * 100U) + (uint32_t)BASE_SPEED_STRAIGHT - 1U)
        / (uint32_t)BASE_SPEED_STRAIGHT;

    if (target_percent < 1U) {
        target_percent = 1U;
    } else if (target_percent > 100U) {
        target_percent = 100U;
    }

    return (uint8_t)target_percent;
}

static void task_apply_cd_slowdown(void)
{
    if (!g_cd_slowdown_applied) {
        if (g_task_state != TASK_KEY3_SLOW_LAP) {
            app_task_set_speed_percent(task_cd_slow_speed_percent());
        }
        g_cd_slowdown_applied = true;
    }
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
    const uint8_t required_mask = (uint8_t)0xC0U; /* CH7 | CH8 */
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

/**
 * 主要判断函数：是否应该停止（仅用于KEY2 A→B）
 * 在直线段检测到停止线特征
 */
static bool should_stop_at_B_point(uint8_t pattern)
{
    /* 检测十字路口或两侧都有传感器 */
    uint8_t cnt = 0;
    uint8_t i;
    for (i = 0; i < 8; i++) {
        if (pattern & (1U << i)) cnt++;
    }

    /* 编码器保护：防止过度运行 */
    float distance = task_segment_distance_cm();
    if (distance >= TASK_STRAIGHT_DISTANCE_MAX_CM) {
        return true;
    }

    /* 红外判断：检测到明显的路口特征 */
    if (is_vertex_like_pattern(pattern) && (cnt >= 6U)) {
        return true;
    }

    return false;
}

/* ========== UI显示 ========== */

static uint32_t task_timeout_ms(void)
{
    switch (g_task_state) {
    case TASK_KEY1_ONE_LAP:
        return 30000U;  /* 30秒 */
    case TASK_KEY2_A_TO_B:
        return 8000U;   /* 8秒 */
    case TASK_KEY3_SLOW_LAP:
        return 40000U;  /* 40秒 */
    default:
        return 0U;
    }
}

static void task_show_switch_sources(void)
{
    char line[22];

    snprintf(line, sizeof(line), "AB:%u                 ",
             (unsigned int)g_ab_bc_switch_source);
    oled_display_string(0U, 0U, line);

    snprintf(line, sizeof(line), "CD:%u                 ",
             (unsigned int)g_cd_da_switch_source);
    oled_display_string(1U, 0U, line);
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

static void task_enter_phase(RoutePhase next)
{
    /* 保存刚完成路段的编码器距离 */
    g_distance_base_cm += task_segment_distance_cm();
    g_route_phase = next;
    g_phase_start_yaw_deg = g_yaw_accum_deg;

    /* 重置红外检测计数 */
    reset_right_turn_detect();
    g_line_lost_cnt = 0U;

    if (next == ROUTE_DA_ARC) {
        /* 红外提前触发DA时，也必须先应用CD末段降速。 */
        task_apply_cd_slowdown();
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
    uint8_t speed_percent =
        (state == TASK_KEY3_SLOW_LAP)
        ? TASK_SLOW_SPEED_PERCENT
        : TASK_NORMAL_SPEED_PERCENT;

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
    g_cd_slowdown_applied = false;
    g_ab_bc_switch_source = 0U;
    g_cd_da_switch_source = 0U;

    pid_set_key3_profile(state == TASK_KEY3_SLOW_LAP);
    app_task_set_speed_percent(speed_percent);
    app_task_straight_prepare(0.0f);

    oled_clear();
    number_show_time_ms(0U);
    task_show_switch_sources();
}

static void task_finish(FinishReason reason)
{
    (void)reason;

    chassis_stop();
    number_show_time_ms(g_task_elapsed_ms);
    g_task_state = TASK_FINISHED;

    oled_clear();
    task_show_switch_sources();
}

/* ========== 核心路线执行逻辑 ========== */

static void task_run_route(void)
{
    uint8_t pattern = line_read_pattern();

    switch (g_route_phase) {
    case ROUTE_AB_STRAIGHT:
        /* 直线阶段：红外特征优先，编码器保护 */
        if (g_task_state == TASK_KEY2_A_TO_B) {
            /* KEY2：直到检测到B点停止线 */
            if (should_stop_at_B_point(pattern)) {
                task_finish(FINISH_SUCCESS);
                return;
            }
        } else {
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
        if (task_segment_distance_cm() >= TASK_CD_SLOWDOWN_DISTANCE_CM) {
            task_apply_cd_slowdown();
        }

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
            /* KEY1：累计转角达到340度后立即停车 */
            if (abs_f(g_yaw_accum_deg) >= TASK_KEY1_TARGET_YAW_DEG) {
                task_finish(FINISH_SUCCESS);
                return;
            }
        } else {
            /* KEY3：累计转角达到340度后立即停车 */
            if (abs_f(g_yaw_accum_deg) >= TASK_KEY3_TARGET_YAW_DEG) {
                task_finish(FINISH_SUCCESS);
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
    g_yaw_last_deg = 0.0f;
    g_yaw_accum_deg = 0.0f;
    g_phase_start_yaw_deg = 0.0f;
    g_distance_base_cm = 0.0f;
    reset_right_turn_detect();
    g_line_lost_cnt = 0U;
    g_ab_bc_switch_source = 0U;
    g_cd_da_switch_source = 0U;

    chassis_stop();
    app_task_set_oled_debug_enabled(false);
    task_show_select();
}

void task_step(void)
{
    uint8_t key;
    uint32_t timeout;

    if ((g_task_state == TASK_IDLE)
        || (g_task_state == TASK_FINISHED)) {
        key = key_scan();

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
        return;
    }

    /* 运行中按键停止 */
    key = key_scan();
    if (key != 0U) {
        task_finish(FINISH_CANCELLED);
        return;
    }

    g_task_elapsed_ms = motor_millis() - g_task_start_ms;
    number_show_time_ms(g_task_elapsed_ms);
    task_update_yaw();

    /* 超时保护 */
    timeout = task_timeout_ms();
    if ((timeout != 0U) && (g_task_elapsed_ms >= timeout)) {
        task_finish(FINISH_TIMEOUT);
        return;
    }

    /* 执行路线 */
    task_run_route();
    if (g_task_state != TASK_FINISHED) {
        task_show_running();
    }
}
