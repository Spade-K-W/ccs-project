#include "task.h"
#include "key.h"
#include "app_task.h"
#include "encoder.h"
#include "line_sensor.h"
#include "motor.h"
#include "mpu6050.h"
#include "number.h"
#include "oled.h"

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
#define TURN_PATTERN_CONFIRM_CNT         (3U)      /* 转弯特征确认帧数 */
#define TURN_PATTERN_CONFIRM_TIMEOUT_MS  (500U)    /* 特征确认超时 */
#define LINE_LOST_CONFIRM_CNT            (5U)      /* 丢线确认帧数 */

/* 陀螺仪参数 */
#define TASK_ARC_YAW_DEG                 (180.0f)  /* 弧线转弯180度 */
#define TASK_KEY1_TARGET_YAW_DEG         (340.0f)  /* KEY1一圈的目标角度 */

/* 编码器保护参数 */
#define TASK_STRAIGHT_DISTANCE_MAX_CM    (200.0f)  /* 直线段编码器保护上限 */
#define TASK_ARC_PREP_DISTANCE_CM        (120.0f)  /* 提前进弧线的编码器阈值 */

/* 速度参数 */
#define TASK_NORMAL_SPEED_PERCENT        (100U)
#define TASK_SLOW_SPEED_PERCENT          (60U)

/* 显示参数 */
#define TASK_KEY_NOTICE_MS               (2000U)
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
static uint8_t g_right_turn_pattern_cnt = 0U;      /* 右转特征连续检测计数 */
static uint32_t g_right_turn_last_detect_ms = 0U;  /* 上次检测到右转的时间 */
static uint8_t g_line_lost_cnt = 0U;                /* 丢线连续计数 */

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

static float task_total_distance_cm(void)
{
    return g_distance_base_cm + task_segment_distance_cm();
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

/**
 * 检测右转特征：CH1/CH2/CH3 全亮（黑线在左，即将右转）
 */
static bool detect_right_turn_feature(uint8_t pattern)
{
    return line_ch123_all_on(pattern);
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

/**
 * 处理右转特征确认（多帧确认防止抖动）
 * 返回 true 表示已确认转弯
 */
static bool process_right_turn_detect(uint8_t pattern)
{
    uint32_t now_ms = motor_millis();

    if (detect_right_turn_feature(pattern)) {
        /* 继续检测右转特征 */
        if (g_right_turn_pattern_cnt == 0U) {
            g_right_turn_last_detect_ms = now_ms;
        }
        g_right_turn_pattern_cnt++;
        
        /* 连续确认达到阈值 */
        if (g_right_turn_pattern_cnt >= TURN_PATTERN_CONFIRM_CNT) {
            g_right_turn_pattern_cnt = 0U;
            return true;
        }
    } else {
        /* 未检测到或中断，检查是否超时 */
        if (g_right_turn_pattern_cnt > 0U) {
            if ((now_ms - g_right_turn_last_detect_ms) 
                > TURN_PATTERN_CONFIRM_TIMEOUT_MS) {
                /* 超时，清零计数 */
                g_right_turn_pattern_cnt = 0U;
            }
        }
    }

    return false;
}

/**
 * 主要判断函数：是否应该进入弧线阶段
 * 优先级：红外特征 > 编码器保护
 */
static bool should_enter_arc_straight_to_arc(uint8_t pattern)
{
    /* 第1优先级：红外特征检测 */
    if (process_right_turn_detect(pattern)) {
        return true;
    }

    /* 第2优先级：编码器保护（防止红外失效） */
    float distance = task_segment_distance_cm();
    if (distance >= TASK_ARC_PREP_DISTANCE_CM) {
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

static const char *task_key_text(void)
{
    switch (g_task_state) {
    case TASK_KEY1_ONE_LAP:
        return "KEY1 ONE LAP";
    case TASK_KEY2_A_TO_B:
        return "KEY2 A TO B";
    case TASK_KEY3_SLOW_LAP:
        return "KEY3 SLOW LAP";
    default:
        return "";
    }
}

static const char *route_phase_text(void)
{
    switch (g_route_phase) {
    case ROUTE_AB_STRAIGHT:
        return "AB";
    case ROUTE_BC_ARC:
        return "BC";
    case ROUTE_CD_STRAIGHT:
        return "CD";
    case ROUTE_DA_ARC:
        return "DA";
    default:
        return "--";
    }
}

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

static void task_show_select(void)
{
    number_show_time_ms(0U);
    oled_clear();
    oled_display_string(0U, 0U, "Select task");
    oled_display_string(2U, 0U, "KEY1 One lap");
    oled_display_string(4U, 0U, "KEY2 A to B");
    oled_display_string(6U, 0U, "KEY3 Slow lap");
}

static void task_show_running(void)
{
    char line[22];
    uint8_t pattern = line_read_pattern();

    if ((g_task_elapsed_ms - g_display_last_ms)
        < TASK_DISPLAY_PERIOD_MS) {
        return;
    }
    g_display_last_ms = g_task_elapsed_ms;

    /* 第1行：任务名称/路段 */
    if (g_task_elapsed_ms < TASK_KEY_NOTICE_MS) {
        snprintf(line, sizeof(line), "%-21s", task_key_text());
    } else {
        snprintf(line, sizeof(line), "%s Y:%+6.1f D:%4.0f",
                 route_phase_text(), g_yaw_accum_deg,
                 task_total_distance_cm());
    }
    oled_display_string(6U, 0U, line);

    /* 第2行：时间 + 红外模式 */
    snprintf(line, sizeof(line), "T:%lu.%03lu P:%02X    ",
             (unsigned long)(g_task_elapsed_ms / 1000U),
             (unsigned long)(g_task_elapsed_ms % 1000U),
             pattern);
    oled_display_string(7U, 0U, line);
}

/* ========== 状态机 ========== */

static void task_enter_phase(RoutePhase next)
{
    /* 保存刚完成路段的编码器距离 */
    g_distance_base_cm += task_segment_distance_cm();
    g_route_phase = next;
    g_phase_start_yaw_deg = g_yaw_accum_deg;

    /* 重置红外检测计数 */
    g_right_turn_pattern_cnt = 0U;
    g_line_lost_cnt = 0U;

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
    g_right_turn_pattern_cnt = 0U;
    g_line_lost_cnt = 0U;

    app_task_set_speed_percent(speed_percent);
    app_task_straight_prepare(0.0f);

    oled_clear();
    number_show_time_ms(0U);

    oled_display_string(6U, 0U, task_key_text());
    oled_display_string(7U, 0U, "T:0.000s");
}

static void task_finish(FinishReason reason)
{
    char line[22];
    const char *name = task_key_text();
    const char *result;

    switch (reason) {
    case FINISH_SUCCESS:
        result = "TASK FINISHED";
        break;
    case FINISH_TIMEOUT:
        result = "TASK TIMEOUT";
        break;
    default:
        result = "TASK CANCELLED";
        break;
    }

    chassis_stop();
    number_show_time_ms(g_task_elapsed_ms);
    g_task_state = TASK_FINISHED;

    oled_clear();
    oled_display_string(0U, 0U, result);
    oled_display_string(1U, 0U, name);

    snprintf(line, sizeof(line), "TIME:%lu.%03lus",
             (unsigned long)(g_task_elapsed_ms / 1000U),
             (unsigned long)(g_task_elapsed_ms % 1000U));
    oled_display_string(3U, 0U, line);

    snprintf(line, sizeof(line), "YAW:%+7.1f deg", g_yaw_accum_deg);
    oled_display_string(5U, 0U, line);

    snprintf(line, sizeof(line), "DIST:%6.1f cm",
             task_total_distance_cm());
    oled_display_string(6U, 0U, line);
    oled_display_string(7U, 0U, "Press next key");
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
            /* KEY1：整圈模式，目标角度340度 */
            if (abs_f(g_yaw_accum_deg) >= TASK_KEY1_TARGET_YAW_DEG) {
                task_finish(FINISH_SUCCESS);
                return;
            }
        } else {
            /* KEY3：标准180度转弯 */
            if (task_phase_yaw_deg() >= TASK_ARC_YAW_DEG) {
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
    g_right_turn_pattern_cnt = 0U;
    g_line_lost_cnt = 0U;

    chassis_stop();
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