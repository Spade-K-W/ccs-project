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
 * 连续黑色环线的路线状态：
 *   A→B：红外直线循迹 150 cm
 *   B→C：红外右弧循迹 180°
 *   C→D：红外直线循迹 150 cm
 *   D→A：红外右弧循迹 180°
 *
 * 不再把“看见黑线/丢失黑线”作为直线与弧线的切换条件。
 */
#define TASK_STRAIGHT_DISTANCE_CM    (160.0f)
#define TASK_ARC_YAW_DEG             (180.0f)
/*
 * 提前进入弧线准备：随后由 line_follow_arc() 普通巡线 1 秒，
 * 再启用强制右转。慢速模式一秒走得更短，所以切换点更靠后。
 */
#define TASK_ARC_PREP_NORMAL_CM      (125.0f)
#define TASK_ARC_PREP_SLOW_CM        (132.0f)
#define TASK_NORMAL_SPEED_PERCENT    (100U)
#define TASK_SLOW_SPEED_PERCENT      (60U)
#define TASK_KEY_NOTICE_MS           (2000U)
#define TASK_DISPLAY_PERIOD_MS       (100U)

#define TASK_KEY1_TARGET_YAW_DEG     (340.0f)
#define TASK_KEY2_TIMEOUT_MS         (8000U)
#define TASK_KEY3_TIMEOUT_MS         (30000U)

/* app_task.c 约定：+1 为左弧，-1 为右弧；顺时针使用右弧。 */
#define TASK_ARC_MIRROR_DIR          (-1.0f)

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

static TaskState  g_task_state = TASK_IDLE;
static RoutePhase g_route_phase = ROUTE_AB_STRAIGHT;

static uint32_t g_task_start_ms = 0U;
static uint32_t g_task_elapsed_ms = 0U;
static uint32_t g_display_last_ms = 0U;

/* MPU 当前角度会在 ±180°回绕；这里保存解回绕后的整段累计角度。 */
static float g_yaw_last_deg = 0.0f;
static float g_yaw_accum_deg = 0.0f;
static float g_phase_start_yaw_deg = 0.0f;

/*
 * 每次直线/弧线 prepare 都会清编码器。
 * 已完成路段距离放在 base 中，当前路段直接读取复位后的编码器。
 */
static float g_distance_base_cm = 0.0f;

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
        /* KEY1 stops by accumulated MPU yaw, not by elapsed time. */
        return 0U;
    case TASK_KEY2_A_TO_B:
        return TASK_KEY2_TIMEOUT_MS;
    case TASK_KEY3_SLOW_LAP:
        return TASK_KEY3_TIMEOUT_MS;
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

    if ((g_task_elapsed_ms - g_display_last_ms)
        < TASK_DISPLAY_PERIOD_MS) {
        return;
    }
    g_display_last_ms = g_task_elapsed_ms;

    if (g_task_elapsed_ms < TASK_KEY_NOTICE_MS) {
        snprintf(line, sizeof(line), "%-21s", task_key_text());
    } else {
        snprintf(line, sizeof(line), "%s Y:%+6.1f D:%4.0f",
                 route_phase_text(), g_yaw_accum_deg,
                 task_total_distance_cm());
    }
    oled_display_string(6U, 0U, line);

    snprintf(line, sizeof(line), "T:%lu.%03lus          ",
             (unsigned long)(g_task_elapsed_ms / 1000U),
             (unsigned long)(g_task_elapsed_ms % 1000U));
    oled_display_string(7U, 0U, line);
}

static float task_arc_prep_distance_cm(void)
{
    return (g_task_state == TASK_KEY3_SLOW_LAP)
        ? TASK_ARC_PREP_SLOW_CM
        : TASK_ARC_PREP_NORMAL_CM;
}

static void task_drive_straight(void)
{
    uint8_t pattern = line_read_pattern();
    float error = 0.0f;
    bool line_valid = line_calc_error_f(pattern, &error);

    line_follow_drive(pattern, error, line_valid);
}

static void task_drive_arc(void)
{
    uint8_t pattern = line_read_pattern();
    float error = 0.0f;
    bool line_valid = line_calc_error_arc_f(pattern, &error);

    app_task_arc_step(pattern, error, line_valid);
}

static void task_enter_phase(RoutePhase next)
{
    /*
     * prepare 会清编码器；先保存刚完成路段的实际编码器距离。
     * 初始相位不走本函数，因此不会重复累计。
     */
    g_distance_base_cm += task_segment_distance_cm();
    g_route_phase = next;
    g_phase_start_yaw_deg = g_yaw_accum_deg;

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

static void task_run_route(void)
{
    float distance;

    switch (g_route_phase) {
    case ROUTE_AB_STRAIGHT:
        distance = task_segment_distance_cm();
        if (g_task_state == TASK_KEY2_A_TO_B) {
            if (distance >= TASK_STRAIGHT_DISTANCE_CM) {
                task_finish(FINISH_SUCCESS);
                return;
            }
        } else {
            if (distance >= task_arc_prep_distance_cm()) {
                task_enter_phase(ROUTE_BC_ARC);
                return;
            }
        }
        task_drive_straight();
        break;

    case ROUTE_BC_ARC:
        if (task_phase_yaw_deg() >= TASK_ARC_YAW_DEG) {
            task_enter_phase(ROUTE_CD_STRAIGHT);
            return;
        }
        task_drive_arc();
        break;

    case ROUTE_CD_STRAIGHT:
        distance = task_segment_distance_cm();
        if (distance >= task_arc_prep_distance_cm()) {
            task_enter_phase(ROUTE_DA_ARC);
            return;
        }
        task_drive_straight();
        break;

    case ROUTE_DA_ARC:
        if (((g_task_state == TASK_KEY1_ONE_LAP)
             && (abs_f(g_yaw_accum_deg) >= TASK_KEY1_TARGET_YAW_DEG))
            || ((g_task_state != TASK_KEY1_ONE_LAP)
                && (task_phase_yaw_deg() >= TASK_ARC_YAW_DEG))) {
            task_finish(FINISH_SUCCESS);
            return;
        }
        task_drive_arc();
        break;

    default:
        task_finish(FINISH_CANCELLED);
        break;
    }
}

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

    /*
     * 运行中再次按任意按键可立即停车。
     * 启动按键必须先松开，再次按下才会产生新事件。
     */
    key = key_scan();
    if (key != 0U) {
        task_finish(FINISH_CANCELLED);
        return;
    }

    g_task_elapsed_ms = motor_millis() - g_task_start_ms;
    number_show_time_ms(g_task_elapsed_ms);
    task_update_yaw();

    timeout = task_timeout_ms();
    if ((timeout != 0U) && (g_task_elapsed_ms >= timeout)) {
        task_finish(FINISH_TIMEOUT);
        return;
    }

    task_run_route();
    if (g_task_state != TASK_FINISHED) {
        task_show_running();
    }
}
