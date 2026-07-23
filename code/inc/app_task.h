#ifndef APP_TASK_H
#define APP_TASK_H

#include <stdbool.h>
#include <stdint.h>

#if 0 /* 比赛状态机未再被调用，暂禁用 */
/* =========================================================
 * 小车运行状态枚举
 * PHASE_AC_STRAIGHT : A -> C 直线段
 * PHASE_CB_ARC      : C -> B 弧线段
 * PHASE_BD_STRAIGHT : B -> D 直线段
 * PHASE_DA_ARC      : D -> A 弧线段
 * PHASE_FINISH      : 完成任务，停车
 * ========================================================= */
typedef enum {
    PHASE_START = 0,

    /* 题(1) */
    PH1_AB_STRAIGHT,        /* A→B 无线直行，传感器到B */

    /* 题(2) 顺时针 A→B→C→D→A */
    PH2_AB_STRAIGHT,        /* A→B 无线直行 */
    PH2_BC_ARC,             /* B→C 右半圆有线循迹，顺时针 */
    PH2_CD_STRAIGHT,        /* C→D 无线直行 */
    PH2_DA_ARC,             /* D→A 左半圆有线循迹，顺时针 */

    /* 题(3)(4) 八字 A→C→B→D→A */
    PH34_AC_STRAIGHT,       /* A→C 无线对角直行 */
    PH34_CB_ARC,            /* C→B 右半圆有线循迹，逆时针 */
    PH34_BD_STRAIGHT,       /* B→D 无线对角直行 */
    PH34_DA_ARC,            /* D→A 左半圆有线循迹，逆时针 */

    PHASE_FINISH
} RunPhase;

/* 初始化任务状态机 */
void app_task_init(void);

/* 主任务单步执行函数
 * 建议在主循环中每隔固定时间调用一次
 */
void app_task_run_once(void);

/* 判断任务是否已经完成 */
bool app_task_is_finished(void);

/* 直线段单步：红外循迹 + 差速 */
void do_straight_drive_step(void);
#endif

/* 设置直线段目标航向（走直前调用，通常传 0.0f） */
void app_task_straight_prepare(float targetHeadingDeg);

/* 根据加权误差驱动左右轮，并刷新 OLED/串口 */
void line_follow_drive(uint8_t pattern, float error, bool lineValid);

/* 读取最近一次外环目标左右轮速度（未走速度环） */
void line_follow_get_wheels(int16_t *left, int16_t *right);

/* 读取最近一次速度环修正后实际下发的左右轮速度 */
void line_follow_get_out_wheels(int16_t *left, int16_t *right);

/* 左转弯并刷新 OLED/串口 */
void line_follow_left_turn(uint8_t pattern, float error, bool lineValid);

/* 右转弯并刷新 OLED/串口 */
void line_follow_right_turn(uint8_t pattern, float error, bool lineValid);

/*
 * 循迹状态机单步（主循环每拍调用）：
 *  状态一 直线循迹；CH123 → 预延时 TURN_DETECT_DELAY_MS 后左转；
 *  CH678 → 预延时后右转；转满后回状态一。
 *  每次 CH123/CH678 触发计 1 路口，满 CORNERS_PER_LAP 次记 1 圈。
 */
void app_task_line_step(uint8_t pattern, float error, bool lineValid);

/* 当前是否处于直线循迹阶段（非原地转弯） */
bool app_task_line_is_straight(void);

/* 路口累计圈数：每 4 次 CH123/CH678 记 1 圈 */
uint32_t app_task_get_lap_count(void);

/* 路口计数与圈数清零 */
void app_task_reset_lap_count(void);

#endif
