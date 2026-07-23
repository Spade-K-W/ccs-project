#include "chapter.h"
#include "app_config.h"
#include "app_task.h"
#include "USER_BUTTON.h"
#include "BUZZER.h"
#include "Delay.h"
#include "line_sensor.h"
#include "motor.h"
#include "oled.h"
#include "uart_vision.h"

#include <stdint.h>
#include <stdbool.h>

/* 第 2~5 关共用：握手后停车死循环 */
static void chapter_handshake_then_hold(uint8_t mode)
{
    uart_vision_handshake_mode(mode);
    chassis_stop();
    motor_driver_disable();
    while (1) {
        Delay_ms(500U);
    }
}

void chapter1_run(void)
{
    uint8_t  pattern;
    float    error;
    bool     lineValid;
    uint8_t  targetLaps;
    uint32_t finishElapsedMs = 0U;

    targetLaps = USER_BUTTON_InputTargetLaps();
    app_task_reset_lap_count();
    app_task_straight_prepare(0.0f);
    oled_clear();

    /*
     * 第 1 关主循环：
     * - 状态一：红外循迹 PID
     * - CH123 → 左转；CH678 → 右转（每次计 1 路口，满 4 次 = 1 圈）
     * - 达到目标圈数后：等末弯转完回到直线循迹，再延时后停
     */
    while (1) {
        pattern   = line_read_pattern();
        lineValid = line_calc_error_f(pattern, &error);

        app_task_line_step(pattern, error, lineValid);

        if ((app_task_get_lap_count() >= (uint32_t)targetLaps)
            && app_task_line_is_straight()) {
            finishElapsedMs += (uint32_t)LOOP_PERIOD_MS;
            if (finishElapsedMs >= (uint32_t)LAP_FINISH_DELAY_MS) {
                chassis_stop();
                motor_driver_disable();
                oled_display_string(0, 0, "Laps Done       ");
                BUZZER_BeepShort();
                while (1) {
                    Delay_ms(500U);
                }
            }
        } else {
            finishElapsedMs = 0U;
        }

        Delay_ms(LOOP_PERIOD_MS);
    }
}

void chapter2_run(void)
{
    chapter_handshake_then_hold(2U);
}

void chapter3_run(void)
{
    chapter_handshake_then_hold(3U);
}

void chapter4_run(void)
{
    chapter_handshake_then_hold(4U);
}

void chapter5_run(void)
{
    chapter_handshake_then_hold(5U);
}
