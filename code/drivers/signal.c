#include "signal.h"
#include "BUZZER.h"
#include "app_config.h"
#include "app_utils.h"
#include "board_defs.h"

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    SIGNAL_IDLE = 0,
    SIGNAL_ON,
    SIGNAL_OFF
} SignalState;

static SignalState g_signalState = SIGNAL_IDLE;

static uint32_t g_signalTimerMs = 0U;
static uint32_t g_signalOnMs = 0U;
static uint32_t g_signalOffMs = 0U;
static uint8_t  g_signalRemainTimes = 0U;


/* ==================== 蜂鸣器控制（转发至 BUZZER 驱动） ==================== */

void buzzer_set(bool on)
{
    BUZZER_Set(on);
}

void buzzer_beep(uint32_t ms)
{
    BUZZER_Beep(ms);
}


/* ==================== 提示输出控制：蜂鸣器 + LED ==================== */

static void signal_output_on(void)
{
    BUZZER_Set(true);
    board_led_set(true);
}

static void signal_output_off(void)
{
    BUZZER_Set(false);
    board_led_set(false);
}


/* ==================== 非阻塞提示启动 ==================== */

static void signal_start_pattern(uint32_t onMs, uint32_t offMs, uint8_t times)
{
    if (times == 0U) {
        signal_output_off();
        g_signalState = SIGNAL_IDLE;
        return;
    }

    g_signalOnMs = onMs;
    g_signalOffMs = offMs;
    g_signalRemainTimes = times;
    g_signalTimerMs = 0U;

    signal_output_on();
    g_signalState = SIGNAL_ON;
}


void signal_start_async(void)
{
    signal_start_pattern(POINT_SIGNAL_ON_MS,
                         POINT_SIGNAL_OFF_MS,
                         START_SIGNAL_TIMES);
}


void signal_pass_point_async(void)
{
    signal_start_pattern(POINT_SIGNAL_ON_MS,
                         POINT_SIGNAL_OFF_MS,
                         POINT_SIGNAL_TIMES);
}


void signal_finish_async(void)
{
    signal_start_pattern(POINT_SIGNAL_ON_MS,
                         POINT_SIGNAL_OFF_MS,
                         FINISH_SIGNAL_TIMES);
}


/* ==================== 非阻塞提示任务 ==================== */

void signal_task_run_once(void)
{
    if (g_signalState == SIGNAL_IDLE) {
        return;
    }

    g_signalTimerMs += LOOP_PERIOD_MS;

    if (g_signalState == SIGNAL_ON) {
        if (g_signalTimerMs >= g_signalOnMs) {
            g_signalTimerMs = 0U;

            signal_output_off();

            if (g_signalRemainTimes > 0U) {
                g_signalRemainTimes--;
            }

            if (g_signalRemainTimes == 0U) {
                g_signalState = SIGNAL_IDLE;
            } else {
                g_signalState = SIGNAL_OFF;
            }
        }
    } else if (g_signalState == SIGNAL_OFF) {
        if (g_signalTimerMs >= g_signalOffMs) {
            g_signalTimerMs = 0U;

            signal_output_on();
            g_signalState = SIGNAL_ON;
        }
    }
}


/* ==================== 模式选择阶段阻塞反馈 ====================
 * 这个只在开跑前选择模式时调用，所以可以保留阻塞 delay。
 */
void signal_blink(uint32_t onMs, uint32_t offMs, uint8_t times)
{
    uint8_t i;

    for (i = 0U; i < times; i++) {
        signal_output_on();
        delay_ms(onMs);

        signal_output_off();
        delay_ms(offMs);
    }
}