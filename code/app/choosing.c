#include "choosing.h"
#include "USER_BUTTON.h"
#include "BUZZER.h"
#include "app_config.h"
#include "Delay.h"
#include "oled.h"

#include <stdbool.h>
#include <stdio.h>

#define PROJECT_MAX  (5U)

/* 等待按键出现一次按下边沿（松开→按下），带消抖 */
static void choosing_wait_press_edge(void)
{
    while (USER_BUTTON_IsPressed()) {
        Delay_ms(5U);
    }

    for (;;) {
        if (USER_BUTTON_IsPressed()) {
            Delay_ms((uint32_t)USER_KEY_DEBOUNCE_MS);
            if (USER_BUTTON_IsPressed()) {
                return;
            }
        }
        Delay_ms(5U);
    }
}

/* 短鸣 times 声，声间留空隙便于分辨 */
static void choosing_beep_times(uint8_t times)
{
    uint8_t i;

    if (times == 0U) {
        times = 1U;
    }

    for (i = 0U; i < times; i++) {
        BUZZER_BeepShort();
        if ((i + 1U) < times) {
            Delay_ms(150U);
        }
    }
}

/* 进入关卡提示：OLED 显示 project:N，蜂鸣器鸣叫 N 声 */
static void choosing_enter_announce(uint8_t project)
{
    char line[20];

    oled_clear();
    snprintf(line, sizeof(line), "project:%u       ", (unsigned)project);
    oled_display_string(0, 0, line);
    oled_display_string(1, 0, "                ");
    oled_display_string(2, 0, "                ");
    oled_display_string(3, 0, "                ");

    choosing_beep_times(project);
    Delay_ms(300U);

    /* 等松手，避免长按残留边沿带进下一阶段（设圈数） */
    while (USER_BUTTON_IsPressed()) {
        Delay_ms(5U);
    }
    Delay_ms(100U);
}

uint8_t choosing_select_project(void)
{
    uint8_t  pressCnt;
    uint32_t elapsed;
    uint32_t idleMs;
    bool     wasPressed;
    bool     nowPressed;
    char     line[20];

    oled_clear();
    oled_display_string(0, 0, "Select Project  ");
    oled_display_string(1, 0, "1~5 KEY press  ");
    oled_display_string(2, 0, "release=confirm ");
    oled_display_string(3, 0, "Wait 1st KEY... ");

    /* 第一次按下起开启窗口，并计入第 1 次 */
    choosing_wait_press_edge();
    pressCnt   = 1U;
    wasPressed = true;
    elapsed    = 0U;
    idleMs     = 0U;

    snprintf(line, sizeof(line), "Project: %u      ", (unsigned)pressCnt);
    oled_display_string(3, 0, line);

    while (elapsed < (uint32_t)LAP_SET_WINDOW_MS) {
        nowPressed = USER_BUTTON_IsPressed();

        if (nowPressed && !wasPressed) {
            Delay_ms((uint32_t)USER_KEY_DEBOUNCE_MS);
            elapsed += (uint32_t)USER_KEY_DEBOUNCE_MS;
            if (elapsed >= (uint32_t)LAP_SET_WINDOW_MS) {
                break;
            }
            if (USER_BUTTON_IsPressed()) {
                if (pressCnt < PROJECT_MAX) {
                    pressCnt++;
                }
                snprintf(line, sizeof(line), "Project: %u      ",
                         (unsigned)pressCnt);
                oled_display_string(3, 0, line);
                wasPressed = true;
                idleMs     = 0U;
            }
        } else if (!nowPressed) {
            wasPressed = false;
            idleMs += 5U;
            /* 松手静置一段时间 → 确认当前关卡（可早于 2s 窗口结束） */
            if (idleMs >= (uint32_t)KEY_CONFIRM_IDLE_MS) {
                break;
            }
        } else {
            idleMs = 0U;
        }

        Delay_ms(5U);
        elapsed += 5U;
    }

    if (pressCnt < 1U) {
        pressCnt = 1U;
    }
    if (pressCnt > PROJECT_MAX) {
        pressCnt = PROJECT_MAX;
    }

    choosing_enter_announce(pressCnt);
    return pressCnt;
}
