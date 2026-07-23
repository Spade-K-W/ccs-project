#include "USER_BUTTON.h"
#include "BUZZER.h"
#include "app_config.h"
#include "board_defs.h"
#include "Delay.h"
#include "oled.h"

#include <stdio.h>

bool USER_BUTTON_IsPressed(void)
{
    bool raw = pin_read_raw(&USER_KEY);
    return USER_KEY_ACTIVE_LOW ? (!raw) : raw;
}

/* 等待按键出现一次按下边沿（松开→按下），带消抖 */
static void USER_BUTTON_WaitPressEdge(void)
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

uint8_t USER_BUTTON_InputTargetLaps(void)
{
    uint8_t  pressCnt;
    uint32_t elapsed;
    uint32_t idleMs;
    bool     wasPressed;
    bool     nowPressed;
    char     line[20];

    oled_display_string(0, 0, "Set Laps        ");
    oled_display_string(1, 0, "Wait 1st KEY... ");
    oled_display_string(2, 0, "release=confirm ");
    oled_display_string(3, 0, "                ");

    /* 第一次按下起开启窗口，并计入第 1 次 */
    USER_BUTTON_WaitPressEdge();
    pressCnt   = 1U;
    wasPressed = true;
    elapsed    = 0U;
    idleMs     = 0U;

    snprintf(line, sizeof(line), "Laps: %u         ", (unsigned)pressCnt);
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
                if (pressCnt < 255U) {
                    pressCnt++;
                }
                snprintf(line, sizeof(line), "Laps: %u         ", (unsigned)pressCnt);
                oled_display_string(3, 0, line);
                wasPressed = true;
                idleMs     = 0U;
            }
        } else if (!nowPressed) {
            wasPressed = false;
            idleMs += 5U;
            if (idleMs >= (uint32_t)KEY_CONFIRM_IDLE_MS) {
                break;
            }
        } else {
            idleMs = 0U;
        }

        Delay_ms(5U);
        elapsed += 5U;
    }

    if (pressCnt == 0U) {
        pressCnt = 1U;
    }

    snprintf(line, sizeof(line), "Target: %u       ", (unsigned)pressCnt);
    oled_display_string(1, 0, "Laps locked     ");
    oled_display_string(3, 0, line);
    BUZZER_BeepShort();

    while (USER_BUTTON_IsPressed()) {
        Delay_ms(5U);
    }

    return pressCnt;
}
