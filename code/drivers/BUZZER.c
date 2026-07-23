#include "BUZZER.h"
#include "app_config.h"
#include "board_defs.h"
#include "Delay.h"

/* =========================================================
 * 蜂鸣器驱动
 * 硬件：高电平发声，低电平静音（BUZZER_ACTIVE_HIGH=1）
 * ========================================================= */

void BUZZER_Set(bool on)
{
#if BUZZER_ACTIVE_HIGH
    if (on) {
        pin_high(&BUZZER);  /* 拉高发声 */
    } else {
        pin_low(&BUZZER);   /* 拉低静音 */
    }
#else
    if (on) {
        pin_low(&BUZZER);
    } else {
        pin_high(&BUZZER);
    }
#endif
}

void BUZZER_Mute(void)
{
    BUZZER_Set(false);
}

void BUZZER_Beep(uint32_t on_ms)
{
    BUZZER_Set(true);
    Delay_ms(on_ms);
    BUZZER_Set(false);
}

void BUZZER_BeepShort(void)
{
    BUZZER_Beep((uint32_t)BUZZER_SHORT_BEEP_MS);
}
