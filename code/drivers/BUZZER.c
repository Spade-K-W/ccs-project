#include "BUZZER.h"

/* PB13已改作循迹X1；保留空实现以兼容现有调用。 */

void BUZZER_Set(bool on)
{
    (void)on;
}

void BUZZER_Mute(void)
{
}

void BUZZER_Beep(uint32_t on_ms)
{
    (void)on_ms;
}

void BUZZER_BeepShort(void)
{
}
