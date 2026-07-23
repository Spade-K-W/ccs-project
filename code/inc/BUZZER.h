#ifndef BUZZER_H
#define BUZZER_H

#include <stdint.h>
#include <stdbool.h>

/* 蜂鸣器开/关（当前高电平发声、低电平静音） */
void BUZZER_Set(bool on);

/* 立即静音（拉低 PB13） */
void BUZZER_Mute(void);

/* 短促鸣叫一次（阻塞），on_ms 为发声持续时间 */
void BUZZER_Beep(uint32_t on_ms);

/* 默认短提示音（时长见 BUZZER_SHORT_BEEP_MS） */
void BUZZER_BeepShort(void);

#endif /* BUZZER_H */
