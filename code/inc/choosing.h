#ifndef CHOOSING_H
#define CHOOSING_H

#include <stdint.h>

/*
 * 关卡选择：
 * 1) 等待第一次按下 USER_KEY，从该时刻起开统计窗口
 * 2) 窗口内按下次数 → 关卡号（1~5）；松手静置 KEY_CONFIRM_IDLE_MS 即确认
 * 3) OLED 显示 project:N，蜂鸣器短鸣 N 声
 * 返回值：1 ~ 5
 */
uint8_t choosing_select_project(void);

#endif /* CHOOSING_H */
