#ifndef USER_BUTTON_H
#define USER_BUTTON_H

#include <stdint.h>
#include <stdbool.h>

/* 读取 USER 按键是否按下（已按 ACTIVE_LOW 语义转换） */
bool USER_BUTTON_IsPressed(void);

/*
 * 行驶圈数设定：
 * 1) 蜂鸣器短鸣一声，提示开始输入
 * 2) 等待第一次按下，从该时刻起开 2s 窗口
 * 3) 统计窗口内（含第一次）按下次数，作为目标圈数
 * 返回值：目标圈数（至少为 1）
 */
uint8_t USER_BUTTON_InputTargetLaps(void);

#endif /* USER_BUTTON_H */
