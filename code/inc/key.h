#ifndef KEY_H
#define KEY_H

#include <stdint.h>

/* 初始化 KEY1/KEY2/KEY3 输入及消抖状态 */
void key_init(void);

/*
 * 非阻塞扫描，主循环周期调用：
 *   0：无新按键
 *   1：KEY1 新按下一次
 *   2：KEY2 新按下一次
 *   3：KEY3 新按下一次
 */
uint8_t key_scan(void);

#endif /* KEY_H */
