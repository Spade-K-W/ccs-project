#ifndef KEY_H
#define KEY_H

#include <stdint.h>

/* 第 1~3 关：数码管显示关号后死循环等待 */
void key1_run(void);
void key2_run(void);
void key3_run(void);

/*
 * 按键选关总入口：
 * 等待 key_1~key_3 任一按下 → 进入对应关（显示 1~3 后卡死）
 * 本函数不返回
 */
void key_control(void);

#endif /* KEY_H */
