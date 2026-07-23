#ifndef SIGNAL_H
#define SIGNAL_H

#include <stdbool.h>
#include <stdint.h>

/* 控制蜂鸣器开关 */
void buzzer_set(bool on);

/* 阻塞鸣叫一次 */
void buzzer_beep(uint32_t ms);

/* 同时控制 LED + 蜂鸣器 */
void signal_set(bool on);

/* 按指定时间和次数进行闪烁/鸣叫 */
void signal_blink(uint32_t onMs, uint32_t offMs, uint8_t times);

/* 启动提示 */
void signal_start_async(void);

/* 经过关键点提示 */
void signal_pass_point_async(void);

/* 完成任务提示 */
void signal_finish_async(void);

/* 主循环中周期调用 */
void signal_task_run_once(void);

#endif