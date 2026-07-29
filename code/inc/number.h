#ifndef NUMBER_H
#define NUMBER_H

#include <stdint.h>

/* 共阳四位七段数码管（双 74HC595）：SysConfig 组 number → SCLK/RCLK/DIO */

void number_init(void);

/* 仅在第 1 位显示 0~9，其余位熄灭 */
void number_show_digit(uint8_t digit);

/* 显示运行时间 SS.hh（秒.百分秒），最大锁定为 99.99 秒 */
void number_show_time_ms(uint32_t elapsed_ms);

/* 由 1ms 定时中断调用，完成四位动态扫描 */
void number_refresh_1ms_isr(void);

/*
 * 调试用：USER_KEY 每按一次数码管 +1（0→9），本函数不返回。
 * 主流程不调用，仅保留备用。
 */
void number_user_key_count_demo(void);

#endif /* NUMBER_H */
