#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include "uart_debug.h"

#include <stddef.h>
#include <stdint.h>

/*
 * UART通信：SysConfig TI_UART
 *   RX = PA9，TX = PB4（115200）
 * 本板作发送端，把 OLED 六行状态发给另一块 TI 板。
 */

void bluetooth_init(void);
void bluetooth_putc(char c);
void bluetooth_puts(const char *s);
void bluetooth_write(const char *data, size_t len);

/* 按键通信：发送单个 ASCII 字符 '1'、'2' 或 '3'，不附加换行。 */
void bluetooth_send_key(uint8_t key);

/* 缓存当前 OLED 六行（由 app_task 刷新 UI 时调用） */
void bluetooth_cache_oled_status(const UartDebugStatus *st);

/* 主循环每 LOOP_PERIOD_MS 调用；内部按 BLUETOOTH_SEND_INTERVAL_MS 节流发送 */
void bluetooth_send_oled_now(void);

/*
 * 阻塞等待对端：互发 "ok\r\n"，收到带 ok 的一行即成功。
 */
void bluetooth_wait_peer(void);

#endif /* BLUETOOTH_H */
