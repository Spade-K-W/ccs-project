#include "bluetooth.h"
#include "app_config.h"
#include "Delay.h"
#include "ti_msp_dl_config.h"

#include <string.h>
#include <stdbool.h>

/* SysConfig：Bluetooth_UART_1，RX=PA9，TX=PB4 */
#ifndef Bluetooth_UART_1_INST
#error "Bluetooth_UART_1_INST not defined — check SysConfig Bluetooth_UART_1 (PA9/PB4)"
#endif

#define BT_UART_INST Bluetooth_UART_1_INST

static UartDebugStatus s_oledCache;
static uint8_t s_oledCacheValid;
static uint32_t s_sendElapsedMs;

void bluetooth_init(void)
{
    /* UART 引脚与时钟由 SysConfig 在 SYSCFG_DL_init() 中完成初始化 */
    memset(&s_oledCache, 0, sizeof(s_oledCache));
    s_oledCacheValid = 0U;
    s_sendElapsedMs = BLUETOOTH_SEND_INTERVAL_MS;
}

void bluetooth_putc(char c)
{
    uint32_t guard = 50000U;

    while (DL_UART_Main_isTXFIFOFull(BT_UART_INST) != false) {
        if (--guard == 0U) {
            return;
        }
    }
    DL_UART_Main_transmitData(BT_UART_INST, (uint8_t)c);
}

void bluetooth_puts(const char *s)
{
    while ((s != NULL) && (*s != '\0')) {
        bluetooth_putc(*s++);
    }
}

void bluetooth_write(const char *data, size_t len)
{
    size_t i;

    if (data == NULL) {
        return;
    }

    for (i = 0U; i < len; i++) {
        bluetooth_putc(data[i]);
    }
}

void bluetooth_cache_oled_status(const UartDebugStatus *st)
{
    if (st == NULL) {
        return;
    }
    s_oledCache = *st;
    s_oledCacheValid = 1U;
}

void bluetooth_send_oled_now(void)
{
    if (s_oledCacheValid == 0U) {
        return;
    }

    s_sendElapsedMs += (uint32_t)LOOP_PERIOD_MS;
    if (s_sendElapsedMs < (uint32_t)BLUETOOTH_SEND_INTERVAL_MS) {
        return;
    }
    s_sendElapsedMs = 0U;

    /* 与本地 OLED / uart_debug 六行格式一致，便于对端直接刷屏 */
    bluetooth_puts(s_oledCache.line_state);
    bluetooth_puts("\r\n");
    bluetooth_puts(s_oledCache.line_pat);
    bluetooth_puts("\r\n");
    bluetooth_puts(s_oledCache.line_cmd);
    bluetooth_puts("\r\n");
    bluetooth_puts(s_oledCache.line_mea);
    bluetooth_puts("\r\n");
    bluetooth_puts(s_oledCache.line_corr);
    bluetooth_puts("\r\n");
    bluetooth_puts(s_oledCache.line_angle);
    bluetooth_puts("\r\n");
    /* 空行作为帧结束标记，方便对端分包 */
    bluetooth_puts("\r\n");
}

static void bluetooth_flush_rx(void)
{
    while (DL_UART_Main_isRXFIFOEmpty(BT_UART_INST) == false) {
        (void)DL_UART_Main_receiveData(BT_UART_INST);
    }
}

static bool bluetooth_try_getc(char *out)
{
    if (DL_UART_Main_isRXFIFOEmpty(BT_UART_INST) != false) {
        return false;
    }
    *out = (char)DL_UART_Main_receiveData(BT_UART_INST);
    return true;
}

static bool bluetooth_line_is_sync(const char *line)
{
    return (strcmp(line, "BT_READY") == 0) || (strcmp(line, "BT_ACK") == 0);
}

void bluetooth_wait_peer(void)
{
    char line[24];
    uint8_t idx = 0U;
    char c;

    bluetooth_flush_rx();

    for (;;) {
        bluetooth_puts("BT_READY\r\n");
        idx = 0U;

        {
            uint32_t t = 0U;
            while (t < (uint32_t)BLUETOOTH_HANDSHAKE_RETRY_MS) {
                if (bluetooth_try_getc(&c)) {
                    if ((c == '\r') || (c == '\n')) {
                        if (idx > 0U) {
                            line[idx] = '\0';
                            if (bluetooth_line_is_sync(line)) {
                                bluetooth_puts("BT_ACK\r\n");
                                Delay_ms(30U);
                                bluetooth_flush_rx();
                                return;
                            }
                            idx = 0U;
                        }
                    } else if (idx < (sizeof(line) - 1U)) {
                        line[idx++] = c;
                    } else {
                        idx = 0U;
                    }
                } else {
                    Delay_ms(1U);
                    t++;
                }
            }
        }
    }
}
