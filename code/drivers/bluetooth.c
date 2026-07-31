#include "bluetooth.h"
#include "app_config.h"
#include "Delay.h"
#include "uart_debug.h"
#include "ti_msp_dl_config.h"

#include <string.h>
#include <stdbool.h>

/* SysConfig：TI_UART，RX=PA9，TX=PB4，115200 */
#ifndef TI_UART_INST
#error "TI_UART_INST not defined - check SysConfig TI_UART (PA9/PB4)"
#endif

#define BT_UART_INST TI_UART_INST

static UartDebugStatus s_oledCache;
static uint8_t s_oledCacheValid;
static uint32_t s_sendElapsedMs;

void bluetooth_init(void)
{
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

void bluetooth_send_key(uint8_t key)
{
    if ((key >= 1U) && (key <= 3U)) {
        bluetooth_putc((char)('0' + key));
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
    uint8_t b;

    if (DL_UART_Main_receiveDataCheck(BT_UART_INST, &b) == false) {
        return false;
    }
    *out = (char)b;
    return true;
}

/*
 * 握手：周期性发 "ok\r\n"
 * 用字节状态机识别连续 o/k（不依赖整行，避免 \r\n 拆散）
 * 任一侧看到 ok 后连续补发约 1.5s，让后成功的一侧也能收到
 */
void bluetooth_wait_peer(void)
{
    char c;
    uint8_t saw_o = 0U;
    uint8_t n;
    uint32_t idle_ms = 0U;
    uint32_t rounds = 0U;

    bluetooth_flush_rx();
    uart_debug_puts("BT HS: send/wait ok\r\n");

    for (;;) {
        bluetooth_puts("ok\r\n");
        rounds++;
        if ((rounds % 4U) == 0U) {
            uart_debug_puts("BT HS: still linking...\r\n");
        }

        idle_ms = 0U;
        while (idle_ms < (uint32_t)BLUETOOTH_HANDSHAKE_RETRY_MS) {
            if (bluetooth_try_getc(&c)) {
                if ((c >= 0x20) && (c <= 0x7E)) {
                    uart_debug_putc(c);
                }

                if ((c == 'o') || (c == 'O')) {
                    saw_o = 1U;
                } else if (saw_o && ((c == 'k') || (c == 'K'))) {
                    uart_debug_puts("\r\nBT HS: linked!\r\n");
                    /* 多发一会儿，对端若还在 linking 能吃到 */
                    for (n = 0U; n < 30U; n++) {
                        bluetooth_puts("ok\r\n");
                        Delay_ms(50U);
                        while (bluetooth_try_getc(&c)) {
                            /* 排空 */
                        }
                    }
                    bluetooth_flush_rx();
                    return;
                } else if ((c != '\r') && (c != '\n')) {
                    saw_o = 0U;
                }
            } else {
                Delay_ms(1U);
                idle_ms++;
            }
        }
    }
}
