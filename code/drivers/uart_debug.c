#include "uart_debug.h"
#include "app_config.h"
#include "ti_msp_dl_config.h"

#include <stdio.h>

static uint32_t s_uartPrintElapsedMs = UART_DEBUG_PRINT_INTERVAL_MS;

void uart_debug_init(void)
{
    /* UART 引脚与时钟由 SysConfig 在 SYSCFG_DL_init() 中完成初始化 */
    s_uartPrintElapsedMs = UART_DEBUG_PRINT_INTERVAL_MS;
}

void uart_debug_putc(char c)
{
    uint32_t guard = 50000U;

    while (DL_UART_Main_isTXFIFOFull(UART_0_INST) != false) {
        if (--guard == 0U) {
            return;
        }
    }
    DL_UART_Main_transmitData(UART_0_INST, (uint8_t)c);
}

void uart_debug_puts(const char *s)
{
    while ((s != NULL) && (*s != '\0')) {
        uart_debug_putc(*s++);
    }
}

void uart_debug_write(const char *data, size_t len)
{
    size_t i;

    if (data == NULL) {
        return;
    }

    for (i = 0U; i < len; i++) {
        uart_debug_putc(data[i]);
    }
}

void uart_debug_print_boot_ok(void)
{
    uart_debug_puts("MPU6050 ready\r\n");
    uart_debug_puts("UART debug ready\r\n");
}

void uart_debug_print_mpu_init_fail(void)
{
    uart_debug_puts("MPU6050 init failed\r\n");
}

void uart_debug_print_mpu_calib_fail(void)
{
    uart_debug_puts("MPU6050 calibrate failed\r\n");
}

static void pattern_to_bits(char *buf, size_t n, uint8_t pattern)
{
    uint8_t i;

    if ((buf == NULL) || (n < 9U)) {
        return;
    }
    /* CH1..CH8 = bit0..bit7，显示成 00011000 */
    for (i = 0U; i < 8U; i++) {
        buf[i] = ((pattern & (1U << i)) != 0U) ? '1' : '0';
    }
    buf[8] = '\0';
}

void uart_debug_build_status6(UartDebugStatus *st,
                              const char *state,
                              uint8_t pattern,
                              int16_t cmdL, int16_t cmdR,
                              float meaL, float meaR,
                              int16_t corrL, int16_t corrR,
                              float zAngle)
{
    char bits[9];

    if (st == NULL) {
        return;
    }

    pattern_to_bits(bits, sizeof(bits), pattern);

    if (state != NULL) {
        snprintf(st->line_state, sizeof(st->line_state), "%-16s", state);
    } else {
        snprintf(st->line_state, sizeof(st->line_state), "Straight        ");
    }

    snprintf(st->line_pat, sizeof(st->line_pat), "%s        ", bits);
    snprintf(st->line_cmd, sizeof(st->line_cmd),
             "Cmd L:%3d R:%3d", (int)cmdL, (int)cmdR);
    snprintf(st->line_mea, sizeof(st->line_mea),
             "Mea L:%4.1f R:%4.1f", meaL, meaR);
    snprintf(st->line_corr, sizeof(st->line_corr),
             "CorrL:%+3d R:%+3d", (int)corrL, (int)corrR);
    snprintf(st->line_angle, sizeof(st->line_angle),
             "Z:%+7.1f deg   ", zAngle);
}

static void uart_debug_print_status_now(const UartDebugStatus *st)
{
    uart_debug_puts(st->line_state);
    uart_debug_puts("\r\n");
    uart_debug_puts(st->line_pat);
    uart_debug_puts("\r\n");
    uart_debug_puts(st->line_cmd);
    uart_debug_puts("\r\n");
    uart_debug_puts(st->line_mea);
    uart_debug_puts("\r\n");
    uart_debug_puts(st->line_corr);
    uart_debug_puts("\r\n");
    uart_debug_puts(st->line_angle);
    uart_debug_puts("\r\n");
}

void uart_debug_print_status(const UartDebugStatus *st)
{
    if (st == NULL) {
        return;
    }

    s_uartPrintElapsedMs += (uint32_t)LOOP_PERIOD_MS;
    if (s_uartPrintElapsedMs < (uint32_t)UART_DEBUG_PRINT_INTERVAL_MS) {
        return;
    }

    s_uartPrintElapsedMs = 0U;
    uart_debug_print_status_now(st);
}
