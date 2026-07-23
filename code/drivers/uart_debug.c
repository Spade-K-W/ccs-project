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

    /* 非阻塞：TX FIFO 满超时就丢弃，避免调试口卡住整车 */
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

void uart_debug_build_status(UartDebugStatus *st, float error, bool lineValid,
                             uint8_t pattern, int16_t leftSpd, int16_t rightSpd,
                             float zAngle, uint32_t laps, const char *mode)
{
    if (st == NULL) {
        return;
    }

    if (lineValid) {
        snprintf(st->line_err, sizeof(st->line_err),
                 "err:%+.1f pat:%02X", error, (unsigned)pattern);
    } else {
        snprintf(st->line_err, sizeof(st->line_err),
                 "LOST  pat:%02X", (unsigned)pattern);
    }

    snprintf(st->line_spd, sizeof(st->line_spd),
             "L:%4d R:%4d    ", (int)leftSpd, (int)rightSpd);
    snprintf(st->line_angle, sizeof(st->line_angle),
             "ZAngle:%+6.1f deg ", zAngle);
    snprintf(st->line_laps, sizeof(st->line_laps),
             "Laps:%lu         ", (unsigned long)laps);

    if (mode != NULL) {
        snprintf(st->line_mode, sizeof(st->line_mode), "%-16s", mode);
    } else {
        snprintf(st->line_mode, sizeof(st->line_mode), "go straight     ");
    }
}

static void uart_debug_print_status_now(const UartDebugStatus *st)
{
    /* 与 OLED 对齐：1标题 2误差 3轮速 4状态 5 Z角 6圈数 */
    uart_debug_puts("Line Follow");
    uart_debug_puts("\r\n");
    uart_debug_puts(st->line_err);
    uart_debug_puts("\r\n");
    uart_debug_puts(st->line_spd);
    uart_debug_puts("\r\n");
    uart_debug_puts(st->line_mode);
    uart_debug_puts("\r\n");
    uart_debug_puts(st->line_angle);
    uart_debug_puts("\r\n");
    uart_debug_puts(st->line_laps);
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
