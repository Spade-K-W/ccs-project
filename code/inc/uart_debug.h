#ifndef UART_DEBUG_H
#define UART_DEBUG_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct {
    char line_err[24];
    char line_spd[24];
    char line_angle[24];
    char line_mode[24];
    char line_laps[24];
} UartDebugStatus;

void uart_debug_init(void);
void uart_debug_putc(char c);
void uart_debug_puts(const char *s);
void uart_debug_write(const char *data, size_t len);

void uart_debug_print_boot_ok(void);
void uart_debug_print_mpu_init_fail(void);
void uart_debug_print_mpu_calib_fail(void);

void uart_debug_build_status(UartDebugStatus *st, float error, bool lineValid,
                             uint8_t pattern, int16_t leftSpd, int16_t rightSpd,
                             float zAngle, uint32_t laps, const char *mode);
void uart_debug_print_status(const UartDebugStatus *st);

#endif /* UART_DEBUG_H */
