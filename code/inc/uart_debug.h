#ifndef UART_DEBUG_H
#define UART_DEBUG_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct {
    char line_state[24];
    char line_pat[24];
    char line_cmd[24];
    char line_mea[24];
    char line_corr[24];
    char line_angle[24];
} UartDebugStatus;

void uart_debug_init(void);
void uart_debug_putc(char c);
void uart_debug_puts(const char *s);
void uart_debug_write(const char *data, size_t len);

void uart_debug_print_boot_ok(void);
void uart_debug_print_mpu_init_fail(void);
void uart_debug_print_mpu_calib_fail(void);

void uart_debug_build_status6(UartDebugStatus *st,
                              const char *state,
                              uint8_t pattern,
                              int16_t cmdL, int16_t cmdR,
                              float meaL, float meaR,
                              int16_t corrL, int16_t corrR,
                              float zAngle);
void uart_debug_print_status(const UartDebugStatus *st);

#endif /* UART_DEBUG_H */
