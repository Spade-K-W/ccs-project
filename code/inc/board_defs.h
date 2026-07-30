#ifndef BOARD_DEFS_H
#define BOARD_DEFS_H

#include "ti_msp_dl_config.h"
#include "app_config.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    GPIO_Regs *port;
    uint32_t pin;
} GpioPin;

typedef struct {
    GpioPin pwm;
    GpioPin in1;
    GpioPin in2;
    bool reversed;
    volatile uint8_t duty;
    volatile int8_t dir;
} Motor;

/* 亚博 YB-MUX04-1.0 八路数字输出 */
extern const GpioPin LINE_X1;
extern const GpioPin LINE_X2;
extern const GpioPin LINE_X3;
extern const GpioPin LINE_X4;
extern const GpioPin LINE_X5;
extern const GpioPin LINE_X6;
extern const GpioPin LINE_X7;
extern const GpioPin LINE_X8;

extern const GpioPin ELECTROMAGNET;  /* PA24，高电平吸合 */
extern const GpioPin MOTOR_STBY;
extern const GpioPin USER_KEY;

extern Motor motor1;
extern Motor motor2;
extern Motor motor3;
extern Motor motor4;
extern Motor *motors[4];

void line_sensor_gpio_init(void);

/* 视觉 SPI：TIMG0 50ms 刷新 + SPI0 从机中断/FIFO（引脚由 SysConfig 完成） */
void vision_spi_hw_init(void);

void pin_high(const GpioPin *p);
void pin_low(const GpioPin *p);
bool pin_read_raw(const GpioPin *p);

void board_led_set(bool on);
void board_safe_state(void);

#endif
