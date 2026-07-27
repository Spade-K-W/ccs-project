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

/* 八路巡线 4051 引脚（与 SysConfig GPIO_GRP_LINE 一致：AD0=PB15 AD1=PB16 AD2=PA17 OUT=PA28） */
extern const GpioPin LINE_AD0;
extern const GpioPin LINE_AD1;
extern const GpioPin LINE_AD2;
extern const GpioPin LINE_OUT;

extern const GpioPin BUZZER;
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
