#include "board_defs.h"
#include "app_config.h"
#include "rgb_sk6812.h"

/* 亚博 YB-MUX04-1.0：X1~X8 全部配置为 MCU 输入。 */
const GpioPin LINE_X1 = {GPIO_GRP_LINE_X1_PORT, GPIO_GRP_LINE_X1_PIN};
const GpioPin LINE_X2 = {GPIO_GRP_LINE_X2_PORT, GPIO_GRP_LINE_X2_PIN};
const GpioPin LINE_X3 = {GPIO_GRP_LINE_X3_PORT, GPIO_GRP_LINE_X3_PIN};
const GpioPin LINE_X4 = {GPIO_GRP_LINE_X4_PORT, GPIO_GRP_LINE_X4_PIN};
const GpioPin LINE_X5 = {GPIO_GRP_LINE_X5_PORT, GPIO_GRP_LINE_X5_PIN};
const GpioPin LINE_X6 = {GPIO_GRP_LINE_X6_PORT, GPIO_GRP_LINE_X6_PIN};
const GpioPin LINE_X7 = {GPIO_GRP_LINE_X7_PORT, GPIO_GRP_LINE_X7_PIN};
const GpioPin LINE_X8 = {GPIO_GRP_LINE_X8_PORT, GPIO_GRP_LINE_X8_PIN};

void line_sensor_gpio_init(void)
{
    /* 八路输入已由 SYSCFG_DL_init() 根据 GPIO_GRP_LINE 完成初始化。 */
}

/*
 * 视觉模块硬件补充初始化（引脚复用已在 SYSCFG_DL_SPI_1_init 完成）：
 * - SPI0 从机中断 + FIFO 阈值
 * - TIMG0 周期定时，按 UART_VISION_SEND_INTERVAL_MS 刷新发送缓冲
 */
#if UART_VISION_ENABLE
#define VISION_TIM_PRESCALE (249U)
#define VISION_TIM_CLK_HZ   (40000U)
#define VISION_TIM_LOAD_VALUE \
    (((VISION_TIM_CLK_HZ * (uint32_t)UART_VISION_SEND_INTERVAL_MS) / 1000U) - 1U)
#endif

void vision_spi_hw_init(void)
{
#if UART_VISION_ENABLE
    static const DL_TimerG_ClockConfig clockCfg = {
        .clockSel    = DL_TIMER_CLOCK_BUSCLK,
        .divideRatio = DL_TIMER_CLOCK_DIVIDE_8,
        .prescale    = VISION_TIM_PRESCALE,
    };
    static const DL_TimerG_TimerConfig timerCfg = {
        .period     = VISION_TIM_LOAD_VALUE,
        .timerMode  = DL_TIMER_TIMER_MODE_PERIODIC,
        .startTimer = DL_TIMER_STOP,
    };

    NVIC_ClearPendingIRQ(SPI_1_INST_INT_IRQN);
    NVIC_EnableIRQ(SPI_1_INST_INT_IRQN);
    DL_SPI_enableInterrupt(SPI_1_INST,
                           DL_SPI_INTERRUPT_RX | DL_SPI_INTERRUPT_IDLE |
                               DL_SPI_INTERRUPT_TX_EMPTY);
    DL_SPI_setFIFOThreshold(SPI_1_INST, DL_SPI_RX_FIFO_LEVEL_ONE_FRAME,
                            DL_SPI_TX_FIFO_LEVEL_ONE_FRAME);

    DL_TimerG_enablePower(TIMG0);
    delay_cycles(POWER_STARTUP_DELAY);
    DL_TimerG_setClockConfig(TIMG0, (DL_TimerG_ClockConfig *)&clockCfg);
    DL_TimerG_initTimerMode(TIMG0, (DL_TimerG_TimerConfig *)&timerCfg);
    DL_TimerG_enableInterrupt(TIMG0, DL_TIMER_INTERRUPT_ZERO_EVENT);
    DL_TimerG_enableClock(TIMG0);

    NVIC_ClearPendingIRQ(TIMG0_INT_IRQn);
    NVIC_EnableIRQ(TIMG0_INT_IRQn);
    DL_TimerG_startCounter(TIMG0);
#endif
}

const GpioPin ELECTROMAGNET = {Electromagnet_PORT, Electromagnet_PIN_0_PIN};
const GpioPin MOTOR_STBY = {GPIO_GRP_MOTOR_L_MOTOR_STBY_PORT, GPIO_GRP_MOTOR_L_MOTOR_STBY_PIN};
const GpioPin USER_KEY = {GPIO_BUTTON_PORT, GPIO_BUTTON_USER_KEY_PIN};

Motor motor1 = {
    {GPIO_GRP_MOTOR_L_M1_PWM_PORT, GPIO_GRP_MOTOR_L_M1_PWM_PIN},
    {GPIO_GRP_MOTOR_L_M1_IN1_PORT, GPIO_GRP_MOTOR_L_M1_IN1_PIN},
    {GPIO_GRP_MOTOR_L_M1_IN2_PORT, GPIO_GRP_MOTOR_L_M1_IN2_PIN},
    M1_REVERSED, 0, 0
};

Motor motor2 = {
    {GPIO_GRP_MOTOR_L_M2_PWM_PORT, GPIO_GRP_MOTOR_L_M2_PWM_PIN},
    {GPIO_GRP_MOTOR_L_M2_IN1_PORT, GPIO_GRP_MOTOR_L_M2_IN1_PIN},
    {GPIO_GRP_MOTOR_L_M2_IN2_PORT, GPIO_GRP_MOTOR_L_M2_IN2_PIN},
    M2_REVERSED, 0, 0
};

Motor motor3 = {
    {GPIO_GRP_MOTOR_R_M3_PWM_PORT, GPIO_GRP_MOTOR_R_M3_PWM_PIN},
    {GPIO_GRP_MOTOR_R_M3_IN1_PORT, GPIO_GRP_MOTOR_R_M3_IN1_PIN},
    {GPIO_GRP_MOTOR_R_M3_IN2_PORT, GPIO_GRP_MOTOR_R_M3_IN2_PIN},
    M3_REVERSED, 0, 0
};

Motor motor4 = {
    {GPIO_GRP_MOTOR_R_M4_PWM_PORT, GPIO_GRP_MOTOR_R_M4_PWM_PIN},
    {GPIO_GRP_MOTOR_R_M4_IN1_PORT, GPIO_GRP_MOTOR_R_M4_IN1_PIN},
    {GPIO_GRP_MOTOR_R_M4_IN2_PORT, GPIO_GRP_MOTOR_R_M4_IN2_PIN},
    M4_REVERSED, 0, 0
};

Motor *motors[4] = {&motor1, &motor2, &motor3, &motor4};

void pin_high(const GpioPin *p)
{
    DL_GPIO_setPins(p->port, p->pin);
}

void pin_low(const GpioPin *p)
{
    DL_GPIO_clearPins(p->port, p->pin);
}

bool pin_read_raw(const GpioPin *p)
{
    return (DL_GPIO_readPins(p->port, p->pin) != 0U);
}

void board_led_set(bool on)
{
    rgb_sk6812_set_on(on);
}

void board_safe_state(void)
{
    uint8_t i;

    board_led_set(false);

    pin_low(&ELECTROMAGNET);  /* 安全态：电磁铁释放 */

    pin_low(&MOTOR_STBY);

    for (i = 0; i < 4; i++) {
        pin_low(&motors[i]->pwm);
        pin_low(&motors[i]->in1);
        pin_low(&motors[i]->in2);
        motors[i]->dir = 0;
        motors[i]->duty = 0U;
    }
}
