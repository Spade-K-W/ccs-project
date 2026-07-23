#include "board_defs.h"
#include "app_config.h"
#include "rgb_sk6812.h"

/*
 * 循迹多路选择：AD0->PB15, AD1->PB16, AD2->PA17, OUT->PA28
 * （与 SysConfig GPIO_GRP_LINE 一致；PA13/PA14 留给视觉 SPI0）
 */
const GpioPin LINE_AD0 = {GPIO_GRP_LINE_AD0_PORT, GPIO_GRP_LINE_AD0_PIN};
const GpioPin LINE_AD1 = {GPIO_GRP_LINE_AD1_PORT, GPIO_GRP_LINE_AD1_PIN};
const GpioPin LINE_AD2 = {GPIO_GRP_LINE_AD2_PORT, GPIO_GRP_LINE_AD2_PIN};
const GpioPin LINE_OUT = {GPIO_GRP_LINE_OUT_PORT, GPIO_GRP_LINE_OUT_PIN};

void line_sensor_gpio_init(void)
{
    DL_GPIO_initDigitalOutput(GPIO_GRP_LINE_AD0_IOMUX);
    DL_GPIO_initDigitalOutput(GPIO_GRP_LINE_AD1_IOMUX);
    DL_GPIO_initDigitalOutput(GPIO_GRP_LINE_AD2_IOMUX);
    DL_GPIO_initDigitalInputFeatures(GPIO_GRP_LINE_OUT_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_clearPins(GPIOB, GPIO_GRP_LINE_AD0_PIN | GPIO_GRP_LINE_AD1_PIN);
    DL_GPIO_enableOutput(GPIOB, GPIO_GRP_LINE_AD0_PIN | GPIO_GRP_LINE_AD1_PIN);
    DL_GPIO_clearPins(GPIOA, GPIO_GRP_LINE_AD2_PIN);
    DL_GPIO_enableOutput(GPIOA, GPIO_GRP_LINE_AD2_PIN);
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

const GpioPin BUZZER = {GPIO_GRP_BUZZER_PORT, GPIO_GRP_BUZZER_BUZZER_PIN};
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
#if BUZZER_ACTIVE_HIGH
    pin_low(&BUZZER);
#else
    pin_high(&BUZZER);
#endif

    pin_low(&MOTOR_STBY);

    for (i = 0; i < 4; i++) {
        pin_low(&motors[i]->pwm);
        pin_low(&motors[i]->in1);
        pin_low(&motors[i]->in2);
        motors[i]->dir = 0;
        motors[i]->duty = 0U;
    }
}
