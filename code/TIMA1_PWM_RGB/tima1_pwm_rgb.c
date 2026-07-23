#include "tima1_pwm_rgb.h"
#include "ti_msp_dl_config.h"

#ifndef DL_TIMER_IIDX_LOAD
#ifdef DL_TIMERA_IIDX_LOAD
#define DL_TIMER_IIDX_LOAD DL_TIMERA_IIDX_LOAD
#endif
#endif

#ifndef DL_TIMER_CC_0_INDEX
#define DL_TIMER_CC_0_INDEX GPIO_RGB_PWM_C0_IDX
#endif

// 80MHz, Period=100, CCR=99 → 输出几乎 0% 占空比 (全低电平, 用于复位)
#define RGB_IDLE_VAL (99U)

static volatile const uint16_t *g_tima1_pwm_rgb_compare_buf = 0;
static volatile uint32_t g_tima1_pwm_rgb_compare_count = 0;
static volatile uint32_t g_tima1_pwm_rgb_compare_index = 1;
static volatile bool g_tima1_pwm_rgb_busy = false;

void tima1_pwm_rgb_init(void)
{
    g_tima1_pwm_rgb_busy = false;

    DL_TimerA_stopCounter(RGB_PWM_INST);
    DL_TimerA_setCaptureCompareValue(RGB_PWM_INST, RGB_IDLE_VAL, DL_TIMER_CC_0_INDEX);
    DL_TimerA_disableInterrupt(RGB_PWM_INST, DL_TIMER_INTERRUPT_LOAD_EVENT);
    NVIC_ClearPendingIRQ(RGB_PWM_INST_INT_IRQN);
}

bool tima1_pwm_rgb_start_frame(const uint16_t *compare_buf, uint32_t compare_count)
{
    if ((compare_buf == 0) || (compare_count == 0U)) {
        return false;
    }

    __disable_irq();

    if (g_tima1_pwm_rgb_busy) {
        __enable_irq();
        return false;
    }

    g_tima1_pwm_rgb_compare_buf = compare_buf;
    g_tima1_pwm_rgb_compare_count = compare_count;
    g_tima1_pwm_rgb_compare_index = 1U;
    g_tima1_pwm_rgb_busy = true;

    DL_TimerA_stopCounter(RGB_PWM_INST);
    DL_TimerA_setCaptureCompareValue(RGB_PWM_INST, g_tima1_pwm_rgb_compare_buf[0], DL_TIMER_CC_0_INDEX);

    NVIC_ClearPendingIRQ(RGB_PWM_INST_INT_IRQN);
    DL_TimerA_enableInterrupt(RGB_PWM_INST, DL_TIMER_INTERRUPT_LOAD_EVENT);
    NVIC_EnableIRQ(RGB_PWM_INST_INT_IRQN);

    DL_TimerA_startCounter(RGB_PWM_INST);

    __enable_irq();

    return true;
}

bool tima1_pwm_rgb_is_busy(void)
{
    return g_tima1_pwm_rgb_busy;
}

void tima1_pwm_rgb_wait_done(void)
{
    while (g_tima1_pwm_rgb_busy) {
    }
}

void RGB_PWM_INST_IRQHandler(void)
{
    switch (DL_TimerA_getPendingInterrupt(RGB_PWM_INST)) {
        case DL_TIMER_IIDX_LOAD:
            if (g_tima1_pwm_rgb_busy) {
                if (g_tima1_pwm_rgb_compare_index < g_tima1_pwm_rgb_compare_count) {
                    DL_TimerA_setCaptureCompareValue(
                        RGB_PWM_INST,
                        g_tima1_pwm_rgb_compare_buf[g_tima1_pwm_rgb_compare_index],
                        DL_TIMER_CC_0_INDEX);
                    g_tima1_pwm_rgb_compare_index++;
                } else {
                    DL_TimerA_setCaptureCompareValue(RGB_PWM_INST, RGB_IDLE_VAL, DL_TIMER_CC_0_INDEX);
                    DL_TimerA_stopCounter(RGB_PWM_INST);
                    DL_TimerA_disableInterrupt(RGB_PWM_INST, DL_TIMER_INTERRUPT_LOAD_EVENT);
                    g_tima1_pwm_rgb_busy = false;
                }
            } else {
                DL_TimerA_setCaptureCompareValue(RGB_PWM_INST, RGB_IDLE_VAL, DL_TIMER_CC_0_INDEX);
                DL_TimerA_stopCounter(RGB_PWM_INST);
                DL_TimerA_disableInterrupt(RGB_PWM_INST, DL_TIMER_INTERRUPT_LOAD_EVENT);
            }
            break;
        default:
            break;
    }
}