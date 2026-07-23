#include "ti_msp_dl_config.h"
#include "mpu6050.h"
#include "encoder.h"

/*
 * 不要用 getPendingGroup 的 switch 漏清中断，否则会中断风暴卡死主循环。
 * GPIOA/GPIOB 同属 GROUP1，每次进入都处理两边即可。
 */
void GROUP1_IRQHandler(void)
{
    uint32_t statusA;

    statusA = DL_GPIO_getEnabledInterruptStatus(GPIOA, GPIO_MPU6050_PIN_INT_PIN);
    if ((statusA & GPIO_MPU6050_PIN_INT_PIN) != 0U) {
        DL_GPIO_clearInterruptStatus(GPIOA, GPIO_MPU6050_PIN_INT_PIN);
        mpu6050_on_data_ready_isr();
    }

    encoder_on_gpio_isr();
}
