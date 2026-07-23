#include "ti_msp_dl_config.h"
#include "board_defs.h"
#include "motor.h"
#include "encoder.h"
#include "oled.h"
#include "line_sensor.h"
#include "uart_debug.h"
#include "uart_vision.h"
#include "mpu6050.h"
#include "BUZZER.h"
#include "Delay.h"
#include "choosing.h"
#include "chapter.h"

int main(void)
{
    uint8_t project;

    SYSCFG_DL_init();
    /* 高电平发声：上电立刻拉低 PB13 静音 */
    BUZZER_Mute();
    uart_debug_init();
    uart_vision_init();
    uart_vision_send_boot_test();  /* 上电立刻测 SPI0：PB18/PA14/PA13 */
    line_sensor_gpio_init();
    board_safe_state();
    BUZZER_Mute();

    oled_init();
    oled_clear();
    mpu6050_startup();

    motor_init();
    encoder_init();

    /* 初始化完成提示一声，再进入关卡选择 */
    BUZZER_BeepShort();

    project = choosing_select_project();

    switch (project) {
        case 1:
            chapter1_run();
            break;
        case 2:
            chapter2_run();
            break;
        case 3:
            chapter3_run();
            break;
        case 4:
            chapter4_run();
            break;
        case 5:
            chapter5_run();
            break;
        default:
            chapter1_run();
            break;
    }

    /* 各关卡函数内部为死循环，正常不会返回 */
    while (1) {
        Delay_ms(500U);
    }
}
