#include "ti_msp_dl_config.h"
#include "board_defs.h"
#include "motor.h"
#include "oled.h"
#include "mpu6050.h"
#include "BUZZER.h"
#include "ELE.h"
#include "app_config.h"
#include "app_task.h"
#include "line_sensor.h"
#include "encoder.h"
#include "Delay.h"
#include "uart_debug.h"
/* #include "bluetooth.h" */

#include <stdint.h>
#include <stdbool.h>

int main(void)
{
    SYSCFG_DL_init();
    line_sensor_gpio_init();
    board_safe_state();

    oled_init();
    oled_clear();

    motor_init();
    chassis_stop();
    encoder_init();
    uart_debug_init();
    /* bluetooth_init(); */

    /* 1. 上电先校准 MPU（车保持静止） */
    mpu6050_startup();

    /* 2. 校准完毕提示（电磁铁保持释放，不拉高） */
    ELE_Off();
    oled_display_string(0, 0, "MPU Ready       ");
    oled_display_string(1, 0, "                ");
    BUZZER_BeepShort();
    uart_debug_print_boot_ok();

    /*
     * 3. Straight ↔ Arc 状态机：
     *    Straight：不循线，陀螺仪锁航向直行
     *    连续见线 → Arc 弧线循迹
     *    累计偏航 ≥ ARC_TARGET_DEG 且连续丢线 → 回 Straight
     *    （未转满角度时丢线会继续强转，不会提前直行）
     */
    app_task_demo_prepare(+1.0f); /* +1 左圆弧；贴反了改 -1.0f */

    while (1) {
        app_task_demo_step();
        /* #if BLUETOOTH_ENABLE
         * bluetooth_send_oled_now();
         * #endif
         */
        Delay_ms(LOOP_PERIOD_MS);
    }
}
