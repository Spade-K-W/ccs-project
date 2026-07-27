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

#include <stdint.h>
#include <stdbool.h>

int main(void)
{
    uint8_t pattern;
    float   error;
    bool    lineValid;

    SYSCFG_DL_init();
    line_sensor_gpio_init();
    board_safe_state();

    oled_init();
    oled_clear();

    /* 电机初始化并停车，校准期间保持静止 */
    motor_init();
    chassis_stop();
    encoder_init();

    /* 1. 上电先校准 MPU */
    mpu6050_startup();

    /* 2. 校准完毕：电磁铁吸合 + 蜂鸣器响一声 */
    ELE_On();
    oled_display_string(0, 0, "MPU Ready       ");
    oled_display_string(1, 0, "                ");
    BUZZER_BeepShort();

    /* 3. 进入弧线循迹（+1 正向；若贴线方向反了改为 -1.0f） */
    app_task_arc_prepare(-1.0f);
    oled_display_string(0, 0, "Arc Follow      ");

    while (1) {
        pattern   = line_read_pattern();
        lineValid = line_calc_error_f(pattern, &error);
        line_follow_arc(pattern, error, lineValid);

        Delay_ms(LOOP_PERIOD_MS);
    }
}
