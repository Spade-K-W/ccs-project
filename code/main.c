#include "ti_msp_dl_config.h"
#include "number.h"
#include "key.h"

/*
#include "USER_BUTTON.h"
#include "Delay.h"
#include "app_config.h"
#include "board_defs.h"
#include "motor.h"
#include "oled.h"
#include "mpu6050.h"
#include "BUZZER.h"
#include "ELE.h"
#include "app_task.h"
#include "line_sensor.h"
#include "encoder.h"
*/

int main(void)
{
    SYSCFG_DL_init();
    number_init();

    /* 按 KEY_1~4 选关：显示关号后卡死（不返回） */
    key_control();

    /*
     * 备用：USER_KEY 按一下数码管 +1（0~9）
     * number_user_key_count_demo();
     */

    /*
    line_sensor_gpio_init();
    board_safe_state();
    ...
    */
}
