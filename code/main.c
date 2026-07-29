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
#include "uart_vision.h"
#include "bluetooth.h"

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
    bluetooth_init();
    uart_vision_init();

    /* 1. MPU 校准 */
    oled_display_string(0, 0, "MPU Calibrating ");
    oled_display_string(1, 0, "Keep still...   ");
    mpu6050_startup();

    ELE_Off();
    oled_display_string(0, 0, "MPU Ready       ");
    uart_debug_puts("MPU ready\r\n");

    /*
     * 2. 蓝牙握手（暂时注释，改走视觉 SPI Mode:N 握手）
     *
     * oled_display_string(1, 0, "BT linking...   ");
     * uart_debug_puts("BT linking...\r\n");
     * #if BLUETOOTH_ENABLE
     * bluetooth_wait_peer();
     * #endif
     * oled_display_string(1, 0, "BT linked       ");
     * uart_debug_puts("BT linked\r\n");
     */

    /* 3. 视觉 SPI 握手：TI 发 Mode:N，等泰山派 MOSI 回 ok，成功后再巡线 */
#if UART_VISION_ENABLE
    oled_display_string(1, 0, "SPI handshake   ");
    uart_debug_puts("SPI vision handshake...\r\n");
    uart_vision_handshake_mode((uint8_t)VISION_HANDSHAKE_MODE);
    uart_debug_puts("SPI handshake OK, start line follow\r\n");
#else
    oled_display_string(1, 0, "No SPI vision   ");
    BUZZER_BeepShort();
#endif

    uart_debug_print_boot_ok();

    /* 4. 握手通过 → 循迹 */
    app_task_demo_prepare(+1.0f);

    while (1) {
        app_task_demo_step();
#if UART_VISION_ENABLE
        uart_vision_flush_rx_to_uart();
#endif
#if BLUETOOTH_ENABLE
        bluetooth_send_oled_now();
#endif
        Delay_ms(LOOP_PERIOD_MS);
    }
}
