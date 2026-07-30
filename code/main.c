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
#include "key.h"
#include "task.h"
#include "number.h"

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief 主程序入口
 * 
 * 初始化流程：
 * 1. 系统配置 & 硬件初始化
 * 2. MPU6050 陀螺仪校准（约1秒）
 * 3. 视觉/蓝牙握手（可选）
 * 4. 循迹任务初始化
 * 5. 主循环
 */
int main(void)
{
    /* ========== 1. 系统与硬件初始化 ========== */
    SYSCFG_DL_init();           /* TI MCU系统初始化 */
    line_sensor_gpio_init();    /* 八路红外 GPIO 输入初始化 */
    board_safe_state();         /* 板级安全状态设置 */

    oled_init();                /* OLED初始化 */
    oled_clear();
    number_init();              /* 数字显示模块初始化 */

    motor_init();               /* 电机初始化 */
    chassis_stop();             /* 停止小车 */
    encoder_init();             /* 编码器初始化 */
    uart_debug_init();          /* 调试串口初始化 */
    uart_vision_init();         /* 视觉/SPI串口初始化 */

    /* ========== 2. MPU6050 陀螺仪校准 ========== */
    oled_display_string(0, 0, "MPU Calibrating ");
    oled_display_string(1, 0, "Keep still...   ");
    
    mpu6050_startup();          /* MPU校准（保持静止约1秒） */

    ELE_Off();
    oled_display_string(0, 0, "MPU Ready       ");
    uart_debug_puts("MPU ready\r\n");

    /* ========== 3. 视觉/蓝牙握手（可选） ========== */
    /*
     * 蓝牙握手（暂时注释）：
     * oled_display_string(1, 0, "BT linking...   ");
     * uart_debug_puts("BT linking...\r\n");
     * #if BLUETOOTH_ENABLE
     * bluetooth_wait_peer();
     * #endif
     * oled_display_string(1, 0, "BT linked       ");
     */

    /* 视觉 SPI 握手 */
#if UART_VISION_ENABLE
    oled_display_string(1, 0, "SPI vision ready");
    uart_debug_puts("SPI vision ready, skip blocking handshake\r\n");
#else
    oled_display_string(1, 0, "No SPI vision   ");
    BUZZER_BeepShort();
#endif

    uart_debug_print_boot_ok();

    /* ========== 4. 循迹任务初始化 ========== */
    key_init();                 /* 按键初始化 */
    task_init();                /* 循迹任务状态机初始化 */

    /* ========== 5. 主循环 ========== */
    /*
     * 循环周期：LOOP_PERIOD_MS（通常 20ms，对应 50Hz）
     * 
     * 循环内容：
     * - task_step()        : 循迹任务状态机更新
     * - 视觉/蓝牙数据处理 : 如果启用
     */
    while (1) {
        /* 循迹任务主函数 */
        task_step();

        /* 视觉数据处理 */
#if UART_VISION_ENABLE
        uart_vision_flush_rx_to_uart();
#endif

        /* 蓝牙数据处理 */
#if BLUETOOTH_ENABLE
        bluetooth_send_oled_now();
#endif

        /* 保持恒定周期 */
        Delay_ms(LOOP_PERIOD_MS);
    }

    return 0;
}
