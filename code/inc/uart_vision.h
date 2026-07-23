#ifndef UART_VISION_H
#define UART_VISION_H

#include <stddef.h>
#include <stdint.h>

/*
 * 视觉 SPI（SPI_1 = 硬件 SPI0）— TI 从机，泰山派主机，三线无 CS
 *   SCLK = PB18 ← 主机时钟
 *   PICO = PA14 ← 主机 MOSI
 *   POCI = PA13 → 主机 MISO（帧数据）
 *   MOTO3 Mode0，8bit，MSB
 *
 * 帧：sState:0,Angle:+12.3t
 *   State: 0=直行，1=转弯
 *   Angle: 陀螺仪 Z 角（度）
 *
 * 第 2~5 关握手：TI 发 Mode:N，泰山派 MOSI 回小写 ok
 */

void uart_vision_init(void);
void uart_vision_send(float angle_deg, float speed_cm_s);
void uart_vision_send_boot_test(void);
void uart_vision_send_status(void);

void uart_vision_hold_speed(float speed_cm_s);
void uart_vision_release_speed(void);

/*
 * 第 2~5 关进入握手（阻塞）：
 * OLED/串口显示 project:N + waiting，SPI 发 Mode:N，
 * 等到泰山派 MOSI 回小写 "ok" 后蜂鸣一声，显示 ok。
 */
void uart_vision_handshake_mode(uint8_t mode);

uint32_t uart_vision_get_tx_ok_count(void);
uint32_t uart_vision_get_tx_fail_count(void);

#endif /* UART_VISION_H */
