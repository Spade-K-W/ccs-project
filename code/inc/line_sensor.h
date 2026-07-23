#ifndef LINE_SENSOR_H
#define LINE_SENSOR_H

#include <stdbool.h>
#include <stdint.h>

/* 八路巡线模块通道数 */
#define LINE_SENSOR_CHANNEL_COUNT   (8U)

/* 读取 8 路红外当前状态，并打包成 1 个 8 位模式值
 * bit0 -> 最左侧探头 (通道 0)
 * bit7 -> 最右侧探头 (通道 7)
 *
 * 硬件说明（74HC4051 多路复用）：
 *   AD0 / AD1 / AD2 -> 通道选择地址 (S0 / S1 / S2)
 *   OUT             -> 公共数字输出 (Z)
 */
uint8_t line_read_pattern(void);

/* 根据 8 路红外模式计算循迹误差（整数） */
bool line_calc_error(uint8_t pattern, int16_t *error);

/* 根据 8 路红外模式计算循迹误差（浮点，OLED 显示用） */
bool line_calc_error_f(uint8_t pattern, float *error);

/* 判断当前红外模式是否像顶点/路口特征 */
bool is_vertex_like_pattern(uint8_t pattern);

/* 通道 1/2/3（bit0~2，偏左）是否同时压线 */
bool line_ch123_all_on(uint8_t pattern);

/* 通道 6/7/8（bit5~7，偏右）是否同时压线 */
bool line_ch678_all_on(uint8_t pattern);

/* 八路全部压线（大黑区 / 十字中心） */
bool line_all_on(uint8_t pattern);

#endif
