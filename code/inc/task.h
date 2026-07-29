#ifndef TASK_H
#define TASK_H

/**
 * @file task.h
 * @brief 循迹任务状态机接口
 * 
 * 功能：
 * - KEY1：红外+陀螺仪，一整圈循迹（整圈约20秒）
 * - KEY2：红外+编码器，A→B直线段（约8秒内）
 * - KEY3：红外+陀螺仪，慢速一整圈（约40秒）
 * 
 * 设计原则：
 * - 红外传感器作为主决策 (检测转弯、停止线)
 * - 编码器作为保护机制 (防止红外失效)
 * - 陀螺仪精确控制转弯角度
 */

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief 初始化循迹任务状态机
 * 
 * 说明：
 * - 关闭电机
 * - 复位所有状态变量
 * - 显示任务选择菜单到OLED
 * - 需在main中MPU校准之后调用
 * 
 * @note 无参数，无返回值
 */
void task_init(void);

/**
 * @brief 循迹任务主循环（每拍调用）
 * 
 * 流程：
 * 1. 扫描按键，根据按键启动相应任务
 * 2. 执行当前路线阶段（直线或弧线）
 * 3. 更新OLED显示信息
 * 4. 检查超时/完成条件
 * 
 * OLED显示格式：
 *   第6行：任务名称 / 路段(AB/BC/CD/DA) 陀螺仪角度 距离
 *   第7行：经过时间 T:8.234s  红外模式 P:07
 * 
 * 红外模式 P值参考：
 *   00 = 完全丢线
 *   01 = CH1亮（单独）
 *   07 = CH1/2/3亮（**右转特征！**）
 *   E0 = CH6/7/8亮（左转特征）
 *   FF = 所有都亮（十字路口）
 * 
 * @note 应该在 main() 的 while(1) 循环中以恒定周期（如50Hz）调用
 *       调用周期应为 LOOP_PERIOD_MS（通常 20ms）
 * 
 * @example
 * while(1) {
 *     task_step();     // 每 20ms 调用一次
 *     Delay_ms(LOOP_PERIOD_MS);
 * }
 */
void task_step(void);

#endif /* TASK_H */
