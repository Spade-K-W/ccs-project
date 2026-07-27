#ifndef ELE_H
#define ELE_H

#include <stdbool.h>
#include <stdint.h>

/* 电磁铁开/关（高电平吸合，低电平释放） */
void ELE_Set(bool on);

/* 吸合 */
void ELE_On(void);

/* 释放 */
void ELE_Off(void);

/* 查询当前是否吸合 */
bool ELE_IsOn(void);

#endif /* ELE_H */
