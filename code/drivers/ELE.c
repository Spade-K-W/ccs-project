#include "ELE.h"
#include "board_defs.h"

/* =========================================================
 * 电磁铁驱动
 * 硬件：PA24（SysConfig: Electromagnet / PIN_0）
 * 逻辑：拉高吸合，拉低释放
 * ========================================================= */

static bool s_ele_on = false;

void ELE_Set(bool on)
{
    if (on) {
        pin_high(&ELECTROMAGNET);
        s_ele_on = true;
    } else {
        pin_low(&ELECTROMAGNET);
        s_ele_on = false;
    }
}

void ELE_On(void)
{
    ELE_Set(true);
}

void ELE_Off(void)
{
    ELE_Set(false);
}

bool ELE_IsOn(void)
{
    return s_ele_on;
}
