#include "uart_vision.h"
#include "app_config.h"
#include "app_task.h"
#include "board_defs.h"
#include "mpu6050.h"
#include "uart_debug.h"
#include "oled.h"
#include "BUZZER.h"
#include "Delay.h"
#include "ti_msp_dl_config.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

/*
 * 视觉 SPI 协议层（引脚 / TIMG0 / SPI 中断在 board_defs::vision_spi_hw_init）
 *   SCLK PB18 ← 主机时钟
 *   PICO PA14 ← 主机 MOSI（握手时收 "ok"）
 *   POCI PA13 → 主机 MISO（常态状态帧 / 握手 Mode:N）
 *
 * 常态帧：sState:0,Angle:+12.3t
 * 握手帧：Mode:2  （等主机 MOSI 回小写 ok）
 */
#define VISION_FRAME_MAX (64U)

static volatile uint32_t s_txOkCount   = 0U;
static volatile uint32_t s_txFailCount = 0U;

#if UART_VISION_ENABLE
static uint8_t          s_frame[2][VISION_FRAME_MAX];
static volatile uint8_t s_frameLen[2];
static volatile uint8_t s_active;
static volatile uint8_t s_txIdx;

/* 关卡握手：持续发 Mode:N，解析 MOSI 中的小写 "ok" */
static volatile bool    s_handshake;
static volatile bool    s_gotOk;
static volatile uint8_t s_handshakeMode;
static volatile uint8_t s_okMatch; /* 0=等'o'，1=等'k' */
static bool             s_modeMirrored;

static void spi_fill_tx_fifo(void)
{
    uint8_t which = s_active;
    uint8_t len   = s_frameLen[which];

    while (DL_SPI_isTXFIFOFull(SPI_1_INST) == false) {
        if (s_txIdx < len) {
            DL_SPI_transmitData8(SPI_1_INST, s_frame[which][s_txIdx]);
            s_txIdx++;
        } else {
            /* 垫 0x00，避免欠载时 MISO 飘成 0xFF */
            DL_SPI_transmitData8(SPI_1_INST, 0x00U);
        }
    }
}

/* 收 MOSI：握手阶段匹配小写 "ok"，否则丢弃 */
static void spi_process_rx(void)
{
    uint32_t n = 0U;

    while (DL_SPI_isRXFIFOEmpty(SPI_1_INST) == false) {
        uint8_t b = DL_SPI_receiveData8(SPI_1_INST);

        if (s_handshake && !s_gotOk) {
            if (s_okMatch == 0U) {
                if (b == (uint8_t)'o') {
                    s_okMatch = 1U;
                }
            } else if (b == (uint8_t)'k') {
                s_gotOk   = true;
                s_okMatch = 0U;
            } else if (b == (uint8_t)'o') {
                s_okMatch = 1U;
            } else {
                s_okMatch = 0U;
            }
        }

        if (++n >= 16U) {
            break;
        }
    }
}

static void vision_commit_frame(const char *tmp, int n, bool mirrorUart)
{
    uint8_t next;

    if (n <= 0) {
        s_txFailCount++;
        return;
    }
    if (n >= (int)VISION_FRAME_MAX) {
        n = (int)VISION_FRAME_MAX - 1;
    }

    next = (uint8_t)(1U - s_active);
    memcpy(s_frame[next], tmp, (size_t)n);
    s_frame[next][n] = '\0';
    s_frameLen[next] = (uint8_t)n;

    __disable_irq();
    s_active = next;
    s_txIdx  = 0U;
    spi_process_rx();
    spi_fill_tx_fifo();
    __enable_irq();

    s_txOkCount++;

#if SPI_VISION_MIRROR_UART0
    if (mirrorUart) {
        uart_debug_puts(tmp);
        uart_debug_puts("\r\n");
    }
#else
    (void)mirrorUart;
#endif
}

static void vision_build_mode_frame(bool mirrorUart)
{
    char tmp[VISION_FRAME_MAX];
    int  n;

    n = snprintf(tmp, sizeof(tmp), "Mode:%u", (unsigned)s_handshakeMode);
    vision_commit_frame(tmp, n, mirrorUart);
}

static void vision_build_frame(void)
{
    char    tmp[VISION_FRAME_MAX];
    int     n;
    uint8_t state;
    float   angleDeg;

    if (s_handshake) {
        /* 握手中持续重发 Mode:N；串口只镜像一次，避免刷屏 */
        vision_build_mode_frame(!s_modeMirrored);
        s_modeMirrored = true;
        return;
    }

    /* 0=直行循迹，1=原地转弯（含左右转） */
    state    = app_task_line_is_straight() ? 0U : 1U;
    angleDeg = mpu6050_get_z_angle_deg();

    n = snprintf(tmp, sizeof(tmp), "sState:%u,Angle:%+.1ft",
                 (unsigned)state, (double)angleDeg);
    vision_commit_frame(tmp, n, true);
}
#endif /* UART_VISION_ENABLE */

void uart_vision_init(void)
{
    s_txOkCount   = 0U;
    s_txFailCount = 0U;

#if UART_VISION_ENABLE
    s_active         = 0U;
    s_txIdx          = 0U;
    s_frameLen[0]    = 0U;
    s_frameLen[1]    = 0U;
    s_frame[0][0]    = '\0';
    s_frame[1][0]    = '\0';
    s_handshake      = false;
    s_gotOk          = false;
    s_handshakeMode  = 0U;
    s_okMatch        = 0U;
    s_modeMirrored   = false;

    vision_build_frame();
    vision_spi_hw_init();
#endif
}

void uart_vision_send(float angle_deg, float speed_cm_s)
{
    /* 兼容旧接口：忽略参数，按当前状态/角度刷新一帧 */
    (void)angle_deg;
    (void)speed_cm_s;
#if UART_VISION_ENABLE
    vision_build_frame();
#endif
}

void uart_vision_send_boot_test(void)
{
#if UART_VISION_ENABLE
#if SPI_VISION_MIRROR_UART0
    uart_debug_puts("SPI0 slave ready\r\n");
#endif
    vision_build_frame();
#endif
}

void uart_vision_hold_speed(float speed_cm_s)
{
    /* 帧已不含速度，保留空实现以兼容转弯预延时调用 */
    (void)speed_cm_s;
}

void uart_vision_release_speed(void)
{
}

uint32_t uart_vision_get_tx_ok_count(void)
{
    return s_txOkCount;
}

uint32_t uart_vision_get_tx_fail_count(void)
{
    return s_txFailCount;
}

void uart_vision_send_status(void)
{
}

/*
 * 第 2~5 关握手：
 * OLED/串口：project:N + waiting → SPI 发 Mode:N → 等 MOSI 小写 ok
 * → 蜂鸣一声 → OLED/串口显示 ok，然后恢复常态状态帧
 */
void uart_vision_handshake_mode(uint8_t mode)
{
    char line[20];

    if (mode < 2U) {
        mode = 2U;
    }
    if (mode > 5U) {
        mode = 5U;
    }

    oled_clear();
    snprintf(line, sizeof(line), "project:%u       ", (unsigned)mode);
    oled_display_string(0, 0, line);
    oled_display_string(1, 0, "waiting         ");
    oled_display_string(2, 0, "                ");
    oled_display_string(3, 0, "                ");

    uart_debug_puts(line);
    uart_debug_puts("\r\nwaiting\r\n");

#if UART_VISION_ENABLE
    s_gotOk         = false;
    s_okMatch       = 0U;
    s_handshakeMode = mode;
    s_modeMirrored  = false;
    s_handshake     = true;

    vision_build_mode_frame(true);
    s_modeMirrored = true;

    while (!s_gotOk) {
        Delay_ms(10U);
    }

    s_handshake = false;
    vision_build_frame();
#else
    (void)mode;
    /* 无视觉 SPI 时直接放行，便于单板调试 */
#endif

    BUZZER_BeepShort();
    oled_display_string(1, 0, "ok              ");
    uart_debug_puts("ok\r\n");
    Delay_ms(300U);
}

#if UART_VISION_ENABLE
void TIMG0_IRQHandler(void)
{
    switch (DL_TimerG_getPendingInterrupt(TIMG0)) {
        case DL_TIMER_IIDX_ZERO:
            vision_build_frame();
            break;
        default:
            break;
    }
}

void SPI_1_INST_IRQHandler(void)
{
    switch (DL_SPI_getPendingInterrupt(SPI_1_INST)) {
        case DL_SPI_IIDX_RX:
            spi_process_rx();
            spi_fill_tx_fifo();
            break;
        case DL_SPI_IIDX_TX_EMPTY:
            spi_fill_tx_fifo();
            break;
        case DL_SPI_IIDX_IDLE:
            s_txIdx = 0U;
            spi_fill_tx_fifo();
            break;
        default:
            break;
    }
}
#endif
