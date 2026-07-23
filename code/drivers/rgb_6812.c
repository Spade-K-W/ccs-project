#include "rgb_sk6812.h"
#include "tima1_pwm_rgb.h"

// ===== 80MHz, Period=100, Down Counting, Initial High =====
// 发 0: CCR=75, 高电平 = (99-75)*12.5ns = 0.3us  ✓
// 发 1: CCR=50, 高电平 = (99-50)*12.5ns = 0.6us  ✓
#define RGB_BIT_0_VAL    (75U)
#define RGB_BIT_1_VAL    (50U)

#define RGB_SK6812_BITS_PER_LED       (24U)
#define RGB_SK6812_FRAME_WORDS        (RGB_SK6812_LED_COUNT * RGB_SK6812_BITS_PER_LED)

static rgb_sk6812_color_t g_pixels[RGB_SK6812_LED_COUNT];
static uint16_t g_frame[RGB_SK6812_FRAME_WORDS];
static bool g_inited = false;

static void rgb_sk6812_append_byte(uint8_t data, uint32_t *index)
{
    uint8_t mask;
    for (mask = 0x80U; mask != 0U; mask >>= 1) {
        g_frame[*index] = ((data & mask) != 0U) ? RGB_BIT_1_VAL : RGB_BIT_0_VAL;
        (*index)++;
    }
}

static void rgb_sk6812_build_frame(void)
{
    uint32_t i;
    uint32_t index = 0U;

    for (i = 0U; i < RGB_SK6812_LED_COUNT; i++) {
        rgb_sk6812_append_byte(g_pixels[i].g, &index);
        rgb_sk6812_append_byte(g_pixels[i].b, &index);
        rgb_sk6812_append_byte(g_pixels[i].r, &index);
    }
}

void rgb_sk6812_init(void)
{
    uint16_t i;
    tima1_pwm_rgb_init();

    for (i = 0U; i < RGB_SK6812_LED_COUNT; i++) {
        g_pixels[i].r = 0U;
        g_pixels[i].g = 0U;
        g_pixels[i].b = 0U;

    }
    g_inited = true;
    rgb_sk6812_show();
}

void rgb_sk6812_set_rgb(uint8_t r, uint8_t g, uint8_t b)
{
    uint16_t i;
    for (i = 0U; i < RGB_SK6812_LED_COUNT; i++) {
        g_pixels[i].r = r;
        g_pixels[i].g = g;
        g_pixels[i].b = b;
    }
    rgb_sk6812_show();
}

void rgb_sk6812_show(void)
{
    if (!g_inited) return;

    while (tima1_pwm_rgb_is_busy()) {}

    rgb_sk6812_build_frame();

    while (!tima1_pwm_rgb_start_frame(g_frame, RGB_SK6812_FRAME_WORDS)) {}

    tima1_pwm_rgb_wait_done();
}

void rgb_sk6812_clear_buffer(void)
{
    uint16_t i;
    for (i = 0U; i < RGB_SK6812_LED_COUNT; i++) {
        g_pixels[i].r = 0U;
        g_pixels[i].g = 0U;
        g_pixels[i].b = 0U;
    }
}

void rgb_sk6812_fill(uint8_t r, uint8_t g, uint8_t b)
{
    uint16_t i;
    for (i = 0U; i < RGB_SK6812_LED_COUNT; i++) {
        g_pixels[i].r = r;
        g_pixels[i].g = g;
        g_pixels[i].b = b;
    }
}

void rgb_sk6812_set_pixel(uint16_t index, uint8_t r, uint8_t g, uint8_t b)
{
    if (index >= RGB_SK6812_LED_COUNT) return;
    g_pixels[index].r = r;
    g_pixels[index].g = g;
    g_pixels[index].b = b;
}

rgb_sk6812_color_t rgb_sk6812_get_pixel(uint16_t index)
{
    rgb_sk6812_color_t empty = {0, 0, 0};
    if (index >= RGB_SK6812_LED_COUNT) return empty;
    return g_pixels[index];
}

bool rgb_sk6812_is_busy(void)
{
    return tima1_pwm_rgb_is_busy();
}

void rgb_sk6812_off(void)
{
    rgb_sk6812_set_rgb(0U, 0U, 0U);
}

void rgb_sk6812_set_on(bool on)
{
    if (!on) {
        rgb_sk6812_off();
    }
}