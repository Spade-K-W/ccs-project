#ifndef RGB_SK6812_H
#define RGB_SK6812_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef RGB_SK6812_LED_COUNT
#define RGB_SK6812_LED_COUNT      (1U)
#endif

typedef struct
{
    uint8_t r;
    uint8_t g;
    uint8_t b;
} rgb_sk6812_color_t;

void rgb_sk6812_init(void);
void rgb_sk6812_clear_buffer(void);
void rgb_sk6812_fill(uint8_t r, uint8_t g, uint8_t b);
void rgb_sk6812_set_pixel(uint16_t index, uint8_t r, uint8_t g, uint8_t b);
rgb_sk6812_color_t rgb_sk6812_get_pixel(uint16_t index);
void rgb_sk6812_show(void);
bool rgb_sk6812_is_busy(void);

/* 兼容旧接口 */
void rgb_sk6812_set_rgb(uint8_t r, uint8_t g, uint8_t b);
void rgb_sk6812_off(void);
void rgb_sk6812_set_on(bool on);

#ifdef __cplusplus
}
#endif

#endif