#ifndef MODULE_WS2812_H
#define MODULE_WS2812_H

#include "bsp_spi.h"
#include "module_device.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define MODULE_WS2812_ENCODED_BYTES_PER_LED (9U)
#define MODULE_WS2812_REQUIRED_BUFFER_SIZE(led_count, reset_byte_count)                            \
    ((size_t)(led_count) * MODULE_WS2812_ENCODED_BYTES_PER_LED + (size_t)(reset_byte_count))

    typedef enum
    {
        MODULE_WS2812_STATUS_OK = 0,
        MODULE_WS2812_STATUS_INVALID_ARGUMENT,
        MODULE_WS2812_STATUS_NOT_INITIALIZED,
        MODULE_WS2812_STATUS_NOT_STARTED,
        MODULE_WS2812_STATUS_BUSY,
        MODULE_WS2812_STATUS_TRANSPORT_ERROR
    } module_ws2812_status_t;

    typedef struct
    {
        uint8_t red;
        uint8_t green;
        uint8_t blue;
    } module_ws2812_color_t;

    typedef enum
    {
        MODULE_WS2812_EFFECT_NONE = 0,
        MODULE_WS2812_EFFECT_BLINK,
        MODULE_WS2812_EFFECT_COLOR_WIPE,
        MODULE_WS2812_EFFECT_BREATH,
        MODULE_WS2812_EFFECT_RAINBOW,
        MODULE_WS2812_EFFECT_THEATER_CHASE
    } module_ws2812_effect_t;

    typedef struct
    {
        module_ws2812_effect_t type;
        module_ws2812_color_t color;
        uint32_t step_time_ms;
        uint32_t elapsed_time_ms;
        size_t led_index;
        uint16_t color_offset;
        uint8_t phase;
        uint8_t brightness;
        int8_t brightness_direction;
        bool is_enabled;
    } module_ws2812_effect_state_t;

    typedef struct
    {
        bsp_spi_t *spi;
        module_ws2812_color_t *pixels;
        size_t led_count;
        uint8_t *transmit_buffer;
        size_t transmit_buffer_size;
        size_t reset_byte_count;
        uint32_t transmit_timeout_ms;
        bsp_transfer_mode_t transfer_mode;
        const char *logical_name;
        uint32_t registration_key;
    } module_ws2812_config_t;

    typedef struct
    {
        module_device_t super;
        bsp_spi_t *spi;
        module_ws2812_color_t *pixels;
        size_t led_count;
        uint8_t *transmit_buffer;
        size_t transmit_buffer_size;
        size_t reset_byte_count;
        uint32_t transmit_timeout_ms;
        bsp_transfer_mode_t transfer_mode;
        uint8_t brightness;
        module_ws2812_effect_state_t effect;
        volatile bool is_busy;
        bool is_started;
    } module_ws2812_t;

    module_ws2812_status_t module_ws2812_init(module_ws2812_t *me,
                                              const module_ws2812_config_t *config);
    module_ws2812_status_t module_ws2812_start(module_ws2812_t *me);
    module_ws2812_status_t module_ws2812_stop(module_ws2812_t *me);
    module_ws2812_status_t module_ws2812_set_pixel(module_ws2812_t *me, size_t led_index,
                                                   module_ws2812_color_t color);
    module_ws2812_status_t module_ws2812_fill(module_ws2812_t *me, module_ws2812_color_t color);
    module_ws2812_status_t module_ws2812_clear(module_ws2812_t *me);
    module_ws2812_status_t module_ws2812_set_brightness(module_ws2812_t *me, uint8_t brightness);
    module_ws2812_status_t module_ws2812_show(module_ws2812_t *me);
    module_ws2812_status_t module_ws2812_update(module_ws2812_t *me, uint32_t elapsed_time_ms);
    module_ws2812_status_t module_ws2812_start_blink(module_ws2812_t *me,
                                                     module_ws2812_color_t color,
                                                     uint32_t half_period_ms);
    module_ws2812_status_t module_ws2812_start_color_wipe(module_ws2812_t *me,
                                                          module_ws2812_color_t color,
                                                          uint32_t step_time_ms);
    module_ws2812_status_t module_ws2812_start_breath(module_ws2812_t *me,
                                                      module_ws2812_color_t color,
                                                      uint32_t step_time_ms);
    module_ws2812_status_t module_ws2812_start_rainbow(module_ws2812_t *me, uint32_t step_time_ms);
    module_ws2812_status_t module_ws2812_start_theater_chase(module_ws2812_t *me,
                                                             module_ws2812_color_t color,
                                                             uint32_t step_time_ms);
    module_ws2812_status_t module_ws2812_stop_effect(module_ws2812_t *me);
    void module_ws2812_notify_transmit_complete(module_ws2812_t *me, bsp_status_t status);
    bool module_ws2812_is_busy(const module_ws2812_t *me);
    module_ws2812_color_t module_ws2812_make_color(uint8_t red, uint8_t green, uint8_t blue);
    module_ws2812_color_t module_ws2812_color_wheel(uint8_t position);

#ifdef __cplusplus
}
#endif

#endif /* MODULE_WS2812_H */
