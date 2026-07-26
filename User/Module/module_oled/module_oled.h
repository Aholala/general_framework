#ifndef MODULE_OLED_H
#define MODULE_OLED_H

#include "bsp_i2c.h"
#include "module_device.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef enum
    {
        MODULE_OLED_STATUS_OK = 0,
        MODULE_OLED_STATUS_INVALID_ARGUMENT,
        MODULE_OLED_STATUS_NOT_INITIALIZED,
        MODULE_OLED_STATUS_NOT_STARTED,
        MODULE_OLED_STATUS_TRANSPORT_ERROR
    } module_oled_status_t;

    typedef struct
    {
        bsp_i2c_t *i2c;
        uint16_t address_7bit;
        uint16_t width_pixels;
        uint16_t height_pixels;
        uint8_t *frame_buffer;
        size_t frame_buffer_size;
        uint32_t timeout_ms;
        const char *logical_name;
        uint32_t registration_key;
    } module_oled_config_t;

    typedef struct
    {
        module_device_t super;
        bsp_i2c_t *i2c;
        uint16_t address_7bit;
        uint16_t width_pixels;
        uint16_t height_pixels;
        uint8_t *frame_buffer;
        size_t frame_buffer_size;
        uint32_t timeout_ms;
        bool is_started;
    } module_oled_t;

    module_oled_status_t module_oled_init(module_oled_t *me, const module_oled_config_t *config);
    module_oled_status_t module_oled_start(module_oled_t *me);
    module_oled_status_t module_oled_stop(module_oled_t *me);
    module_oled_status_t module_oled_clear(module_oled_t *me, bool is_on);
    module_oled_status_t module_oled_set_pixel(module_oled_t *me, int32_t position_x,
                                               int32_t position_y, bool is_on);
    module_oled_status_t module_oled_draw_line(module_oled_t *me, int32_t start_x, int32_t start_y,
                                               int32_t end_x, int32_t end_y, bool is_on);
    module_oled_status_t module_oled_draw_rectangle(module_oled_t *me, int32_t position_x,
                                                    int32_t position_y, uint16_t width_pixels,
                                                    uint16_t height_pixels, bool is_filled,
                                                    bool is_on);
    module_oled_status_t module_oled_draw_bitmap(module_oled_t *me, int32_t position_x,
                                                 int32_t position_y, uint16_t width_pixels,
                                                 uint16_t height_pixels, const uint8_t *bitmap,
                                                 size_t bitmap_size, bool is_on);
    module_oled_status_t module_oled_flush(module_oled_t *me);
    module_oled_status_t module_oled_set_contrast(module_oled_t *me, uint8_t contrast);

#ifdef __cplusplus
}
#endif

#endif /* MODULE_OLED_H */
