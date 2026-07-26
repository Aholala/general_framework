#include "module_oled.h"

#include <stddef.h>
#include <string.h>

#define MODULE_OLED_CONTROL_COMMAND (0x00U)
#define MODULE_OLED_CONTROL_DATA (0x40U)
#define MODULE_OLED_MAXIMUM_WIDTH_PIXELS (128U)
#define MODULE_OLED_MAXIMUM_PAGE_COUNT (8U)

static module_oled_status_t module_oled_write(module_oled_t *me, uint8_t control_byte,
                                              const uint8_t *payload, size_t payload_size)
{
    uint8_t transfer_buffer[MODULE_OLED_MAXIMUM_WIDTH_PIXELS + 1U];

    if ((payload == NULL) || (payload_size == 0U) ||
        (payload_size > MODULE_OLED_MAXIMUM_WIDTH_PIXELS))
    {
        return MODULE_OLED_STATUS_INVALID_ARGUMENT;
    }
    transfer_buffer[0] = control_byte;
    memcpy(&transfer_buffer[1], payload, payload_size);
    return (bsp_i2c_transmit(me->i2c, me->address_7bit, transfer_buffer, payload_size + 1U,
                             BSP_TRANSFER_MODE_BLOCKING, me->timeout_ms) == BSP_STATUS_OK)
               ? MODULE_OLED_STATUS_OK
               : MODULE_OLED_STATUS_TRANSPORT_ERROR;
}

static module_oled_status_t module_oled_write_commands(module_oled_t *me, const uint8_t *commands,
                                                       size_t command_count)
{
    return module_oled_write(me, MODULE_OLED_CONTROL_COMMAND, commands, command_count);
}

static module_device_status_t module_oled_device_start(module_device_t *const device_base)
{
    module_oled_t *const me = MODULE_CONTAINER_OF(device_base, module_oled_t, super);
    return (module_oled_start(me) == MODULE_OLED_STATUS_OK) ? MODULE_DEVICE_STATUS_OK
                                                            : MODULE_DEVICE_STATUS_OPERATION_FAILED;
}

static module_device_status_t module_oled_device_stop(module_device_t *const device_base)
{
    module_oled_t *const me = MODULE_CONTAINER_OF(device_base, module_oled_t, super);
    return (module_oled_stop(me) == MODULE_OLED_STATUS_OK) ? MODULE_DEVICE_STATUS_OK
                                                           : MODULE_DEVICE_STATUS_OPERATION_FAILED;
}

static module_device_status_t module_oled_device_update(module_device_t *const device_base,
                                                        uint32_t elapsed_time_ms)
{
    (void)device_base;
    (void)elapsed_time_ms;
    return MODULE_DEVICE_STATUS_OK;
}

static const module_device_ops_t s_module_oled_ops = {
    .start = module_oled_device_start,
    .stop = module_oled_device_stop,
    .update = module_oled_device_update,
};

module_oled_status_t module_oled_init(module_oled_t *me, const module_oled_config_t *config)
{
    size_t required_buffer_size;

    if ((me == NULL) || (config == NULL) || (config->i2c == NULL) ||
        (config->address_7bit > 0x7FU) || (config->width_pixels == 0U) ||
        (config->width_pixels > MODULE_OLED_MAXIMUM_WIDTH_PIXELS) ||
        (config->height_pixels == 0U) || (config->height_pixels > 64U) ||
        ((config->height_pixels % 8U) != 0U) || (config->frame_buffer == NULL))
    {
        return MODULE_OLED_STATUS_INVALID_ARGUMENT;
    }
    required_buffer_size = (size_t)config->width_pixels * ((size_t)config->height_pixels / 8U);
    if (config->frame_buffer_size < required_buffer_size)
    {
        return MODULE_OLED_STATUS_INVALID_ARGUMENT;
    }
    *me = (module_oled_t){0};
    me->i2c = config->i2c;
    me->address_7bit = config->address_7bit;
    me->width_pixels = config->width_pixels;
    me->height_pixels = config->height_pixels;
    me->frame_buffer = config->frame_buffer;
    me->frame_buffer_size = required_buffer_size;
    me->timeout_ms = config->timeout_ms;
    memset(me->frame_buffer, 0, required_buffer_size);
    if (module_device_init_base(&me->super, &s_module_oled_ops, config->logical_name,
                                config->registration_key) != MODULE_DEVICE_STATUS_OK)
    {
        return MODULE_OLED_STATUS_INVALID_ARGUMENT;
    }
    if (module_device_complete_init(&me->super) != MODULE_DEVICE_STATUS_OK)
    {
        module_device_abort_init(&me->super);
        return MODULE_OLED_STATUS_INVALID_ARGUMENT;
    }
    return MODULE_OLED_STATUS_OK;
}

module_oled_status_t module_oled_start(module_oled_t *me)
{
    uint8_t commands[] = {
        0xAEU,
        0xD5U,
        0x80U,
        0xA8U,
        (uint8_t)(me != NULL ? me->height_pixels - 1U : 0U),
        0xD3U,
        0x00U,
        0x40U,
        0x8DU,
        0x14U,
        0x20U,
        0x02U,
        0xA1U,
        0xC8U,
        0xDAU,
        (uint8_t)((me != NULL && me->height_pixels == 64U) ? 0x12U : 0x02U),
        0x81U,
        0x7FU,
        0xD9U,
        0xF1U,
        0xDBU,
        0x40U,
        0xA4U,
        0xA6U,
        0xAFU,
    };

    if ((me == NULL) || !module_device_is_initialized(&me->super))
    {
        return MODULE_OLED_STATUS_NOT_INITIALIZED;
    }
    if (module_oled_write_commands(me, commands, sizeof(commands)) != MODULE_OLED_STATUS_OK)
    {
        return MODULE_OLED_STATUS_TRANSPORT_ERROR;
    }
    me->is_started = true;
    return module_oled_flush(me);
}

module_oled_status_t module_oled_stop(module_oled_t *me)
{
    const uint8_t command = 0xAEU;

    if ((me == NULL) || !module_device_is_initialized(&me->super))
    {
        return MODULE_OLED_STATUS_NOT_INITIALIZED;
    }
    if (!me->is_started)
    {
        return MODULE_OLED_STATUS_OK;
    }
    if (module_oled_write_commands(me, &command, 1U) != MODULE_OLED_STATUS_OK)
    {
        return MODULE_OLED_STATUS_TRANSPORT_ERROR;
    }
    me->is_started = false;
    return MODULE_OLED_STATUS_OK;
}

module_oled_status_t module_oled_clear(module_oled_t *me, bool is_on)
{
    if ((me == NULL) || !module_device_is_initialized(&me->super))
    {
        return MODULE_OLED_STATUS_NOT_INITIALIZED;
    }
    memset(me->frame_buffer, is_on ? 0xFF : 0x00, me->frame_buffer_size);
    return MODULE_OLED_STATUS_OK;
}

module_oled_status_t module_oled_set_pixel(module_oled_t *me, int32_t position_x,
                                           int32_t position_y, bool is_on)
{
    size_t buffer_index;
    uint8_t pixel_mask;

    if ((me == NULL) || !module_device_is_initialized(&me->super))
    {
        return MODULE_OLED_STATUS_NOT_INITIALIZED;
    }
    if ((position_x < 0) || (position_y < 0) || (position_x >= (int32_t)me->width_pixels) ||
        (position_y >= (int32_t)me->height_pixels))
    {
        return MODULE_OLED_STATUS_INVALID_ARGUMENT;
    }
    buffer_index = (size_t)position_x + ((size_t)position_y / 8U) * me->width_pixels;
    pixel_mask = (uint8_t)(1U << ((uint32_t)position_y & 7U));
    if (is_on)
    {
        me->frame_buffer[buffer_index] |= pixel_mask;
    }
    else
    {
        me->frame_buffer[buffer_index] &= (uint8_t)(~pixel_mask);
    }
    return MODULE_OLED_STATUS_OK;
}

module_oled_status_t module_oled_draw_line(module_oled_t *me, int32_t start_x, int32_t start_y,
                                           int32_t end_x, int32_t end_y, bool is_on)
{
    int32_t delta_x = (end_x > start_x) ? end_x - start_x : start_x - end_x;
    int32_t delta_y = (end_y > start_y) ? start_y - end_y : end_y - start_y;
    const int32_t step_x = (start_x < end_x) ? 1 : -1;
    const int32_t step_y = (start_y < end_y) ? 1 : -1;
    int32_t error = delta_x + delta_y;

    if ((me == NULL) || !module_device_is_initialized(&me->super))
    {
        return MODULE_OLED_STATUS_NOT_INITIALIZED;
    }
    if ((start_x < 0) || (start_y < 0) || (end_x < 0) || (end_y < 0) ||
        (start_x >= (int32_t)me->width_pixels) || (end_x >= (int32_t)me->width_pixels) ||
        (start_y >= (int32_t)me->height_pixels) || (end_y >= (int32_t)me->height_pixels))
    {
        return MODULE_OLED_STATUS_INVALID_ARGUMENT;
    }
    for (;;)
    {
        (void)module_oled_set_pixel(me, start_x, start_y, is_on);
        if ((start_x == end_x) && (start_y == end_y))
        {
            break;
        }
        if ((2 * error) >= delta_y)
        {
            error += delta_y;
            start_x += step_x;
        }
        if ((2 * error) <= delta_x)
        {
            error += delta_x;
            start_y += step_y;
        }
    }
    return MODULE_OLED_STATUS_OK;
}

module_oled_status_t module_oled_draw_rectangle(module_oled_t *me, int32_t position_x,
                                                int32_t position_y, uint16_t width_pixels,
                                                uint16_t height_pixels, bool is_filled, bool is_on)
{
    uint16_t row_index;

    if ((me == NULL) || (width_pixels == 0U) || (height_pixels == 0U))
    {
        return MODULE_OLED_STATUS_INVALID_ARGUMENT;
    }
    if (!module_device_is_initialized(&me->super))
    {
        return MODULE_OLED_STATUS_NOT_INITIALIZED;
    }
    if ((position_x < 0) || (position_y < 0) ||
        ((uint32_t)position_x + width_pixels > me->width_pixels) ||
        ((uint32_t)position_y + height_pixels > me->height_pixels))
    {
        return MODULE_OLED_STATUS_INVALID_ARGUMENT;
    }
    if (is_filled)
    {
        for (row_index = 0U; row_index < height_pixels; ++row_index)
        {
            (void)module_oled_draw_line(me, position_x, position_y + row_index,
                                        position_x + width_pixels - 1, position_y + row_index,
                                        is_on);
        }
    }
    else
    {
        (void)module_oled_draw_line(me, position_x, position_y, position_x + width_pixels - 1,
                                    position_y, is_on);
        (void)module_oled_draw_line(me, position_x, position_y + height_pixels - 1,
                                    position_x + width_pixels - 1, position_y + height_pixels - 1,
                                    is_on);
        (void)module_oled_draw_line(me, position_x, position_y, position_x,
                                    position_y + height_pixels - 1, is_on);
        (void)module_oled_draw_line(me, position_x + width_pixels - 1, position_y,
                                    position_x + width_pixels - 1, position_y + height_pixels - 1,
                                    is_on);
    }
    return MODULE_OLED_STATUS_OK;
}

module_oled_status_t module_oled_draw_bitmap(module_oled_t *me, int32_t position_x,
                                             int32_t position_y, uint16_t width_pixels,
                                             uint16_t height_pixels, const uint8_t *bitmap,
                                             size_t bitmap_size, bool is_on)
{
    size_t required_size = ((size_t)width_pixels * height_pixels + 7U) / 8U;
    uint16_t pixel_y;
    uint16_t pixel_x;

    if ((me == NULL) || (bitmap == NULL) || (width_pixels == 0U) || (height_pixels == 0U) ||
        (bitmap_size < required_size))
    {
        return MODULE_OLED_STATUS_INVALID_ARGUMENT;
    }
    if (!module_device_is_initialized(&me->super))
    {
        return MODULE_OLED_STATUS_NOT_INITIALIZED;
    }
    for (pixel_y = 0U; pixel_y < height_pixels; ++pixel_y)
    {
        for (pixel_x = 0U; pixel_x < width_pixels; ++pixel_x)
        {
            const size_t bit_index = (size_t)pixel_y * width_pixels + pixel_x;
            if ((bitmap[bit_index / 8U] & (uint8_t)(0x80U >> (bit_index & 7U))) != 0U)
            {
                const int32_t target_x = position_x + (int32_t)pixel_x;
                const int32_t target_y = position_y + (int32_t)pixel_y;
                if ((target_x >= 0) && (target_y >= 0) && (target_x < (int32_t)me->width_pixels) &&
                    (target_y < (int32_t)me->height_pixels))
                {
                    (void)module_oled_set_pixel(me, target_x, target_y, is_on);
                }
            }
        }
    }
    return MODULE_OLED_STATUS_OK;
}

module_oled_status_t module_oled_flush(module_oled_t *me)
{
    uint16_t page_index;
    const uint16_t page_count = (me != NULL) ? me->height_pixels / 8U : 0U;

    if ((me == NULL) || !module_device_is_initialized(&me->super))
    {
        return MODULE_OLED_STATUS_NOT_INITIALIZED;
    }
    if (!me->is_started)
    {
        return MODULE_OLED_STATUS_NOT_STARTED;
    }
    for (page_index = 0U; page_index < page_count; ++page_index)
    {
        const uint8_t commands[3] = {(uint8_t)(0xB0U + page_index), 0x00U, 0x10U};
        if ((module_oled_write_commands(me, commands, 3U) != MODULE_OLED_STATUS_OK) ||
            (module_oled_write(me, MODULE_OLED_CONTROL_DATA,
                               &me->frame_buffer[(size_t)page_index * me->width_pixels],
                               me->width_pixels) != MODULE_OLED_STATUS_OK))
        {
            return MODULE_OLED_STATUS_TRANSPORT_ERROR;
        }
    }
    return MODULE_OLED_STATUS_OK;
}

module_oled_status_t module_oled_set_contrast(module_oled_t *me, uint8_t contrast)
{
    const uint8_t commands[2] = {0x81U, contrast};

    if ((me == NULL) || !module_device_is_initialized(&me->super))
    {
        return MODULE_OLED_STATUS_NOT_INITIALIZED;
    }
    if (!me->is_started)
    {
        return MODULE_OLED_STATUS_NOT_STARTED;
    }
    return module_oled_write_commands(me, commands, 2U);
}
