#include "module_ws2812.h"

#include <stddef.h>
#include <string.h>

static uint8_t module_ws2812_scale_component(uint8_t component, uint8_t brightness)
{
    return (uint8_t)(((uint16_t)component * brightness + 127U) / 255U);
}

static module_ws2812_color_t module_ws2812_scale_color(module_ws2812_color_t color,
                                                       uint8_t brightness)
{
    return module_ws2812_make_color(module_ws2812_scale_component(color.red, brightness),
                                    module_ws2812_scale_component(color.green, brightness),
                                    module_ws2812_scale_component(color.blue, brightness));
}

static module_ws2812_status_t module_ws2812_configure_effect(module_ws2812_t *me,
                                                             module_ws2812_effect_t effect_type,
                                                             module_ws2812_color_t color,
                                                             uint32_t step_time_ms)
{
    if ((me == NULL) || (step_time_ms == 0U) || (effect_type == MODULE_WS2812_EFFECT_NONE) ||
        (effect_type > MODULE_WS2812_EFFECT_THEATER_CHASE))
    {
        return MODULE_WS2812_STATUS_INVALID_ARGUMENT;
    }
    if (!module_device_is_initialized(&me->super))
    {
        return MODULE_WS2812_STATUS_NOT_INITIALIZED;
    }
    if (!me->is_started)
    {
        return MODULE_WS2812_STATUS_NOT_STARTED;
    }
    me->effect = (module_ws2812_effect_state_t){
        .type = effect_type,
        .color = color,
        .step_time_ms = step_time_ms,
        .brightness = 0U,
        .brightness_direction = 1,
        .is_enabled = true,
    };
    return MODULE_WS2812_STATUS_OK;
}

static void module_ws2812_encode_byte(uint8_t value, uint8_t encoded_bytes[3])
{
    uint32_t encoded_value = 0U;
    uint8_t bit_index;

    for (bit_index = 0U; bit_index < 8U; ++bit_index)
    {
        encoded_value <<= 3U;
        encoded_value |= ((value & (uint8_t)(0x80U >> bit_index)) != 0U) ? 0x06U : 0x04U;
    }
    encoded_bytes[0] = (uint8_t)(encoded_value >> 16U);
    encoded_bytes[1] = (uint8_t)(encoded_value >> 8U);
    encoded_bytes[2] = (uint8_t)encoded_value;
}

static module_device_status_t module_ws2812_device_start(module_device_t *const device_base)
{
    module_ws2812_t *const me = MODULE_CONTAINER_OF(device_base, module_ws2812_t, super);
    return (module_ws2812_start(me) == MODULE_WS2812_STATUS_OK)
               ? MODULE_DEVICE_STATUS_OK
               : MODULE_DEVICE_STATUS_OPERATION_FAILED;
}

static module_device_status_t module_ws2812_device_stop(module_device_t *const device_base)
{
    module_ws2812_t *const me = MODULE_CONTAINER_OF(device_base, module_ws2812_t, super);
    return (module_ws2812_stop(me) == MODULE_WS2812_STATUS_OK)
               ? MODULE_DEVICE_STATUS_OK
               : MODULE_DEVICE_STATUS_OPERATION_FAILED;
}

static module_device_status_t module_ws2812_device_update(module_device_t *const device_base,
                                                          uint32_t elapsed_time_ms)
{
    module_ws2812_t *const me = MODULE_CONTAINER_OF(device_base, module_ws2812_t, super);
    const module_ws2812_status_t status = module_ws2812_update(me, elapsed_time_ms);
    return ((status == MODULE_WS2812_STATUS_OK) || (status == MODULE_WS2812_STATUS_BUSY))
               ? MODULE_DEVICE_STATUS_OK
               : MODULE_DEVICE_STATUS_OPERATION_FAILED;
}

static const module_device_ops_t s_module_ws2812_ops = {
    .start = module_ws2812_device_start,
    .stop = module_ws2812_device_stop,
    .update = module_ws2812_device_update,
};

module_ws2812_status_t module_ws2812_init(module_ws2812_t *me, const module_ws2812_config_t *config)
{
    size_t required_buffer_size;

    if ((me == NULL) || (config == NULL) || (config->spi == NULL) ||
        !bsp_device_is_initialized(&config->spi->super) || (config->pixels == NULL) ||
        (config->led_count == 0U) || (config->transmit_buffer == NULL) ||
        (config->reset_byte_count == 0U) || !bsp_transfer_mode_is_valid(config->transfer_mode))
    {
        return MODULE_WS2812_STATUS_INVALID_ARGUMENT;
    }
    required_buffer_size =
        MODULE_WS2812_REQUIRED_BUFFER_SIZE(config->led_count, config->reset_byte_count);
    if (config->transmit_buffer_size < required_buffer_size)
    {
        return MODULE_WS2812_STATUS_INVALID_ARGUMENT;
    }
    *me = (module_ws2812_t){0};
    me->spi = config->spi;
    me->pixels = config->pixels;
    me->led_count = config->led_count;
    me->transmit_buffer = config->transmit_buffer;
    me->transmit_buffer_size = required_buffer_size;
    me->reset_byte_count = config->reset_byte_count;
    me->transmit_timeout_ms = config->transmit_timeout_ms;
    me->transfer_mode = config->transfer_mode;
    me->brightness = 255U;
    memset(me->pixels, 0, me->led_count * sizeof(*me->pixels));
    memset(me->transmit_buffer, 0, required_buffer_size);
    if (module_device_init_base(&me->super, &s_module_ws2812_ops, config->logical_name,
                                config->registration_key) != MODULE_DEVICE_STATUS_OK)
    {
        return MODULE_WS2812_STATUS_INVALID_ARGUMENT;
    }
    if (module_device_complete_init(&me->super) != MODULE_DEVICE_STATUS_OK)
    {
        module_device_abort_init(&me->super);
        return MODULE_WS2812_STATUS_INVALID_ARGUMENT;
    }
    return MODULE_WS2812_STATUS_OK;
}

module_ws2812_status_t module_ws2812_start(module_ws2812_t *me)
{
    if ((me == NULL) || !module_device_is_initialized(&me->super))
    {
        return MODULE_WS2812_STATUS_NOT_INITIALIZED;
    }
    me->is_started = true;
    return MODULE_WS2812_STATUS_OK;
}

module_ws2812_status_t module_ws2812_stop(module_ws2812_t *me)
{
    if ((me == NULL) || !module_device_is_initialized(&me->super))
    {
        return MODULE_WS2812_STATUS_NOT_INITIALIZED;
    }
    if (me->is_busy)
    {
        (void)bsp_spi_abort(me->spi);
    }
    me->is_busy = false;
    me->effect.is_enabled = false;
    me->is_started = false;
    return MODULE_WS2812_STATUS_OK;
}

module_ws2812_status_t module_ws2812_set_pixel(module_ws2812_t *me, size_t led_index,
                                               module_ws2812_color_t color)
{
    if (me == NULL)
    {
        return MODULE_WS2812_STATUS_INVALID_ARGUMENT;
    }
    if (!module_device_is_initialized(&me->super))
    {
        return MODULE_WS2812_STATUS_NOT_INITIALIZED;
    }
    if (led_index >= me->led_count)
    {
        return MODULE_WS2812_STATUS_INVALID_ARGUMENT;
    }
    me->pixels[led_index] = color;
    return MODULE_WS2812_STATUS_OK;
}

module_ws2812_status_t module_ws2812_fill(module_ws2812_t *me, module_ws2812_color_t color)
{
    size_t led_index;

    if ((me == NULL) || !module_device_is_initialized(&me->super))
    {
        return MODULE_WS2812_STATUS_NOT_INITIALIZED;
    }
    for (led_index = 0U; led_index < me->led_count; ++led_index)
    {
        me->pixels[led_index] = color;
    }
    return MODULE_WS2812_STATUS_OK;
}

module_ws2812_status_t module_ws2812_clear(module_ws2812_t *me)
{
    return module_ws2812_fill(me, (module_ws2812_color_t){0U, 0U, 0U});
}

module_ws2812_status_t module_ws2812_set_brightness(module_ws2812_t *me, uint8_t brightness)
{
    if ((me == NULL) || !module_device_is_initialized(&me->super))
    {
        return MODULE_WS2812_STATUS_NOT_INITIALIZED;
    }
    me->brightness = brightness;
    return MODULE_WS2812_STATUS_OK;
}

module_ws2812_status_t module_ws2812_show(module_ws2812_t *me)
{
    size_t led_index;
    size_t output_index = 0U;
    bsp_status_t status;

    if ((me == NULL) || !module_device_is_initialized(&me->super))
    {
        return MODULE_WS2812_STATUS_NOT_INITIALIZED;
    }
    if (!me->is_started)
    {
        return MODULE_WS2812_STATUS_NOT_STARTED;
    }
    if (me->is_busy)
    {
        return MODULE_WS2812_STATUS_BUSY;
    }
    for (led_index = 0U; led_index < me->led_count; ++led_index)
    {
        uint8_t encoded_bytes[3];
        const module_ws2812_color_t color = me->pixels[led_index];
        const uint8_t components[3] = {
            module_ws2812_scale_component(color.green, me->brightness),
            module_ws2812_scale_component(color.red, me->brightness),
            module_ws2812_scale_component(color.blue, me->brightness),
        };
        size_t component_index;

        for (component_index = 0U; component_index < 3U; ++component_index)
        {
            module_ws2812_encode_byte(components[component_index], encoded_bytes);
            memcpy(&me->transmit_buffer[output_index], encoded_bytes, sizeof(encoded_bytes));
            output_index += sizeof(encoded_bytes);
        }
    }
    memset(&me->transmit_buffer[output_index], 0, me->reset_byte_count);
    me->is_busy = me->transfer_mode != BSP_TRANSFER_MODE_BLOCKING;
    status = bsp_spi_transmit(me->spi, me->transmit_buffer, me->transmit_buffer_size,
                              me->transfer_mode, me->transmit_timeout_ms);
    if (status != BSP_STATUS_OK)
    {
        me->is_busy = false;
        return MODULE_WS2812_STATUS_TRANSPORT_ERROR;
    }
    return MODULE_WS2812_STATUS_OK;
}

module_ws2812_status_t module_ws2812_update(module_ws2812_t *me, uint32_t elapsed_time_ms)
{
    module_ws2812_effect_state_t *effect;

    if ((me == NULL) || !module_device_is_initialized(&me->super))
    {
        return MODULE_WS2812_STATUS_NOT_INITIALIZED;
    }
    if (!me->is_started)
    {
        return MODULE_WS2812_STATUS_NOT_STARTED;
    }
    if (!me->effect.is_enabled)
    {
        return MODULE_WS2812_STATUS_OK;
    }
    if (me->is_busy)
    {
        return MODULE_WS2812_STATUS_BUSY;
    }
    effect = &me->effect;
    if (UINT32_MAX - effect->elapsed_time_ms < elapsed_time_ms)
    {
        effect->elapsed_time_ms = UINT32_MAX;
    }
    else
    {
        effect->elapsed_time_ms += elapsed_time_ms;
    }
    if (effect->elapsed_time_ms < effect->step_time_ms)
    {
        return MODULE_WS2812_STATUS_OK;
    }
    effect->elapsed_time_ms %= effect->step_time_ms;

    if (effect->type == MODULE_WS2812_EFFECT_BLINK)
    {
        effect->phase ^= 1U;
        (void)module_ws2812_fill(me, (effect->phase != 0U) ? effect->color
                                                           : module_ws2812_make_color(0U, 0U, 0U));
    }
    else if (effect->type == MODULE_WS2812_EFFECT_COLOR_WIPE)
    {
        (void)module_ws2812_set_pixel(me, effect->led_index, effect->color);
        ++effect->led_index;
        if (effect->led_index >= me->led_count)
        {
            effect->is_enabled = false;
        }
    }
    else if (effect->type == MODULE_WS2812_EFFECT_BREATH)
    {
        int32_t new_brightness =
            (int32_t)effect->brightness + ((int32_t)effect->brightness_direction * 5);
        if (new_brightness >= 255)
        {
            new_brightness = 255;
            effect->brightness_direction = -1;
        }
        else if (new_brightness <= 0)
        {
            new_brightness = 0;
            effect->brightness_direction = 1;
        }
        effect->brightness = (uint8_t)new_brightness;
        (void)module_ws2812_fill(me, module_ws2812_scale_color(effect->color, effect->brightness));
    }
    else if (effect->type == MODULE_WS2812_EFFECT_RAINBOW)
    {
        size_t led_index;
        for (led_index = 0U; led_index < me->led_count; ++led_index)
        {
            const uint8_t color_position =
                (uint8_t)(effect->color_offset + (uint16_t)((led_index * 256U) / me->led_count));
            me->pixels[led_index] = module_ws2812_color_wheel(color_position);
        }
        ++effect->color_offset;
    }
    else if (effect->type == MODULE_WS2812_EFFECT_THEATER_CHASE)
    {
        size_t led_index;
        (void)module_ws2812_clear(me);
        for (led_index = effect->phase; led_index < me->led_count; led_index += 3U)
        {
            me->pixels[led_index] = effect->color;
        }
        effect->phase = (uint8_t)((effect->phase + 1U) % 3U);
    }
    return module_ws2812_show(me);
}

module_ws2812_status_t module_ws2812_start_blink(module_ws2812_t *me, module_ws2812_color_t color,
                                                 uint32_t half_period_ms)
{
    return module_ws2812_configure_effect(me, MODULE_WS2812_EFFECT_BLINK, color, half_period_ms);
}

module_ws2812_status_t module_ws2812_start_color_wipe(module_ws2812_t *me,
                                                      module_ws2812_color_t color,
                                                      uint32_t step_time_ms)
{
    const module_ws2812_status_t status =
        module_ws2812_configure_effect(me, MODULE_WS2812_EFFECT_COLOR_WIPE, color, step_time_ms);
    if (status == MODULE_WS2812_STATUS_OK)
    {
        (void)module_ws2812_clear(me);
    }
    return status;
}

module_ws2812_status_t module_ws2812_start_breath(module_ws2812_t *me, module_ws2812_color_t color,
                                                  uint32_t step_time_ms)
{
    return module_ws2812_configure_effect(me, MODULE_WS2812_EFFECT_BREATH, color, step_time_ms);
}

module_ws2812_status_t module_ws2812_start_rainbow(module_ws2812_t *me, uint32_t step_time_ms)
{
    return module_ws2812_configure_effect(me, MODULE_WS2812_EFFECT_RAINBOW,
                                          module_ws2812_make_color(0U, 0U, 0U), step_time_ms);
}

module_ws2812_status_t module_ws2812_start_theater_chase(module_ws2812_t *me,
                                                         module_ws2812_color_t color,
                                                         uint32_t step_time_ms)
{
    return module_ws2812_configure_effect(me, MODULE_WS2812_EFFECT_THEATER_CHASE, color,
                                          step_time_ms);
}

module_ws2812_status_t module_ws2812_stop_effect(module_ws2812_t *me)
{
    if ((me == NULL) || !module_device_is_initialized(&me->super))
    {
        return MODULE_WS2812_STATUS_NOT_INITIALIZED;
    }
    me->effect.is_enabled = false;
    me->effect.type = MODULE_WS2812_EFFECT_NONE;
    return MODULE_WS2812_STATUS_OK;
}

void module_ws2812_notify_transmit_complete(module_ws2812_t *me, bsp_status_t status)
{
    if ((me != NULL) && module_device_is_initialized(&me->super))
    {
        me->is_busy = false;
        if (status != BSP_STATUS_OK)
        {
            (void)bsp_spi_abort(me->spi);
        }
    }
}

bool module_ws2812_is_busy(const module_ws2812_t *me)
{
    return (me != NULL) && module_device_is_initialized(&me->super) && me->is_busy;
}

module_ws2812_color_t module_ws2812_make_color(uint8_t red, uint8_t green, uint8_t blue)
{
    return (module_ws2812_color_t){.red = red, .green = green, .blue = blue};
}

module_ws2812_color_t module_ws2812_color_wheel(uint8_t position)
{
    if (position < 85U)
    {
        return module_ws2812_make_color((uint8_t)(position * 3U), (uint8_t)(255U - position * 3U),
                                        0U);
    }
    if (position < 170U)
    {
        position = (uint8_t)(position - 85U);
        return module_ws2812_make_color((uint8_t)(255U - position * 3U), 0U,
                                        (uint8_t)(position * 3U));
    }
    position = (uint8_t)(position - 170U);
    return module_ws2812_make_color(0U, (uint8_t)(position * 3U), (uint8_t)(255U - position * 3U));
}
