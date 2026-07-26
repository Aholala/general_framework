#include "module_referee.h"

#include "module_referee_crc.h"

#include <stddef.h>
#include <string.h>

static void module_referee_increment_counter(uint32_t *counter)
{
    if (*counter != UINT32_MAX)
    {
        ++(*counter);
    }
}

static void module_referee_discard_stream_prefix(module_referee_t *me, size_t discarded_size)
{
    if (discarded_size >= me->stream_size)
    {
        me->stream_size = 0U;
        return;
    }
    memmove(me->stream_buffer, &me->stream_buffer[discarded_size],
            me->stream_size - discarded_size);
    me->stream_size -= discarded_size;
}

static void module_referee_dispatch_frame(module_referee_t *me, uint16_t command_id,
                                          const uint8_t *payload, size_t payload_size,
                                          uint8_t sequence)
{
    size_t route_index;

    for (route_index = 0U; route_index < me->route_count; ++route_index)
    {
        if ((me->routes[route_index].command_id == command_id) &&
            (me->routes[route_index].handler != NULL))
        {
            me->routes[route_index].handler(command_id, payload, payload_size, sequence,
                                            me->routes[route_index].user_context);
            module_referee_increment_counter(&me->statistics.handled_frame_count);
            return;
        }
    }
    if (me->default_handler != NULL)
    {
        me->default_handler(command_id, payload, payload_size, sequence, me->default_user_context);
        module_referee_increment_counter(&me->statistics.handled_frame_count);
    }
    else
    {
        module_referee_increment_counter(&me->statistics.unknown_command_count);
    }
}

static module_referee_status_t module_referee_process_stream(module_referee_t *me)
{
    bool handled_frame = false;

    while (me->stream_size >= MODULE_REFEREE_HEADER_SIZE)
    {
        uint16_t payload_size;
        size_t frame_size;
        uint16_t command_id;
        uint8_t sequence;

        if (me->stream_buffer[0] != MODULE_REFEREE_START_OF_FRAME)
        {
            module_referee_discard_stream_prefix(me, 1U);
            module_referee_increment_counter(&me->statistics.discarded_byte_count);
            continue;
        }
        if (!module_referee_crc8_verify(me->stream_buffer, MODULE_REFEREE_HEADER_SIZE))
        {
            module_referee_discard_stream_prefix(me, 1U);
            module_referee_increment_counter(&me->statistics.crc8_error_count);
            continue;
        }
        payload_size = module_referee_read_uint16_le(&me->stream_buffer[1]);
        frame_size = MODULE_REFEREE_FRAME_SIZE(payload_size);
        if (frame_size > me->stream_capacity)
        {
            module_referee_discard_stream_prefix(me, 1U);
            module_referee_increment_counter(&me->statistics.oversize_frame_count);
            continue;
        }
        if (me->stream_size < frame_size)
        {
            break;
        }
        if (!module_referee_crc16_verify(me->stream_buffer, frame_size))
        {
            module_referee_discard_stream_prefix(me, 1U);
            module_referee_increment_counter(&me->statistics.crc16_error_count);
            continue;
        }
        sequence = me->stream_buffer[3];
        command_id = module_referee_read_uint16_le(&me->stream_buffer[MODULE_REFEREE_HEADER_SIZE]);
        module_referee_increment_counter(&me->statistics.received_frame_count);
        module_referee_dispatch_frame(
            me, command_id,
            &me->stream_buffer[MODULE_REFEREE_HEADER_SIZE + MODULE_REFEREE_COMMAND_ID_SIZE],
            payload_size, sequence);
        module_referee_discard_stream_prefix(me, frame_size);
        me->receive_elapsed_time_ms = 0U;
        me->is_online = true;
        handled_frame = true;
    }
    return handled_frame ? MODULE_REFEREE_STATUS_FRAME_HANDLED : MODULE_REFEREE_STATUS_OK;
}

static void module_referee_usart_callback(bsp_event_t event, bsp_status_t status,
                                          size_t transferred_size, void *user_context)
{
    module_referee_t *const me = (module_referee_t *)user_context;

    if (me == NULL)
    {
        return;
    }
    if ((event == BSP_EVENT_RECEIVE_COMPLETE) || (event == BSP_EVENT_RECEIVE_PENDING))
    {
        if ((status == BSP_STATUS_OK) && (transferred_size > 0U) &&
            (transferred_size <= me->receive_capacity) &&
            (transferred_size <= me->processing_capacity))
        {
            if (!me->is_receive_pending)
            {
                (void)memcpy(me->processing_buffer, me->receive_buffer, transferred_size);
                me->pending_receive_size = transferred_size;
                me->is_receive_pending = true;
            }
            else
            {
                module_referee_increment_counter(&me->statistics.receive_overrun_count);
            }
        }
        else
        {
            module_referee_increment_counter(&me->statistics.receive_overrun_count);
        }
        if (me->is_started &&
            (bsp_usart_receive_to_idle(me->usart, me->receive_buffer, me->receive_capacity,
                                       me->receive_mode, me->receive_timeout_ms) != BSP_STATUS_OK))
        {
            module_referee_increment_counter(&me->statistics.receive_restart_error_count);
        }
    }
    else if ((event == BSP_EVENT_TRANSMIT_COMPLETE) || (event == BSP_EVENT_ABORT_COMPLETE) ||
             (event == BSP_EVENT_ERROR))
    {
        me->is_transmit_busy = false;
    }
}

static module_device_status_t module_referee_device_start(module_device_t *const device_base)
{
    module_referee_t *const me = MODULE_CONTAINER_OF(device_base, module_referee_t, super);
    return (module_referee_start(me) == MODULE_REFEREE_STATUS_OK)
               ? MODULE_DEVICE_STATUS_OK
               : MODULE_DEVICE_STATUS_OPERATION_FAILED;
}

static module_device_status_t module_referee_device_stop(module_device_t *const device_base)
{
    module_referee_t *const me = MODULE_CONTAINER_OF(device_base, module_referee_t, super);
    return (module_referee_stop(me) == MODULE_REFEREE_STATUS_OK)
               ? MODULE_DEVICE_STATUS_OK
               : MODULE_DEVICE_STATUS_OPERATION_FAILED;
}

static module_device_status_t module_referee_device_update(module_device_t *const device_base,
                                                           uint32_t elapsed_time_ms)
{
    module_referee_t *const me = MODULE_CONTAINER_OF(device_base, module_referee_t, super);
    return (module_referee_update(me, elapsed_time_ms) == MODULE_REFEREE_STATUS_OK)
               ? MODULE_DEVICE_STATUS_OK
               : MODULE_DEVICE_STATUS_OPERATION_FAILED;
}

static const module_device_ops_t s_module_referee_ops = {
    .start = module_referee_device_start,
    .stop = module_referee_device_stop,
    .update = module_referee_device_update,
};

module_referee_status_t module_referee_init(module_referee_t *me,
                                            const module_referee_config_t *config)
{
    size_t route_index;

    if ((me == NULL) || (config == NULL) || (config->usart == NULL) ||
        !bsp_device_is_initialized(&config->usart->super) || (config->receive_buffer == NULL) ||
        (config->receive_capacity == 0U) || (config->processing_buffer == NULL) ||
        (config->processing_capacity < config->receive_capacity) ||
        (config->stream_buffer == NULL) ||
        (config->stream_capacity < MODULE_REFEREE_FRAME_OVERHEAD_SIZE) ||
        (config->transmit_buffer == NULL) ||
        (config->transmit_capacity < MODULE_REFEREE_FRAME_OVERHEAD_SIZE) ||
        !bsp_transfer_mode_is_valid(config->receive_mode) ||
        (config->receive_mode == BSP_TRANSFER_MODE_BLOCKING) ||
        ((config->route_count > 0U) && (config->routes == NULL)) ||
        (config->offline_timeout_ms == 0U))
    {
        return MODULE_REFEREE_STATUS_INVALID_ARGUMENT;
    }
    for (route_index = 0U; route_index < config->route_count; ++route_index)
    {
        size_t comparison_index;
        if (config->routes[route_index].handler == NULL)
        {
            return MODULE_REFEREE_STATUS_INVALID_ARGUMENT;
        }
        for (comparison_index = route_index + 1U; comparison_index < config->route_count;
             ++comparison_index)
        {
            if (config->routes[route_index].command_id ==
                config->routes[comparison_index].command_id)
            {
                return MODULE_REFEREE_STATUS_INVALID_ARGUMENT;
            }
        }
    }
    *me = (module_referee_t){0};
    me->usart = config->usart;
    me->receive_buffer = config->receive_buffer;
    me->receive_capacity = config->receive_capacity;
    me->processing_buffer = config->processing_buffer;
    me->processing_capacity = config->processing_capacity;
    me->stream_buffer = config->stream_buffer;
    me->stream_capacity = config->stream_capacity;
    me->transmit_buffer = config->transmit_buffer;
    me->transmit_capacity = config->transmit_capacity;
    me->routes = config->routes;
    me->route_count = config->route_count;
    me->default_handler = config->default_handler;
    me->default_user_context = config->default_user_context;
    me->receive_timeout_ms = config->receive_timeout_ms;
    me->transmit_timeout_ms = config->transmit_timeout_ms;
    me->offline_timeout_ms = config->offline_timeout_ms;
    me->receive_mode = config->receive_mode;
    if (module_device_init_base(&me->super, &s_module_referee_ops, config->logical_name,
                                config->registration_key) != MODULE_DEVICE_STATUS_OK)
    {
        return MODULE_REFEREE_STATUS_INVALID_ARGUMENT;
    }
    if (module_device_complete_init(&me->super) != MODULE_DEVICE_STATUS_OK)
    {
        module_device_abort_init(&me->super);
        return MODULE_REFEREE_STATUS_INVALID_ARGUMENT;
    }
    return MODULE_REFEREE_STATUS_OK;
}

module_referee_status_t module_referee_start(module_referee_t *me)
{
    if ((me == NULL) || !module_device_is_initialized(&me->super))
    {
        return MODULE_REFEREE_STATUS_NOT_INITIALIZED;
    }
    if (bsp_usart_set_callback(me->usart, module_referee_usart_callback, me) != BSP_STATUS_OK)
    {
        return MODULE_REFEREE_STATUS_TRANSPORT_ERROR;
    }
    if (bsp_usart_receive_to_idle(me->usart, me->receive_buffer, me->receive_capacity,
                                  me->receive_mode, me->receive_timeout_ms) != BSP_STATUS_OK)
    {
        (void)bsp_usart_set_callback(me->usart, NULL, NULL);
        return MODULE_REFEREE_STATUS_TRANSPORT_ERROR;
    }
    me->stream_size = 0U;
    me->receive_elapsed_time_ms = 0U;
    me->is_online = false;
    me->is_receive_pending = false;
    me->pending_receive_size = 0U;
    me->is_transmit_busy = false;
    me->is_started = true;
    return MODULE_REFEREE_STATUS_OK;
}

module_referee_status_t module_referee_stop(module_referee_t *me)
{
    if ((me == NULL) || !module_device_is_initialized(&me->super))
    {
        return MODULE_REFEREE_STATUS_NOT_INITIALIZED;
    }
    (void)bsp_usart_abort(me->usart);
    if (bsp_usart_set_callback(me->usart, NULL, NULL) != BSP_STATUS_OK)
    {
        return MODULE_REFEREE_STATUS_TRANSPORT_ERROR;
    }
    me->is_started = false;
    me->is_online = false;
    me->is_receive_pending = false;
    me->pending_receive_size = 0U;
    me->is_transmit_busy = false;
    return MODULE_REFEREE_STATUS_OK;
}

module_referee_status_t module_referee_feed_data(module_referee_t *me, const uint8_t *receive_data,
                                                 size_t data_size)
{
    if ((me == NULL) || (receive_data == NULL) || (data_size == 0U))
    {
        return MODULE_REFEREE_STATUS_INVALID_ARGUMENT;
    }
    if (!module_device_is_initialized(&me->super))
    {
        return MODULE_REFEREE_STATUS_NOT_INITIALIZED;
    }
    if (data_size > me->stream_capacity - me->stream_size)
    {
        const size_t required_space = data_size - (me->stream_capacity - me->stream_size);
        if (required_space >= me->stream_size)
        {
            me->stream_size = 0U;
        }
        else
        {
            module_referee_discard_stream_prefix(me, required_space);
        }
        module_referee_increment_counter(&me->statistics.oversize_frame_count);
    }
    if (data_size > me->stream_capacity)
    {
        receive_data += data_size - me->stream_capacity;
        data_size = me->stream_capacity;
    }
    memcpy(&me->stream_buffer[me->stream_size], receive_data, data_size);
    me->stream_size += data_size;
    return module_referee_process_stream(me);
}

module_referee_status_t module_referee_build_frame(uint8_t *frame_buffer, size_t frame_capacity,
                                                   uint8_t sequence, uint16_t command_id,
                                                   const uint8_t *payload, size_t payload_size,
                                                   size_t *frame_size)
{
    const size_t required_size = MODULE_REFEREE_FRAME_SIZE(payload_size);

    if ((frame_buffer == NULL) || (frame_size == NULL) ||
        ((payload_size > 0U) && (payload == NULL)) || (payload_size > UINT16_MAX))
    {
        return MODULE_REFEREE_STATUS_INVALID_ARGUMENT;
    }
    if (frame_capacity < required_size)
    {
        return MODULE_REFEREE_STATUS_BUFFER_TOO_SMALL;
    }
    frame_buffer[0] = MODULE_REFEREE_START_OF_FRAME;
    frame_buffer[1] = (uint8_t)payload_size;
    frame_buffer[2] = (uint8_t)(payload_size >> 8U);
    frame_buffer[3] = sequence;
    frame_buffer[4] = 0U;
    (void)module_referee_crc8_append(frame_buffer, MODULE_REFEREE_HEADER_SIZE);
    frame_buffer[5] = (uint8_t)command_id;
    frame_buffer[6] = (uint8_t)(command_id >> 8U);
    if (payload_size > 0U)
    {
        memcpy(&frame_buffer[7], payload, payload_size);
    }
    (void)module_referee_crc16_append(frame_buffer, required_size);
    *frame_size = required_size;
    return MODULE_REFEREE_STATUS_OK;
}

module_referee_status_t module_referee_transmit(module_referee_t *me, uint16_t command_id,
                                                const uint8_t *payload, size_t payload_size,
                                                bsp_transfer_mode_t transfer_mode)
{
    size_t frame_size;
    module_referee_status_t status;

    if (me == NULL)
    {
        return MODULE_REFEREE_STATUS_INVALID_ARGUMENT;
    }
    if (!module_device_is_initialized(&me->super))
    {
        return MODULE_REFEREE_STATUS_NOT_INITIALIZED;
    }
    if (!me->is_started)
    {
        return MODULE_REFEREE_STATUS_NOT_STARTED;
    }
    if (me->is_transmit_busy)
    {
        return MODULE_REFEREE_STATUS_BUSY;
    }
    status = module_referee_build_frame(me->transmit_buffer, me->transmit_capacity,
                                        me->transmit_sequence, command_id, payload, payload_size,
                                        &frame_size);
    if (status != MODULE_REFEREE_STATUS_OK)
    {
        return status;
    }
    me->is_transmit_busy = transfer_mode != BSP_TRANSFER_MODE_BLOCKING;
    if (bsp_usart_transmit(me->usart, me->transmit_buffer, frame_size, transfer_mode,
                           me->transmit_timeout_ms) != BSP_STATUS_OK)
    {
        me->is_transmit_busy = false;
        return MODULE_REFEREE_STATUS_TRANSPORT_ERROR;
    }
    ++me->transmit_sequence;
    return MODULE_REFEREE_STATUS_OK;
}

module_referee_status_t module_referee_update(module_referee_t *me, uint32_t elapsed_time_ms)
{
    if ((me == NULL) || !module_device_is_initialized(&me->super))
    {
        return MODULE_REFEREE_STATUS_NOT_INITIALIZED;
    }
    if (!me->is_started)
    {
        return MODULE_REFEREE_STATUS_NOT_STARTED;
    }
    if (me->is_receive_pending)
    {
        const size_t received_size = me->pending_receive_size;
        (void)module_referee_feed_data(me, me->processing_buffer, received_size);
        me->pending_receive_size = 0U;
        me->is_receive_pending = false;
    }
    if (UINT32_MAX - me->receive_elapsed_time_ms < elapsed_time_ms)
    {
        me->receive_elapsed_time_ms = UINT32_MAX;
    }
    else
    {
        me->receive_elapsed_time_ms += elapsed_time_ms;
    }
    if (me->receive_elapsed_time_ms >= me->offline_timeout_ms)
    {
        me->is_online = false;
    }
    return MODULE_REFEREE_STATUS_OK;
}

bool module_referee_is_online(const module_referee_t *me)
{
    return (me != NULL) && module_device_is_initialized(&me->super) && me->is_started &&
           me->is_online;
}

module_referee_status_t module_referee_get_statistics(const module_referee_t *me,
                                                      module_referee_statistics_t *statistics)
{
    if ((me == NULL) || (statistics == NULL))
    {
        return MODULE_REFEREE_STATUS_INVALID_ARGUMENT;
    }
    if (!module_device_is_initialized(&me->super))
    {
        return MODULE_REFEREE_STATUS_NOT_INITIALIZED;
    }
    *statistics = me->statistics;
    return MODULE_REFEREE_STATUS_OK;
}

uint16_t module_referee_read_uint16_le(const uint8_t *data)
{
    uint16_t value = 0U;

    if (data != NULL)
    {
        value = (uint16_t)((uint16_t)data[0] | (uint16_t)((uint16_t)data[1] << 8U));
    }
    return value;
}

uint32_t module_referee_read_uint32_le(const uint8_t *data)
{
    return (data != NULL) ? (uint32_t)data[0] | ((uint32_t)data[1] << 8U) |
                                ((uint32_t)data[2] << 16U) | ((uint32_t)data[3] << 24U)
                          : 0U;
}

float module_referee_read_float_le(const uint8_t *data)
{
    uint32_t raw_value = module_referee_read_uint32_le(data);
    float value;
    memcpy(&value, &raw_value, sizeof(value));
    return value;
}
