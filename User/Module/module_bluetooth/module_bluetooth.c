#include "module_bluetooth.h"

#include <stddef.h>
#include <string.h>

static void module_bluetooth_usart_callback(bsp_event_t event, bsp_status_t status,
                                            size_t transferred_size, void *user_context)
{
    module_bluetooth_t *const me = (module_bluetooth_t *)user_context;

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
                if (me->receive_overrun_count != UINT32_MAX)
                {
                    ++me->receive_overrun_count;
                }
            }
        }
        else
        {
            if (me->receive_overrun_count != UINT32_MAX)
            {
                ++me->receive_overrun_count;
            }
        }
        if (me->is_started &&
            (bsp_usart_receive_to_idle(me->usart, me->receive_buffer, me->receive_capacity,
                                       me->receive_mode, me->receive_timeout_ms) != BSP_STATUS_OK))
        {
            if (me->receive_restart_error_count != UINT32_MAX)
            {
                ++me->receive_restart_error_count;
            }
        }
    }
}

static module_device_status_t module_bluetooth_device_start(module_device_t *const device_base)
{
    module_bluetooth_t *const me = MODULE_CONTAINER_OF(device_base, module_bluetooth_t, super);
    return (module_bluetooth_start(me) == MODULE_BLUETOOTH_STATUS_OK)
               ? MODULE_DEVICE_STATUS_OK
               : MODULE_DEVICE_STATUS_OPERATION_FAILED;
}

static module_device_status_t module_bluetooth_device_stop(module_device_t *const device_base)
{
    module_bluetooth_t *const me = MODULE_CONTAINER_OF(device_base, module_bluetooth_t, super);
    return (module_bluetooth_stop(me) == MODULE_BLUETOOTH_STATUS_OK)
               ? MODULE_DEVICE_STATUS_OK
               : MODULE_DEVICE_STATUS_OPERATION_FAILED;
}

static module_device_status_t module_bluetooth_device_update(module_device_t *const device_base,
                                                             uint32_t elapsed_time_ms)
{
    module_bluetooth_t *const me = MODULE_CONTAINER_OF(device_base, module_bluetooth_t, super);
    return (module_bluetooth_update(me, elapsed_time_ms) == MODULE_BLUETOOTH_STATUS_OK)
               ? MODULE_DEVICE_STATUS_OK
               : MODULE_DEVICE_STATUS_OPERATION_FAILED;
}

static const module_device_ops_t s_module_bluetooth_ops = {
    .start = module_bluetooth_device_start,
    .stop = module_bluetooth_device_stop,
    .update = module_bluetooth_device_update,
};

module_bluetooth_status_t module_bluetooth_init(module_bluetooth_t *me,
                                                const module_bluetooth_config_t *config)
{
    if ((me == NULL) || (config == NULL) || (config->usart == NULL) ||
        !bsp_device_is_initialized(&config->usart->super) || (config->receive_buffer == NULL) ||
        (config->receive_capacity == 0U) || (config->processing_buffer == NULL) ||
        (config->processing_capacity < config->receive_capacity) ||
        !bsp_transfer_mode_is_valid(config->receive_mode) ||
        (config->receive_mode == BSP_TRANSFER_MODE_BLOCKING) || (config->offline_timeout_ms == 0U))
    {
        return MODULE_BLUETOOTH_STATUS_INVALID_ARGUMENT;
    }
    *me = (module_bluetooth_t){0};
    me->usart = config->usart;
    me->receive_buffer = config->receive_buffer;
    me->receive_capacity = config->receive_capacity;
    me->processing_buffer = config->processing_buffer;
    me->processing_capacity = config->processing_capacity;
    me->transmit_timeout_ms = config->transmit_timeout_ms;
    me->receive_timeout_ms = config->receive_timeout_ms;
    me->offline_timeout_ms = config->offline_timeout_ms;
    me->receive_mode = config->receive_mode;
    me->receive_callback = config->receive_callback;
    me->user_context = config->user_context;
    if (module_device_init_base(&me->super, &s_module_bluetooth_ops, config->logical_name,
                                config->registration_key) != MODULE_DEVICE_STATUS_OK)
    {
        return MODULE_BLUETOOTH_STATUS_INVALID_ARGUMENT;
    }
    if (module_device_complete_init(&me->super) != MODULE_DEVICE_STATUS_OK)
    {
        module_device_abort_init(&me->super);
        return MODULE_BLUETOOTH_STATUS_INVALID_ARGUMENT;
    }
    return MODULE_BLUETOOTH_STATUS_OK;
}

module_bluetooth_status_t module_bluetooth_start(module_bluetooth_t *me)
{
    if ((me == NULL) || !module_device_is_initialized(&me->super))
    {
        return MODULE_BLUETOOTH_STATUS_NOT_INITIALIZED;
    }
    if (bsp_usart_set_callback(me->usart, module_bluetooth_usart_callback, me) != BSP_STATUS_OK)
    {
        return MODULE_BLUETOOTH_STATUS_TRANSPORT_ERROR;
    }
    if (bsp_usart_receive_to_idle(me->usart, me->receive_buffer, me->receive_capacity,
                                  me->receive_mode, me->receive_timeout_ms) != BSP_STATUS_OK)
    {
        (void)bsp_usart_set_callback(me->usart, NULL, NULL);
        return MODULE_BLUETOOTH_STATUS_TRANSPORT_ERROR;
    }
    me->receive_elapsed_time_ms = 0U;
    me->is_online = false;
    me->is_receive_pending = false;
    me->pending_receive_size = 0U;
    me->is_started = true;
    return MODULE_BLUETOOTH_STATUS_OK;
}

module_bluetooth_status_t module_bluetooth_stop(module_bluetooth_t *me)
{
    if ((me == NULL) || !module_device_is_initialized(&me->super))
    {
        return MODULE_BLUETOOTH_STATUS_NOT_INITIALIZED;
    }
    (void)bsp_usart_abort(me->usart);
    if (bsp_usart_set_callback(me->usart, NULL, NULL) != BSP_STATUS_OK)
    {
        return MODULE_BLUETOOTH_STATUS_TRANSPORT_ERROR;
    }
    me->is_started = false;
    me->is_online = false;
    me->is_receive_pending = false;
    me->pending_receive_size = 0U;
    return MODULE_BLUETOOTH_STATUS_OK;
}

module_bluetooth_status_t module_bluetooth_transmit(module_bluetooth_t *me,
                                                    const uint8_t *transmit_data, size_t data_size,
                                                    bsp_transfer_mode_t transfer_mode)
{
    if ((me == NULL) || (transmit_data == NULL) || (data_size == 0U))
    {
        return MODULE_BLUETOOTH_STATUS_INVALID_ARGUMENT;
    }
    if (!module_device_is_initialized(&me->super))
    {
        return MODULE_BLUETOOTH_STATUS_NOT_INITIALIZED;
    }
    if (!me->is_started)
    {
        return MODULE_BLUETOOTH_STATUS_NOT_STARTED;
    }
    return (bsp_usart_transmit(me->usart, transmit_data, data_size, transfer_mode,
                               me->transmit_timeout_ms) == BSP_STATUS_OK)
               ? MODULE_BLUETOOTH_STATUS_OK
               : MODULE_BLUETOOTH_STATUS_TRANSPORT_ERROR;
}

module_bluetooth_status_t module_bluetooth_send_command(module_bluetooth_t *me, const char *command)
{
    if (command == NULL)
    {
        return MODULE_BLUETOOTH_STATUS_INVALID_ARGUMENT;
    }
    return module_bluetooth_transmit(me, (const uint8_t *)command, strlen(command),
                                     BSP_TRANSFER_MODE_BLOCKING);
}

module_bluetooth_status_t module_bluetooth_update(module_bluetooth_t *me, uint32_t elapsed_time_ms)
{
    if ((me == NULL) || !module_device_is_initialized(&me->super))
    {
        return MODULE_BLUETOOTH_STATUS_NOT_INITIALIZED;
    }
    if (!me->is_started)
    {
        return MODULE_BLUETOOTH_STATUS_NOT_STARTED;
    }
    if (me->is_receive_pending)
    {
        const size_t received_size = me->pending_receive_size;
        me->receive_elapsed_time_ms = 0U;
        me->is_online = true;
        if ((me->receive_callback != NULL) && (received_size > 0U))
        {
            me->receive_callback(me->processing_buffer, received_size, me->user_context);
        }
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
    return MODULE_BLUETOOTH_STATUS_OK;
}

bool module_bluetooth_is_online(const module_bluetooth_t *me)
{
    return (me != NULL) && module_device_is_initialized(&me->super) && me->is_started &&
           me->is_online;
}
