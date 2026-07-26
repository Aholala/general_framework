#include "bsp_can_dispatcher.h"

#include <stddef.h>

static bool bsp_can_dispatcher_route_matches(const bsp_can_route_t *const route,
                                             const bsp_can_frame_t *const frame)
{
    return route->is_enabled && (route->id_type == frame->id_type) &&
           ((frame->identifier & route->mask) == (route->identifier & route->mask));
}

static void bsp_can_dispatcher_event_callback(bsp_event_t event, bsp_status_t status,
                                              size_t transferred_size, void *user_context)
{
    bsp_can_dispatcher_t *const me = (bsp_can_dispatcher_t *)user_context;
    (void)transferred_size;

    if (me == NULL)
    {
        return;
    }
    if ((event == BSP_EVENT_RECEIVE_COMPLETE) || (event == BSP_EVENT_RECEIVE_PENDING))
    {
        me->receive_pending = true;
    }
    if ((event == BSP_EVENT_ERROR) || (status != BSP_STATUS_OK))
    {
        ++me->receive_error_count;
    }
}

bsp_status_t bsp_can_dispatcher_init(bsp_can_dispatcher_t *const me,
                                     const bsp_can_dispatcher_config_t *const config)
{
    size_t route_index;
    bsp_status_t status;

    if ((me == NULL) || (config == NULL) || (config->can == NULL) ||
        !bsp_device_is_initialized(&config->can->super) || (config->route_storage == NULL) ||
        (config->route_capacity == 0U) || (config->maximum_frames_per_process == 0U) ||
        ((config->receive_fifo != BSP_CAN_RX_FIFO_0) &&
         (config->receive_fifo != BSP_CAN_RX_FIFO_1)))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }

    me->is_initialized = false;
    me->can = config->can;
    me->receive_fifo = config->receive_fifo;
    me->route_storage = config->route_storage;
    me->route_capacity = config->route_capacity;
    me->route_count = 0U;
    me->maximum_frames_per_process = config->maximum_frames_per_process;
    me->received_frame_count = 0U;
    me->unmatched_frame_count = 0U;
    me->receive_error_count = 0U;
    me->receive_pending = false;
    me->is_processing = false;
    for (route_index = 0U; route_index < me->route_capacity; ++route_index)
    {
        me->route_storage[route_index] = (bsp_can_route_t){0};
    }

    status = bsp_can_set_callback(config->can, bsp_can_dispatcher_event_callback, me);
    if (status == BSP_STATUS_OK)
    {
        me->is_initialized = true;
    }
    return status;
}

bsp_status_t bsp_can_dispatcher_remove_route(bsp_can_dispatcher_t *const me,
                                             size_t route_index)
{
    size_t move_index;

    if (me == NULL)
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return BSP_STATUS_NOT_INITIALIZED;
    }
    if (me->is_processing)
    {
        return BSP_STATUS_BUSY;
    }
    if (route_index >= me->route_count)
    {
        return BSP_STATUS_OUT_OF_RANGE;
    }
    for (move_index = route_index; (move_index + 1U) < me->route_count;
         ++move_index)
    {
        me->route_storage[move_index] =
            me->route_storage[move_index + 1U];
    }
    --me->route_count;
    me->route_storage[me->route_count] = (bsp_can_route_t){0};
    return BSP_STATUS_OK;
}

bsp_status_t bsp_can_dispatcher_clear_routes(bsp_can_dispatcher_t *const me)
{
    size_t route_index;

    if (me == NULL)
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return BSP_STATUS_NOT_INITIALIZED;
    }
    if (me->is_processing)
    {
        return BSP_STATUS_BUSY;
    }
    for (route_index = 0U; route_index < me->route_count; ++route_index)
    {
        me->route_storage[route_index] = (bsp_can_route_t){0};
    }
    me->route_count = 0U;
    return BSP_STATUS_OK;
}

bsp_status_t bsp_can_dispatcher_deinit(bsp_can_dispatcher_t *const me)
{
    bsp_status_t status;

    if (me == NULL)
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return BSP_STATUS_NOT_INITIALIZED;
    }
    if (me->is_processing)
    {
        return BSP_STATUS_BUSY;
    }
    status = bsp_can_set_callback(me->can, NULL, NULL);
    if (status != BSP_STATUS_OK)
    {
        return status;
    }
    (void)bsp_can_dispatcher_clear_routes(me);
    me->receive_pending = false;
    me->is_initialized = false;
    me->can = NULL;
    return BSP_STATUS_OK;
}

bsp_status_t bsp_can_dispatcher_add_route(bsp_can_dispatcher_t *const me, uint32_t identifier,
                                          uint32_t mask, bsp_can_id_type_t id_type,
                                          bsp_can_frame_callback_t callback, void *user_context,
                                          size_t *route_index)
{
    bsp_can_route_t *route;

    if ((me == NULL) || (callback == NULL))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return BSP_STATUS_NOT_INITIALIZED;
    }
    if ((id_type > BSP_CAN_ID_EXTENDED) ||
        ((id_type == BSP_CAN_ID_STANDARD) && ((identifier > 0x7FFU) || (mask > 0x7FFU))) ||
        ((id_type == BSP_CAN_ID_EXTENDED) && ((identifier > 0x1FFFFFFFU) || (mask > 0x1FFFFFFFU))))
    {
        return BSP_STATUS_OUT_OF_RANGE;
    }
    if (me->route_count >= me->route_capacity)
    {
        return BSP_STATUS_NO_RESOURCE;
    }

    route = &me->route_storage[me->route_count];
    *route = (bsp_can_route_t){
        .identifier = identifier,
        .mask = mask,
        .id_type = id_type,
        .callback = callback,
        .user_context = user_context,
        .is_enabled = true,
    };
    if (route_index != NULL)
    {
        *route_index = me->route_count;
    }
    ++me->route_count;
    return BSP_STATUS_OK;
}

bsp_status_t bsp_can_dispatcher_set_route_enabled(bsp_can_dispatcher_t *const me,
                                                  size_t route_index, bool is_enabled)
{
    if (me == NULL)
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return BSP_STATUS_NOT_INITIALIZED;
    }
    if (route_index >= me->route_count)
    {
        return BSP_STATUS_OUT_OF_RANGE;
    }
    me->route_storage[route_index].is_enabled = is_enabled;
    return BSP_STATUS_OK;
}

bsp_status_t bsp_can_dispatcher_process(bsp_can_dispatcher_t *const me,
                                        size_t *processed_frame_count)
{
    size_t frame_index;
    size_t route_index;
    size_t processed_count = 0U;
    bsp_status_t receive_status = BSP_STATUS_OK;

    if (me == NULL)
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return BSP_STATUS_NOT_INITIALIZED;
    }
    if (me->is_processing)
    {
        return BSP_STATUS_BUSY;
    }

    me->is_processing = true;
    me->receive_pending = false;
    for (frame_index = 0U; frame_index < me->maximum_frames_per_process; ++frame_index)
    {
        bsp_can_frame_t frame;
        bool was_matched = false;

        receive_status = bsp_can_receive(me->can, me->receive_fifo, &frame);
        if (receive_status != BSP_STATUS_OK)
        {
            break;
        }
        ++processed_count;
        ++me->received_frame_count;
        for (route_index = 0U; route_index < me->route_count; ++route_index)
        {
            const bsp_can_route_t *const route = &me->route_storage[route_index];
            if (bsp_can_dispatcher_route_matches(route, &frame))
            {
                route->callback(&frame, route->user_context);
                was_matched = true;
            }
        }
        if (!was_matched)
        {
            ++me->unmatched_frame_count;
        }
    }

    if (processed_frame_count != NULL)
    {
        *processed_frame_count = processed_count;
    }
    if ((processed_count > 0U) && (processed_count == me->maximum_frames_per_process))
    {
        me->receive_pending = true;
    }
    me->is_processing = false;
    return (processed_count > 0U) ? BSP_STATUS_OK : receive_status;
}

bool bsp_can_dispatcher_has_pending_receive(const bsp_can_dispatcher_t *const me)
{
    return (me != NULL) && me->is_initialized && me->receive_pending;
}
