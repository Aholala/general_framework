#include "module_dm_motor_bus.h"

module_motor_status_t module_dm_motor_bus_init(module_dm_motor_bus_t *me, bsp_can_t *can,
                                               module_dm_motor_t **motor_storage,
                                               size_t motor_capacity,
                                               size_t maximum_transmits_per_cycle)
{
    size_t motor_index;
    if ((me == NULL) || (can == NULL) || !bsp_device_is_initialized(&can->super) ||
        (motor_storage == NULL) || (motor_capacity == 0U) || (maximum_transmits_per_cycle == 0U))
    {
        return MODULE_MOTOR_STATUS_INVALID_ARGUMENT;
    }
    for (motor_index = 0U; motor_index < motor_capacity; ++motor_index)
    {
        motor_storage[motor_index] = NULL;
    }
    *me = (module_dm_motor_bus_t){
        .can = can,
        .motor_storage = motor_storage,
        .motor_capacity = motor_capacity,
        .maximum_transmits_per_cycle = maximum_transmits_per_cycle,
        .is_initialized = true,
    };
    return MODULE_MOTOR_STATUS_OK;
}

module_motor_status_t module_dm_motor_bus_register(module_dm_motor_bus_t *me,
                                                   module_dm_motor_t *motor)
{
    size_t motor_index;
    if ((me == NULL) || (motor == NULL) || !me->is_initialized || !motor->super.is_initialized ||
        !motor->super.is_registered || (motor->can != me->can))
    {
        return MODULE_MOTOR_STATUS_INVALID_ARGUMENT;
    }
    for (motor_index = 0U; motor_index < me->motor_count; ++motor_index)
    {
        if ((me->motor_storage[motor_index] == motor) ||
            (me->motor_storage[motor_index]->feedback_identifier == motor->feedback_identifier) ||
            (me->motor_storage[motor_index]->mode_vptr->get_transmit_identifier(
                 me->motor_storage[motor_index]) ==
             motor->mode_vptr->get_transmit_identifier(motor)))
        {
            return MODULE_MOTOR_STATUS_DUPLICATE_KEY;
        }
    }
    if (me->motor_count >= me->motor_capacity)
    {
        return MODULE_MOTOR_STATUS_NO_RESOURCE;
    }
    me->motor_storage[me->motor_count] = motor;
    ++me->motor_count;
    return MODULE_MOTOR_STATUS_OK;
}

module_motor_status_t module_dm_motor_bus_unregister(module_dm_motor_bus_t *me,
                                                     module_dm_motor_t *motor)
{
    size_t motor_index;
    if ((me == NULL) || (motor == NULL) || !me->is_initialized)
    {
        return MODULE_MOTOR_STATUS_INVALID_ARGUMENT;
    }
    for (motor_index = 0U; motor_index < me->motor_count; ++motor_index)
    {
        if (me->motor_storage[motor_index] == motor)
        {
            size_t move_index;
            for (move_index = motor_index; move_index + 1U < me->motor_count; ++move_index)
            {
                me->motor_storage[move_index] = me->motor_storage[move_index + 1U];
            }
            --me->motor_count;
            me->motor_storage[me->motor_count] = NULL;
            if (me->next_transmit_index >= me->motor_count)
            {
                me->next_transmit_index = 0U;
            }
            return MODULE_MOTOR_STATUS_OK;
        }
    }
    return MODULE_MOTOR_STATUS_NOT_REGISTERED;
}

module_motor_status_t module_dm_motor_bus_handle_feedback(module_dm_motor_bus_t *me,
                                                          const bsp_can_frame_t *frame)
{
    size_t motor_index;
    if ((me == NULL) || (frame == NULL) || !me->is_initialized)
    {
        return MODULE_MOTOR_STATUS_INVALID_ARGUMENT;
    }
    for (motor_index = 0U; motor_index < me->motor_count; ++motor_index)
    {
        if (me->motor_storage[motor_index]->feedback_identifier == frame->identifier)
        {
            const module_motor_status_t status =
                module_dm_motor_handle_feedback(me->motor_storage[motor_index], frame);
            if (status == MODULE_MOTOR_STATUS_OK)
            {
                ++me->routed_frame_count;
            }
            return status;
        }
    }
    ++me->unknown_frame_count;
    return MODULE_MOTOR_STATUS_FEEDBACK_UNAVAILABLE;
}

module_motor_status_t module_dm_motor_bus_update(module_dm_motor_bus_t *me, float delta_time_s)
{
    size_t transmit_count;
    bool had_error = false;
    if ((me == NULL) || !me->is_initialized || (delta_time_s <= 0.0F))
    {
        return MODULE_MOTOR_STATUS_INVALID_ARGUMENT;
    }
    if (me->motor_count == 0U)
    {
        return MODULE_MOTOR_STATUS_OK;
    }
    for (transmit_count = 0U;
         (transmit_count < me->maximum_transmits_per_cycle) && (transmit_count < me->motor_count);
         ++transmit_count)
    {
        module_dm_motor_t *const motor = me->motor_storage[me->next_transmit_index];
        const module_motor_status_t status = module_motor_update(&motor->super, delta_time_s);
        me->next_transmit_index = (me->next_transmit_index + 1U) % me->motor_count;
        if (status != MODULE_MOTOR_STATUS_OK)
        {
            ++me->transmit_error_count;
            had_error = true;
        }
    }
    return had_error ? MODULE_MOTOR_STATUS_TRANSPORT_ERROR : MODULE_MOTOR_STATUS_OK;
}
