#include "module_motor.h"

#include <math.h>
#include <stddef.h>

static module_motor_status_t module_motor_validate_registered(const module_motor_t *const me)
{
    if (me == NULL)
    {
        return MODULE_MOTOR_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized || (me->vptr == NULL))
    {
        return MODULE_MOTOR_STATUS_NOT_INITIALIZED;
    }
    return me->is_registered ? MODULE_MOTOR_STATUS_OK : MODULE_MOTOR_STATUS_NOT_REGISTERED;
}

static module_motor_status_t module_motor_enter_feedback_fault(module_motor_t *const me)
{
    module_motor_status_t status;

    if (me->state != MODULE_MOTOR_STATE_ENABLED)
    {
        return MODULE_MOTOR_STATUS_FEEDBACK_UNAVAILABLE;
    }
    status = me->vptr->disable(me);
    me->state = MODULE_MOTOR_STATE_FAULT;
    return (status == MODULE_MOTOR_STATUS_OK) ? MODULE_MOTOR_STATUS_FEEDBACK_UNAVAILABLE : status;
}

module_motor_status_t module_motor_init_base(module_motor_t *const me,
                                             const module_motor_ops_t *const vptr,
                                             const char *const logical_name,
                                             uint32_t registration_key)
{
    if ((me == NULL) || (vptr == NULL) || (logical_name == NULL) || (vptr->enable == NULL) ||
        (vptr->disable == NULL) || (vptr->set_target == NULL) || (vptr->update == NULL))
    {
        return MODULE_MOTOR_STATUS_INVALID_ARGUMENT;
    }

    me->vptr = vptr;
    me->logical_name = logical_name;
    me->registration_key = registration_key;
    me->registry_index = SIZE_MAX;
    me->state = MODULE_MOTOR_STATE_DISABLED;
    me->feedback = (module_motor_feedback_t){0};
    me->feedback_timeout_ms = 0U;
    me->is_registered = false;
    me->is_initialized = true;
    return MODULE_MOTOR_STATUS_OK;
}

module_motor_status_t module_motor_registry_init(module_motor_registry_t *const me,
                                                 module_motor_t **const motor_storage,
                                                 size_t motor_capacity)
{
    size_t motor_index;

    if ((me == NULL) || (motor_storage == NULL) || (motor_capacity == 0U))
    {
        return MODULE_MOTOR_STATUS_INVALID_ARGUMENT;
    }
    for (motor_index = 0U; motor_index < motor_capacity; ++motor_index)
    {
        motor_storage[motor_index] = NULL;
    }
    me->motor_storage = motor_storage;
    me->motor_capacity = motor_capacity;
    me->motor_count = 0U;
    me->is_initialized = true;
    return MODULE_MOTOR_STATUS_OK;
}

module_motor_status_t module_motor_registry_register(module_motor_registry_t *const me,
                                                     module_motor_t *const motor)
{
    size_t motor_index;

    if ((me == NULL) || (motor == NULL))
    {
        return MODULE_MOTOR_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized || !motor->is_initialized)
    {
        return MODULE_MOTOR_STATUS_NOT_INITIALIZED;
    }
    if (motor->is_registered)
    {
        return MODULE_MOTOR_STATUS_ALREADY_REGISTERED;
    }
    if (me->motor_count >= me->motor_capacity)
    {
        return MODULE_MOTOR_STATUS_NO_RESOURCE;
    }
    for (motor_index = 0U; motor_index < me->motor_count; ++motor_index)
    {
        if (me->motor_storage[motor_index]->registration_key == motor->registration_key)
        {
            return MODULE_MOTOR_STATUS_DUPLICATE_KEY;
        }
    }

    motor->registry_index = me->motor_count;
    me->motor_storage[me->motor_count] = motor;
    ++me->motor_count;
    motor->is_registered = true;
    return MODULE_MOTOR_STATUS_OK;
}

module_motor_status_t module_motor_registry_unregister(module_motor_registry_t *const me,
                                                       module_motor_t *const motor)
{
    size_t motor_index;

    if ((me == NULL) || (motor == NULL))
    {
        return MODULE_MOTOR_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized || !motor->is_registered ||
        (motor->registry_index >= me->motor_count) ||
        (me->motor_storage[motor->registry_index] != motor))
    {
        return MODULE_MOTOR_STATUS_NOT_REGISTERED;
    }

    for (motor_index = motor->registry_index; (motor_index + 1U) < me->motor_count; ++motor_index)
    {
        me->motor_storage[motor_index] = me->motor_storage[motor_index + 1U];
        me->motor_storage[motor_index]->registry_index = motor_index;
    }
    --me->motor_count;
    me->motor_storage[me->motor_count] = NULL;
    motor->registry_index = SIZE_MAX;
    motor->is_registered = false;
    motor->state = MODULE_MOTOR_STATE_DISABLED;
    return MODULE_MOTOR_STATUS_OK;
}

module_motor_t *module_motor_registry_find(const module_motor_registry_t *const me,
                                           uint32_t registration_key)
{
    size_t motor_index;

    if ((me == NULL) || !me->is_initialized)
    {
        return NULL;
    }
    for (motor_index = 0U; motor_index < me->motor_count; ++motor_index)
    {
        if (me->motor_storage[motor_index]->registration_key == registration_key)
        {
            return me->motor_storage[motor_index];
        }
    }
    return NULL;
}

size_t module_motor_registry_get_count(const module_motor_registry_t *const me)
{
    return ((me != NULL) && me->is_initialized) ? me->motor_count : 0U;
}

module_motor_status_t module_motor_enable(module_motor_t *const me)
{
    module_motor_status_t status = module_motor_validate_registered(me);
    if ((status == MODULE_MOTOR_STATUS_OK) && (me->state == MODULE_MOTOR_STATE_FAULT))
    {
        return MODULE_MOTOR_STATUS_FEEDBACK_UNAVAILABLE;
    }
    if ((status == MODULE_MOTOR_STATUS_OK) && !me->feedback.is_online)
    {
        return MODULE_MOTOR_STATUS_FEEDBACK_UNAVAILABLE;
    }
    return (status == MODULE_MOTOR_STATUS_OK) ? me->vptr->enable(me) : status;
}

module_motor_status_t module_motor_disable(module_motor_t *const me)
{
    module_motor_status_t status = module_motor_validate_registered(me);
    return (status == MODULE_MOTOR_STATUS_OK) ? me->vptr->disable(me) : status;
}

module_motor_status_t module_motor_clear_fault(module_motor_t *const me)
{
    module_motor_status_t status = module_motor_validate_registered(me);

    if (status != MODULE_MOTOR_STATUS_OK)
    {
        return status;
    }
    if (!me->feedback.is_online)
    {
        return MODULE_MOTOR_STATUS_FEEDBACK_UNAVAILABLE;
    }
    if (me->state == MODULE_MOTOR_STATE_FAULT)
    {
        me->state = MODULE_MOTOR_STATE_DISABLED;
    }
    return MODULE_MOTOR_STATUS_OK;
}

module_motor_status_t module_motor_set_target(module_motor_t *const me, float target_value)
{
    module_motor_status_t status = module_motor_validate_registered(me);
    return (status == MODULE_MOTOR_STATUS_OK) ? me->vptr->set_target(me, target_value) : status;
}

module_motor_status_t module_motor_update(module_motor_t *const me, float delta_time_s)
{
    module_motor_status_t status = module_motor_validate_registered(me);
    if ((status == MODULE_MOTOR_STATUS_OK) && (me->state == MODULE_MOTOR_STATE_ENABLED) &&
        !me->feedback.is_online)
    {
        return module_motor_enter_feedback_fault(me);
    }
    if ((status == MODULE_MOTOR_STATUS_OK) && (!isfinite(delta_time_s) || (delta_time_s <= 0.0F)))
    {
        return MODULE_MOTOR_STATUS_OUT_OF_RANGE;
    }
    return (status == MODULE_MOTOR_STATUS_OK) ? me->vptr->update(me, delta_time_s) : status;
}

module_motor_status_t module_motor_set_feedback_timeout(module_motor_t *const me,
                                                        uint32_t feedback_timeout_ms)
{
    if (me == NULL)
    {
        return MODULE_MOTOR_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return MODULE_MOTOR_STATUS_NOT_INITIALIZED;
    }
    me->feedback_timeout_ms = feedback_timeout_ms;
    return MODULE_MOTOR_STATUS_OK;
}

module_motor_status_t module_motor_update_feedback_time(module_motor_t *const me,
                                                        uint32_t elapsed_time_ms)
{
    if (me == NULL)
    {
        return MODULE_MOTOR_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return MODULE_MOTOR_STATUS_NOT_INITIALIZED;
    }
    if (!me->feedback.is_online)
    {
        return MODULE_MOTOR_STATUS_OK;
    }

    if (elapsed_time_ms > (UINT32_MAX - me->feedback.elapsed_time_since_update_ms))
    {
        me->feedback.elapsed_time_since_update_ms = UINT32_MAX;
    }
    else
    {
        me->feedback.elapsed_time_since_update_ms += elapsed_time_ms;
    }
    if ((me->feedback_timeout_ms > 0U) &&
        (me->feedback.elapsed_time_since_update_ms >= me->feedback_timeout_ms))
    {
        me->feedback.is_online = false;
        if (me->state == MODULE_MOTOR_STATE_ENABLED)
        {
            return module_motor_enter_feedback_fault(me);
        }
    }
    return MODULE_MOTOR_STATUS_OK;
}

module_motor_status_t module_motor_notify_feedback(module_motor_t *const me)
{
    if (me == NULL)
    {
        return MODULE_MOTOR_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return MODULE_MOTOR_STATUS_NOT_INITIALIZED;
    }
    me->feedback.elapsed_time_since_update_ms = 0U;
    me->feedback.is_online = true;
    if (me->feedback.update_count != UINT32_MAX)
    {
        ++me->feedback.update_count;
    }
    return MODULE_MOTOR_STATUS_OK;
}

const module_motor_feedback_t *module_motor_get_feedback(const module_motor_t *const me)
{
    return (module_motor_validate_registered(me) == MODULE_MOTOR_STATUS_OK) ? &me->feedback : NULL;
}
