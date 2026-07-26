#include "module_motor_health.h"

#include <math.h>
#include <stddef.h>

static uint32_t module_motor_health_add_time(uint32_t accumulated_time_ms,
                                             uint32_t elapsed_time_ms)
{
    return (elapsed_time_ms > (UINT32_MAX - accumulated_time_ms))
               ? UINT32_MAX
               : accumulated_time_ms + elapsed_time_ms;
}

static uint32_t module_motor_health_evaluate_reason(
    const module_motor_health_t *me, size_t motor_index)
{
    const module_motor_t *const motor = me->motors[motor_index];
    const module_motor_feedback_t *feedback;
    uint32_t reason_mask = MODULE_MOTOR_HEALTH_REASON_NONE;

    if (!motor->is_registered)
    {
        return MODULE_MOTOR_HEALTH_REASON_NOT_REGISTERED;
    }
    feedback = module_motor_get_feedback(motor);
    if ((feedback == NULL) || !feedback->is_online)
    {
        reason_mask |= MODULE_MOTOR_HEALTH_REASON_OFFLINE;
    }
    if (motor->state == MODULE_MOTOR_STATE_FAULT)
    {
        reason_mask |= MODULE_MOTOR_HEALTH_REASON_MOTOR_FAULT;
    }
    if (me->require_enabled_state &&
        (motor->state != MODULE_MOTOR_STATE_ENABLED))
    {
        reason_mask |= MODULE_MOTOR_HEALTH_REASON_NOT_ENABLED;
    }
    if ((feedback != NULL) && (me->maximum_temperature_c != NULL) &&
        (feedback->motor_temperature_c >
         me->maximum_temperature_c[motor_index]))
    {
        reason_mask |= MODULE_MOTOR_HEALTH_REASON_OVER_TEMPERATURE;
    }
    return reason_mask;
}

module_motor_health_status_t module_motor_health_init(
    module_motor_health_t *me,
    const module_motor_health_config_t *config)
{
    size_t motor_index;

    if ((me == NULL) || (config == NULL) ||
        (config->motors == NULL) || (config->motor_count == 0U) ||
        (config->state_storage == NULL))
    {
        return MODULE_MOTOR_HEALTH_STATUS_INVALID_ARGUMENT;
    }
    me->is_initialized = false;
    for (motor_index = 0U; motor_index < config->motor_count;
         ++motor_index)
    {
        if ((config->motors[motor_index] == NULL) ||
            !config->motors[motor_index]->is_initialized ||
            ((config->maximum_temperature_c != NULL) &&
             (!isfinite(config->maximum_temperature_c[motor_index]) ||
              (config->maximum_temperature_c[motor_index] <= 0.0F))))
        {
            return MODULE_MOTOR_HEALTH_STATUS_INVALID_ARGUMENT;
        }
        config->state_storage[motor_index] =
            (module_motor_health_state_t){
                .reason_mask =
                    MODULE_MOTOR_HEALTH_REASON_NOT_REGISTERED,
            };
    }
    me->motors = config->motors;
    me->motor_count = config->motor_count;
    me->states = config->state_storage;
    me->maximum_temperature_c = config->maximum_temperature_c;
    me->fault_confirmation_time_ms =
        config->fault_confirmation_time_ms;
    me->recovery_confirmation_time_ms =
        config->recovery_confirmation_time_ms;
    me->require_enabled_state = config->require_enabled_state;
    me->manage_feedback_time = config->manage_feedback_time;
    me->is_initialized = true;
    return MODULE_MOTOR_HEALTH_STATUS_OK;
}

module_motor_health_status_t module_motor_health_update(
    module_motor_health_t *me, uint32_t elapsed_time_ms)
{
    size_t motor_index;
    size_t available_motor_count = 0U;

    if (me == NULL)
    {
        return MODULE_MOTOR_HEALTH_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return MODULE_MOTOR_HEALTH_STATUS_NOT_INITIALIZED;
    }
    for (motor_index = 0U; motor_index < me->motor_count; ++motor_index)
    {
        module_motor_health_state_t *const health_state =
            &me->states[motor_index];
        uint32_t reason_mask;

        if (me->manage_feedback_time &&
            (module_motor_update_feedback_time(
                 me->motors[motor_index], elapsed_time_ms) !=
             MODULE_MOTOR_STATUS_OK))
        {
            return MODULE_MOTOR_HEALTH_STATUS_MOTOR_ERROR;
        }
        reason_mask =
            module_motor_health_evaluate_reason(me, motor_index);
        if (reason_mask == MODULE_MOTOR_HEALTH_REASON_NONE)
        {
            health_state->fault_elapsed_time_ms = 0U;
            health_state->recovery_elapsed_time_ms =
                module_motor_health_add_time(
                    health_state->recovery_elapsed_time_ms,
                    elapsed_time_ms);
            if (health_state->recovery_elapsed_time_ms >=
                me->recovery_confirmation_time_ms)
            {
                health_state->is_available = true;
                health_state->reason_mask =
                    MODULE_MOTOR_HEALTH_REASON_NONE;
            }
        }
        else
        {
            health_state->recovery_elapsed_time_ms = 0U;
            health_state->reason_mask = reason_mask;
            health_state->fault_elapsed_time_ms =
                module_motor_health_add_time(
                    health_state->fault_elapsed_time_ms,
                    elapsed_time_ms);
            if (health_state->fault_elapsed_time_ms >=
                me->fault_confirmation_time_ms)
            {
                health_state->is_available = false;
            }
        }
        if (health_state->is_available)
        {
            ++available_motor_count;
        }
    }
    return (available_motor_count == me->motor_count)
               ? MODULE_MOTOR_HEALTH_STATUS_OK
               : MODULE_MOTOR_HEALTH_STATUS_DEGRADED;
}

module_motor_health_status_t module_motor_health_get_availability(
    const module_motor_health_t *me, bool *motor_is_available,
    size_t output_capacity)
{
    size_t motor_index;
    size_t available_motor_count = 0U;

    if ((me == NULL) || (motor_is_available == NULL))
    {
        return MODULE_MOTOR_HEALTH_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return MODULE_MOTOR_HEALTH_STATUS_NOT_INITIALIZED;
    }
    if (output_capacity < me->motor_count)
    {
        return MODULE_MOTOR_HEALTH_STATUS_INVALID_ARGUMENT;
    }
    for (motor_index = 0U; motor_index < me->motor_count; ++motor_index)
    {
        motor_is_available[motor_index] =
            me->states[motor_index].is_available;
        if (motor_is_available[motor_index])
        {
            ++available_motor_count;
        }
    }
    return (available_motor_count == me->motor_count)
               ? MODULE_MOTOR_HEALTH_STATUS_OK
               : MODULE_MOTOR_HEALTH_STATUS_DEGRADED;
}

const module_motor_health_state_t *module_motor_health_get_state(
    const module_motor_health_t *me, size_t motor_index)
{
    return ((me != NULL) && me->is_initialized &&
            (motor_index < me->motor_count))
               ? &me->states[motor_index]
               : NULL;
}
