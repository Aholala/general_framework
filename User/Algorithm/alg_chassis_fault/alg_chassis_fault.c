#include "alg_chassis_fault.h"

#include <math.h>
#include <stddef.h>

static uint32_t alg_chassis_fault_increment_counter(uint32_t counter)
{
    return (counter < UINT32_MAX) ? counter + 1U : UINT32_MAX;
}

alg_chassis_status_t alg_chassis_fault_init(
    alg_chassis_fault_t *me,
    const alg_chassis_fault_config_t *config)
{
    size_t wheel_index;

    if ((me == NULL) || (config == NULL) ||
        (config->wheel_count == 0U) ||
        (config->wheel_state_storage == NULL) ||
        !isfinite(config->fault_residual_threshold_m_per_s) ||
        !isfinite(config->recovery_residual_threshold_m_per_s) ||
        (config->fault_residual_threshold_m_per_s <= 0.0F) ||
        (config->recovery_residual_threshold_m_per_s < 0.0F) ||
        (config->recovery_residual_threshold_m_per_s >=
         config->fault_residual_threshold_m_per_s) ||
        (config->fault_confirmation_samples == 0U) ||
        (config->recovery_confirmation_samples == 0U))
    {
        return ALG_CHASSIS_STATUS_INVALID_ARGUMENT;
    }
    me->is_initialized = false;
    me->wheel_count = config->wheel_count;
    me->fault_residual_threshold_m_per_s =
        config->fault_residual_threshold_m_per_s;
    me->recovery_residual_threshold_m_per_s =
        config->recovery_residual_threshold_m_per_s;
    me->fault_confirmation_samples =
        config->fault_confirmation_samples;
    me->recovery_confirmation_samples =
        config->recovery_confirmation_samples;
    me->wheel_states = config->wheel_state_storage;
    for (wheel_index = 0U; wheel_index < me->wheel_count; ++wheel_index)
    {
        me->wheel_states[wheel_index] =
            (alg_chassis_fault_wheel_state_t){0};
    }
    me->is_initialized = true;
    return ALG_CHASSIS_STATUS_OK;
}

alg_chassis_status_t alg_chassis_fault_update(
    alg_chassis_fault_t *me,
    const float *wheel_residuals_m_per_s,
    const bool *sensor_is_available,
    bool *wheel_is_available, size_t output_capacity)
{
    size_t wheel_index;
    size_t available_wheel_count = 0U;

    if ((me == NULL) || (wheel_residuals_m_per_s == NULL) ||
        (wheel_is_available == NULL))
    {
        return ALG_CHASSIS_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return ALG_CHASSIS_STATUS_NOT_INITIALIZED;
    }
    if (output_capacity < me->wheel_count)
    {
        return ALG_CHASSIS_STATUS_INVALID_ARGUMENT;
    }
    for (wheel_index = 0U; wheel_index < me->wheel_count; ++wheel_index)
    {
        alg_chassis_fault_wheel_state_t *const wheel_state =
            &me->wheel_states[wheel_index];
        const bool sensor_available =
            (sensor_is_available == NULL) ||
            sensor_is_available[wheel_index];
        const float residual_m_per_s =
            fabsf(wheel_residuals_m_per_s[wheel_index]);

        if (!isfinite(residual_m_per_s))
        {
            return ALG_CHASSIS_STATUS_INVALID_ARGUMENT;
        }
        wheel_state->residual_m_per_s = residual_m_per_s;
        if (!sensor_available)
        {
            wheel_state->is_faulted = true;
            wheel_state->fault_confirmation_count =
                me->fault_confirmation_samples;
            wheel_state->recovery_confirmation_count = 0U;
        }
        else if (!wheel_state->is_faulted)
        {
            wheel_state->recovery_confirmation_count = 0U;
            if (residual_m_per_s >=
                me->fault_residual_threshold_m_per_s)
            {
                wheel_state->fault_confirmation_count =
                    alg_chassis_fault_increment_counter(
                        wheel_state->fault_confirmation_count);
                if (wheel_state->fault_confirmation_count >=
                    me->fault_confirmation_samples)
                {
                    wheel_state->is_faulted = true;
                    wheel_state->recovery_confirmation_count = 0U;
                }
            }
            else
            {
                wheel_state->fault_confirmation_count = 0U;
            }
        }
        else
        {
            wheel_state->fault_confirmation_count = 0U;
            if (residual_m_per_s <=
                me->recovery_residual_threshold_m_per_s)
            {
                wheel_state->recovery_confirmation_count =
                    alg_chassis_fault_increment_counter(
                        wheel_state->recovery_confirmation_count);
                if (wheel_state->recovery_confirmation_count >=
                    me->recovery_confirmation_samples)
                {
                    wheel_state->is_faulted = false;
                    wheel_state->recovery_confirmation_count = 0U;
                }
            }
            else
            {
                wheel_state->recovery_confirmation_count = 0U;
            }
        }
        wheel_is_available[wheel_index] = !wheel_state->is_faulted;
        if (wheel_is_available[wheel_index])
        {
            ++available_wheel_count;
        }
    }
    return (available_wheel_count == me->wheel_count)
               ? ALG_CHASSIS_STATUS_OK
               : ALG_CHASSIS_STATUS_DEGRADED;
}

alg_chassis_status_t alg_chassis_fault_reset_wheel(
    alg_chassis_fault_t *me, size_t wheel_index,
    bool assume_available)
{
    if (me == NULL)
    {
        return ALG_CHASSIS_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return ALG_CHASSIS_STATUS_NOT_INITIALIZED;
    }
    if (wheel_index >= me->wheel_count)
    {
        return ALG_CHASSIS_STATUS_INVALID_ARGUMENT;
    }
    me->wheel_states[wheel_index] =
        (alg_chassis_fault_wheel_state_t){
            .is_faulted = !assume_available,
        };
    return ALG_CHASSIS_STATUS_OK;
}

const alg_chassis_fault_wheel_state_t *
alg_chassis_fault_get_wheel_state(
    const alg_chassis_fault_t *me, size_t wheel_index)
{
    return ((me != NULL) && me->is_initialized &&
            (wheel_index < me->wheel_count))
               ? &me->wheel_states[wheel_index]
               : NULL;
}
