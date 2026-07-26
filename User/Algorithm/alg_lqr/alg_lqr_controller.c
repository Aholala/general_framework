#include "alg_lqr_internal.h"

#include <math.h>
#include <stddef.h>

alg_lqr_status_t alg_lqr_controller_init(alg_lqr_controller_t *me,
                                         const alg_lqr_controller_config_t *config)
{
    size_t control_index;

    if ((me == NULL) || (config == NULL) || (config->gain_matrix == NULL))
    {
        return ALG_LQR_STATUS_INVALID_ARGUMENT;
    }

    me->is_initialized = false;
    if ((config->state_dimension == 0U) || (config->control_dimension == 0U))
    {
        return ALG_LQR_STATUS_OUT_OF_RANGE;
    }
    if ((config->control_min == NULL) != (config->control_max == NULL))
    {
        return ALG_LQR_STATUS_INVALID_ARGUMENT;
    }
    if (!alg_lqr_internal_is_finite_array(config->gain_matrix,
                                          config->control_dimension * config->state_dimension))
    {
        return ALG_LQR_STATUS_OUT_OF_RANGE;
    }
    if (config->control_min != NULL)
    {
        for (control_index = 0U; control_index < config->control_dimension; ++control_index)
        {
            if (!isfinite(config->control_min[control_index]) ||
                !isfinite(config->control_max[control_index]) ||
                (config->control_min[control_index] >= config->control_max[control_index]))
            {
                return ALG_LQR_STATUS_OUT_OF_RANGE;
            }
        }
    }

    me->config = *config;
    me->is_initialized = true;
    return ALG_LQR_STATUS_OK;
}

alg_lqr_status_t alg_lqr_controller_update(const alg_lqr_controller_t *me, const float *state,
                                           const float *reference_state,
                                           const float *equilibrium_control,
                                           const float *feedforward_control, float *control_output)
{
    size_t control_index;
    size_t state_index;
    float state_error;
    float output;

    if ((me == NULL) || (state == NULL) || (control_output == NULL))
    {
        return ALG_LQR_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return ALG_LQR_STATUS_NOT_INITIALIZED;
    }
    if (!alg_lqr_internal_is_finite_array(state, me->config.state_dimension) ||
        ((reference_state != NULL) &&
         !alg_lqr_internal_is_finite_array(reference_state, me->config.state_dimension)) ||
        ((equilibrium_control != NULL) &&
         !alg_lqr_internal_is_finite_array(equilibrium_control, me->config.control_dimension)) ||
        ((feedforward_control != NULL) &&
         !alg_lqr_internal_is_finite_array(feedforward_control, me->config.control_dimension)))
    {
        return ALG_LQR_STATUS_OUT_OF_RANGE;
    }

    for (control_index = 0U; control_index < me->config.control_dimension; ++control_index)
    {
        output = (equilibrium_control != NULL) ? equilibrium_control[control_index] : 0.0F;
        if (feedforward_control != NULL)
        {
            output += feedforward_control[control_index];
        }
        for (state_index = 0U; state_index < me->config.state_dimension; ++state_index)
        {
            state_error = state[state_index] -
                          ((reference_state != NULL) ? reference_state[state_index] : 0.0F);
            output -=
                me->config.gain_matrix[(control_index * me->config.state_dimension) + state_index] *
                state_error;
        }

        if (!isfinite(output))
        {
            return ALG_LQR_STATUS_NUMERICAL_ERROR;
        }
        if (me->config.control_min != NULL)
        {
            if (output < me->config.control_min[control_index])
            {
                output = me->config.control_min[control_index];
            }
            else if (output > me->config.control_max[control_index])
            {
                output = me->config.control_max[control_index];
            }
        }
        control_output[control_index] = output;
    }
    return ALG_LQR_STATUS_OK;
}
