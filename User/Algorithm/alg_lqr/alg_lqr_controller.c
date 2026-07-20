#include "alg_lqr_internal.h"

#include <math.h>
#include <stddef.h>

AlgLqrStatus_t AlgLqrController_Init(
    AlgLqrController_t *self,
    const AlgLqrControllerConfig_t *config)
{
    size_t control_index;

    if ((self == NULL) || (config == NULL) || (config->gain_matrix == NULL))
    {
        return ALG_LQR_STATUS_INVALID_ARGUMENT;
    }

    self->is_initialized = false;
    if ((config->state_dimension == 0U) || (config->control_dimension == 0U))
    {
        return ALG_LQR_STATUS_OUT_OF_RANGE;
    }
    if ((config->control_min == NULL) != (config->control_max == NULL))
    {
        return ALG_LQR_STATUS_INVALID_ARGUMENT;
    }
    if (!AlgLqrInternal_IsFiniteArray(
            config->gain_matrix,
            config->control_dimension * config->state_dimension))
    {
        return ALG_LQR_STATUS_OUT_OF_RANGE;
    }
    if (config->control_min != NULL)
    {
        for (control_index = 0U; control_index < config->control_dimension;
             ++control_index)
        {
            if (!isfinite(config->control_min[control_index]) ||
                !isfinite(config->control_max[control_index]) ||
                (config->control_min[control_index] >=
                 config->control_max[control_index]))
            {
                return ALG_LQR_STATUS_OUT_OF_RANGE;
            }
        }
    }

    self->config = *config;
    self->is_initialized = true;
    return ALG_LQR_STATUS_OK;
}

AlgLqrStatus_t AlgLqrController_Update(
    const AlgLqrController_t *self,
    const float *state,
    const float *reference_state,
    const float *equilibrium_control,
    const float *feedforward_control,
    float *control_output)
{
    size_t control_index;
    size_t state_index;
    float state_error;
    float output;

    if ((self == NULL) || (state == NULL) || (control_output == NULL))
    {
        return ALG_LQR_STATUS_INVALID_ARGUMENT;
    }
    if (!self->is_initialized)
    {
        return ALG_LQR_STATUS_NOT_INITIALIZED;
    }
    if (!AlgLqrInternal_IsFiniteArray(state, self->config.state_dimension) ||
        ((reference_state != NULL) &&
         !AlgLqrInternal_IsFiniteArray(reference_state,
                                       self->config.state_dimension)) ||
        ((equilibrium_control != NULL) &&
         !AlgLqrInternal_IsFiniteArray(equilibrium_control,
                                       self->config.control_dimension)) ||
        ((feedforward_control != NULL) &&
         !AlgLqrInternal_IsFiniteArray(feedforward_control,
                                       self->config.control_dimension)))
    {
        return ALG_LQR_STATUS_OUT_OF_RANGE;
    }

    for (control_index = 0U; control_index < self->config.control_dimension;
         ++control_index)
    {
        output = (equilibrium_control != NULL)
                     ? equilibrium_control[control_index]
                     : 0.0F;
        if (feedforward_control != NULL)
        {
            output += feedforward_control[control_index];
        }
        for (state_index = 0U; state_index < self->config.state_dimension;
             ++state_index)
        {
            state_error = state[state_index] -
                          ((reference_state != NULL)
                               ? reference_state[state_index]
                               : 0.0F);
            output -= self->config.gain_matrix[
                          (control_index * self->config.state_dimension) + state_index] *
                      state_error;
        }

        if (!isfinite(output))
        {
            return ALG_LQR_STATUS_NUMERICAL_ERROR;
        }
        if (self->config.control_min != NULL)
        {
            if (output < self->config.control_min[control_index])
            {
                output = self->config.control_min[control_index];
            }
            else if (output > self->config.control_max[control_index])
            {
                output = self->config.control_max[control_index];
            }
        }
        control_output[control_index] = output;
    }
    return ALG_LQR_STATUS_OK;
}
