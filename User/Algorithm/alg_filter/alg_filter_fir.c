#include "alg_filter.h"

#include <math.h>
#include <stddef.h>

AlgFilterStatus_t AlgFilterFir_Init(AlgFilterFir_t *self,
                                    const float *coefficients,
                                    float *state_buffer,
                                    size_t tap_count)
{
    size_t tap_index;

    if ((self == NULL) || (coefficients == NULL) || (state_buffer == NULL))
    {
        return ALG_FILTER_STATUS_INVALID_ARGUMENT;
    }
    if (tap_count == 0U)
    {
        return ALG_FILTER_STATUS_OUT_OF_RANGE;
    }

    self->is_initialized = false;
    for (tap_index = 0U; tap_index < tap_count; ++tap_index)
    {
        if (!isfinite(coefficients[tap_index]))
        {
            return ALG_FILTER_STATUS_OUT_OF_RANGE;
        }
    }

    self->coefficients = coefficients;
    self->state_buffer = state_buffer;
    self->tap_count = tap_count;
    self->is_initialized = true;
    return AlgFilterFir_Reset(self);
}

AlgFilterStatus_t AlgFilterFir_Reset(AlgFilterFir_t *self)
{
    size_t tap_index;

    if (self == NULL)
    {
        return ALG_FILTER_STATUS_INVALID_ARGUMENT;
    }
    if (!self->is_initialized)
    {
        return ALG_FILTER_STATUS_NOT_INITIALIZED;
    }

    for (tap_index = 0U; tap_index < self->tap_count; ++tap_index)
    {
        self->state_buffer[tap_index] = 0.0F;
    }
    self->write_index = 0U;
    return ALG_FILTER_STATUS_OK;
}

AlgFilterStatus_t AlgFilterFir_Update(AlgFilterFir_t *self,
                                      float input,
                                      float *output)
{
    size_t coefficient_index;
    size_t state_index;
    float accumulator = 0.0F;

    if ((self == NULL) || (output == NULL))
    {
        return ALG_FILTER_STATUS_INVALID_ARGUMENT;
    }
    if (!self->is_initialized)
    {
        return ALG_FILTER_STATUS_NOT_INITIALIZED;
    }
    if (!isfinite(input))
    {
        return ALG_FILTER_STATUS_OUT_OF_RANGE;
    }

    self->state_buffer[self->write_index] = input;
    state_index = self->write_index;
    for (coefficient_index = 0U; coefficient_index < self->tap_count;
         ++coefficient_index)
    {
        accumulator += self->coefficients[coefficient_index] *
                       self->state_buffer[state_index];
        state_index = (state_index == 0U) ? (self->tap_count - 1U) : (state_index - 1U);
    }

    self->write_index = (self->write_index + 1U) % self->tap_count;
    if (!isfinite(accumulator))
    {
        return ALG_FILTER_STATUS_NUMERICAL_ERROR;
    }

    *output = accumulator;
    return ALG_FILTER_STATUS_OK;
}
