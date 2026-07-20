#include "alg_filter.h"

#include <math.h>
#include <stddef.h>

#define ALG_FILTER_TWO_PI_F (6.28318530717958647692F)

static bool AlgFilterBasic_IsPositiveFinite(float value)
{
    return isfinite(value) && (value > 0.0F);
}

AlgFilterStatus_t AlgFilterLowPass_Init(AlgFilterLowPass_t *self,
                                        float cutoff_frequency_hz)
{
    if (self == NULL)
    {
        return ALG_FILTER_STATUS_INVALID_ARGUMENT;
    }

    self->is_initialized = false;
    self->has_previous_sample = false;
    self->output = 0.0F;
    if (!AlgFilterBasic_IsPositiveFinite(cutoff_frequency_hz))
    {
        return ALG_FILTER_STATUS_OUT_OF_RANGE;
    }

    self->cutoff_frequency_hz = cutoff_frequency_hz;
    self->is_initialized = true;
    return ALG_FILTER_STATUS_OK;
}

AlgFilterStatus_t AlgFilterLowPass_SetCutoff(AlgFilterLowPass_t *self,
                                             float cutoff_frequency_hz)
{
    if (self == NULL)
    {
        return ALG_FILTER_STATUS_INVALID_ARGUMENT;
    }
    if (!self->is_initialized)
    {
        return ALG_FILTER_STATUS_NOT_INITIALIZED;
    }
    if (!AlgFilterBasic_IsPositiveFinite(cutoff_frequency_hz))
    {
        return ALG_FILTER_STATUS_OUT_OF_RANGE;
    }

    self->cutoff_frequency_hz = cutoff_frequency_hz;
    return ALG_FILTER_STATUS_OK;
}

AlgFilterStatus_t AlgFilterLowPass_Reset(AlgFilterLowPass_t *self,
                                         float initial_output)
{
    if (self == NULL)
    {
        return ALG_FILTER_STATUS_INVALID_ARGUMENT;
    }
    if (!self->is_initialized)
    {
        return ALG_FILTER_STATUS_NOT_INITIALIZED;
    }
    if (!isfinite(initial_output))
    {
        return ALG_FILTER_STATUS_OUT_OF_RANGE;
    }

    self->output = initial_output;
    self->has_previous_sample = true;
    return ALG_FILTER_STATUS_OK;
}

AlgFilterStatus_t AlgFilterLowPass_Update(AlgFilterLowPass_t *self,
                                          float input,
                                          float delta_time_s,
                                          float *output)
{
    float time_constant_s;
    float smoothing_factor;

    if ((self == NULL) || (output == NULL))
    {
        return ALG_FILTER_STATUS_INVALID_ARGUMENT;
    }
    if (!self->is_initialized)
    {
        return ALG_FILTER_STATUS_NOT_INITIALIZED;
    }
    if (!isfinite(input) || !AlgFilterBasic_IsPositiveFinite(delta_time_s))
    {
        return ALG_FILTER_STATUS_OUT_OF_RANGE;
    }

    if (!self->has_previous_sample)
    {
        self->output = input;
        self->has_previous_sample = true;
    }
    else
    {
        time_constant_s = 1.0F / (ALG_FILTER_TWO_PI_F * self->cutoff_frequency_hz);
        smoothing_factor = delta_time_s / (time_constant_s + delta_time_s);
        self->output += smoothing_factor * (input - self->output);
    }

    if (!isfinite(self->output))
    {
        return ALG_FILTER_STATUS_NUMERICAL_ERROR;
    }

    *output = self->output;
    return ALG_FILTER_STATUS_OK;
}

AlgFilterStatus_t AlgFilterHighPass_Init(AlgFilterHighPass_t *self,
                                         float cutoff_frequency_hz)
{
    if (self == NULL)
    {
        return ALG_FILTER_STATUS_INVALID_ARGUMENT;
    }

    self->is_initialized = false;
    self->has_previous_sample = false;
    self->previous_input = 0.0F;
    self->output = 0.0F;
    if (!AlgFilterBasic_IsPositiveFinite(cutoff_frequency_hz))
    {
        return ALG_FILTER_STATUS_OUT_OF_RANGE;
    }

    self->cutoff_frequency_hz = cutoff_frequency_hz;
    self->is_initialized = true;
    return ALG_FILTER_STATUS_OK;
}

AlgFilterStatus_t AlgFilterHighPass_SetCutoff(AlgFilterHighPass_t *self,
                                              float cutoff_frequency_hz)
{
    if (self == NULL)
    {
        return ALG_FILTER_STATUS_INVALID_ARGUMENT;
    }
    if (!self->is_initialized)
    {
        return ALG_FILTER_STATUS_NOT_INITIALIZED;
    }
    if (!AlgFilterBasic_IsPositiveFinite(cutoff_frequency_hz))
    {
        return ALG_FILTER_STATUS_OUT_OF_RANGE;
    }

    self->cutoff_frequency_hz = cutoff_frequency_hz;
    return ALG_FILTER_STATUS_OK;
}

AlgFilterStatus_t AlgFilterHighPass_Reset(AlgFilterHighPass_t *self,
                                          float initial_input)
{
    if (self == NULL)
    {
        return ALG_FILTER_STATUS_INVALID_ARGUMENT;
    }
    if (!self->is_initialized)
    {
        return ALG_FILTER_STATUS_NOT_INITIALIZED;
    }
    if (!isfinite(initial_input))
    {
        return ALG_FILTER_STATUS_OUT_OF_RANGE;
    }

    self->previous_input = initial_input;
    self->output = 0.0F;
    self->has_previous_sample = true;
    return ALG_FILTER_STATUS_OK;
}

AlgFilterStatus_t AlgFilterHighPass_Update(AlgFilterHighPass_t *self,
                                           float input,
                                           float delta_time_s,
                                           float *output)
{
    float time_constant_s;
    float smoothing_factor;

    if ((self == NULL) || (output == NULL))
    {
        return ALG_FILTER_STATUS_INVALID_ARGUMENT;
    }
    if (!self->is_initialized)
    {
        return ALG_FILTER_STATUS_NOT_INITIALIZED;
    }
    if (!isfinite(input) || !AlgFilterBasic_IsPositiveFinite(delta_time_s))
    {
        return ALG_FILTER_STATUS_OUT_OF_RANGE;
    }

    if (!self->has_previous_sample)
    {
        self->previous_input = input;
        self->output = 0.0F;
        self->has_previous_sample = true;
    }
    else
    {
        time_constant_s = 1.0F / (ALG_FILTER_TWO_PI_F * self->cutoff_frequency_hz);
        smoothing_factor = time_constant_s / (time_constant_s + delta_time_s);
        self->output = smoothing_factor * (self->output + input - self->previous_input);
        self->previous_input = input;
    }

    if (!isfinite(self->output))
    {
        return ALG_FILTER_STATUS_NUMERICAL_ERROR;
    }

    *output = self->output;
    return ALG_FILTER_STATUS_OK;
}

AlgFilterStatus_t AlgFilterExponential_Init(AlgFilterExponential_t *self,
                                            float smoothing_factor)
{
    if (self == NULL)
    {
        return ALG_FILTER_STATUS_INVALID_ARGUMENT;
    }

    self->is_initialized = false;
    self->has_previous_sample = false;
    self->output = 0.0F;
    if (!isfinite(smoothing_factor) || (smoothing_factor <= 0.0F) ||
        (smoothing_factor > 1.0F))
    {
        return ALG_FILTER_STATUS_OUT_OF_RANGE;
    }

    self->smoothing_factor = smoothing_factor;
    self->is_initialized = true;
    return ALG_FILTER_STATUS_OK;
}

AlgFilterStatus_t AlgFilterExponential_SetFactor(AlgFilterExponential_t *self,
                                                 float smoothing_factor)
{
    if (self == NULL)
    {
        return ALG_FILTER_STATUS_INVALID_ARGUMENT;
    }
    if (!self->is_initialized)
    {
        return ALG_FILTER_STATUS_NOT_INITIALIZED;
    }
    if (!isfinite(smoothing_factor) || (smoothing_factor <= 0.0F) ||
        (smoothing_factor > 1.0F))
    {
        return ALG_FILTER_STATUS_OUT_OF_RANGE;
    }

    self->smoothing_factor = smoothing_factor;
    return ALG_FILTER_STATUS_OK;
}

AlgFilterStatus_t AlgFilterExponential_Reset(AlgFilterExponential_t *self,
                                             float initial_output)
{
    if (self == NULL)
    {
        return ALG_FILTER_STATUS_INVALID_ARGUMENT;
    }
    if (!self->is_initialized)
    {
        return ALG_FILTER_STATUS_NOT_INITIALIZED;
    }
    if (!isfinite(initial_output))
    {
        return ALG_FILTER_STATUS_OUT_OF_RANGE;
    }

    self->output = initial_output;
    self->has_previous_sample = true;
    return ALG_FILTER_STATUS_OK;
}

AlgFilterStatus_t AlgFilterExponential_Update(AlgFilterExponential_t *self,
                                              float input,
                                              float *output)
{
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

    if (!self->has_previous_sample)
    {
        self->output = input;
        self->has_previous_sample = true;
    }
    else
    {
        self->output += self->smoothing_factor * (input - self->output);
    }

    if (!isfinite(self->output))
    {
        return ALG_FILTER_STATUS_NUMERICAL_ERROR;
    }

    *output = self->output;
    return ALG_FILTER_STATUS_OK;
}
