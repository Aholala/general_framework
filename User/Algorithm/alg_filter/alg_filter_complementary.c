#include "alg_filter.h"

#include <math.h>
#include <stddef.h>

AlgFilterStatus_t AlgFilterComplementary_Init(AlgFilterComplementary_t *self,
                                              float prediction_weight,
                                              float initial_output)
{
    if (self == NULL)
    {
        return ALG_FILTER_STATUS_INVALID_ARGUMENT;
    }

    self->is_initialized = false;
    if (!isfinite(prediction_weight) || (prediction_weight < 0.0F) ||
        (prediction_weight > 1.0F) || !isfinite(initial_output))
    {
        return ALG_FILTER_STATUS_OUT_OF_RANGE;
    }

    self->prediction_weight = prediction_weight;
    self->output = initial_output;
    self->is_initialized = true;
    return ALG_FILTER_STATUS_OK;
}

AlgFilterStatus_t AlgFilterComplementary_SetWeight(AlgFilterComplementary_t *self,
                                                   float prediction_weight)
{
    if (self == NULL)
    {
        return ALG_FILTER_STATUS_INVALID_ARGUMENT;
    }
    if (!self->is_initialized)
    {
        return ALG_FILTER_STATUS_NOT_INITIALIZED;
    }
    if (!isfinite(prediction_weight) || (prediction_weight < 0.0F) ||
        (prediction_weight > 1.0F))
    {
        return ALG_FILTER_STATUS_OUT_OF_RANGE;
    }

    self->prediction_weight = prediction_weight;
    return ALG_FILTER_STATUS_OK;
}

AlgFilterStatus_t AlgFilterComplementary_Reset(AlgFilterComplementary_t *self,
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
    return ALG_FILTER_STATUS_OK;
}

AlgFilterStatus_t AlgFilterComplementary_Update(AlgFilterComplementary_t *self,
                                                float measured_value,
                                                float measured_rate_per_s,
                                                float delta_time_s,
                                                float *output)
{
    float predicted_value;

    if ((self == NULL) || (output == NULL))
    {
        return ALG_FILTER_STATUS_INVALID_ARGUMENT;
    }
    if (!self->is_initialized)
    {
        return ALG_FILTER_STATUS_NOT_INITIALIZED;
    }
    if (!isfinite(measured_value) || !isfinite(measured_rate_per_s) ||
        !isfinite(delta_time_s) || (delta_time_s <= 0.0F))
    {
        return ALG_FILTER_STATUS_OUT_OF_RANGE;
    }

    predicted_value = self->output + (measured_rate_per_s * delta_time_s);
    self->output = (self->prediction_weight * predicted_value) +
                   ((1.0F - self->prediction_weight) * measured_value);
    if (!isfinite(self->output))
    {
        return ALG_FILTER_STATUS_NUMERICAL_ERROR;
    }

    *output = self->output;
    return ALG_FILTER_STATUS_OK;
}
