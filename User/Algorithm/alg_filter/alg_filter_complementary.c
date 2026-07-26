#include "alg_filter.h"

#include <math.h>
#include <stddef.h>

alg_filter_status_t alg_filter_complementary_init(alg_filter_complementary_t *me,
                                                  float prediction_weight, float initial_output)
{
    if (me == NULL)
    {
        return ALG_FILTER_STATUS_INVALID_ARGUMENT;
    }

    me->is_initialized = false;
    if (!isfinite(prediction_weight) || (prediction_weight < 0.0F) || (prediction_weight > 1.0F) ||
        !isfinite(initial_output))
    {
        return ALG_FILTER_STATUS_OUT_OF_RANGE;
    }

    me->prediction_weight = prediction_weight;
    me->output = initial_output;
    me->is_initialized = true;
    return ALG_FILTER_STATUS_OK;
}

alg_filter_status_t alg_filter_complementary_set_weight(alg_filter_complementary_t *me,
                                                        float prediction_weight)
{
    if (me == NULL)
    {
        return ALG_FILTER_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return ALG_FILTER_STATUS_NOT_INITIALIZED;
    }
    if (!isfinite(prediction_weight) || (prediction_weight < 0.0F) || (prediction_weight > 1.0F))
    {
        return ALG_FILTER_STATUS_OUT_OF_RANGE;
    }

    me->prediction_weight = prediction_weight;
    return ALG_FILTER_STATUS_OK;
}

alg_filter_status_t alg_filter_complementary_reset(alg_filter_complementary_t *me,
                                                   float initial_output)
{
    if (me == NULL)
    {
        return ALG_FILTER_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return ALG_FILTER_STATUS_NOT_INITIALIZED;
    }
    if (!isfinite(initial_output))
    {
        return ALG_FILTER_STATUS_OUT_OF_RANGE;
    }

    me->output = initial_output;
    return ALG_FILTER_STATUS_OK;
}

alg_filter_status_t alg_filter_complementary_update(alg_filter_complementary_t *me,
                                                    float measured_value, float measured_rate_per_s,
                                                    float delta_time_s, float *output)
{
    float predicted_value;

    if ((me == NULL) || (output == NULL))
    {
        return ALG_FILTER_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return ALG_FILTER_STATUS_NOT_INITIALIZED;
    }
    if (!isfinite(measured_value) || !isfinite(measured_rate_per_s) || !isfinite(delta_time_s) ||
        (delta_time_s <= 0.0F))
    {
        return ALG_FILTER_STATUS_OUT_OF_RANGE;
    }

    predicted_value = me->output + (measured_rate_per_s * delta_time_s);
    me->output = (me->prediction_weight * predicted_value) +
                 ((1.0F - me->prediction_weight) * measured_value);
    if (!isfinite(me->output))
    {
        return ALG_FILTER_STATUS_NUMERICAL_ERROR;
    }

    *output = me->output;
    return ALG_FILTER_STATUS_OK;
}
