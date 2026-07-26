#include "alg_filter.h"

#include <math.h>
#include <stddef.h>

#define ALG_FILTER_TWO_PI_F (6.28318530717958647692F)

static bool alg_filter_basic_is_positive_finite(float value)
{
    return isfinite(value) && (value > 0.0F);
}

alg_filter_status_t alg_filter_low_pass_init(alg_filter_low_pass_t *me, float cutoff_frequency_hz)
{
    if (me == NULL)
    {
        return ALG_FILTER_STATUS_INVALID_ARGUMENT;
    }

    me->is_initialized = false;
    me->has_previous_sample = false;
    me->output = 0.0F;
    if (!alg_filter_basic_is_positive_finite(cutoff_frequency_hz))
    {
        return ALG_FILTER_STATUS_OUT_OF_RANGE;
    }

    me->cutoff_frequency_hz = cutoff_frequency_hz;
    me->is_initialized = true;
    return ALG_FILTER_STATUS_OK;
}

alg_filter_status_t alg_filter_low_pass_set_cutoff(alg_filter_low_pass_t *me,
                                                   float cutoff_frequency_hz)
{
    if (me == NULL)
    {
        return ALG_FILTER_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return ALG_FILTER_STATUS_NOT_INITIALIZED;
    }
    if (!alg_filter_basic_is_positive_finite(cutoff_frequency_hz))
    {
        return ALG_FILTER_STATUS_OUT_OF_RANGE;
    }

    me->cutoff_frequency_hz = cutoff_frequency_hz;
    return ALG_FILTER_STATUS_OK;
}

alg_filter_status_t alg_filter_low_pass_reset(alg_filter_low_pass_t *me, float initial_output)
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
    me->has_previous_sample = true;
    return ALG_FILTER_STATUS_OK;
}

alg_filter_status_t alg_filter_low_pass_update(alg_filter_low_pass_t *me, float input,
                                               float delta_time_s, float *output)
{
    float time_constant_s;
    float smoothing_factor;

    if ((me == NULL) || (output == NULL))
    {
        return ALG_FILTER_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return ALG_FILTER_STATUS_NOT_INITIALIZED;
    }
    if (!isfinite(input) || !alg_filter_basic_is_positive_finite(delta_time_s))
    {
        return ALG_FILTER_STATUS_OUT_OF_RANGE;
    }

    if (!me->has_previous_sample)
    {
        me->output = input;
        me->has_previous_sample = true;
    }
    else
    {
        time_constant_s = 1.0F / (ALG_FILTER_TWO_PI_F * me->cutoff_frequency_hz);
        smoothing_factor = delta_time_s / (time_constant_s + delta_time_s);
        me->output += smoothing_factor * (input - me->output);
    }

    if (!isfinite(me->output))
    {
        return ALG_FILTER_STATUS_NUMERICAL_ERROR;
    }

    *output = me->output;
    return ALG_FILTER_STATUS_OK;
}

alg_filter_status_t alg_filter_high_pass_init(alg_filter_high_pass_t *me, float cutoff_frequency_hz)
{
    if (me == NULL)
    {
        return ALG_FILTER_STATUS_INVALID_ARGUMENT;
    }

    me->is_initialized = false;
    me->has_previous_sample = false;
    me->previous_input = 0.0F;
    me->output = 0.0F;
    if (!alg_filter_basic_is_positive_finite(cutoff_frequency_hz))
    {
        return ALG_FILTER_STATUS_OUT_OF_RANGE;
    }

    me->cutoff_frequency_hz = cutoff_frequency_hz;
    me->is_initialized = true;
    return ALG_FILTER_STATUS_OK;
}

alg_filter_status_t alg_filter_high_pass_set_cutoff(alg_filter_high_pass_t *me,
                                                    float cutoff_frequency_hz)
{
    if (me == NULL)
    {
        return ALG_FILTER_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return ALG_FILTER_STATUS_NOT_INITIALIZED;
    }
    if (!alg_filter_basic_is_positive_finite(cutoff_frequency_hz))
    {
        return ALG_FILTER_STATUS_OUT_OF_RANGE;
    }

    me->cutoff_frequency_hz = cutoff_frequency_hz;
    return ALG_FILTER_STATUS_OK;
}

alg_filter_status_t alg_filter_high_pass_reset(alg_filter_high_pass_t *me, float initial_input)
{
    if (me == NULL)
    {
        return ALG_FILTER_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return ALG_FILTER_STATUS_NOT_INITIALIZED;
    }
    if (!isfinite(initial_input))
    {
        return ALG_FILTER_STATUS_OUT_OF_RANGE;
    }

    me->previous_input = initial_input;
    me->output = 0.0F;
    me->has_previous_sample = true;
    return ALG_FILTER_STATUS_OK;
}

alg_filter_status_t alg_filter_high_pass_update(alg_filter_high_pass_t *me, float input,
                                                float delta_time_s, float *output)
{
    float time_constant_s;
    float smoothing_factor;

    if ((me == NULL) || (output == NULL))
    {
        return ALG_FILTER_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return ALG_FILTER_STATUS_NOT_INITIALIZED;
    }
    if (!isfinite(input) || !alg_filter_basic_is_positive_finite(delta_time_s))
    {
        return ALG_FILTER_STATUS_OUT_OF_RANGE;
    }

    if (!me->has_previous_sample)
    {
        me->previous_input = input;
        me->output = 0.0F;
        me->has_previous_sample = true;
    }
    else
    {
        time_constant_s = 1.0F / (ALG_FILTER_TWO_PI_F * me->cutoff_frequency_hz);
        smoothing_factor = time_constant_s / (time_constant_s + delta_time_s);
        me->output = smoothing_factor * (me->output + input - me->previous_input);
        me->previous_input = input;
    }

    if (!isfinite(me->output))
    {
        return ALG_FILTER_STATUS_NUMERICAL_ERROR;
    }

    *output = me->output;
    return ALG_FILTER_STATUS_OK;
}

alg_filter_status_t alg_filter_exponential_init(alg_filter_exponential_t *me,
                                                float smoothing_factor)
{
    if (me == NULL)
    {
        return ALG_FILTER_STATUS_INVALID_ARGUMENT;
    }

    me->is_initialized = false;
    me->has_previous_sample = false;
    me->output = 0.0F;
    if (!isfinite(smoothing_factor) || (smoothing_factor <= 0.0F) || (smoothing_factor > 1.0F))
    {
        return ALG_FILTER_STATUS_OUT_OF_RANGE;
    }

    me->smoothing_factor = smoothing_factor;
    me->is_initialized = true;
    return ALG_FILTER_STATUS_OK;
}

alg_filter_status_t alg_filter_exponential_set_factor(alg_filter_exponential_t *me,
                                                      float smoothing_factor)
{
    if (me == NULL)
    {
        return ALG_FILTER_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return ALG_FILTER_STATUS_NOT_INITIALIZED;
    }
    if (!isfinite(smoothing_factor) || (smoothing_factor <= 0.0F) || (smoothing_factor > 1.0F))
    {
        return ALG_FILTER_STATUS_OUT_OF_RANGE;
    }

    me->smoothing_factor = smoothing_factor;
    return ALG_FILTER_STATUS_OK;
}

alg_filter_status_t alg_filter_exponential_reset(alg_filter_exponential_t *me, float initial_output)
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
    me->has_previous_sample = true;
    return ALG_FILTER_STATUS_OK;
}

alg_filter_status_t alg_filter_exponential_update(alg_filter_exponential_t *me, float input,
                                                  float *output)
{
    if ((me == NULL) || (output == NULL))
    {
        return ALG_FILTER_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return ALG_FILTER_STATUS_NOT_INITIALIZED;
    }
    if (!isfinite(input))
    {
        return ALG_FILTER_STATUS_OUT_OF_RANGE;
    }

    if (!me->has_previous_sample)
    {
        me->output = input;
        me->has_previous_sample = true;
    }
    else
    {
        me->output += me->smoothing_factor * (input - me->output);
    }

    if (!isfinite(me->output))
    {
        return ALG_FILTER_STATUS_NUMERICAL_ERROR;
    }

    *output = me->output;
    return ALG_FILTER_STATUS_OK;
}
