#include "alg_motion.h"

#include <math.h>
#include <stddef.h>

static float alg_motion_clamp(float value, float minimum, float maximum)
{
    if (value < minimum)
    {
        return minimum;
    }
    if (value > maximum)
    {
        return maximum;
    }
    return value;
}

static bool
alg_motion_rate_limiter_config_is_valid(const alg_motion_rate_limiter_config_t *const config)
{
    return (config != NULL) && isfinite(config->rising_rate_per_s) &&
           isfinite(config->falling_rate_per_s) && isfinite(config->output_min) &&
           isfinite(config->output_max) && (config->rising_rate_per_s > 0.0F) &&
           (config->falling_rate_per_s > 0.0F) && (config->output_min < config->output_max);
}

alg_motion_status_t
alg_motion_rate_limiter_init(alg_motion_rate_limiter_t *const me,
                             const alg_motion_rate_limiter_config_t *const config,
                             float initial_output)
{
    if ((me == NULL) || (config == NULL))
    {
        return ALG_MOTION_STATUS_INVALID_ARGUMENT;
    }
    me->is_initialized = false;
    if (!alg_motion_rate_limiter_config_is_valid(config) || !isfinite(initial_output) ||
        (initial_output < config->output_min) || (initial_output > config->output_max))
    {
        return ALG_MOTION_STATUS_OUT_OF_RANGE;
    }
    me->config = *config;
    me->output = initial_output;
    me->is_initialized = true;
    return ALG_MOTION_STATUS_OK;
}

alg_motion_status_t alg_motion_rate_limiter_reset(alg_motion_rate_limiter_t *const me,
                                                  float initial_output)
{
    if (me == NULL)
    {
        return ALG_MOTION_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return ALG_MOTION_STATUS_NOT_INITIALIZED;
    }
    if (!isfinite(initial_output) || (initial_output < me->config.output_min) ||
        (initial_output > me->config.output_max))
    {
        return ALG_MOTION_STATUS_OUT_OF_RANGE;
    }
    me->output = initial_output;
    return ALG_MOTION_STATUS_OK;
}

alg_motion_status_t alg_motion_rate_limiter_update(alg_motion_rate_limiter_t *const me,
                                                   float target, float delta_time_s, float *output)
{
    float difference;
    float maximum_change;

    if ((me == NULL) || (output == NULL))
    {
        return ALG_MOTION_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return ALG_MOTION_STATUS_NOT_INITIALIZED;
    }
    if (!isfinite(target) || !isfinite(delta_time_s) || (delta_time_s <= 0.0F))
    {
        return ALG_MOTION_STATUS_OUT_OF_RANGE;
    }

    target = alg_motion_clamp(target, me->config.output_min, me->config.output_max);
    difference = target - me->output;
    maximum_change =
        ((difference >= 0.0F) ? me->config.rising_rate_per_s : me->config.falling_rate_per_s) *
        delta_time_s;
    if (fabsf(difference) <= maximum_change)
    {
        me->output = target;
    }
    else
    {
        me->output += copysignf(maximum_change, difference);
    }
    if (!isfinite(me->output))
    {
        return ALG_MOTION_STATUS_NUMERICAL_ERROR;
    }
    *output = me->output;
    return ALG_MOTION_STATUS_OK;
}

alg_motion_status_t alg_motion_unwrapper_init(alg_motion_unwrapper_t *const me, float period)
{
    if (me == NULL)
    {
        return ALG_MOTION_STATUS_INVALID_ARGUMENT;
    }
    me->is_initialized = false;
    if (!isfinite(period) || (period <= 0.0F))
    {
        return ALG_MOTION_STATUS_OUT_OF_RANGE;
    }
    me->period = period;
    me->previous_wrapped_value = 0.0F;
    me->continuous_value = 0.0F;
    me->has_previous_value = false;
    me->is_initialized = true;
    return ALG_MOTION_STATUS_OK;
}

alg_motion_status_t alg_motion_unwrapper_reset(alg_motion_unwrapper_t *const me,
                                               float wrapped_value, float continuous_value)
{
    if (me == NULL)
    {
        return ALG_MOTION_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return ALG_MOTION_STATUS_NOT_INITIALIZED;
    }
    if (!isfinite(wrapped_value) || !isfinite(continuous_value))
    {
        return ALG_MOTION_STATUS_OUT_OF_RANGE;
    }
    me->previous_wrapped_value = wrapped_value;
    me->continuous_value = continuous_value;
    me->has_previous_value = true;
    return ALG_MOTION_STATUS_OK;
}

alg_motion_status_t alg_motion_unwrapper_update(alg_motion_unwrapper_t *const me,
                                                float wrapped_value, float *continuous_value)
{
    float difference;
    float half_period;

    if ((me == NULL) || (continuous_value == NULL))
    {
        return ALG_MOTION_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return ALG_MOTION_STATUS_NOT_INITIALIZED;
    }
    if (!isfinite(wrapped_value))
    {
        return ALG_MOTION_STATUS_OUT_OF_RANGE;
    }
    if (!me->has_previous_value)
    {
        me->previous_wrapped_value = wrapped_value;
        me->continuous_value = wrapped_value;
        me->has_previous_value = true;
        *continuous_value = me->continuous_value;
        return ALG_MOTION_STATUS_OK;
    }

    half_period = 0.5F * me->period;
    difference = wrapped_value - me->previous_wrapped_value;
    while (difference > half_period)
    {
        difference -= me->period;
    }
    while (difference < -half_period)
    {
        difference += me->period;
    }
    me->continuous_value += difference;
    me->previous_wrapped_value = wrapped_value;
    if (!isfinite(me->continuous_value))
    {
        return ALG_MOTION_STATUS_NUMERICAL_ERROR;
    }
    *continuous_value = me->continuous_value;
    return ALG_MOTION_STATUS_OK;
}
