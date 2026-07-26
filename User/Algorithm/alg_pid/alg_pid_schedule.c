#include "alg_pid_internal.h"

#include <math.h>
#include <stddef.h>

static alg_pid_status_t
alg_pid_gain_schedule_validate_points(const alg_pid_gain_point_t *gain_points,
                                      size_t gain_point_count)
{
    size_t point_index;

    if (gain_points == NULL)
    {
        return ALG_PID_STATUS_INVALID_ARGUMENT;
    }
    if (gain_point_count == 0U)
    {
        return ALG_PID_STATUS_OUT_OF_RANGE;
    }

    for (point_index = 0U; point_index < gain_point_count; ++point_index)
    {
        if (!isfinite(gain_points[point_index].operating_point) ||
            !isfinite(gain_points[point_index].proportional_gain) ||
            !isfinite(gain_points[point_index].integral_gain) ||
            !isfinite(gain_points[point_index].derivative_gain))
        {
            return ALG_PID_STATUS_OUT_OF_RANGE;
        }
        if ((point_index > 0U) && (gain_points[point_index].operating_point <=
                                   gain_points[point_index - 1U].operating_point))
        {
            return ALG_PID_STATUS_OUT_OF_RANGE;
        }
    }
    return ALG_PID_STATUS_OK;
}

static void alg_pid_gain_schedule_interpolate(const alg_pid_gain_schedule_t *me,
                                              float operating_point, float *proportional_gain,
                                              float *integral_gain, float *derivative_gain)
{
    const alg_pid_gain_point_t *lower;
    const alg_pid_gain_point_t *upper;
    size_t point_index;
    float interpolation_factor;

    if ((me->gain_point_count == 1U) || (operating_point <= me->gain_points[0].operating_point))
    {
        lower = &me->gain_points[0];
        upper = lower;
    }
    else if (operating_point >= me->gain_points[me->gain_point_count - 1U].operating_point)
    {
        lower = &me->gain_points[me->gain_point_count - 1U];
        upper = lower;
    }
    else
    {
        lower = &me->gain_points[0];
        upper = &me->gain_points[1];
        for (point_index = 1U; point_index < me->gain_point_count; ++point_index)
        {
            if (operating_point <= me->gain_points[point_index].operating_point)
            {
                lower = &me->gain_points[point_index - 1U];
                upper = &me->gain_points[point_index];
                break;
            }
        }
    }

    interpolation_factor = 0.0F;
    if (upper != lower)
    {
        interpolation_factor = (operating_point - lower->operating_point) /
                               (upper->operating_point - lower->operating_point);
    }
    *proportional_gain =
        lower->proportional_gain +
        interpolation_factor * (upper->proportional_gain - lower->proportional_gain);
    *integral_gain =
        lower->integral_gain + interpolation_factor * (upper->integral_gain - lower->integral_gain);
    *derivative_gain = lower->derivative_gain +
                       interpolation_factor * (upper->derivative_gain - lower->derivative_gain);
}

alg_pid_status_t alg_pid_gain_schedule_init(alg_pid_gain_schedule_t *me,
                                            const alg_pid_config_t *base_config,
                                            const alg_pid_gain_point_t *gain_points,
                                            size_t gain_point_count)
{
    alg_pid_status_t status;

    if (me == NULL)
    {
        return ALG_PID_STATUS_INVALID_ARGUMENT;
    }

    me->is_initialized = false;
    status = alg_pid_gain_schedule_validate_points(gain_points, gain_point_count);
    if (status != ALG_PID_STATUS_OK)
    {
        return status;
    }
    status = alg_pid_init(&me->controller, base_config);
    if (status != ALG_PID_STATUS_OK)
    {
        return status;
    }

    me->gain_points = gain_points;
    me->gain_point_count = gain_point_count;
    me->is_initialized = true;
    return ALG_PID_STATUS_OK;
}

alg_pid_status_t alg_pid_gain_schedule_update(alg_pid_gain_schedule_t *me, float operating_point,
                                              const alg_pid_input_t *input, float *output)
{
    if ((me == NULL) || (input == NULL) || (output == NULL))
    {
        return ALG_PID_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return ALG_PID_STATUS_NOT_INITIALIZED;
    }
    if (!isfinite(operating_point))
    {
        return ALG_PID_STATUS_OUT_OF_RANGE;
    }

    alg_pid_gain_schedule_interpolate(me, operating_point, &me->controller.config.proportional_gain,
                                      &me->controller.config.integral_gain,
                                      &me->controller.config.derivative_gain);
    return alg_pid_update_advanced(&me->controller, input, output);
}

alg_pid_status_t alg_pid_gain_schedule_reset(alg_pid_gain_schedule_t *me, float measurement,
                                             float initial_output)
{
    if (me == NULL)
    {
        return ALG_PID_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return ALG_PID_STATUS_NOT_INITIALIZED;
    }
    return alg_pid_reset(&me->controller, measurement, initial_output);
}
