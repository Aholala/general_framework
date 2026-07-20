#include "alg_pid_internal.h"

#include <math.h>
#include <stddef.h>

static AlgPidStatus_t AlgPidGainSchedule_ValidatePoints(
    const AlgPidGainPoint_t *gain_points,
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
        if ((point_index > 0U) &&
            (gain_points[point_index].operating_point <=
             gain_points[point_index - 1U].operating_point))
        {
            return ALG_PID_STATUS_OUT_OF_RANGE;
        }
    }
    return ALG_PID_STATUS_OK;
}

static void AlgPidGainSchedule_Interpolate(const AlgPidGainSchedule_t *self,
                                           float operating_point,
                                           float *proportional_gain,
                                           float *integral_gain,
                                           float *derivative_gain)
{
    const AlgPidGainPoint_t *lower;
    const AlgPidGainPoint_t *upper;
    size_t point_index;
    float interpolation_factor;

    if ((self->gain_point_count == 1U) ||
        (operating_point <= self->gain_points[0].operating_point))
    {
        lower = &self->gain_points[0];
        upper = lower;
    }
    else if (operating_point >=
             self->gain_points[self->gain_point_count - 1U].operating_point)
    {
        lower = &self->gain_points[self->gain_point_count - 1U];
        upper = lower;
    }
    else
    {
        lower = &self->gain_points[0];
        upper = &self->gain_points[1];
        for (point_index = 1U; point_index < self->gain_point_count; ++point_index)
        {
            if (operating_point <= self->gain_points[point_index].operating_point)
            {
                lower = &self->gain_points[point_index - 1U];
                upper = &self->gain_points[point_index];
                break;
            }
        }
    }

    interpolation_factor = 0.0F;
    if (upper != lower)
    {
        interpolation_factor =
            (operating_point - lower->operating_point) /
            (upper->operating_point - lower->operating_point);
    }
    *proportional_gain = lower->proportional_gain +
                         interpolation_factor *
                             (upper->proportional_gain - lower->proportional_gain);
    *integral_gain = lower->integral_gain +
                     interpolation_factor *
                         (upper->integral_gain - lower->integral_gain);
    *derivative_gain = lower->derivative_gain +
                       interpolation_factor *
                           (upper->derivative_gain - lower->derivative_gain);
}

AlgPidStatus_t AlgPidGainSchedule_Init(AlgPidGainSchedule_t *self,
                                       const AlgPidConfig_t *base_config,
                                       const AlgPidGainPoint_t *gain_points,
                                       size_t gain_point_count)
{
    AlgPidStatus_t status;

    if (self == NULL)
    {
        return ALG_PID_STATUS_INVALID_ARGUMENT;
    }

    self->is_initialized = false;
    status = AlgPidGainSchedule_ValidatePoints(gain_points, gain_point_count);
    if (status != ALG_PID_STATUS_OK)
    {
        return status;
    }
    status = AlgPid_Init(&self->controller, base_config);
    if (status != ALG_PID_STATUS_OK)
    {
        return status;
    }

    self->gain_points = gain_points;
    self->gain_point_count = gain_point_count;
    self->is_initialized = true;
    return ALG_PID_STATUS_OK;
}

AlgPidStatus_t AlgPidGainSchedule_Update(AlgPidGainSchedule_t *self,
                                         float operating_point,
                                         const AlgPidInput_t *input,
                                         float *output)
{
    if ((self == NULL) || (input == NULL) || (output == NULL))
    {
        return ALG_PID_STATUS_INVALID_ARGUMENT;
    }
    if (!self->is_initialized)
    {
        return ALG_PID_STATUS_NOT_INITIALIZED;
    }
    if (!isfinite(operating_point))
    {
        return ALG_PID_STATUS_OUT_OF_RANGE;
    }

    AlgPidGainSchedule_Interpolate(self,
                                   operating_point,
                                   &self->controller.config.proportional_gain,
                                   &self->controller.config.integral_gain,
                                   &self->controller.config.derivative_gain);
    return AlgPid_UpdateAdvanced(&self->controller, input, output);
}

AlgPidStatus_t AlgPidGainSchedule_Reset(AlgPidGainSchedule_t *self,
                                        float measurement,
                                        float initial_output)
{
    if (self == NULL)
    {
        return ALG_PID_STATUS_INVALID_ARGUMENT;
    }
    if (!self->is_initialized)
    {
        return ALG_PID_STATUS_NOT_INITIALIZED;
    }
    return AlgPid_Reset(&self->controller, measurement, initial_output);
}
