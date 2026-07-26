#include "alg_kalman.h"

#include <math.h>
#include <stddef.h>

static bool alg_kalman_scalar_is_nonnegative_finite(float value)
{
    return isfinite(value) && (value >= 0.0F);
}

alg_kalman_status_t alg_kalman_scalar_init(alg_kalman_scalar_t *me, float process_noise,
                                           float measurement_noise, float initial_estimate,
                                           float initial_covariance)
{
    if (me == NULL)
    {
        return ALG_KALMAN_STATUS_INVALID_ARGUMENT;
    }

    me->is_initialized = false;
    if (!alg_kalman_scalar_is_nonnegative_finite(process_noise) || !isfinite(measurement_noise) ||
        (measurement_noise <= 0.0F) || !isfinite(initial_estimate) ||
        !alg_kalman_scalar_is_nonnegative_finite(initial_covariance))
    {
        return ALG_KALMAN_STATUS_OUT_OF_RANGE;
    }

    me->process_noise = process_noise;
    me->measurement_noise = measurement_noise;
    me->estimate = initial_estimate;
    me->covariance = initial_covariance;
    me->gain = 0.0F;
    me->is_initialized = true;
    return ALG_KALMAN_STATUS_OK;
}

alg_kalman_status_t alg_kalman_scalar_set_noise(alg_kalman_scalar_t *me, float process_noise,
                                                float measurement_noise)
{
    if (me == NULL)
    {
        return ALG_KALMAN_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return ALG_KALMAN_STATUS_NOT_INITIALIZED;
    }
    if (!alg_kalman_scalar_is_nonnegative_finite(process_noise) || !isfinite(measurement_noise) ||
        (measurement_noise <= 0.0F))
    {
        return ALG_KALMAN_STATUS_OUT_OF_RANGE;
    }

    me->process_noise = process_noise;
    me->measurement_noise = measurement_noise;
    return ALG_KALMAN_STATUS_OK;
}

alg_kalman_status_t alg_kalman_scalar_reset(alg_kalman_scalar_t *me, float initial_estimate,
                                            float initial_covariance)
{
    if (me == NULL)
    {
        return ALG_KALMAN_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return ALG_KALMAN_STATUS_NOT_INITIALIZED;
    }
    if (!isfinite(initial_estimate) || !alg_kalman_scalar_is_nonnegative_finite(initial_covariance))
    {
        return ALG_KALMAN_STATUS_OUT_OF_RANGE;
    }

    me->estimate = initial_estimate;
    me->covariance = initial_covariance;
    me->gain = 0.0F;
    return ALG_KALMAN_STATUS_OK;
}

alg_kalman_status_t alg_kalman_scalar_predict(alg_kalman_scalar_t *me, float state_delta)
{
    if (me == NULL)
    {
        return ALG_KALMAN_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return ALG_KALMAN_STATUS_NOT_INITIALIZED;
    }
    if (!isfinite(state_delta))
    {
        return ALG_KALMAN_STATUS_OUT_OF_RANGE;
    }

    me->estimate += state_delta;
    me->covariance += me->process_noise;
    if (!isfinite(me->estimate) || !alg_kalman_scalar_is_nonnegative_finite(me->covariance))
    {
        return ALG_KALMAN_STATUS_NUMERICAL_ERROR;
    }
    return ALG_KALMAN_STATUS_OK;
}

alg_kalman_status_t alg_kalman_scalar_correct(alg_kalman_scalar_t *me, float measurement,
                                              float *output)
{
    float innovation_covariance;

    if ((me == NULL) || (output == NULL))
    {
        return ALG_KALMAN_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return ALG_KALMAN_STATUS_NOT_INITIALIZED;
    }
    if (!isfinite(measurement))
    {
        return ALG_KALMAN_STATUS_OUT_OF_RANGE;
    }

    innovation_covariance = me->covariance + me->measurement_noise;
    if (!isfinite(innovation_covariance) || (innovation_covariance <= 0.0F))
    {
        return ALG_KALMAN_STATUS_NUMERICAL_ERROR;
    }

    me->gain = me->covariance / innovation_covariance;
    me->estimate += me->gain * (measurement - me->estimate);
    me->covariance = (1.0F - me->gain) * me->covariance;
    if (!isfinite(me->estimate) || !alg_kalman_scalar_is_nonnegative_finite(me->covariance))
    {
        return ALG_KALMAN_STATUS_NUMERICAL_ERROR;
    }

    *output = me->estimate;
    return ALG_KALMAN_STATUS_OK;
}

alg_kalman_status_t alg_kalman_scalar_update(alg_kalman_scalar_t *me, float measurement,
                                             float *output)
{
    alg_kalman_status_t status;

    if ((me == NULL) || (output == NULL))
    {
        return ALG_KALMAN_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return ALG_KALMAN_STATUS_NOT_INITIALIZED;
    }
    if (!isfinite(measurement))
    {
        return ALG_KALMAN_STATUS_OUT_OF_RANGE;
    }

    status = alg_kalman_scalar_predict(me, 0.0F);

    if (status != ALG_KALMAN_STATUS_OK)
    {
        return status;
    }
    return alg_kalman_scalar_correct(me, measurement, output);
}
