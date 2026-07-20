#include "alg_kalman.h"

#include <math.h>
#include <stddef.h>

static bool AlgKalmanScalar_IsNonnegativeFinite(float value)
{
    return isfinite(value) && (value >= 0.0F);
}

AlgKalmanStatus_t AlgKalmanScalar_Init(AlgKalmanScalar_t *self,
                                       float process_noise,
                                       float measurement_noise,
                                       float initial_estimate,
                                       float initial_covariance)
{
    if (self == NULL)
    {
        return ALG_KALMAN_STATUS_INVALID_ARGUMENT;
    }

    self->is_initialized = false;
    if (!AlgKalmanScalar_IsNonnegativeFinite(process_noise) ||
        !isfinite(measurement_noise) || (measurement_noise <= 0.0F) ||
        !isfinite(initial_estimate) ||
        !AlgKalmanScalar_IsNonnegativeFinite(initial_covariance))
    {
        return ALG_KALMAN_STATUS_OUT_OF_RANGE;
    }

    self->process_noise = process_noise;
    self->measurement_noise = measurement_noise;
    self->estimate = initial_estimate;
    self->covariance = initial_covariance;
    self->gain = 0.0F;
    self->is_initialized = true;
    return ALG_KALMAN_STATUS_OK;
}

AlgKalmanStatus_t AlgKalmanScalar_SetNoise(AlgKalmanScalar_t *self,
                                           float process_noise,
                                           float measurement_noise)
{
    if (self == NULL)
    {
        return ALG_KALMAN_STATUS_INVALID_ARGUMENT;
    }
    if (!self->is_initialized)
    {
        return ALG_KALMAN_STATUS_NOT_INITIALIZED;
    }
    if (!AlgKalmanScalar_IsNonnegativeFinite(process_noise) ||
        !isfinite(measurement_noise) || (measurement_noise <= 0.0F))
    {
        return ALG_KALMAN_STATUS_OUT_OF_RANGE;
    }

    self->process_noise = process_noise;
    self->measurement_noise = measurement_noise;
    return ALG_KALMAN_STATUS_OK;
}

AlgKalmanStatus_t AlgKalmanScalar_Reset(AlgKalmanScalar_t *self,
                                        float initial_estimate,
                                        float initial_covariance)
{
    if (self == NULL)
    {
        return ALG_KALMAN_STATUS_INVALID_ARGUMENT;
    }
    if (!self->is_initialized)
    {
        return ALG_KALMAN_STATUS_NOT_INITIALIZED;
    }
    if (!isfinite(initial_estimate) ||
        !AlgKalmanScalar_IsNonnegativeFinite(initial_covariance))
    {
        return ALG_KALMAN_STATUS_OUT_OF_RANGE;
    }

    self->estimate = initial_estimate;
    self->covariance = initial_covariance;
    self->gain = 0.0F;
    return ALG_KALMAN_STATUS_OK;
}

AlgKalmanStatus_t AlgKalmanScalar_Predict(AlgKalmanScalar_t *self,
                                          float state_delta)
{
    if (self == NULL)
    {
        return ALG_KALMAN_STATUS_INVALID_ARGUMENT;
    }
    if (!self->is_initialized)
    {
        return ALG_KALMAN_STATUS_NOT_INITIALIZED;
    }
    if (!isfinite(state_delta))
    {
        return ALG_KALMAN_STATUS_OUT_OF_RANGE;
    }

    self->estimate += state_delta;
    self->covariance += self->process_noise;
    if (!isfinite(self->estimate) ||
        !AlgKalmanScalar_IsNonnegativeFinite(self->covariance))
    {
        return ALG_KALMAN_STATUS_NUMERICAL_ERROR;
    }
    return ALG_KALMAN_STATUS_OK;
}

AlgKalmanStatus_t AlgKalmanScalar_Correct(AlgKalmanScalar_t *self,
                                          float measurement,
                                          float *output)
{
    float innovation_covariance;

    if ((self == NULL) || (output == NULL))
    {
        return ALG_KALMAN_STATUS_INVALID_ARGUMENT;
    }
    if (!self->is_initialized)
    {
        return ALG_KALMAN_STATUS_NOT_INITIALIZED;
    }
    if (!isfinite(measurement))
    {
        return ALG_KALMAN_STATUS_OUT_OF_RANGE;
    }

    innovation_covariance = self->covariance + self->measurement_noise;
    if (!isfinite(innovation_covariance) || (innovation_covariance <= 0.0F))
    {
        return ALG_KALMAN_STATUS_NUMERICAL_ERROR;
    }

    self->gain = self->covariance / innovation_covariance;
    self->estimate += self->gain * (measurement - self->estimate);
    self->covariance = (1.0F - self->gain) * self->covariance;
    if (!isfinite(self->estimate) ||
        !AlgKalmanScalar_IsNonnegativeFinite(self->covariance))
    {
        return ALG_KALMAN_STATUS_NUMERICAL_ERROR;
    }

    *output = self->estimate;
    return ALG_KALMAN_STATUS_OK;
}

AlgKalmanStatus_t AlgKalmanScalar_Update(AlgKalmanScalar_t *self,
                                         float measurement,
                                         float *output)
{
    AlgKalmanStatus_t status;

    if ((self == NULL) || (output == NULL))
    {
        return ALG_KALMAN_STATUS_INVALID_ARGUMENT;
    }
    if (!self->is_initialized)
    {
        return ALG_KALMAN_STATUS_NOT_INITIALIZED;
    }
    if (!isfinite(measurement))
    {
        return ALG_KALMAN_STATUS_OUT_OF_RANGE;
    }

    status = AlgKalmanScalar_Predict(self, 0.0F);

    if (status != ALG_KALMAN_STATUS_OK)
    {
        return status;
    }
    return AlgKalmanScalar_Correct(self, measurement, output);
}
