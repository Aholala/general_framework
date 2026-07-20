#include "alg_imu_ekf_internal.h"

#include <math.h>
#include <stddef.h>

#define ALG_IMU_EKF_MINIMUM_NORM (1.0e-6F)

static void AlgImuEkf_UpdateProcessNoise(AlgImuEkf_t *self,
                                         float delta_time_s)
{
    float gyro_mapping[4U * 3U];
    float gyro_variance_factor;
    float bias_variance;
    size_t row;
    size_t column;
    size_t axis;
    float accumulator;

    for (row = 0U; row <
                         (ALG_IMU_EKF_STATE_DIMENSION *
                          ALG_IMU_EKF_STATE_DIMENSION);
         ++row)
    {
        self->process_noise[row] = 0.0F;
    }

    gyro_mapping[0] = -self->state[1];
    gyro_mapping[1] = -self->state[2];
    gyro_mapping[2] = -self->state[3];
    gyro_mapping[3] = self->state[0];
    gyro_mapping[4] = -self->state[3];
    gyro_mapping[5] = self->state[2];
    gyro_mapping[6] = self->state[3];
    gyro_mapping[7] = self->state[0];
    gyro_mapping[8] = -self->state[1];
    gyro_mapping[9] = -self->state[2];
    gyro_mapping[10] = self->state[1];
    gyro_mapping[11] = self->state[0];

    gyro_variance_factor = 0.25F *
                           self->config.gyro_noise_std_rad_s *
                           self->config.gyro_noise_std_rad_s *
                           delta_time_s * delta_time_s;
    for (row = 0U; row < 4U; ++row)
    {
        for (column = 0U; column < 4U; ++column)
        {
            accumulator = 0.0F;
            for (axis = 0U; axis < 3U; ++axis)
            {
                accumulator += gyro_mapping[(row * 3U) + axis] *
                               gyro_mapping[(column * 3U) + axis];
            }
            self->process_noise[(row * ALG_IMU_EKF_STATE_DIMENSION) + column] =
                gyro_variance_factor * accumulator;
        }
    }

    bias_variance = self->config.gyro_bias_random_walk_std_rad_s2 *
                    self->config.gyro_bias_random_walk_std_rad_s2 *
                    delta_time_s * delta_time_s;
    for (axis = 4U; axis < ALG_IMU_EKF_STATE_DIMENSION; ++axis)
    {
        self->process_noise[(axis * ALG_IMU_EKF_STATE_DIMENSION) + axis] =
            bias_variance;
    }
}

AlgImuEkfStatus_t AlgImuEkf_Predict(
    AlgImuEkf_t *self,
    const float gyroscope_rad_s[3],
    float delta_time_s)
{
    AlgKalmanStatus_t kalman_status;
    AlgImuEkfStatus_t status;

    if ((self == NULL) || (gyroscope_rad_s == NULL))
    {
        return ALG_IMU_EKF_STATUS_INVALID_ARGUMENT;
    }
    if (!self->is_initialized)
    {
        return ALG_IMU_EKF_STATUS_NOT_INITIALIZED;
    }
    if (!AlgImuEkfInternal_IsFiniteArray(gyroscope_rad_s, 3U) ||
        !isfinite(delta_time_s) || (delta_time_s <= 0.0F))
    {
        return ALG_IMU_EKF_STATUS_OUT_OF_RANGE;
    }

    AlgImuEkf_UpdateProcessNoise(self, delta_time_s);
    kalman_status = AlgKalmanExtended_Predict(&self->kalman,
                                              gyroscope_rad_s,
                                              delta_time_s);
    if (kalman_status != ALG_KALMAN_STATUS_OK)
    {
        return AlgImuEkfInternal_MapKalmanStatus(kalman_status);
    }

    status = AlgImuEkfInternal_NormalizeAndProject(self);
    self->was_accelerometer_used = false;
    return status;
}

AlgImuEkfStatus_t AlgImuEkf_CorrectAccelerometer(
    AlgImuEkf_t *self,
    const float accelerometer_m_s2[3])
{
    float accelerometer_norm;
    float relative_deviation;
    float normalized_measurement[3];
    float noise_scale;
    float measurement_variance;
    size_t index;
    AlgKalmanStatus_t kalman_status;
    AlgImuEkfStatus_t status;

    if ((self == NULL) || (accelerometer_m_s2 == NULL))
    {
        return ALG_IMU_EKF_STATUS_INVALID_ARGUMENT;
    }
    if (!self->is_initialized)
    {
        return ALG_IMU_EKF_STATUS_NOT_INITIALIZED;
    }
    if (!AlgImuEkfInternal_IsFiniteArray(accelerometer_m_s2, 3U))
    {
        return ALG_IMU_EKF_STATUS_OUT_OF_RANGE;
    }

    accelerometer_norm = sqrtf(
        (accelerometer_m_s2[0] * accelerometer_m_s2[0]) +
        (accelerometer_m_s2[1] * accelerometer_m_s2[1]) +
        (accelerometer_m_s2[2] * accelerometer_m_s2[2]));
    if (!isfinite(accelerometer_norm) ||
        (accelerometer_norm <= ALG_IMU_EKF_MINIMUM_NORM))
    {
        self->was_accelerometer_used = false;
        return ALG_IMU_EKF_STATUS_ACCELEROMETER_REJECTED;
    }

    relative_deviation = fabsf(accelerometer_norm - self->config.gravity_m_s2) /
                         self->config.gravity_m_s2;
    self->last_accelerometer_norm_m_s2 = accelerometer_norm;
    self->last_accelerometer_deviation_g = relative_deviation;
    if (relative_deviation > self->config.accelerometer_rejection_threshold_g)
    {
        self->was_accelerometer_used = false;
        return ALG_IMU_EKF_STATUS_ACCELEROMETER_REJECTED;
    }

    for (index = 0U; index < 3U; ++index)
    {
        normalized_measurement[index] =
            accelerometer_m_s2[index] / accelerometer_norm;
    }
    noise_scale = 1.0F +
                  (self->config.accelerometer_noise_multiplier *
                   relative_deviation * relative_deviation /
                   (self->config.accelerometer_rejection_threshold_g *
                    self->config.accelerometer_rejection_threshold_g));
    measurement_variance =
        self->config.accelerometer_direction_noise_std *
        self->config.accelerometer_direction_noise_std * noise_scale;
    for (index = 0U; index <
                         (ALG_IMU_EKF_MEASUREMENT_DIMENSION *
                          ALG_IMU_EKF_MEASUREMENT_DIMENSION);
         ++index)
    {
        self->measurement_noise[index] = 0.0F;
    }
    for (index = 0U; index < ALG_IMU_EKF_MEASUREMENT_DIMENSION; ++index)
    {
        self->measurement_noise[
            (index * ALG_IMU_EKF_MEASUREMENT_DIMENSION) + index] =
            measurement_variance;
    }

    kalman_status = AlgKalmanExtended_Correct(&self->kalman,
                                              normalized_measurement);
    if (kalman_status != ALG_KALMAN_STATUS_OK)
    {
        self->was_accelerometer_used = false;
        return AlgImuEkfInternal_MapKalmanStatus(kalman_status);
    }
    status = AlgImuEkfInternal_NormalizeAndProject(self);
    self->was_accelerometer_used = (status == ALG_IMU_EKF_STATUS_OK);
    return status;
}

AlgImuEkfStatus_t AlgImuEkf_Update(
    AlgImuEkf_t *self,
    const float gyroscope_rad_s[3],
    const float accelerometer_m_s2[3],
    float delta_time_s,
    bool *accelerometer_used)
{
    AlgImuEkfStatus_t status;

    if ((self == NULL) || (gyroscope_rad_s == NULL) ||
        (accelerometer_m_s2 == NULL))
    {
        return ALG_IMU_EKF_STATUS_INVALID_ARGUMENT;
    }

    status = AlgImuEkf_Predict(self, gyroscope_rad_s, delta_time_s);
    if (status != ALG_IMU_EKF_STATUS_OK)
    {
        return status;
    }
    status = AlgImuEkf_CorrectAccelerometer(self, accelerometer_m_s2);
    if (status == ALG_IMU_EKF_STATUS_ACCELEROMETER_REJECTED)
    {
        if (accelerometer_used != NULL)
        {
            *accelerometer_used = false;
        }
        return ALG_IMU_EKF_STATUS_OK;
    }
    if (status != ALG_IMU_EKF_STATUS_OK)
    {
        return status;
    }
    if (accelerometer_used != NULL)
    {
        *accelerometer_used = true;
    }
    return ALG_IMU_EKF_STATUS_OK;
}
