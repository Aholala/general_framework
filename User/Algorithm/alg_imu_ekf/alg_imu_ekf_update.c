#include "alg_imu_ekf_internal.h"

#include <math.h>
#include <stddef.h>

#define ALG_IMU_EKF_MINIMUM_NORM      (1.0e-6F)
#define ALG_IMU_EKF_SINGULAR_THRESHOLD (1.0e-12F)

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
    self->process_noise[(4U * ALG_IMU_EKF_STATE_DIMENSION) + 4U] = bias_variance;
    self->process_noise[(5U * ALG_IMU_EKF_STATE_DIMENSION) + 5U] = bias_variance;
}

static void AlgImuEkf_ApplyBiasFading(AlgImuEkf_t *self)
{
    const float bias_scale = sqrtf(self->config.gyro_bias_fading_factor);
    size_t row;
    size_t column;
    float scale;

    for (row = 0U; row < ALG_IMU_EKF_STATE_DIMENSION; ++row)
    {
        for (column = 0U; column < ALG_IMU_EKF_STATE_DIMENSION; ++column)
        {
            scale = 1.0F;
            if (row >= 4U)
            {
                scale *= bias_scale;
            }
            if (column >= 4U)
            {
                scale *= bias_scale;
            }
            self->covariance[(row * ALG_IMU_EKF_STATE_DIMENSION) + column] *= scale;
        }
    }
}

static bool AlgImuEkf_InvertSymmetric3x3(const float matrix[9], float inverse[9])
{
    const float cofactor_00 = (matrix[4] * matrix[8]) - (matrix[5] * matrix[7]);
    const float cofactor_01 = (matrix[5] * matrix[6]) - (matrix[3] * matrix[8]);
    const float cofactor_02 = (matrix[3] * matrix[7]) - (matrix[4] * matrix[6]);
    const float determinant = (matrix[0] * cofactor_00) +
                              (matrix[1] * cofactor_01) +
                              (matrix[2] * cofactor_02);

    if (!isfinite(determinant) ||
        (fabsf(determinant) <= ALG_IMU_EKF_SINGULAR_THRESHOLD))
    {
        return false;
    }

    inverse[0] = cofactor_00 / determinant;
    inverse[1] = ((matrix[2] * matrix[7]) - (matrix[1] * matrix[8])) /
                 determinant;
    inverse[2] = ((matrix[1] * matrix[5]) - (matrix[2] * matrix[4])) /
                 determinant;
    inverse[3] = cofactor_01 / determinant;
    inverse[4] = ((matrix[0] * matrix[8]) - (matrix[2] * matrix[6])) /
                 determinant;
    inverse[5] = ((matrix[2] * matrix[3]) - (matrix[0] * matrix[5])) /
                 determinant;
    inverse[6] = cofactor_02 / determinant;
    inverse[7] = ((matrix[1] * matrix[6]) - (matrix[0] * matrix[7])) /
                 determinant;
    inverse[8] = ((matrix[0] * matrix[4]) - (matrix[1] * matrix[3])) /
                 determinant;
    return AlgImuEkfInternal_IsFiniteArray(inverse, 9U);
}

static AlgImuEkfStatus_t AlgImuEkf_ComputeInnovationStatistics(
    AlgImuEkf_t *self,
    const float normalized_measurement[3],
    float base_measurement_variance)
{
    float *predicted_measurement = self->innovation_workspace;
    float *measurement_jacobian = predicted_measurement + 3U;
    float *measurement_covariance_product = measurement_jacobian + 18U;
    float *innovation_covariance = measurement_covariance_product + 18U;
    float *innovation_covariance_inverse = innovation_covariance + 9U;
    size_t row;
    size_t column;
    size_t shared_index;
    float accumulator;
    float transformed_innovation[3];
    AlgKalmanStatus_t kalman_status;

    kalman_status = AlgImuEkfInternal_MeasurementFunction(
        self->state,
        ALG_IMU_EKF_STATE_DIMENSION,
        ALG_IMU_EKF_MEASUREMENT_DIMENSION,
        predicted_measurement,
        self);
    if (kalman_status != ALG_KALMAN_STATUS_OK)
    {
        return ALG_IMU_EKF_STATUS_KALMAN_ERROR;
    }
    kalman_status = AlgImuEkfInternal_MeasurementJacobian(
        self->state,
        ALG_IMU_EKF_STATE_DIMENSION,
        ALG_IMU_EKF_MEASUREMENT_DIMENSION,
        measurement_jacobian,
        self);
    if (kalman_status != ALG_KALMAN_STATUS_OK)
    {
        return ALG_IMU_EKF_STATUS_KALMAN_ERROR;
    }

    for (row = 0U; row < 3U; ++row)
    {
        self->innovation[row] = normalized_measurement[row] -
                                predicted_measurement[row];
        for (column = 0U; column < ALG_IMU_EKF_STATE_DIMENSION; ++column)
        {
            accumulator = 0.0F;
            for (shared_index = 0U;
                 shared_index < ALG_IMU_EKF_STATE_DIMENSION;
                 ++shared_index)
            {
                accumulator += measurement_jacobian[
                                   (row * ALG_IMU_EKF_STATE_DIMENSION) + shared_index] *
                               self->covariance[
                                   (shared_index * ALG_IMU_EKF_STATE_DIMENSION) + column];
            }
            measurement_covariance_product[
                (row * ALG_IMU_EKF_STATE_DIMENSION) + column] = accumulator;
        }
    }

    for (row = 0U; row < 3U; ++row)
    {
        for (column = 0U; column < 3U; ++column)
        {
            accumulator = 0.0F;
            for (shared_index = 0U;
                 shared_index < ALG_IMU_EKF_STATE_DIMENSION;
                 ++shared_index)
            {
                accumulator += measurement_covariance_product[
                                   (row * ALG_IMU_EKF_STATE_DIMENSION) + shared_index] *
                               measurement_jacobian[
                                   (column * ALG_IMU_EKF_STATE_DIMENSION) + shared_index];
            }
            innovation_covariance[(row * 3U) + column] = accumulator +
                ((row == column) ? base_measurement_variance : 0.0F);
        }
    }

    if (!AlgImuEkf_InvertSymmetric3x3(innovation_covariance,
                                      innovation_covariance_inverse))
    {
        return ALG_IMU_EKF_STATUS_NUMERICAL_ERROR;
    }
    for (row = 0U; row < 3U; ++row)
    {
        transformed_innovation[row] =
            (innovation_covariance_inverse[(row * 3U)] * self->innovation[0]) +
            (innovation_covariance_inverse[(row * 3U) + 1U] * self->innovation[1]) +
            (innovation_covariance_inverse[(row * 3U) + 2U] * self->innovation[2]);
    }
    self->last_normalized_innovation_squared =
        (self->innovation[0] * transformed_innovation[0]) +
        (self->innovation[1] * transformed_innovation[1]) +
        (self->innovation[2] * transformed_innovation[2]);
    return isfinite(self->last_normalized_innovation_squared)
               ? ALG_IMU_EKF_STATUS_OK
               : ALG_IMU_EKF_STATUS_NUMERICAL_ERROR;
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

    AlgImuEkf_ApplyBiasFading(self);
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
    const float accelerometer_m_s2[3],
    float delta_time_s)
{
    float raw_norm;
    float filtered_norm;
    float relative_deviation;
    float normalized_measurement[3];
    float base_measurement_variance;
    float adaptation_ratio;
    float noise_scale;
    size_t index;
    AlgFilterStatus_t filter_status;
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
    if (!AlgImuEkfInternal_IsFiniteArray(accelerometer_m_s2, 3U) ||
        !isfinite(delta_time_s) || (delta_time_s <= 0.0F))
    {
        return ALG_IMU_EKF_STATUS_OUT_OF_RANGE;
    }

    raw_norm = sqrtf((accelerometer_m_s2[0] * accelerometer_m_s2[0]) +
                     (accelerometer_m_s2[1] * accelerometer_m_s2[1]) +
                     (accelerometer_m_s2[2] * accelerometer_m_s2[2]));
    if (!isfinite(raw_norm) || (raw_norm <= ALG_IMU_EKF_MINIMUM_NORM))
    {
        self->was_accelerometer_used = false;
        return ALG_IMU_EKF_STATUS_ACCELEROMETER_REJECTED;
    }

    relative_deviation = fabsf(raw_norm - self->config.gravity_m_s2) /
                         self->config.gravity_m_s2;
    self->last_accelerometer_norm_m_s2 = raw_norm;
    self->last_accelerometer_deviation_g = relative_deviation;
    if (relative_deviation > self->config.accelerometer_rejection_threshold_g)
    {
        self->was_accelerometer_used = false;
        return ALG_IMU_EKF_STATUS_ACCELEROMETER_REJECTED;
    }

    for (index = 0U; index < 3U; ++index)
    {
        filter_status = AlgFilterLowPass_Update(
            &self->accelerometer_filter[index],
            accelerometer_m_s2[index],
            delta_time_s,
            &self->filtered_accelerometer_m_s2[index]);
        if (filter_status != ALG_FILTER_STATUS_OK)
        {
            return ALG_IMU_EKF_STATUS_NUMERICAL_ERROR;
        }
    }

    filtered_norm = sqrtf(
        (self->filtered_accelerometer_m_s2[0] *
         self->filtered_accelerometer_m_s2[0]) +
        (self->filtered_accelerometer_m_s2[1] *
         self->filtered_accelerometer_m_s2[1]) +
        (self->filtered_accelerometer_m_s2[2] *
         self->filtered_accelerometer_m_s2[2]));
    if (!isfinite(filtered_norm) || (filtered_norm <= ALG_IMU_EKF_MINIMUM_NORM))
    {
        self->was_accelerometer_used = false;
        return ALG_IMU_EKF_STATUS_ACCELEROMETER_REJECTED;
    }
    for (index = 0U; index < 3U; ++index)
    {
        normalized_measurement[index] =
            self->filtered_accelerometer_m_s2[index] / filtered_norm;
    }

    base_measurement_variance =
        self->config.accelerometer_direction_noise_std *
        self->config.accelerometer_direction_noise_std;
    status = AlgImuEkf_ComputeInnovationStatistics(self,
                                                   normalized_measurement,
                                                   base_measurement_variance);
    if (status != ALG_IMU_EKF_STATUS_OK)
    {
        return status;
    }
    if (self->last_normalized_innovation_squared >
        self->config.chi_square_rejection_threshold)
    {
        self->last_measurement_noise_scale =
            self->config.maximum_measurement_noise_scale;
        self->was_accelerometer_used = false;
        return ALG_IMU_EKF_STATUS_ACCELEROMETER_REJECTED;
    }

    noise_scale = 1.0F;
    if (self->last_normalized_innovation_squared >
        self->config.chi_square_adaptation_threshold)
    {
        adaptation_ratio =
            (self->last_normalized_innovation_squared -
             self->config.chi_square_adaptation_threshold) /
            (self->config.chi_square_rejection_threshold -
             self->config.chi_square_adaptation_threshold);
        noise_scale += (self->config.maximum_measurement_noise_scale - 1.0F) *
                       adaptation_ratio * adaptation_ratio;
    }
    self->last_measurement_noise_scale = noise_scale;
    for (index = 0U; index < 9U; ++index)
    {
        self->measurement_noise[index] = 0.0F;
    }
    for (index = 0U; index < 3U; ++index)
    {
        self->measurement_noise[(index * 3U) + index] =
            base_measurement_variance * noise_scale;
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
    status = AlgImuEkf_CorrectAccelerometer(self,
                                            accelerometer_m_s2,
                                            delta_time_s);
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
