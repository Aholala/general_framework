#include "alg_imu_ekf_internal.h"

#include <math.h>
#include <stddef.h>

#define ALG_IMU_EKF_MINIMUM_NORM (1.0e-6F)
#define ALG_IMU_EKF_SINGULAR_THRESHOLD (1.0e-12F)

static void alg_imu_ekf_update_process_noise(alg_imu_ekf_t *me, float delta_time_s)
{
    float gyro_mapping[4U * 3U];
    float gyro_variance_factor;
    float bias_variance;
    size_t row;
    size_t column;
    size_t axis;
    float accumulator;

    for (row = 0U; row < (ALG_IMU_EKF_STATE_DIMENSION * ALG_IMU_EKF_STATE_DIMENSION); ++row)
    {
        me->process_noise[row] = 0.0F;
    }

    gyro_mapping[0] = -me->state[1];
    gyro_mapping[1] = -me->state[2];
    gyro_mapping[2] = -me->state[3];
    gyro_mapping[3] = me->state[0];
    gyro_mapping[4] = -me->state[3];
    gyro_mapping[5] = me->state[2];
    gyro_mapping[6] = me->state[3];
    gyro_mapping[7] = me->state[0];
    gyro_mapping[8] = -me->state[1];
    gyro_mapping[9] = -me->state[2];
    gyro_mapping[10] = me->state[1];
    gyro_mapping[11] = me->state[0];

    gyro_variance_factor = 0.25F * me->config.gyro_noise_std_rad_s *
                           me->config.gyro_noise_std_rad_s * delta_time_s * delta_time_s;
    for (row = 0U; row < 4U; ++row)
    {
        for (column = 0U; column < 4U; ++column)
        {
            accumulator = 0.0F;
            for (axis = 0U; axis < 3U; ++axis)
            {
                accumulator += gyro_mapping[(row * 3U) + axis] * gyro_mapping[(column * 3U) + axis];
            }
            me->process_noise[(row * ALG_IMU_EKF_STATE_DIMENSION) + column] =
                gyro_variance_factor * accumulator;
        }
    }

    bias_variance = me->config.gyro_bias_random_walk_std_rad_s2 *
                    me->config.gyro_bias_random_walk_std_rad_s2 * delta_time_s * delta_time_s;
    me->process_noise[(4U * ALG_IMU_EKF_STATE_DIMENSION) + 4U] = bias_variance;
    me->process_noise[(5U * ALG_IMU_EKF_STATE_DIMENSION) + 5U] = bias_variance;
}

static void alg_imu_ekf_apply_bias_fading(alg_imu_ekf_t *me)
{
    const float bias_scale = sqrtf(me->config.gyro_bias_fading_factor);
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
            me->covariance[(row * ALG_IMU_EKF_STATE_DIMENSION) + column] *= scale;
        }
    }
}

static bool alg_imu_ekf_invert_symmetric3x3(const float matrix[9], float inverse[9])
{
    const float cofactor_00 = (matrix[4] * matrix[8]) - (matrix[5] * matrix[7]);
    const float cofactor_01 = (matrix[5] * matrix[6]) - (matrix[3] * matrix[8]);
    const float cofactor_02 = (matrix[3] * matrix[7]) - (matrix[4] * matrix[6]);
    const float determinant =
        (matrix[0] * cofactor_00) + (matrix[1] * cofactor_01) + (matrix[2] * cofactor_02);

    if (!isfinite(determinant) || (fabsf(determinant) <= ALG_IMU_EKF_SINGULAR_THRESHOLD))
    {
        return false;
    }

    inverse[0] = cofactor_00 / determinant;
    inverse[1] = ((matrix[2] * matrix[7]) - (matrix[1] * matrix[8])) / determinant;
    inverse[2] = ((matrix[1] * matrix[5]) - (matrix[2] * matrix[4])) / determinant;
    inverse[3] = cofactor_01 / determinant;
    inverse[4] = ((matrix[0] * matrix[8]) - (matrix[2] * matrix[6])) / determinant;
    inverse[5] = ((matrix[2] * matrix[3]) - (matrix[0] * matrix[5])) / determinant;
    inverse[6] = cofactor_02 / determinant;
    inverse[7] = ((matrix[1] * matrix[6]) - (matrix[0] * matrix[7])) / determinant;
    inverse[8] = ((matrix[0] * matrix[4]) - (matrix[1] * matrix[3])) / determinant;
    return alg_imu_ekf_internal_is_finite_array(inverse, 9U);
}

static alg_imu_ekf_status_t
alg_imu_ekf_compute_innovation_statistics(alg_imu_ekf_t *me, const float normalized_measurement[3],
                                          float base_measurement_variance)
{
    float *predicted_measurement = me->innovation_workspace;
    float *measurement_jacobian = predicted_measurement + 3U;
    float *measurement_covariance_product = measurement_jacobian + 18U;
    float *innovation_covariance = measurement_covariance_product + 18U;
    float *innovation_covariance_inverse = innovation_covariance + 9U;
    size_t row;
    size_t column;
    size_t shared_index;
    float accumulator;
    float transformed_innovation[3];
    alg_kalman_status_t kalman_status;

    kalman_status = alg_imu_ekf_internal_measurement_function(
        me->state, ALG_IMU_EKF_STATE_DIMENSION, ALG_IMU_EKF_MEASUREMENT_DIMENSION,
        predicted_measurement, me);
    if (kalman_status != ALG_KALMAN_STATUS_OK)
    {
        return ALG_IMU_EKF_STATUS_KALMAN_ERROR;
    }
    kalman_status = alg_imu_ekf_internal_measurement_jacobian(
        me->state, ALG_IMU_EKF_STATE_DIMENSION, ALG_IMU_EKF_MEASUREMENT_DIMENSION,
        measurement_jacobian, me);
    if (kalman_status != ALG_KALMAN_STATUS_OK)
    {
        return ALG_IMU_EKF_STATUS_KALMAN_ERROR;
    }

    for (row = 0U; row < 3U; ++row)
    {
        me->innovation[row] = normalized_measurement[row] - predicted_measurement[row];
        for (column = 0U; column < ALG_IMU_EKF_STATE_DIMENSION; ++column)
        {
            accumulator = 0.0F;
            for (shared_index = 0U; shared_index < ALG_IMU_EKF_STATE_DIMENSION; ++shared_index)
            {
                accumulator +=
                    measurement_jacobian[(row * ALG_IMU_EKF_STATE_DIMENSION) + shared_index] *
                    me->covariance[(shared_index * ALG_IMU_EKF_STATE_DIMENSION) + column];
            }
            measurement_covariance_product[(row * ALG_IMU_EKF_STATE_DIMENSION) + column] =
                accumulator;
        }
    }

    for (row = 0U; row < 3U; ++row)
    {
        for (column = 0U; column < 3U; ++column)
        {
            accumulator = 0.0F;
            for (shared_index = 0U; shared_index < ALG_IMU_EKF_STATE_DIMENSION; ++shared_index)
            {
                accumulator +=
                    measurement_covariance_product[(row * ALG_IMU_EKF_STATE_DIMENSION) +
                                                   shared_index] *
                    measurement_jacobian[(column * ALG_IMU_EKF_STATE_DIMENSION) + shared_index];
            }
            innovation_covariance[(row * 3U) + column] =
                accumulator + ((row == column) ? base_measurement_variance : 0.0F);
        }
    }

    if (!alg_imu_ekf_invert_symmetric3x3(innovation_covariance, innovation_covariance_inverse))
    {
        return ALG_IMU_EKF_STATUS_NUMERICAL_ERROR;
    }
    for (row = 0U; row < 3U; ++row)
    {
        transformed_innovation[row] =
            (innovation_covariance_inverse[(row * 3U)] * me->innovation[0]) +
            (innovation_covariance_inverse[(row * 3U) + 1U] * me->innovation[1]) +
            (innovation_covariance_inverse[(row * 3U) + 2U] * me->innovation[2]);
    }
    me->last_normalized_innovation_squared = (me->innovation[0] * transformed_innovation[0]) +
                                             (me->innovation[1] * transformed_innovation[1]) +
                                             (me->innovation[2] * transformed_innovation[2]);
    return isfinite(me->last_normalized_innovation_squared) ? ALG_IMU_EKF_STATUS_OK
                                                            : ALG_IMU_EKF_STATUS_NUMERICAL_ERROR;
}

alg_imu_ekf_status_t alg_imu_ekf_predict(alg_imu_ekf_t *me, const float gyroscope_rad_s[3],
                                         float delta_time_s)
{
    alg_kalman_status_t kalman_status;
    alg_imu_ekf_status_t status;

    if ((me == NULL) || (gyroscope_rad_s == NULL))
    {
        return ALG_IMU_EKF_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return ALG_IMU_EKF_STATUS_NOT_INITIALIZED;
    }
    if (!alg_imu_ekf_internal_is_finite_array(gyroscope_rad_s, 3U) || !isfinite(delta_time_s) ||
        (delta_time_s <= 0.0F))
    {
        return ALG_IMU_EKF_STATUS_OUT_OF_RANGE;
    }

    alg_imu_ekf_apply_bias_fading(me);
    alg_imu_ekf_update_process_noise(me, delta_time_s);
    kalman_status = alg_kalman_extended_predict(&me->kalman, gyroscope_rad_s, delta_time_s);
    if (kalman_status != ALG_KALMAN_STATUS_OK)
    {
        return alg_imu_ekf_internal_map_kalman_status(kalman_status);
    }

    status = alg_imu_ekf_internal_normalize_and_project(me);
    me->was_accelerometer_used = false;
    return status;
}

alg_imu_ekf_status_t alg_imu_ekf_correct_accelerometer(alg_imu_ekf_t *me,
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
    alg_filter_status_t filter_status;
    alg_kalman_status_t kalman_status;
    alg_imu_ekf_status_t status;

    if ((me == NULL) || (accelerometer_m_s2 == NULL))
    {
        return ALG_IMU_EKF_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return ALG_IMU_EKF_STATUS_NOT_INITIALIZED;
    }
    if (!alg_imu_ekf_internal_is_finite_array(accelerometer_m_s2, 3U) || !isfinite(delta_time_s) ||
        (delta_time_s <= 0.0F))
    {
        return ALG_IMU_EKF_STATUS_OUT_OF_RANGE;
    }

    raw_norm = sqrtf((accelerometer_m_s2[0] * accelerometer_m_s2[0]) +
                     (accelerometer_m_s2[1] * accelerometer_m_s2[1]) +
                     (accelerometer_m_s2[2] * accelerometer_m_s2[2]));
    if (!isfinite(raw_norm) || (raw_norm <= ALG_IMU_EKF_MINIMUM_NORM))
    {
        me->was_accelerometer_used = false;
        return ALG_IMU_EKF_STATUS_ACCELEROMETER_REJECTED;
    }

    relative_deviation = fabsf(raw_norm - me->config.gravity_m_s2) / me->config.gravity_m_s2;
    me->last_accelerometer_norm_m_s2 = raw_norm;
    me->last_accelerometer_deviation_g = relative_deviation;
    if (relative_deviation > me->config.accelerometer_rejection_threshold_g)
    {
        me->was_accelerometer_used = false;
        return ALG_IMU_EKF_STATUS_ACCELEROMETER_REJECTED;
    }

    for (index = 0U; index < 3U; ++index)
    {
        filter_status =
            alg_filter_low_pass_update(&me->accelerometer_filter[index], accelerometer_m_s2[index],
                                       delta_time_s, &me->filtered_accelerometer_m_s2[index]);
        if (filter_status != ALG_FILTER_STATUS_OK)
        {
            return ALG_IMU_EKF_STATUS_NUMERICAL_ERROR;
        }
    }

    filtered_norm =
        sqrtf((me->filtered_accelerometer_m_s2[0] * me->filtered_accelerometer_m_s2[0]) +
              (me->filtered_accelerometer_m_s2[1] * me->filtered_accelerometer_m_s2[1]) +
              (me->filtered_accelerometer_m_s2[2] * me->filtered_accelerometer_m_s2[2]));
    if (!isfinite(filtered_norm) || (filtered_norm <= ALG_IMU_EKF_MINIMUM_NORM))
    {
        me->was_accelerometer_used = false;
        return ALG_IMU_EKF_STATUS_ACCELEROMETER_REJECTED;
    }
    for (index = 0U; index < 3U; ++index)
    {
        normalized_measurement[index] = me->filtered_accelerometer_m_s2[index] / filtered_norm;
    }

    base_measurement_variance =
        me->config.accelerometer_direction_noise_std * me->config.accelerometer_direction_noise_std;
    status = alg_imu_ekf_compute_innovation_statistics(me, normalized_measurement,
                                                       base_measurement_variance);
    if (status != ALG_IMU_EKF_STATUS_OK)
    {
        return status;
    }
    if (me->last_normalized_innovation_squared > me->config.chi_square_rejection_threshold)
    {
        me->last_measurement_noise_scale = me->config.maximum_measurement_noise_scale;
        me->was_accelerometer_used = false;
        return ALG_IMU_EKF_STATUS_ACCELEROMETER_REJECTED;
    }

    noise_scale = 1.0F;
    if (me->last_normalized_innovation_squared > me->config.chi_square_adaptation_threshold)
    {
        adaptation_ratio =
            (me->last_normalized_innovation_squared - me->config.chi_square_adaptation_threshold) /
            (me->config.chi_square_rejection_threshold -
             me->config.chi_square_adaptation_threshold);
        noise_scale += (me->config.maximum_measurement_noise_scale - 1.0F) * adaptation_ratio *
                       adaptation_ratio;
    }
    me->last_measurement_noise_scale = noise_scale;
    for (index = 0U; index < 9U; ++index)
    {
        me->measurement_noise[index] = 0.0F;
    }
    for (index = 0U; index < 3U; ++index)
    {
        me->measurement_noise[(index * 3U) + index] = base_measurement_variance * noise_scale;
    }

    kalman_status = alg_kalman_extended_correct(&me->kalman, normalized_measurement);
    if (kalman_status != ALG_KALMAN_STATUS_OK)
    {
        me->was_accelerometer_used = false;
        return alg_imu_ekf_internal_map_kalman_status(kalman_status);
    }
    status = alg_imu_ekf_internal_normalize_and_project(me);
    me->was_accelerometer_used = (status == ALG_IMU_EKF_STATUS_OK);
    return status;
}

alg_imu_ekf_status_t alg_imu_ekf_update(alg_imu_ekf_t *me, const float gyroscope_rad_s[3],
                                        const float accelerometer_m_s2[3], float delta_time_s,
                                        bool *accelerometer_used)
{
    alg_imu_ekf_status_t status;

    if ((me == NULL) || (gyroscope_rad_s == NULL) || (accelerometer_m_s2 == NULL))
    {
        return ALG_IMU_EKF_STATUS_INVALID_ARGUMENT;
    }

    status = alg_imu_ekf_predict(me, gyroscope_rad_s, delta_time_s);
    if (status != ALG_IMU_EKF_STATUS_OK)
    {
        return status;
    }
    status = alg_imu_ekf_correct_accelerometer(me, accelerometer_m_s2, delta_time_s);
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
