#include "alg_imu_ekf_internal.h"

#include <math.h>
#include <stddef.h>

#define ALG_IMU_EKF_STANDARD_GRAVITY_M_S2 (9.80665F)
#define ALG_IMU_EKF_MINIMUM_NORM (1.0e-6F)

bool alg_imu_ekf_internal_is_finite_array(const float *values, size_t value_count)
{
    size_t index;

    if (values == NULL)
    {
        return false;
    }
    for (index = 0U; index < value_count; ++index)
    {
        if (!isfinite(values[index]))
        {
            return false;
        }
    }
    return true;
}

alg_imu_ekf_status_t alg_imu_ekf_internal_map_kalman_status(alg_kalman_status_t status)
{
    switch (status)
    {
    case ALG_KALMAN_STATUS_OK:
        return ALG_IMU_EKF_STATUS_OK;
    case ALG_KALMAN_STATUS_INVALID_ARGUMENT:
        return ALG_IMU_EKF_STATUS_INVALID_ARGUMENT;
    case ALG_KALMAN_STATUS_OUT_OF_RANGE:
        return ALG_IMU_EKF_STATUS_OUT_OF_RANGE;
    case ALG_KALMAN_STATUS_NOT_INITIALIZED:
        return ALG_IMU_EKF_STATUS_NOT_INITIALIZED;
    case ALG_KALMAN_STATUS_NUMERICAL_ERROR:
        return ALG_IMU_EKF_STATUS_NUMERICAL_ERROR;
    default:
        return ALG_IMU_EKF_STATUS_KALMAN_ERROR;
    }
}

static void alg_imu_ekf_clear(float *values, size_t value_count)
{
    size_t index;

    for (index = 0U; index < value_count; ++index)
    {
        values[index] = 0.0F;
    }
}

static void alg_imu_ekf_reset_covariance(alg_imu_ekf_t *me)
{
    size_t index;

    alg_imu_ekf_clear(me->covariance, ALG_IMU_EKF_STATE_DIMENSION * ALG_IMU_EKF_STATE_DIMENSION);
    for (index = 0U; index < 4U; ++index)
    {
        me->covariance[(index * ALG_IMU_EKF_STATE_DIMENSION) + index] =
            me->config.initial_attitude_variance;
    }
    for (index = 4U; index < ALG_IMU_EKF_STATE_DIMENSION; ++index)
    {
        me->covariance[(index * ALG_IMU_EKF_STATE_DIMENSION) + index] =
            me->config.initial_gyro_bias_variance;
    }
}

alg_imu_ekf_status_t alg_imu_ekf_internal_normalize_and_project(alg_imu_ekf_t *me)
{
    float quaternion_norm;
    float normalized_quaternion[4];
    float *normalization_jacobian;
    float *temporary_covariance;
    size_t row;
    size_t column;
    size_t shared_index;
    float accumulator;

    quaternion_norm = sqrtf((me->state[0] * me->state[0]) + (me->state[1] * me->state[1]) +
                            (me->state[2] * me->state[2]) + (me->state[3] * me->state[3]));
    if (!isfinite(quaternion_norm) || (quaternion_norm <= ALG_IMU_EKF_MINIMUM_NORM))
    {
        return ALG_IMU_EKF_STATUS_NUMERICAL_ERROR;
    }

    for (row = 0U; row < 4U; ++row)
    {
        normalized_quaternion[row] = me->state[row] / quaternion_norm;
        me->state[row] = normalized_quaternion[row];
    }

    normalization_jacobian = me->normalization_workspace;
    temporary_covariance =
        normalization_jacobian + (ALG_IMU_EKF_STATE_DIMENSION * ALG_IMU_EKF_STATE_DIMENSION);
    alg_imu_ekf_clear(normalization_jacobian,
                      ALG_IMU_EKF_STATE_DIMENSION * ALG_IMU_EKF_STATE_DIMENSION);
    for (row = 0U; row < 4U; ++row)
    {
        for (column = 0U; column < 4U; ++column)
        {
            normalization_jacobian[(row * ALG_IMU_EKF_STATE_DIMENSION) + column] =
                (((row == column) ? 1.0F : 0.0F) -
                 (normalized_quaternion[row] * normalized_quaternion[column])) /
                quaternion_norm;
        }
    }
    for (row = 4U; row < ALG_IMU_EKF_STATE_DIMENSION; ++row)
    {
        normalization_jacobian[(row * ALG_IMU_EKF_STATE_DIMENSION) + row] = 1.0F;
    }

    for (row = 0U; row < ALG_IMU_EKF_STATE_DIMENSION; ++row)
    {
        for (column = 0U; column < ALG_IMU_EKF_STATE_DIMENSION; ++column)
        {
            accumulator = 0.0F;
            for (shared_index = 0U; shared_index < ALG_IMU_EKF_STATE_DIMENSION; ++shared_index)
            {
                accumulator +=
                    normalization_jacobian[(row * ALG_IMU_EKF_STATE_DIMENSION) + shared_index] *
                    me->covariance[(shared_index * ALG_IMU_EKF_STATE_DIMENSION) + column];
            }
            temporary_covariance[(row * ALG_IMU_EKF_STATE_DIMENSION) + column] = accumulator;
        }
    }
    for (row = 0U; row < ALG_IMU_EKF_STATE_DIMENSION; ++row)
    {
        for (column = 0U; column < ALG_IMU_EKF_STATE_DIMENSION; ++column)
        {
            accumulator = 0.0F;
            for (shared_index = 0U; shared_index < ALG_IMU_EKF_STATE_DIMENSION; ++shared_index)
            {
                accumulator +=
                    temporary_covariance[(row * ALG_IMU_EKF_STATE_DIMENSION) + shared_index] *
                    normalization_jacobian[(column * ALG_IMU_EKF_STATE_DIMENSION) + shared_index];
            }
            me->covariance[(row * ALG_IMU_EKF_STATE_DIMENSION) + column] = accumulator;
        }
    }

    for (row = 0U; row < ALG_IMU_EKF_STATE_DIMENSION; ++row)
    {
        for (column = row + 1U; column < ALG_IMU_EKF_STATE_DIMENSION; ++column)
        {
            const float average =
                0.5F * (me->covariance[(row * ALG_IMU_EKF_STATE_DIMENSION) + column] +
                        me->covariance[(column * ALG_IMU_EKF_STATE_DIMENSION) + row]);
            me->covariance[(row * ALG_IMU_EKF_STATE_DIMENSION) + column] = average;
            me->covariance[(column * ALG_IMU_EKF_STATE_DIMENSION) + row] = average;
        }
    }

    return alg_imu_ekf_internal_is_finite_array(me->covariance, ALG_IMU_EKF_STATE_DIMENSION *
                                                                    ALG_IMU_EKF_STATE_DIMENSION)
               ? ALG_IMU_EKF_STATUS_OK
               : ALG_IMU_EKF_STATUS_NUMERICAL_ERROR;
}

alg_imu_ekf_status_t alg_imu_ekf_config_init(alg_imu_ekf_config_t *config)
{
    if (config == NULL)
    {
        return ALG_IMU_EKF_STATUS_INVALID_ARGUMENT;
    }

    *config = (alg_imu_ekf_config_t){.gravity_m_s2 = ALG_IMU_EKF_STANDARD_GRAVITY_M_S2,
                                     .gyro_noise_std_rad_s = 0.015F,
                                     .gyro_bias_random_walk_std_rad_s2 = 0.0005F,
                                     .accelerometer_direction_noise_std = 0.03F,
                                     .accelerometer_lpf_cutoff_hz = 30.0F,
                                     .accelerometer_rejection_threshold_g = 0.20F,
                                     .chi_square_adaptation_threshold = 3.0F,
                                     .chi_square_rejection_threshold = 11.345F,
                                     .maximum_measurement_noise_scale = 20.0F,
                                     .gyro_bias_fading_factor = 1.0001F,
                                     .initial_attitude_variance = 0.10F,
                                     .initial_gyro_bias_variance = 0.01F};
    return ALG_IMU_EKF_STATUS_OK;
}

static alg_imu_ekf_status_t alg_imu_ekf_validate_config(const alg_imu_ekf_config_t *config)
{
    if (config == NULL)
    {
        return ALG_IMU_EKF_STATUS_INVALID_ARGUMENT;
    }
    if (!isfinite(config->gravity_m_s2) || (config->gravity_m_s2 <= 0.0F) ||
        !isfinite(config->gyro_noise_std_rad_s) || (config->gyro_noise_std_rad_s < 0.0F) ||
        !isfinite(config->gyro_bias_random_walk_std_rad_s2) ||
        (config->gyro_bias_random_walk_std_rad_s2 < 0.0F) ||
        !isfinite(config->accelerometer_direction_noise_std) ||
        (config->accelerometer_direction_noise_std <= 0.0F) ||
        !isfinite(config->accelerometer_lpf_cutoff_hz) ||
        (config->accelerometer_lpf_cutoff_hz <= 0.0F) ||
        !isfinite(config->accelerometer_rejection_threshold_g) ||
        (config->accelerometer_rejection_threshold_g <= 0.0F) ||
        !isfinite(config->chi_square_adaptation_threshold) ||
        (config->chi_square_adaptation_threshold < 0.0F) ||
        !isfinite(config->chi_square_rejection_threshold) ||
        (config->chi_square_rejection_threshold <= config->chi_square_adaptation_threshold) ||
        !isfinite(config->maximum_measurement_noise_scale) ||
        (config->maximum_measurement_noise_scale < 1.0F) ||
        !isfinite(config->gyro_bias_fading_factor) || (config->gyro_bias_fading_factor < 1.0F) ||
        !isfinite(config->initial_attitude_variance) ||
        (config->initial_attitude_variance < 0.0F) ||
        !isfinite(config->initial_gyro_bias_variance) ||
        (config->initial_gyro_bias_variance < 0.0F))
    {
        return ALG_IMU_EKF_STATUS_OUT_OF_RANGE;
    }
    return ALG_IMU_EKF_STATUS_OK;
}

alg_imu_ekf_status_t alg_imu_ekf_init(alg_imu_ekf_t *me, const alg_imu_ekf_config_t *config)
{
    alg_kalman_extended_config_t kalman_config;
    alg_kalman_status_t kalman_status;
    alg_imu_ekf_status_t status;
    size_t index;
    float accelerometer_variance;
    alg_filter_status_t filter_status;

    if (me == NULL)
    {
        return ALG_IMU_EKF_STATUS_INVALID_ARGUMENT;
    }

    me->is_initialized = false;
    status = alg_imu_ekf_validate_config(config);
    if (status != ALG_IMU_EKF_STATUS_OK)
    {
        return status;
    }

    me->config = *config;
    alg_imu_ekf_clear(me->state, ALG_IMU_EKF_STATE_DIMENSION);
    me->state[0] = 1.0F;
    alg_imu_ekf_reset_covariance(me);
    alg_imu_ekf_clear(me->process_noise, ALG_IMU_EKF_STATE_DIMENSION * ALG_IMU_EKF_STATE_DIMENSION);
    alg_imu_ekf_clear(me->measurement_noise,
                      ALG_IMU_EKF_MEASUREMENT_DIMENSION * ALG_IMU_EKF_MEASUREMENT_DIMENSION);
    accelerometer_variance =
        config->accelerometer_direction_noise_std * config->accelerometer_direction_noise_std;
    for (index = 0U; index < ALG_IMU_EKF_MEASUREMENT_DIMENSION; ++index)
    {
        me->measurement_noise[(index * ALG_IMU_EKF_MEASUREMENT_DIMENSION) + index] =
            accelerometer_variance;
    }
    for (index = 0U; index < 3U; ++index)
    {
        filter_status = alg_filter_low_pass_init(&me->accelerometer_filter[index],
                                                 config->accelerometer_lpf_cutoff_hz);
        if (filter_status != ALG_FILTER_STATUS_OK)
        {
            return ALG_IMU_EKF_STATUS_OUT_OF_RANGE;
        }
        me->filtered_accelerometer_m_s2[index] = 0.0F;
        me->innovation[index] = 0.0F;
    }

    kalman_config = (alg_kalman_extended_config_t){
        .state_dimension = ALG_IMU_EKF_STATE_DIMENSION,
        .measurement_dimension = ALG_IMU_EKF_MEASUREMENT_DIMENSION,
        .control_dimension = ALG_IMU_EKF_CONTROL_DIMENSION,
        .state = me->state,
        .covariance = me->covariance,
        .process_noise = me->process_noise,
        .measurement_noise = me->measurement_noise,
        .workspace = me->kalman_workspace,
        .workspace_size = sizeof(me->kalman_workspace) / sizeof(me->kalman_workspace[0]),
        .state_function = alg_imu_ekf_internal_state_function,
        .state_jacobian_function = alg_imu_ekf_internal_state_jacobian,
        .measurement_function = alg_imu_ekf_internal_measurement_function,
        .measurement_jacobian_function = alg_imu_ekf_internal_measurement_jacobian,
        .user_context = me};
    kalman_status = alg_kalman_extended_init(&me->kalman, &kalman_config);
    if (kalman_status != ALG_KALMAN_STATUS_OK)
    {
        return alg_imu_ekf_internal_map_kalman_status(kalman_status);
    }

    me->last_accelerometer_norm_m_s2 = config->gravity_m_s2;
    me->last_accelerometer_deviation_g = 0.0F;
    me->last_normalized_innovation_squared = 0.0F;
    me->last_measurement_noise_scale = 1.0F;
    me->was_accelerometer_used = false;
    me->is_initialized = true;
    return alg_imu_ekf_internal_normalize_and_project(me);
}

alg_imu_ekf_status_t alg_imu_ekf_reset(alg_imu_ekf_t *me,
                                       const alg_imu_ekf_quaternion_t *quaternion,
                                       const float gyro_bias_rad_s[2])
{
    size_t index;

    if ((me == NULL) || (quaternion == NULL) || (gyro_bias_rad_s == NULL))
    {
        return ALG_IMU_EKF_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return ALG_IMU_EKF_STATUS_NOT_INITIALIZED;
    }
    if (!isfinite(quaternion->w) || !isfinite(quaternion->x) || !isfinite(quaternion->y) ||
        !isfinite(quaternion->z) || !alg_imu_ekf_internal_is_finite_array(gyro_bias_rad_s, 2U))
    {
        return ALG_IMU_EKF_STATUS_OUT_OF_RANGE;
    }

    me->state[0] = quaternion->w;
    me->state[1] = quaternion->x;
    me->state[2] = quaternion->y;
    me->state[3] = quaternion->z;
    me->state[4] = gyro_bias_rad_s[0];
    me->state[5] = gyro_bias_rad_s[1];
    alg_imu_ekf_reset_covariance(me);
    for (index = 0U; index < 3U; ++index)
    {
        (void)alg_filter_low_pass_init(&me->accelerometer_filter[index],
                                       me->config.accelerometer_lpf_cutoff_hz);
        me->filtered_accelerometer_m_s2[index] = 0.0F;
        me->innovation[index] = 0.0F;
    }
    me->last_accelerometer_norm_m_s2 = me->config.gravity_m_s2;
    me->last_accelerometer_deviation_g = 0.0F;
    me->last_normalized_innovation_squared = 0.0F;
    me->last_measurement_noise_scale = 1.0F;
    me->was_accelerometer_used = false;
    return alg_imu_ekf_internal_normalize_and_project(me);
}

alg_imu_ekf_status_t alg_imu_ekf_reset_from_accelerometer(alg_imu_ekf_t *me,
                                                          const float accelerometer_m_s2[3])
{
    float norm;
    float roll;
    float pitch;
    float half_roll;
    float half_pitch;
    alg_imu_ekf_quaternion_t quaternion;
    const float zero_bias[2] = {0.0F, 0.0F};
    alg_imu_ekf_status_t status;

    if ((me == NULL) || (accelerometer_m_s2 == NULL))
    {
        return ALG_IMU_EKF_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return ALG_IMU_EKF_STATUS_NOT_INITIALIZED;
    }
    if (!alg_imu_ekf_internal_is_finite_array(accelerometer_m_s2, 3U))
    {
        return ALG_IMU_EKF_STATUS_OUT_OF_RANGE;
    }

    norm = sqrtf((accelerometer_m_s2[0] * accelerometer_m_s2[0]) +
                 (accelerometer_m_s2[1] * accelerometer_m_s2[1]) +
                 (accelerometer_m_s2[2] * accelerometer_m_s2[2]));
    if (!isfinite(norm) || (norm <= ALG_IMU_EKF_MINIMUM_NORM))
    {
        return ALG_IMU_EKF_STATUS_OUT_OF_RANGE;
    }

    roll = atan2f(accelerometer_m_s2[1], accelerometer_m_s2[2]);
    pitch = atan2f(-accelerometer_m_s2[0], sqrtf((accelerometer_m_s2[1] * accelerometer_m_s2[1]) +
                                                 (accelerometer_m_s2[2] * accelerometer_m_s2[2])));
    half_roll = 0.5F * roll;
    half_pitch = 0.5F * pitch;
    quaternion.w = cosf(half_roll) * cosf(half_pitch);
    quaternion.x = sinf(half_roll) * cosf(half_pitch);
    quaternion.y = cosf(half_roll) * sinf(half_pitch);
    quaternion.z = -sinf(half_roll) * sinf(half_pitch);
    status = alg_imu_ekf_reset(me, &quaternion, zero_bias);
    if (status != ALG_IMU_EKF_STATUS_OK)
    {
        return status;
    }
    (void)alg_filter_low_pass_reset(&me->accelerometer_filter[0], accelerometer_m_s2[0]);
    (void)alg_filter_low_pass_reset(&me->accelerometer_filter[1], accelerometer_m_s2[1]);
    (void)alg_filter_low_pass_reset(&me->accelerometer_filter[2], accelerometer_m_s2[2]);
    me->filtered_accelerometer_m_s2[0] = accelerometer_m_s2[0];
    me->filtered_accelerometer_m_s2[1] = accelerometer_m_s2[1];
    me->filtered_accelerometer_m_s2[2] = accelerometer_m_s2[2];
    return ALG_IMU_EKF_STATUS_OK;
}
