#include "alg_imu_ekf_internal.h"

#include <math.h>
#include <stddef.h>

#define ALG_IMU_EKF_STANDARD_GRAVITY_M_S2 (9.80665F)
#define ALG_IMU_EKF_MINIMUM_NORM          (1.0e-6F)

bool AlgImuEkfInternal_IsFiniteArray(const float *values, size_t value_count)
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

AlgImuEkfStatus_t AlgImuEkfInternal_MapKalmanStatus(AlgKalmanStatus_t status)
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

static void AlgImuEkf_Clear(float *values, size_t value_count)
{
    size_t index;

    for (index = 0U; index < value_count; ++index)
    {
        values[index] = 0.0F;
    }
}

static void AlgImuEkf_ResetCovariance(AlgImuEkf_t *self)
{
    size_t index;

    AlgImuEkf_Clear(self->covariance,
                    ALG_IMU_EKF_STATE_DIMENSION * ALG_IMU_EKF_STATE_DIMENSION);
    for (index = 0U; index < 4U; ++index)
    {
        self->covariance[(index * ALG_IMU_EKF_STATE_DIMENSION) + index] =
            self->config.initial_attitude_variance;
    }
    for (index = 4U; index < ALG_IMU_EKF_STATE_DIMENSION; ++index)
    {
        self->covariance[(index * ALG_IMU_EKF_STATE_DIMENSION) + index] =
            self->config.initial_gyro_bias_variance;
    }
}

AlgImuEkfStatus_t AlgImuEkfInternal_NormalizeAndProject(AlgImuEkf_t *self)
{
    float quaternion_norm;
    float normalized_quaternion[4];
    float *normalization_jacobian;
    float *temporary_covariance;
    size_t row;
    size_t column;
    size_t shared_index;
    float accumulator;

    quaternion_norm = sqrtf((self->state[0] * self->state[0]) +
                            (self->state[1] * self->state[1]) +
                            (self->state[2] * self->state[2]) +
                            (self->state[3] * self->state[3]));
    if (!isfinite(quaternion_norm) ||
        (quaternion_norm <= ALG_IMU_EKF_MINIMUM_NORM))
    {
        return ALG_IMU_EKF_STATUS_NUMERICAL_ERROR;
    }

    for (row = 0U; row < 4U; ++row)
    {
        normalized_quaternion[row] = self->state[row] / quaternion_norm;
        self->state[row] = normalized_quaternion[row];
    }

    normalization_jacobian = self->normalization_workspace;
    temporary_covariance = normalization_jacobian +
                           (ALG_IMU_EKF_STATE_DIMENSION *
                            ALG_IMU_EKF_STATE_DIMENSION);
    AlgImuEkf_Clear(normalization_jacobian,
                    ALG_IMU_EKF_STATE_DIMENSION * ALG_IMU_EKF_STATE_DIMENSION);
    for (row = 0U; row < 4U; ++row)
    {
        for (column = 0U; column < 4U; ++column)
        {
            normalization_jacobian[
                (row * ALG_IMU_EKF_STATE_DIMENSION) + column] =
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
            for (shared_index = 0U; shared_index < ALG_IMU_EKF_STATE_DIMENSION;
                 ++shared_index)
            {
                accumulator += normalization_jacobian[
                                   (row * ALG_IMU_EKF_STATE_DIMENSION) + shared_index] *
                               self->covariance[
                                   (shared_index * ALG_IMU_EKF_STATE_DIMENSION) + column];
            }
            temporary_covariance[(row * ALG_IMU_EKF_STATE_DIMENSION) + column] =
                accumulator;
        }
    }
    for (row = 0U; row < ALG_IMU_EKF_STATE_DIMENSION; ++row)
    {
        for (column = 0U; column < ALG_IMU_EKF_STATE_DIMENSION; ++column)
        {
            accumulator = 0.0F;
            for (shared_index = 0U; shared_index < ALG_IMU_EKF_STATE_DIMENSION;
                 ++shared_index)
            {
                accumulator += temporary_covariance[
                                   (row * ALG_IMU_EKF_STATE_DIMENSION) + shared_index] *
                               normalization_jacobian[
                                   (column * ALG_IMU_EKF_STATE_DIMENSION) + shared_index];
            }
            self->covariance[(row * ALG_IMU_EKF_STATE_DIMENSION) + column] =
                accumulator;
        }
    }

    for (row = 0U; row < ALG_IMU_EKF_STATE_DIMENSION; ++row)
    {
        for (column = row + 1U; column < ALG_IMU_EKF_STATE_DIMENSION; ++column)
        {
            const float average =
                0.5F *
                (self->covariance[(row * ALG_IMU_EKF_STATE_DIMENSION) + column] +
                 self->covariance[(column * ALG_IMU_EKF_STATE_DIMENSION) + row]);
            self->covariance[(row * ALG_IMU_EKF_STATE_DIMENSION) + column] = average;
            self->covariance[(column * ALG_IMU_EKF_STATE_DIMENSION) + row] = average;
        }
    }

    return AlgImuEkfInternal_IsFiniteArray(
               self->covariance,
               ALG_IMU_EKF_STATE_DIMENSION * ALG_IMU_EKF_STATE_DIMENSION)
               ? ALG_IMU_EKF_STATUS_OK
               : ALG_IMU_EKF_STATUS_NUMERICAL_ERROR;
}

AlgImuEkfStatus_t AlgImuEkfConfig_Init(AlgImuEkfConfig_t *config)
{
    if (config == NULL)
    {
        return ALG_IMU_EKF_STATUS_INVALID_ARGUMENT;
    }

    *config = (AlgImuEkfConfig_t){
        .gravity_m_s2 = ALG_IMU_EKF_STANDARD_GRAVITY_M_S2,
        .gyro_noise_std_rad_s = 0.015F,
        .gyro_bias_random_walk_std_rad_s2 = 0.0005F,
        .accelerometer_direction_noise_std = 0.03F,
        .accelerometer_rejection_threshold_g = 0.20F,
        .accelerometer_noise_multiplier = 20.0F,
        .initial_attitude_variance = 0.10F,
        .initial_gyro_bias_variance = 0.01F};
    return ALG_IMU_EKF_STATUS_OK;
}

static AlgImuEkfStatus_t AlgImuEkf_ValidateConfig(
    const AlgImuEkfConfig_t *config)
{
    if (config == NULL)
    {
        return ALG_IMU_EKF_STATUS_INVALID_ARGUMENT;
    }
    if (!isfinite(config->gravity_m_s2) || (config->gravity_m_s2 <= 0.0F) ||
        !isfinite(config->gyro_noise_std_rad_s) ||
        (config->gyro_noise_std_rad_s < 0.0F) ||
        !isfinite(config->gyro_bias_random_walk_std_rad_s2) ||
        (config->gyro_bias_random_walk_std_rad_s2 < 0.0F) ||
        !isfinite(config->accelerometer_direction_noise_std) ||
        (config->accelerometer_direction_noise_std <= 0.0F) ||
        !isfinite(config->accelerometer_rejection_threshold_g) ||
        (config->accelerometer_rejection_threshold_g <= 0.0F) ||
        !isfinite(config->accelerometer_noise_multiplier) ||
        (config->accelerometer_noise_multiplier < 0.0F) ||
        !isfinite(config->initial_attitude_variance) ||
        (config->initial_attitude_variance < 0.0F) ||
        !isfinite(config->initial_gyro_bias_variance) ||
        (config->initial_gyro_bias_variance < 0.0F))
    {
        return ALG_IMU_EKF_STATUS_OUT_OF_RANGE;
    }
    return ALG_IMU_EKF_STATUS_OK;
}

AlgImuEkfStatus_t AlgImuEkf_Init(AlgImuEkf_t *self,
                                 const AlgImuEkfConfig_t *config)
{
    AlgKalmanExtendedConfig_t kalman_config;
    AlgKalmanStatus_t kalman_status;
    AlgImuEkfStatus_t status;
    size_t index;
    float accelerometer_variance;

    if (self == NULL)
    {
        return ALG_IMU_EKF_STATUS_INVALID_ARGUMENT;
    }

    self->is_initialized = false;
    status = AlgImuEkf_ValidateConfig(config);
    if (status != ALG_IMU_EKF_STATUS_OK)
    {
        return status;
    }

    self->config = *config;
    AlgImuEkf_Clear(self->state, ALG_IMU_EKF_STATE_DIMENSION);
    self->state[0] = 1.0F;
    AlgImuEkf_ResetCovariance(self);
    AlgImuEkf_Clear(self->process_noise,
                    ALG_IMU_EKF_STATE_DIMENSION * ALG_IMU_EKF_STATE_DIMENSION);
    AlgImuEkf_Clear(self->measurement_noise,
                    ALG_IMU_EKF_MEASUREMENT_DIMENSION *
                        ALG_IMU_EKF_MEASUREMENT_DIMENSION);
    accelerometer_variance = config->accelerometer_direction_noise_std *
                             config->accelerometer_direction_noise_std;
    for (index = 0U; index < ALG_IMU_EKF_MEASUREMENT_DIMENSION; ++index)
    {
        self->measurement_noise[
            (index * ALG_IMU_EKF_MEASUREMENT_DIMENSION) + index] =
            accelerometer_variance;
    }

    kalman_config = (AlgKalmanExtendedConfig_t){
        .state_dimension = ALG_IMU_EKF_STATE_DIMENSION,
        .measurement_dimension = ALG_IMU_EKF_MEASUREMENT_DIMENSION,
        .control_dimension = ALG_IMU_EKF_CONTROL_DIMENSION,
        .state = self->state,
        .covariance = self->covariance,
        .process_noise = self->process_noise,
        .measurement_noise = self->measurement_noise,
        .workspace = self->kalman_workspace,
        .workspace_size = sizeof(self->kalman_workspace) /
                          sizeof(self->kalman_workspace[0]),
        .state_function = AlgImuEkfInternal_StateFunction,
        .state_jacobian_function = AlgImuEkfInternal_StateJacobian,
        .measurement_function = AlgImuEkfInternal_MeasurementFunction,
        .measurement_jacobian_function = AlgImuEkfInternal_MeasurementJacobian,
        .user_context = self};
    kalman_status = AlgKalmanExtended_Init(&self->kalman, &kalman_config);
    if (kalman_status != ALG_KALMAN_STATUS_OK)
    {
        return AlgImuEkfInternal_MapKalmanStatus(kalman_status);
    }

    self->last_accelerometer_norm_m_s2 = config->gravity_m_s2;
    self->last_accelerometer_deviation_g = 0.0F;
    self->was_accelerometer_used = false;
    self->is_initialized = true;
    return AlgImuEkfInternal_NormalizeAndProject(self);
}

AlgImuEkfStatus_t AlgImuEkf_Reset(
    AlgImuEkf_t *self,
    const AlgImuEkfQuaternion_t *quaternion,
    const float gyro_bias_rad_s[3])
{
    if ((self == NULL) || (quaternion == NULL) || (gyro_bias_rad_s == NULL))
    {
        return ALG_IMU_EKF_STATUS_INVALID_ARGUMENT;
    }
    if (!self->is_initialized)
    {
        return ALG_IMU_EKF_STATUS_NOT_INITIALIZED;
    }
    if (!isfinite(quaternion->w) || !isfinite(quaternion->x) ||
        !isfinite(quaternion->y) || !isfinite(quaternion->z) ||
        !AlgImuEkfInternal_IsFiniteArray(gyro_bias_rad_s, 3U))
    {
        return ALG_IMU_EKF_STATUS_OUT_OF_RANGE;
    }

    self->state[0] = quaternion->w;
    self->state[1] = quaternion->x;
    self->state[2] = quaternion->y;
    self->state[3] = quaternion->z;
    self->state[4] = gyro_bias_rad_s[0];
    self->state[5] = gyro_bias_rad_s[1];
    self->state[6] = gyro_bias_rad_s[2];
    AlgImuEkf_ResetCovariance(self);
    self->was_accelerometer_used = false;
    return AlgImuEkfInternal_NormalizeAndProject(self);
}

AlgImuEkfStatus_t AlgImuEkf_ResetFromAccelerometer(
    AlgImuEkf_t *self,
    const float accelerometer_m_s2[3])
{
    float norm;
    float roll;
    float pitch;
    float half_roll;
    float half_pitch;
    AlgImuEkfQuaternion_t quaternion;
    const float zero_bias[3] = {0.0F, 0.0F, 0.0F};

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

    norm = sqrtf((accelerometer_m_s2[0] * accelerometer_m_s2[0]) +
                 (accelerometer_m_s2[1] * accelerometer_m_s2[1]) +
                 (accelerometer_m_s2[2] * accelerometer_m_s2[2]));
    if (!isfinite(norm) || (norm <= ALG_IMU_EKF_MINIMUM_NORM))
    {
        return ALG_IMU_EKF_STATUS_OUT_OF_RANGE;
    }

    roll = atan2f(accelerometer_m_s2[1], accelerometer_m_s2[2]);
    pitch = atan2f(-accelerometer_m_s2[0],
                   sqrtf((accelerometer_m_s2[1] * accelerometer_m_s2[1]) +
                         (accelerometer_m_s2[2] * accelerometer_m_s2[2])));
    half_roll = 0.5F * roll;
    half_pitch = 0.5F * pitch;
    quaternion.w = cosf(half_roll) * cosf(half_pitch);
    quaternion.x = sinf(half_roll) * cosf(half_pitch);
    quaternion.y = cosf(half_roll) * sinf(half_pitch);
    quaternion.z = -sinf(half_roll) * sinf(half_pitch);
    return AlgImuEkf_Reset(self, &quaternion, zero_bias);
}
