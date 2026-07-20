#include "alg_imu_ekf_internal.h"

#include <math.h>
#include <stddef.h>

static float AlgImuEkf_ClampUnit(float value)
{
    if (value > 1.0F)
    {
        return 1.0F;
    }
    if (value < -1.0F)
    {
        return -1.0F;
    }
    return value;
}

AlgImuEkfStatus_t AlgImuEkf_GetQuaternion(
    const AlgImuEkf_t *self,
    AlgImuEkfQuaternion_t *quaternion)
{
    if ((self == NULL) || (quaternion == NULL))
    {
        return ALG_IMU_EKF_STATUS_INVALID_ARGUMENT;
    }
    if (!self->is_initialized)
    {
        return ALG_IMU_EKF_STATUS_NOT_INITIALIZED;
    }

    quaternion->w = self->state[0];
    quaternion->x = self->state[1];
    quaternion->y = self->state[2];
    quaternion->z = self->state[3];
    return ALG_IMU_EKF_STATUS_OK;
}

AlgImuEkfStatus_t AlgImuEkf_GetEuler(
    const AlgImuEkf_t *self,
    AlgImuEkfEuler_t *euler)
{
    float sine_pitch;

    if ((self == NULL) || (euler == NULL))
    {
        return ALG_IMU_EKF_STATUS_INVALID_ARGUMENT;
    }
    if (!self->is_initialized)
    {
        return ALG_IMU_EKF_STATUS_NOT_INITIALIZED;
    }

    euler->roll_rad = atan2f(
        2.0F * ((self->state[0] * self->state[1]) +
                (self->state[2] * self->state[3])),
        1.0F - (2.0F * ((self->state[1] * self->state[1]) +
                        (self->state[2] * self->state[2]))));
    sine_pitch = 2.0F * ((self->state[0] * self->state[2]) -
                         (self->state[3] * self->state[1]));
    euler->pitch_rad = asinf(AlgImuEkf_ClampUnit(sine_pitch));
    euler->yaw_rad = atan2f(
        2.0F * ((self->state[0] * self->state[3]) +
                (self->state[1] * self->state[2])),
        1.0F - (2.0F * ((self->state[2] * self->state[2]) +
                        (self->state[3] * self->state[3]))));
    return (isfinite(euler->roll_rad) && isfinite(euler->pitch_rad) &&
            isfinite(euler->yaw_rad))
               ? ALG_IMU_EKF_STATUS_OK
               : ALG_IMU_EKF_STATUS_NUMERICAL_ERROR;
}

AlgImuEkfStatus_t AlgImuEkf_GetGyroBias(
    const AlgImuEkf_t *self,
    float gyro_bias_rad_s[3])
{
    if ((self == NULL) || (gyro_bias_rad_s == NULL))
    {
        return ALG_IMU_EKF_STATUS_INVALID_ARGUMENT;
    }
    if (!self->is_initialized)
    {
        return ALG_IMU_EKF_STATUS_NOT_INITIALIZED;
    }

    gyro_bias_rad_s[0] = self->state[4];
    gyro_bias_rad_s[1] = self->state[5];
    gyro_bias_rad_s[2] = 0.0F;
    return ALG_IMU_EKF_STATUS_OK;
}

AlgImuEkfStatus_t AlgImuEkf_GetCorrectedGyroscope(
    const AlgImuEkf_t *self,
    const float gyroscope_rad_s[3],
    float corrected_gyroscope_rad_s[3])
{
    size_t axis;

    if ((self == NULL) || (gyroscope_rad_s == NULL) ||
        (corrected_gyroscope_rad_s == NULL))
    {
        return ALG_IMU_EKF_STATUS_INVALID_ARGUMENT;
    }
    if (!self->is_initialized)
    {
        return ALG_IMU_EKF_STATUS_NOT_INITIALIZED;
    }
    if (!AlgImuEkfInternal_IsFiniteArray(gyroscope_rad_s, 3U))
    {
        return ALG_IMU_EKF_STATUS_OUT_OF_RANGE;
    }

    for (axis = 0U; axis < 2U; ++axis)
    {
        corrected_gyroscope_rad_s[axis] =
            gyroscope_rad_s[axis] - self->state[4U + axis];
    }
    corrected_gyroscope_rad_s[2] = gyroscope_rad_s[2];
    return ALG_IMU_EKF_STATUS_OK;
}

AlgImuEkfStatus_t AlgImuEkf_GetDiagnostics(
    const AlgImuEkf_t *self,
    AlgImuEkfDiagnostics_t *diagnostics)
{
    size_t index;

    if ((self == NULL) || (diagnostics == NULL))
    {
        return ALG_IMU_EKF_STATUS_INVALID_ARGUMENT;
    }
    if (!self->is_initialized)
    {
        return ALG_IMU_EKF_STATUS_NOT_INITIALIZED;
    }

    for (index = 0U; index < 3U; ++index)
    {
        diagnostics->filtered_accelerometer_m_s2[index] =
            self->filtered_accelerometer_m_s2[index];
        diagnostics->innovation[index] = self->innovation[index];
    }
    diagnostics->accelerometer_norm_m_s2 =
        self->last_accelerometer_norm_m_s2;
    diagnostics->accelerometer_deviation_g =
        self->last_accelerometer_deviation_g;
    diagnostics->normalized_innovation_squared =
        self->last_normalized_innovation_squared;
    diagnostics->measurement_noise_scale =
        self->last_measurement_noise_scale;
    diagnostics->was_accelerometer_used = self->was_accelerometer_used;
    return ALG_IMU_EKF_STATUS_OK;
}

AlgImuEkfStatus_t AlgImuEkf_GetGravityBody(
    const AlgImuEkf_t *self,
    float gravity_body_m_s2[3])
{
    if ((self == NULL) || (gravity_body_m_s2 == NULL))
    {
        return ALG_IMU_EKF_STATUS_INVALID_ARGUMENT;
    }
    if (!self->is_initialized)
    {
        return ALG_IMU_EKF_STATUS_NOT_INITIALIZED;
    }

    gravity_body_m_s2[0] = self->config.gravity_m_s2 *
                           2.0F *
                           ((self->state[1] * self->state[3]) -
                            (self->state[0] * self->state[2]));
    gravity_body_m_s2[1] = self->config.gravity_m_s2 *
                           2.0F *
                           ((self->state[0] * self->state[1]) +
                            (self->state[2] * self->state[3]));
    gravity_body_m_s2[2] = self->config.gravity_m_s2 *
                           ((self->state[0] * self->state[0]) -
                            (self->state[1] * self->state[1]) -
                            (self->state[2] * self->state[2]) +
                            (self->state[3] * self->state[3]));
    return ALG_IMU_EKF_STATUS_OK;
}

AlgImuEkfStatus_t AlgImuEkf_GetLinearAccelerationBody(
    const AlgImuEkf_t *self,
    const float accelerometer_m_s2[3],
    float linear_acceleration_body_m_s2[3])
{
    float gravity_body[3];
    size_t axis;
    AlgImuEkfStatus_t status;

    if ((self == NULL) || (accelerometer_m_s2 == NULL) ||
        (linear_acceleration_body_m_s2 == NULL))
    {
        return ALG_IMU_EKF_STATUS_INVALID_ARGUMENT;
    }
    if (!AlgImuEkfInternal_IsFiniteArray(accelerometer_m_s2, 3U))
    {
        return ALG_IMU_EKF_STATUS_OUT_OF_RANGE;
    }

    status = AlgImuEkf_GetGravityBody(self, gravity_body);
    if (status != ALG_IMU_EKF_STATUS_OK)
    {
        return status;
    }
    for (axis = 0U; axis < 3U; ++axis)
    {
        linear_acceleration_body_m_s2[axis] =
            accelerometer_m_s2[axis] - gravity_body[axis];
    }
    return ALG_IMU_EKF_STATUS_OK;
}

AlgImuEkfStatus_t AlgImuEkf_GetLinearAccelerationWorld(
    const AlgImuEkf_t *self,
    const float accelerometer_m_s2[3],
    float linear_acceleration_world_m_s2[3])
{
    float rotation[9];

    if ((self == NULL) || (accelerometer_m_s2 == NULL) ||
        (linear_acceleration_world_m_s2 == NULL))
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

    rotation[0] = 1.0F -
                  (2.0F * ((self->state[2] * self->state[2]) +
                           (self->state[3] * self->state[3])));
    rotation[1] = 2.0F * ((self->state[1] * self->state[2]) -
                          (self->state[0] * self->state[3]));
    rotation[2] = 2.0F * ((self->state[1] * self->state[3]) +
                          (self->state[0] * self->state[2]));
    rotation[3] = 2.0F * ((self->state[1] * self->state[2]) +
                          (self->state[0] * self->state[3]));
    rotation[4] = 1.0F -
                  (2.0F * ((self->state[1] * self->state[1]) +
                           (self->state[3] * self->state[3])));
    rotation[5] = 2.0F * ((self->state[2] * self->state[3]) -
                          (self->state[0] * self->state[1]));
    rotation[6] = 2.0F * ((self->state[1] * self->state[3]) -
                          (self->state[0] * self->state[2]));
    rotation[7] = 2.0F * ((self->state[2] * self->state[3]) +
                          (self->state[0] * self->state[1]));
    rotation[8] = 1.0F -
                  (2.0F * ((self->state[1] * self->state[1]) +
                           (self->state[2] * self->state[2])));

    linear_acceleration_world_m_s2[0] =
        (rotation[0] * accelerometer_m_s2[0]) +
        (rotation[1] * accelerometer_m_s2[1]) +
        (rotation[2] * accelerometer_m_s2[2]);
    linear_acceleration_world_m_s2[1] =
        (rotation[3] * accelerometer_m_s2[0]) +
        (rotation[4] * accelerometer_m_s2[1]) +
        (rotation[5] * accelerometer_m_s2[2]);
    linear_acceleration_world_m_s2[2] =
        (rotation[6] * accelerometer_m_s2[0]) +
        (rotation[7] * accelerometer_m_s2[1]) +
        (rotation[8] * accelerometer_m_s2[2]) - self->config.gravity_m_s2;
    return ALG_IMU_EKF_STATUS_OK;
}
