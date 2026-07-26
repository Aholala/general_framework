#include "alg_imu_ekf_internal.h"

#include <math.h>
#include <stddef.h>

static float alg_imu_ekf_clamp_unit(float value)
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

alg_imu_ekf_status_t alg_imu_ekf_get_quaternion(const alg_imu_ekf_t *me,
                                                alg_imu_ekf_quaternion_t *quaternion)
{
    if ((me == NULL) || (quaternion == NULL))
    {
        return ALG_IMU_EKF_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return ALG_IMU_EKF_STATUS_NOT_INITIALIZED;
    }

    quaternion->w = me->state[0];
    quaternion->x = me->state[1];
    quaternion->y = me->state[2];
    quaternion->z = me->state[3];
    return ALG_IMU_EKF_STATUS_OK;
}

alg_imu_ekf_status_t alg_imu_ekf_get_euler(const alg_imu_ekf_t *me, alg_imu_ekf_euler_t *euler)
{
    float sine_pitch;

    if ((me == NULL) || (euler == NULL))
    {
        return ALG_IMU_EKF_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return ALG_IMU_EKF_STATUS_NOT_INITIALIZED;
    }

    euler->roll_rad =
        atan2f(2.0F * ((me->state[0] * me->state[1]) + (me->state[2] * me->state[3])),
               1.0F - (2.0F * ((me->state[1] * me->state[1]) + (me->state[2] * me->state[2]))));
    sine_pitch = 2.0F * ((me->state[0] * me->state[2]) - (me->state[3] * me->state[1]));
    euler->pitch_rad = asinf(alg_imu_ekf_clamp_unit(sine_pitch));
    euler->yaw_rad =
        atan2f(2.0F * ((me->state[0] * me->state[3]) + (me->state[1] * me->state[2])),
               1.0F - (2.0F * ((me->state[2] * me->state[2]) + (me->state[3] * me->state[3]))));
    return (isfinite(euler->roll_rad) && isfinite(euler->pitch_rad) && isfinite(euler->yaw_rad))
               ? ALG_IMU_EKF_STATUS_OK
               : ALG_IMU_EKF_STATUS_NUMERICAL_ERROR;
}

alg_imu_ekf_status_t alg_imu_ekf_get_gyro_bias(const alg_imu_ekf_t *me, float gyro_bias_rad_s[3])
{
    if ((me == NULL) || (gyro_bias_rad_s == NULL))
    {
        return ALG_IMU_EKF_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return ALG_IMU_EKF_STATUS_NOT_INITIALIZED;
    }

    gyro_bias_rad_s[0] = me->state[4];
    gyro_bias_rad_s[1] = me->state[5];
    gyro_bias_rad_s[2] = 0.0F;
    return ALG_IMU_EKF_STATUS_OK;
}

alg_imu_ekf_status_t alg_imu_ekf_get_corrected_gyroscope(const alg_imu_ekf_t *me,
                                                         const float gyroscope_rad_s[3],
                                                         float corrected_gyroscope_rad_s[3])
{
    size_t axis;

    if ((me == NULL) || (gyroscope_rad_s == NULL) || (corrected_gyroscope_rad_s == NULL))
    {
        return ALG_IMU_EKF_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return ALG_IMU_EKF_STATUS_NOT_INITIALIZED;
    }
    if (!alg_imu_ekf_internal_is_finite_array(gyroscope_rad_s, 3U))
    {
        return ALG_IMU_EKF_STATUS_OUT_OF_RANGE;
    }

    for (axis = 0U; axis < 2U; ++axis)
    {
        corrected_gyroscope_rad_s[axis] = gyroscope_rad_s[axis] - me->state[4U + axis];
    }
    corrected_gyroscope_rad_s[2] = gyroscope_rad_s[2];
    return ALG_IMU_EKF_STATUS_OK;
}

alg_imu_ekf_status_t alg_imu_ekf_get_diagnostics(const alg_imu_ekf_t *me,
                                                 alg_imu_ekf_diagnostics_t *diagnostics)
{
    size_t index;

    if ((me == NULL) || (diagnostics == NULL))
    {
        return ALG_IMU_EKF_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return ALG_IMU_EKF_STATUS_NOT_INITIALIZED;
    }

    for (index = 0U; index < 3U; ++index)
    {
        diagnostics->filtered_accelerometer_m_s2[index] = me->filtered_accelerometer_m_s2[index];
        diagnostics->innovation[index] = me->innovation[index];
    }
    diagnostics->accelerometer_norm_m_s2 = me->last_accelerometer_norm_m_s2;
    diagnostics->accelerometer_deviation_g = me->last_accelerometer_deviation_g;
    diagnostics->normalized_innovation_squared = me->last_normalized_innovation_squared;
    diagnostics->measurement_noise_scale = me->last_measurement_noise_scale;
    diagnostics->was_accelerometer_used = me->was_accelerometer_used;
    return ALG_IMU_EKF_STATUS_OK;
}

alg_imu_ekf_status_t alg_imu_ekf_get_gravity_body(const alg_imu_ekf_t *me,
                                                  float gravity_body_m_s2[3])
{
    if ((me == NULL) || (gravity_body_m_s2 == NULL))
    {
        return ALG_IMU_EKF_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return ALG_IMU_EKF_STATUS_NOT_INITIALIZED;
    }

    gravity_body_m_s2[0] = me->config.gravity_m_s2 * 2.0F *
                           ((me->state[1] * me->state[3]) - (me->state[0] * me->state[2]));
    gravity_body_m_s2[1] = me->config.gravity_m_s2 * 2.0F *
                           ((me->state[0] * me->state[1]) + (me->state[2] * me->state[3]));
    gravity_body_m_s2[2] =
        me->config.gravity_m_s2 * ((me->state[0] * me->state[0]) - (me->state[1] * me->state[1]) -
                                   (me->state[2] * me->state[2]) + (me->state[3] * me->state[3]));
    return ALG_IMU_EKF_STATUS_OK;
}

alg_imu_ekf_status_t
alg_imu_ekf_get_linear_acceleration_body(const alg_imu_ekf_t *me, const float accelerometer_m_s2[3],
                                         float linear_acceleration_body_m_s2[3])
{
    float gravity_body[3];
    size_t axis;
    alg_imu_ekf_status_t status;

    if ((me == NULL) || (accelerometer_m_s2 == NULL) || (linear_acceleration_body_m_s2 == NULL))
    {
        return ALG_IMU_EKF_STATUS_INVALID_ARGUMENT;
    }
    if (!alg_imu_ekf_internal_is_finite_array(accelerometer_m_s2, 3U))
    {
        return ALG_IMU_EKF_STATUS_OUT_OF_RANGE;
    }

    status = alg_imu_ekf_get_gravity_body(me, gravity_body);
    if (status != ALG_IMU_EKF_STATUS_OK)
    {
        return status;
    }
    for (axis = 0U; axis < 3U; ++axis)
    {
        linear_acceleration_body_m_s2[axis] = accelerometer_m_s2[axis] - gravity_body[axis];
    }
    return ALG_IMU_EKF_STATUS_OK;
}

alg_imu_ekf_status_t
alg_imu_ekf_get_linear_acceleration_world(const alg_imu_ekf_t *me,
                                          const float accelerometer_m_s2[3],
                                          float linear_acceleration_world_m_s2[3])
{
    float rotation[9];

    if ((me == NULL) || (accelerometer_m_s2 == NULL) || (linear_acceleration_world_m_s2 == NULL))
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

    rotation[0] = 1.0F - (2.0F * ((me->state[2] * me->state[2]) + (me->state[3] * me->state[3])));
    rotation[1] = 2.0F * ((me->state[1] * me->state[2]) - (me->state[0] * me->state[3]));
    rotation[2] = 2.0F * ((me->state[1] * me->state[3]) + (me->state[0] * me->state[2]));
    rotation[3] = 2.0F * ((me->state[1] * me->state[2]) + (me->state[0] * me->state[3]));
    rotation[4] = 1.0F - (2.0F * ((me->state[1] * me->state[1]) + (me->state[3] * me->state[3])));
    rotation[5] = 2.0F * ((me->state[2] * me->state[3]) - (me->state[0] * me->state[1]));
    rotation[6] = 2.0F * ((me->state[1] * me->state[3]) - (me->state[0] * me->state[2]));
    rotation[7] = 2.0F * ((me->state[2] * me->state[3]) + (me->state[0] * me->state[1]));
    rotation[8] = 1.0F - (2.0F * ((me->state[1] * me->state[1]) + (me->state[2] * me->state[2])));

    linear_acceleration_world_m_s2[0] = (rotation[0] * accelerometer_m_s2[0]) +
                                        (rotation[1] * accelerometer_m_s2[1]) +
                                        (rotation[2] * accelerometer_m_s2[2]);
    linear_acceleration_world_m_s2[1] = (rotation[3] * accelerometer_m_s2[0]) +
                                        (rotation[4] * accelerometer_m_s2[1]) +
                                        (rotation[5] * accelerometer_m_s2[2]);
    linear_acceleration_world_m_s2[2] =
        (rotation[6] * accelerometer_m_s2[0]) + (rotation[7] * accelerometer_m_s2[1]) +
        (rotation[8] * accelerometer_m_s2[2]) - me->config.gravity_m_s2;
    return ALG_IMU_EKF_STATUS_OK;
}
