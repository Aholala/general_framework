#include "alg_attitude.h"

#include <math.h>

static bool alg_attitude_normalize_quaternion(alg_attitude_quaternion_t *quaternion)
{
    const float norm = sqrtf(quaternion->q0 * quaternion->q0 + quaternion->q1 * quaternion->q1 +
                             quaternion->q2 * quaternion->q2 + quaternion->q3 * quaternion->q3);
    if (!isfinite(norm) || (norm < 1.0e-8F))
    {
        return false;
    }
    quaternion->q0 /= norm;
    quaternion->q1 /= norm;
    quaternion->q2 /= norm;
    quaternion->q3 /= norm;
    return true;
}

static void alg_attitude_integrate_gyro(alg_attitude_quaternion_t *quaternion,
                                        float gyro_x_rad_per_s, float gyro_y_rad_per_s,
                                        float gyro_z_rad_per_s, float delta_time_s)
{
    const float half_delta_time_s = 0.5F * delta_time_s;
    const float q0 = quaternion->q0;
    const float q1 = quaternion->q1;
    const float q2 = quaternion->q2;
    const float q3 = quaternion->q3;
    quaternion->q0 += (-q1 * gyro_x_rad_per_s - q2 * gyro_y_rad_per_s - q3 * gyro_z_rad_per_s) *
                      half_delta_time_s;
    quaternion->q1 +=
        (q0 * gyro_x_rad_per_s + q2 * gyro_z_rad_per_s - q3 * gyro_y_rad_per_s) * half_delta_time_s;
    quaternion->q2 +=
        (q0 * gyro_y_rad_per_s - q1 * gyro_z_rad_per_s + q3 * gyro_x_rad_per_s) * half_delta_time_s;
    quaternion->q3 +=
        (q0 * gyro_z_rad_per_s + q1 * gyro_y_rad_per_s - q2 * gyro_x_rad_per_s) * half_delta_time_s;
}

static void alg_attitude_update_mahony(alg_attitude_t *me, float *gyro_x_rad_per_s,
                                       float *gyro_y_rad_per_s, float *gyro_z_rad_per_s,
                                       float acceleration_x, float acceleration_y,
                                       float acceleration_z, float delta_time_s)
{
    const alg_attitude_quaternion_t *const quaternion = &me->quaternion;
    const float estimated_gravity_x =
        2.0F * (quaternion->q1 * quaternion->q3 - quaternion->q0 * quaternion->q2);
    const float estimated_gravity_y =
        2.0F * (quaternion->q0 * quaternion->q1 + quaternion->q2 * quaternion->q3);
    const float estimated_gravity_z =
        quaternion->q0 * quaternion->q0 - quaternion->q1 * quaternion->q1 -
        quaternion->q2 * quaternion->q2 + quaternion->q3 * quaternion->q3;
    const float error_x =
        acceleration_y * estimated_gravity_z - acceleration_z * estimated_gravity_y;
    const float error_y =
        acceleration_z * estimated_gravity_x - acceleration_x * estimated_gravity_z;
    const float error_z =
        acceleration_x * estimated_gravity_y - acceleration_y * estimated_gravity_x;

    me->integral_error_x += me->config.integral_gain * error_x * delta_time_s;
    me->integral_error_y += me->config.integral_gain * error_y * delta_time_s;
    me->integral_error_z += me->config.integral_gain * error_z * delta_time_s;
    *gyro_x_rad_per_s += me->config.proportional_gain * error_x + me->integral_error_x;
    *gyro_y_rad_per_s += me->config.proportional_gain * error_y + me->integral_error_y;
    *gyro_z_rad_per_s += me->config.proportional_gain * error_z + me->integral_error_z;
}

static void alg_attitude_update_madgwick(alg_attitude_t *me, float *gyro_x_rad_per_s,
                                         float *gyro_y_rad_per_s, float *gyro_z_rad_per_s,
                                         float acceleration_x, float acceleration_y,
                                         float acceleration_z)
{
    const float q0 = me->quaternion.q0;
    const float q1 = me->quaternion.q1;
    const float q2 = me->quaternion.q2;
    const float q3 = me->quaternion.q3;
    float gradient_q0 = 4.0F * q0 * q2 * q2 + 2.0F * q2 * acceleration_x + 4.0F * q0 * q1 * q1 -
                        2.0F * q1 * acceleration_y;
    float gradient_q1 = 4.0F * q1 * q3 * q3 - 2.0F * q3 * acceleration_x + 4.0F * q0 * q0 * q1 -
                        2.0F * q0 * acceleration_y - 4.0F * q1 + 8.0F * q1 * q1 * q1 +
                        8.0F * q1 * q2 * q2 + 4.0F * q1 * acceleration_z;
    float gradient_q2 = 4.0F * q0 * q0 * q2 + 2.0F * q0 * acceleration_x + 4.0F * q2 * q3 * q3 -
                        2.0F * q3 * acceleration_y - 4.0F * q2 + 8.0F * q1 * q1 * q2 +
                        8.0F * q2 * q2 * q2 + 4.0F * q2 * acceleration_z;
    float gradient_q3 = 4.0F * q1 * q1 * q3 - 2.0F * q1 * acceleration_x + 4.0F * q2 * q2 * q3 -
                        2.0F * q2 * acceleration_y;
    const float norm = sqrtf(gradient_q0 * gradient_q0 + gradient_q1 * gradient_q1 +
                             gradient_q2 * gradient_q2 + gradient_q3 * gradient_q3);

    if (norm > 1.0e-8F)
    {
        gradient_q0 /= norm;
        gradient_q1 /= norm;
        gradient_q2 /= norm;
        gradient_q3 /= norm;
        /*
         * Convert the gradient correction into an equivalent angular-rate
         * correction. This keeps the common quaternion integrator in one place.
         */
        *gyro_x_rad_per_s -=
            2.0F * me->config.madgwick_beta *
            (-q1 * gradient_q0 + q0 * gradient_q1 + q3 * gradient_q2 - q2 * gradient_q3);
        *gyro_y_rad_per_s -=
            2.0F * me->config.madgwick_beta *
            (-q2 * gradient_q0 - q3 * gradient_q1 + q0 * gradient_q2 + q1 * gradient_q3);
        *gyro_z_rad_per_s -=
            2.0F * me->config.madgwick_beta *
            (-q3 * gradient_q0 + q2 * gradient_q1 - q1 * gradient_q2 + q0 * gradient_q3);
    }
}

alg_attitude_status_t alg_attitude_init(alg_attitude_t *me, const alg_attitude_config_t *config,
                                        const alg_attitude_quaternion_t *initial_quaternion)
{
    alg_attitude_quaternion_t quaternion = {1.0F, 0.0F, 0.0F, 0.0F};
    if ((me == NULL) || (config == NULL) || (config->method > ALG_ATTITUDE_METHOD_MADGWICK) ||
        !isfinite(config->proportional_gain) || !isfinite(config->integral_gain) ||
        !isfinite(config->madgwick_beta) || !isfinite(config->acceleration_min_m_per_s2) ||
        !isfinite(config->acceleration_max_m_per_s2) || (config->proportional_gain < 0.0F) ||
        (config->integral_gain < 0.0F) || (config->madgwick_beta < 0.0F) ||
        (config->acceleration_min_m_per_s2 < 0.0F) ||
        (config->acceleration_max_m_per_s2 <= config->acceleration_min_m_per_s2))
    {
        return ALG_ATTITUDE_STATUS_INVALID_ARGUMENT;
    }
    if (initial_quaternion != NULL)
    {
        quaternion = *initial_quaternion;
    }
    if (!alg_attitude_normalize_quaternion(&quaternion))
    {
        return ALG_ATTITUDE_STATUS_INVALID_ARGUMENT;
    }
    *me = (alg_attitude_t){
        .config = *config,
        .quaternion = quaternion,
        .is_initialized = true,
    };
    return ALG_ATTITUDE_STATUS_OK;
}

alg_attitude_status_t alg_attitude_reset(alg_attitude_t *me,
                                         const alg_attitude_quaternion_t *quaternion)
{
    alg_attitude_quaternion_t normalized;
    if ((me == NULL) || (quaternion == NULL))
    {
        return ALG_ATTITUDE_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return ALG_ATTITUDE_STATUS_NOT_INITIALIZED;
    }
    normalized = *quaternion;
    if (!alg_attitude_normalize_quaternion(&normalized))
    {
        return ALG_ATTITUDE_STATUS_INVALID_ARGUMENT;
    }
    me->quaternion = normalized;
    me->integral_error_x = 0.0F;
    me->integral_error_y = 0.0F;
    me->integral_error_z = 0.0F;
    return ALG_ATTITUDE_STATUS_OK;
}

alg_attitude_status_t alg_attitude_update(alg_attitude_t *me, float gyro_x_rad_per_s,
                                          float gyro_y_rad_per_s, float gyro_z_rad_per_s,
                                          float acceleration_x_m_per_s2,
                                          float acceleration_y_m_per_s2,
                                          float acceleration_z_m_per_s2, float delta_time_s)
{
    const float acceleration_norm = sqrtf(acceleration_x_m_per_s2 * acceleration_x_m_per_s2 +
                                          acceleration_y_m_per_s2 * acceleration_y_m_per_s2 +
                                          acceleration_z_m_per_s2 * acceleration_z_m_per_s2);
    bool use_acceleration;

    if ((me == NULL) || !isfinite(gyro_x_rad_per_s) || !isfinite(gyro_y_rad_per_s) ||
        !isfinite(gyro_z_rad_per_s) || !isfinite(acceleration_norm) || !isfinite(delta_time_s) ||
        (delta_time_s <= 0.0F))
    {
        return ALG_ATTITUDE_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return ALG_ATTITUDE_STATUS_NOT_INITIALIZED;
    }
    use_acceleration = (acceleration_norm >= me->config.acceleration_min_m_per_s2) &&
                       (acceleration_norm <= me->config.acceleration_max_m_per_s2);
    if (use_acceleration)
    {
        const float acceleration_x = acceleration_x_m_per_s2 / acceleration_norm;
        const float acceleration_y = acceleration_y_m_per_s2 / acceleration_norm;
        const float acceleration_z = acceleration_z_m_per_s2 / acceleration_norm;
        if (me->config.method == ALG_ATTITUDE_METHOD_MAHONY)
        {
            alg_attitude_update_mahony(me, &gyro_x_rad_per_s, &gyro_y_rad_per_s, &gyro_z_rad_per_s,
                                       acceleration_x, acceleration_y, acceleration_z,
                                       delta_time_s);
        }
        else
        {
            alg_attitude_update_madgwick(me, &gyro_x_rad_per_s, &gyro_y_rad_per_s,
                                         &gyro_z_rad_per_s, acceleration_x, acceleration_y,
                                         acceleration_z);
        }
    }
    alg_attitude_integrate_gyro(&me->quaternion, gyro_x_rad_per_s, gyro_y_rad_per_s,
                                gyro_z_rad_per_s, delta_time_s);
    if (!alg_attitude_normalize_quaternion(&me->quaternion))
    {
        return ALG_ATTITUDE_STATUS_NUMERICAL_ERROR;
    }
    return use_acceleration ? ALG_ATTITUDE_STATUS_OK : ALG_ATTITUDE_STATUS_GYRO_ONLY;
}

alg_attitude_status_t alg_attitude_correct_yaw(alg_attitude_t *me, float measured_yaw_rad,
                                               float correction_gain)
{
    float roll_rad;
    float pitch_rad;
    float yaw_rad;
    float yaw_error_rad;
    if ((me == NULL) || !isfinite(measured_yaw_rad) || !isfinite(correction_gain) ||
        (correction_gain < 0.0F) || (correction_gain > 1.0F))
    {
        return ALG_ATTITUDE_STATUS_INVALID_ARGUMENT;
    }
    if (alg_attitude_get_euler(me, &roll_rad, &pitch_rad, &yaw_rad) != ALG_ATTITUDE_STATUS_OK)
    {
        return ALG_ATTITUDE_STATUS_NOT_INITIALIZED;
    }
    yaw_error_rad = remainderf(measured_yaw_rad - yaw_rad, 2.0F * 3.14159265358979323846F);
    alg_attitude_integrate_gyro(&me->quaternion, 0.0F, 0.0F, yaw_error_rad * correction_gain, 1.0F);
    return alg_attitude_normalize_quaternion(&me->quaternion) ? ALG_ATTITUDE_STATUS_OK
                                                              : ALG_ATTITUDE_STATUS_NUMERICAL_ERROR;
}

alg_attitude_status_t alg_attitude_get_euler(const alg_attitude_t *me, float *roll_rad,
                                             float *pitch_rad, float *yaw_rad)
{
    float pitch_sine;
    if ((me == NULL) || (roll_rad == NULL) || (pitch_rad == NULL) || (yaw_rad == NULL))
    {
        return ALG_ATTITUDE_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return ALG_ATTITUDE_STATUS_NOT_INITIALIZED;
    }
    *roll_rad = atan2f(
        2.0F * (me->quaternion.q0 * me->quaternion.q1 + me->quaternion.q2 * me->quaternion.q3),
        1.0F -
            2.0F * (me->quaternion.q1 * me->quaternion.q1 + me->quaternion.q2 * me->quaternion.q2));
    pitch_sine =
        2.0F * (me->quaternion.q0 * me->quaternion.q2 - me->quaternion.q3 * me->quaternion.q1);
    *pitch_rad = asinf(fmaxf(-1.0F, fminf(1.0F, pitch_sine)));
    *yaw_rad = atan2f(
        2.0F * (me->quaternion.q0 * me->quaternion.q3 + me->quaternion.q1 * me->quaternion.q2),
        1.0F -
            2.0F * (me->quaternion.q2 * me->quaternion.q2 + me->quaternion.q3 * me->quaternion.q3));
    return ALG_ATTITUDE_STATUS_OK;
}
