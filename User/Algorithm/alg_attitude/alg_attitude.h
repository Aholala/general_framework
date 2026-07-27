#ifndef ALG_ATTITUDE_H
#define ALG_ATTITUDE_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

    typedef enum
    {
        ALG_ATTITUDE_STATUS_OK = 0,
        ALG_ATTITUDE_STATUS_GYRO_ONLY,
        ALG_ATTITUDE_STATUS_INVALID_ARGUMENT,
        ALG_ATTITUDE_STATUS_NOT_INITIALIZED,
        ALG_ATTITUDE_STATUS_NUMERICAL_ERROR
    } alg_attitude_status_t;

    typedef enum
    {
        ALG_ATTITUDE_METHOD_MAHONY = 0,
        ALG_ATTITUDE_METHOD_MADGWICK
    } alg_attitude_method_t;

    typedef struct
    {
        alg_attitude_method_t method;
        float proportional_gain;
        float integral_gain;
        float madgwick_beta;
        float acceleration_min_m_per_s2;
        float acceleration_max_m_per_s2;
    } alg_attitude_config_t;

    typedef struct
    {
        float q0;
        float q1;
        float q2;
        float q3;
    } alg_attitude_quaternion_t;

    typedef struct
    {
        alg_attitude_config_t config;
        alg_attitude_quaternion_t quaternion;
        float integral_error_x;
        float integral_error_y;
        float integral_error_z;
        bool is_initialized;
    } alg_attitude_t;

    alg_attitude_status_t alg_attitude_init(alg_attitude_t *me, const alg_attitude_config_t *config,
                                            const alg_attitude_quaternion_t *initial_quaternion);
    alg_attitude_status_t alg_attitude_reset(alg_attitude_t *me,
                                             const alg_attitude_quaternion_t *quaternion);
    alg_attitude_status_t alg_attitude_update(alg_attitude_t *me, float gyro_x_rad_per_s,
                                              float gyro_y_rad_per_s, float gyro_z_rad_per_s,
                                              float acceleration_x_m_per_s2,
                                              float acceleration_y_m_per_s2,
                                              float acceleration_z_m_per_s2, float delta_time_s);
    alg_attitude_status_t alg_attitude_correct_yaw(alg_attitude_t *me, float measured_yaw_rad,
                                                   float correction_gain);
    alg_attitude_status_t alg_attitude_get_euler(const alg_attitude_t *me, float *roll_rad,
                                                 float *pitch_rad, float *yaw_rad);

#ifdef __cplusplus
}
#endif

#endif
