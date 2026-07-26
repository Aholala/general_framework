#ifndef ALG_IMU_EKF_H
#define ALG_IMU_EKF_H

#include <stdbool.h>

#include "alg_filter.h"
#include "alg_kalman.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define ALG_IMU_EKF_STATE_DIMENSION (6U)
#define ALG_IMU_EKF_MEASUREMENT_DIMENSION (3U)
#define ALG_IMU_EKF_CONTROL_DIMENSION (3U)

    typedef enum
    {
        ALG_IMU_EKF_STATUS_OK = 0,
        ALG_IMU_EKF_STATUS_INVALID_ARGUMENT,
        ALG_IMU_EKF_STATUS_OUT_OF_RANGE,
        ALG_IMU_EKF_STATUS_NOT_INITIALIZED,
        ALG_IMU_EKF_STATUS_ACCELEROMETER_REJECTED,
        ALG_IMU_EKF_STATUS_NUMERICAL_ERROR,
        ALG_IMU_EKF_STATUS_KALMAN_ERROR
    } alg_imu_ekf_status_t;

    typedef struct
    {
        float w;
        float x;
        float y;
        float z;
    } alg_imu_ekf_quaternion_t;

    typedef struct
    {
        float roll_rad;
        float pitch_rad;
        float yaw_rad;
    } alg_imu_ekf_euler_t;

    typedef struct
    {
        float gravity_m_s2;
        float gyro_noise_std_rad_s;
        float gyro_bias_random_walk_std_rad_s2;
        float accelerometer_direction_noise_std;
        float accelerometer_lpf_cutoff_hz;
        float accelerometer_rejection_threshold_g;
        float chi_square_adaptation_threshold;
        float chi_square_rejection_threshold;
        float maximum_measurement_noise_scale;
        float gyro_bias_fading_factor;
        float initial_attitude_variance;
        float initial_gyro_bias_variance;
    } alg_imu_ekf_config_t;

    typedef struct
    {
        float filtered_accelerometer_m_s2[3];
        float innovation[3];
        float accelerometer_norm_m_s2;
        float accelerometer_deviation_g;
        float normalized_innovation_squared;
        float measurement_noise_scale;
        bool was_accelerometer_used;
    } alg_imu_ekf_diagnostics_t;

    typedef struct
    {
        alg_imu_ekf_config_t config;
        alg_kalman_extended_t kalman;
        float state[ALG_IMU_EKF_STATE_DIMENSION];
        float covariance[ALG_IMU_EKF_STATE_DIMENSION * ALG_IMU_EKF_STATE_DIMENSION];
        float process_noise[ALG_IMU_EKF_STATE_DIMENSION * ALG_IMU_EKF_STATE_DIMENSION];
        float measurement_noise[ALG_IMU_EKF_MEASUREMENT_DIMENSION *
                                ALG_IMU_EKF_MEASUREMENT_DIMENSION];
        float kalman_workspace[ALG_KALMAN_WORKSPACE_SIZE(ALG_IMU_EKF_STATE_DIMENSION,
                                                         ALG_IMU_EKF_MEASUREMENT_DIMENSION)];
        float
            normalization_workspace[2U * ALG_IMU_EKF_STATE_DIMENSION * ALG_IMU_EKF_STATE_DIMENSION];
        float innovation_workspace[60U];
        alg_filter_low_pass_t accelerometer_filter[3];
        float filtered_accelerometer_m_s2[3];
        float innovation[3];
        float last_accelerometer_norm_m_s2;
        float last_accelerometer_deviation_g;
        float last_normalized_innovation_squared;
        float last_measurement_noise_scale;
        bool was_accelerometer_used;
        bool is_initialized;
    } alg_imu_ekf_t;

    alg_imu_ekf_status_t alg_imu_ekf_config_init(alg_imu_ekf_config_t *config);
    alg_imu_ekf_status_t alg_imu_ekf_init(alg_imu_ekf_t *me, const alg_imu_ekf_config_t *config);
    alg_imu_ekf_status_t alg_imu_ekf_reset(alg_imu_ekf_t *me,
                                           const alg_imu_ekf_quaternion_t *quaternion,
                                           const float gyro_bias_rad_s[2]);
    alg_imu_ekf_status_t alg_imu_ekf_reset_from_accelerometer(alg_imu_ekf_t *me,
                                                              const float accelerometer_m_s2[3]);
    alg_imu_ekf_status_t alg_imu_ekf_predict(alg_imu_ekf_t *me, const float gyroscope_rad_s[3],
                                             float delta_time_s);
    alg_imu_ekf_status_t alg_imu_ekf_correct_accelerometer(alg_imu_ekf_t *me,
                                                           const float accelerometer_m_s2[3],
                                                           float delta_time_s);
    alg_imu_ekf_status_t alg_imu_ekf_update(alg_imu_ekf_t *me, const float gyroscope_rad_s[3],
                                            const float accelerometer_m_s2[3], float delta_time_s,
                                            bool *accelerometer_used);
    alg_imu_ekf_status_t alg_imu_ekf_get_quaternion(const alg_imu_ekf_t *me,
                                                    alg_imu_ekf_quaternion_t *quaternion);
    alg_imu_ekf_status_t alg_imu_ekf_get_euler(const alg_imu_ekf_t *me, alg_imu_ekf_euler_t *euler);
    alg_imu_ekf_status_t alg_imu_ekf_get_gyro_bias(const alg_imu_ekf_t *me,
                                                   float gyro_bias_rad_s[3]);
    alg_imu_ekf_status_t alg_imu_ekf_get_corrected_gyroscope(const alg_imu_ekf_t *me,
                                                             const float gyroscope_rad_s[3],
                                                             float corrected_gyroscope_rad_s[3]);
    alg_imu_ekf_status_t alg_imu_ekf_get_diagnostics(const alg_imu_ekf_t *me,
                                                     alg_imu_ekf_diagnostics_t *diagnostics);
    alg_imu_ekf_status_t alg_imu_ekf_get_gravity_body(const alg_imu_ekf_t *me,
                                                      float gravity_body_m_s2[3]);
    alg_imu_ekf_status_t
    alg_imu_ekf_get_linear_acceleration_body(const alg_imu_ekf_t *me,
                                             const float accelerometer_m_s2[3],
                                             float linear_acceleration_body_m_s2[3]);
    alg_imu_ekf_status_t
    alg_imu_ekf_get_linear_acceleration_world(const alg_imu_ekf_t *me,
                                              const float accelerometer_m_s2[3],
                                              float linear_acceleration_world_m_s2[3]);

#ifdef __cplusplus
}
#endif

#endif /* ALG_IMU_EKF_H */
