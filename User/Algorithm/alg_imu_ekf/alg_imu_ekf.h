#ifndef ALG_IMU_EKF_H
#define ALG_IMU_EKF_H

#include <stdbool.h>

#include "alg_kalman.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ALG_IMU_EKF_STATE_DIMENSION       (7U)
#define ALG_IMU_EKF_MEASUREMENT_DIMENSION (3U)
#define ALG_IMU_EKF_CONTROL_DIMENSION     (3U)

typedef enum
{
    ALG_IMU_EKF_STATUS_OK = 0,
    ALG_IMU_EKF_STATUS_INVALID_ARGUMENT,
    ALG_IMU_EKF_STATUS_OUT_OF_RANGE,
    ALG_IMU_EKF_STATUS_NOT_INITIALIZED,
    ALG_IMU_EKF_STATUS_ACCELEROMETER_REJECTED,
    ALG_IMU_EKF_STATUS_NUMERICAL_ERROR,
    ALG_IMU_EKF_STATUS_KALMAN_ERROR
} AlgImuEkfStatus_t;

typedef struct
{
    float w;
    float x;
    float y;
    float z;
} AlgImuEkfQuaternion_t;

typedef struct
{
    float roll_rad;
    float pitch_rad;
    float yaw_rad;
} AlgImuEkfEuler_t;

typedef struct
{
    float gravity_m_s2;
    float gyro_noise_std_rad_s;
    float gyro_bias_random_walk_std_rad_s2;
    float accelerometer_direction_noise_std;
    float accelerometer_rejection_threshold_g;
    float accelerometer_noise_multiplier;
    float initial_attitude_variance;
    float initial_gyro_bias_variance;
} AlgImuEkfConfig_t;

typedef struct
{
    AlgImuEkfConfig_t config;
    AlgKalmanExtended_t kalman;
    float state[ALG_IMU_EKF_STATE_DIMENSION];
    float covariance[ALG_IMU_EKF_STATE_DIMENSION *
                     ALG_IMU_EKF_STATE_DIMENSION];
    float process_noise[ALG_IMU_EKF_STATE_DIMENSION *
                        ALG_IMU_EKF_STATE_DIMENSION];
    float measurement_noise[ALG_IMU_EKF_MEASUREMENT_DIMENSION *
                            ALG_IMU_EKF_MEASUREMENT_DIMENSION];
    float kalman_workspace[
        ALG_KALMAN_WORKSPACE_SIZE(ALG_IMU_EKF_STATE_DIMENSION,
                                  ALG_IMU_EKF_MEASUREMENT_DIMENSION)];
    float normalization_workspace[2U * ALG_IMU_EKF_STATE_DIMENSION *
                                  ALG_IMU_EKF_STATE_DIMENSION];
    float last_accelerometer_norm_m_s2;
    float last_accelerometer_deviation_g;
    bool was_accelerometer_used;
    bool is_initialized;
} AlgImuEkf_t;

AlgImuEkfStatus_t AlgImuEkfConfig_Init(AlgImuEkfConfig_t *config);
AlgImuEkfStatus_t AlgImuEkf_Init(AlgImuEkf_t *self,
                                 const AlgImuEkfConfig_t *config);
AlgImuEkfStatus_t AlgImuEkf_Reset(
    AlgImuEkf_t *self,
    const AlgImuEkfQuaternion_t *quaternion,
    const float gyro_bias_rad_s[3]);
AlgImuEkfStatus_t AlgImuEkf_ResetFromAccelerometer(
    AlgImuEkf_t *self,
    const float accelerometer_m_s2[3]);
AlgImuEkfStatus_t AlgImuEkf_Predict(
    AlgImuEkf_t *self,
    const float gyroscope_rad_s[3],
    float delta_time_s);
AlgImuEkfStatus_t AlgImuEkf_CorrectAccelerometer(
    AlgImuEkf_t *self,
    const float accelerometer_m_s2[3]);
AlgImuEkfStatus_t AlgImuEkf_Update(
    AlgImuEkf_t *self,
    const float gyroscope_rad_s[3],
    const float accelerometer_m_s2[3],
    float delta_time_s,
    bool *accelerometer_used);
AlgImuEkfStatus_t AlgImuEkf_GetQuaternion(
    const AlgImuEkf_t *self,
    AlgImuEkfQuaternion_t *quaternion);
AlgImuEkfStatus_t AlgImuEkf_GetEuler(
    const AlgImuEkf_t *self,
    AlgImuEkfEuler_t *euler);
AlgImuEkfStatus_t AlgImuEkf_GetGyroBias(
    const AlgImuEkf_t *self,
    float gyro_bias_rad_s[3]);
AlgImuEkfStatus_t AlgImuEkf_GetCorrectedGyroscope(
    const AlgImuEkf_t *self,
    const float gyroscope_rad_s[3],
    float corrected_gyroscope_rad_s[3]);
AlgImuEkfStatus_t AlgImuEkf_GetGravityBody(
    const AlgImuEkf_t *self,
    float gravity_body_m_s2[3]);
AlgImuEkfStatus_t AlgImuEkf_GetLinearAccelerationBody(
    const AlgImuEkf_t *self,
    const float accelerometer_m_s2[3],
    float linear_acceleration_body_m_s2[3]);
AlgImuEkfStatus_t AlgImuEkf_GetLinearAccelerationWorld(
    const AlgImuEkf_t *self,
    const float accelerometer_m_s2[3],
    float linear_acceleration_world_m_s2[3]);

#ifdef __cplusplus
}
#endif

#endif /* ALG_IMU_EKF_H */
