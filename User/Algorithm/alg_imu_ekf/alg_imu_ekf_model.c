#include "alg_imu_ekf_internal.h"

#include <math.h>
#include <stddef.h>

AlgKalmanStatus_t AlgImuEkfInternal_StateFunction(
    const float *state,
    size_t state_dimension,
    const float *control_input,
    size_t control_dimension,
    float delta_time_s,
    float *predicted_state,
    void *user_context)
{
    float angular_rate_x;
    float angular_rate_y;
    float angular_rate_z;
    float quaternion_norm;

    (void)user_context;
    if ((state == NULL) || (control_input == NULL) || (predicted_state == NULL) ||
        (state_dimension != ALG_IMU_EKF_STATE_DIMENSION) ||
        (control_dimension != ALG_IMU_EKF_CONTROL_DIMENSION) ||
        !isfinite(delta_time_s) || (delta_time_s <= 0.0F))
    {
        return ALG_KALMAN_STATUS_MODEL_ERROR;
    }

    angular_rate_x = control_input[0] - state[4];
    angular_rate_y = control_input[1] - state[5];
    angular_rate_z = control_input[2];

    predicted_state[0] = state[0] -
                         (0.5F * delta_time_s *
                          ((state[1] * angular_rate_x) +
                           (state[2] * angular_rate_y) +
                           (state[3] * angular_rate_z)));
    predicted_state[1] = state[1] +
                         (0.5F * delta_time_s *
                          ((state[0] * angular_rate_x) +
                           (state[2] * angular_rate_z) -
                           (state[3] * angular_rate_y)));
    predicted_state[2] = state[2] +
                         (0.5F * delta_time_s *
                          ((state[0] * angular_rate_y) -
                           (state[1] * angular_rate_z) +
                           (state[3] * angular_rate_x)));
    predicted_state[3] = state[3] +
                         (0.5F * delta_time_s *
                          ((state[0] * angular_rate_z) +
                           (state[1] * angular_rate_y) -
                           (state[2] * angular_rate_x)));
    predicted_state[4] = state[4];
    predicted_state[5] = state[5];

    quaternion_norm = sqrtf((predicted_state[0] * predicted_state[0]) +
                            (predicted_state[1] * predicted_state[1]) +
                            (predicted_state[2] * predicted_state[2]) +
                            (predicted_state[3] * predicted_state[3]));
    if (!isfinite(quaternion_norm) || (quaternion_norm <= 1.0e-6F))
    {
        return ALG_KALMAN_STATUS_MODEL_ERROR;
    }
    predicted_state[0] /= quaternion_norm;
    predicted_state[1] /= quaternion_norm;
    predicted_state[2] /= quaternion_norm;
    predicted_state[3] /= quaternion_norm;
    return ALG_KALMAN_STATUS_OK;
}

AlgKalmanStatus_t AlgImuEkfInternal_StateJacobian(
    const float *state,
    size_t state_dimension,
    const float *control_input,
    size_t control_dimension,
    float delta_time_s,
    float *state_jacobian,
    void *user_context)
{
    float angular_rate_x;
    float angular_rate_y;
    float angular_rate_z;
    float half_delta_time;
    size_t index;

    (void)user_context;
    if ((state == NULL) || (control_input == NULL) || (state_jacobian == NULL) ||
        (state_dimension != ALG_IMU_EKF_STATE_DIMENSION) ||
        (control_dimension != ALG_IMU_EKF_CONTROL_DIMENSION) ||
        !isfinite(delta_time_s) || (delta_time_s <= 0.0F))
    {
        return ALG_KALMAN_STATUS_MODEL_ERROR;
    }

    angular_rate_x = control_input[0] - state[4];
    angular_rate_y = control_input[1] - state[5];
    angular_rate_z = control_input[2];
    half_delta_time = 0.5F * delta_time_s;
    for (index = 0U; index <
                         (ALG_IMU_EKF_STATE_DIMENSION *
                          ALG_IMU_EKF_STATE_DIMENSION);
         ++index)
    {
        state_jacobian[index] = 0.0F;
    }

#define F(row, column) state_jacobian[((row) * ALG_IMU_EKF_STATE_DIMENSION) + (column)]
    F(0U, 0U) = 1.0F;
    F(0U, 1U) = -half_delta_time * angular_rate_x;
    F(0U, 2U) = -half_delta_time * angular_rate_y;
    F(0U, 3U) = -half_delta_time * angular_rate_z;
    F(1U, 0U) = half_delta_time * angular_rate_x;
    F(1U, 1U) = 1.0F;
    F(1U, 2U) = half_delta_time * angular_rate_z;
    F(1U, 3U) = -half_delta_time * angular_rate_y;
    F(2U, 0U) = half_delta_time * angular_rate_y;
    F(2U, 1U) = -half_delta_time * angular_rate_z;
    F(2U, 2U) = 1.0F;
    F(2U, 3U) = half_delta_time * angular_rate_x;
    F(3U, 0U) = half_delta_time * angular_rate_z;
    F(3U, 1U) = half_delta_time * angular_rate_y;
    F(3U, 2U) = -half_delta_time * angular_rate_x;
    F(3U, 3U) = 1.0F;

    F(0U, 4U) = half_delta_time * state[1];
    F(0U, 5U) = half_delta_time * state[2];
    F(1U, 4U) = -half_delta_time * state[0];
    F(1U, 5U) = half_delta_time * state[3];
    F(2U, 4U) = -half_delta_time * state[3];
    F(2U, 5U) = -half_delta_time * state[0];
    F(3U, 4U) = half_delta_time * state[2];
    F(3U, 5U) = -half_delta_time * state[1];
    F(4U, 4U) = 1.0F;
    F(5U, 5U) = 1.0F;
#undef F

    return ALG_KALMAN_STATUS_OK;
}

AlgKalmanStatus_t AlgImuEkfInternal_MeasurementFunction(
    const float *state,
    size_t state_dimension,
    size_t measurement_dimension,
    float *predicted_measurement,
    void *user_context)
{
    (void)user_context;
    if ((state == NULL) || (predicted_measurement == NULL) ||
        (state_dimension != ALG_IMU_EKF_STATE_DIMENSION) ||
        (measurement_dimension != ALG_IMU_EKF_MEASUREMENT_DIMENSION))
    {
        return ALG_KALMAN_STATUS_MODEL_ERROR;
    }

    predicted_measurement[0] =
        2.0F * ((state[1] * state[3]) - (state[0] * state[2]));
    predicted_measurement[1] =
        2.0F * ((state[0] * state[1]) + (state[2] * state[3]));
    predicted_measurement[2] = (state[0] * state[0]) -
                               (state[1] * state[1]) -
                               (state[2] * state[2]) +
                               (state[3] * state[3]);
    return ALG_KALMAN_STATUS_OK;
}

AlgKalmanStatus_t AlgImuEkfInternal_MeasurementJacobian(
    const float *state,
    size_t state_dimension,
    size_t measurement_dimension,
    float *measurement_jacobian,
    void *user_context)
{
    size_t index;

    (void)user_context;
    if ((state == NULL) || (measurement_jacobian == NULL) ||
        (state_dimension != ALG_IMU_EKF_STATE_DIMENSION) ||
        (measurement_dimension != ALG_IMU_EKF_MEASUREMENT_DIMENSION))
    {
        return ALG_KALMAN_STATUS_MODEL_ERROR;
    }
    for (index = 0U; index <
                         (ALG_IMU_EKF_MEASUREMENT_DIMENSION *
                          ALG_IMU_EKF_STATE_DIMENSION);
         ++index)
    {
        measurement_jacobian[index] = 0.0F;
    }

#define H(row, column) measurement_jacobian[((row) * ALG_IMU_EKF_STATE_DIMENSION) + (column)]
    H(0U, 0U) = -2.0F * state[2];
    H(0U, 1U) = 2.0F * state[3];
    H(0U, 2U) = -2.0F * state[0];
    H(0U, 3U) = 2.0F * state[1];
    H(1U, 0U) = 2.0F * state[1];
    H(1U, 1U) = 2.0F * state[0];
    H(1U, 2U) = 2.0F * state[3];
    H(1U, 3U) = 2.0F * state[2];
    H(2U, 0U) = 2.0F * state[0];
    H(2U, 1U) = -2.0F * state[1];
    H(2U, 2U) = -2.0F * state[2];
    H(2U, 3U) = 2.0F * state[3];
#undef H

    return ALG_KALMAN_STATUS_OK;
}
