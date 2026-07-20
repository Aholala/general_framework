#ifndef ALG_IMU_EKF_INTERNAL_H
#define ALG_IMU_EKF_INTERNAL_H

#include "alg_imu_ekf.h"

bool AlgImuEkfInternal_IsFiniteArray(const float *values, size_t value_count);
AlgImuEkfStatus_t AlgImuEkfInternal_NormalizeAndProject(AlgImuEkf_t *self);
AlgImuEkfStatus_t AlgImuEkfInternal_MapKalmanStatus(AlgKalmanStatus_t status);
AlgKalmanStatus_t AlgImuEkfInternal_StateFunction(
    const float *state,
    size_t state_dimension,
    const float *control_input,
    size_t control_dimension,
    float delta_time_s,
    float *predicted_state,
    void *user_context);
AlgKalmanStatus_t AlgImuEkfInternal_StateJacobian(
    const float *state,
    size_t state_dimension,
    const float *control_input,
    size_t control_dimension,
    float delta_time_s,
    float *state_jacobian,
    void *user_context);
AlgKalmanStatus_t AlgImuEkfInternal_MeasurementFunction(
    const float *state,
    size_t state_dimension,
    size_t measurement_dimension,
    float *predicted_measurement,
    void *user_context);
AlgKalmanStatus_t AlgImuEkfInternal_MeasurementJacobian(
    const float *state,
    size_t state_dimension,
    size_t measurement_dimension,
    float *measurement_jacobian,
    void *user_context);

#endif /* ALG_IMU_EKF_INTERNAL_H */
