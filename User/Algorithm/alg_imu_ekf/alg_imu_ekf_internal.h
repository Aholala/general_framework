#ifndef ALG_IMU_EKF_INTERNAL_H
#define ALG_IMU_EKF_INTERNAL_H

#include "alg_imu_ekf.h"

bool alg_imu_ekf_internal_is_finite_array(const float *values, size_t value_count);
alg_imu_ekf_status_t alg_imu_ekf_internal_normalize_and_project(alg_imu_ekf_t *me);
alg_imu_ekf_status_t alg_imu_ekf_internal_map_kalman_status(alg_kalman_status_t status);
alg_kalman_status_t alg_imu_ekf_internal_state_function(const float *state, size_t state_dimension,
                                                        const float *control_input,
                                                        size_t control_dimension,
                                                        float delta_time_s, float *predicted_state,
                                                        void *user_context);
alg_kalman_status_t alg_imu_ekf_internal_state_jacobian(const float *state, size_t state_dimension,
                                                        const float *control_input,
                                                        size_t control_dimension,
                                                        float delta_time_s, float *state_jacobian,
                                                        void *user_context);
alg_kalman_status_t alg_imu_ekf_internal_measurement_function(const float *state,
                                                              size_t state_dimension,
                                                              size_t measurement_dimension,
                                                              float *predicted_measurement,
                                                              void *user_context);
alg_kalman_status_t alg_imu_ekf_internal_measurement_jacobian(const float *state,
                                                              size_t state_dimension,
                                                              size_t measurement_dimension,
                                                              float *measurement_jacobian,
                                                              void *user_context);

#endif /* ALG_IMU_EKF_INTERNAL_H */
