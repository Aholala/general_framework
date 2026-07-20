#ifndef ALG_KALMAN_INTERNAL_H
#define ALG_KALMAN_INTERNAL_H

#include "alg_kalman.h"

bool AlgKalmanInternal_IsFiniteArray(const float *values, size_t value_count);
bool AlgKalmanInternal_HasNonnegativeDiagonal(const float *matrix,
                                              size_t dimension);
void AlgKalmanInternal_Copy(float *destination,
                            const float *source,
                            size_t value_count);
void AlgKalmanInternal_Multiply(const float *left,
                                size_t left_rows,
                                size_t shared_dimension,
                                const float *right,
                                size_t right_columns,
                                float *output);
void AlgKalmanInternal_MultiplyRightTranspose(const float *left,
                                              size_t left_rows,
                                              size_t shared_dimension,
                                              const float *right,
                                              size_t right_rows,
                                              float *output);
AlgKalmanStatus_t AlgKalmanInternal_Correct(float *state,
                                            float *covariance,
                                            size_t state_dimension,
                                            const float *measurement_matrix,
                                            const float *measurement_noise,
                                            const float *measurement,
                                            const float *predicted_measurement,
                                            size_t measurement_dimension,
                                            float *workspace,
                                            size_t workspace_size);
void AlgKalmanInternal_Symmetrize(float *matrix, size_t dimension);

#endif /* ALG_KALMAN_INTERNAL_H */
