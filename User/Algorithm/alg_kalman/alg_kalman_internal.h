#ifndef ALG_KALMAN_INTERNAL_H
#define ALG_KALMAN_INTERNAL_H

#include "alg_kalman.h"

bool alg_kalman_internal_is_finite_array(const float *values, size_t value_count);
bool alg_kalman_internal_has_nonnegative_diagonal(const float *matrix, size_t dimension);
void alg_kalman_internal_copy(float *destination, const float *source, size_t value_count);
void alg_kalman_internal_multiply(const float *left, size_t left_rows, size_t shared_dimension,
                                  const float *right, size_t right_columns, float *output);
void alg_kalman_internal_multiply_right_transpose(const float *left, size_t left_rows,
                                                  size_t shared_dimension, const float *right,
                                                  size_t right_rows, float *output);
alg_kalman_status_t
alg_kalman_internal_correct(float *state, float *covariance, size_t state_dimension,
                            const float *measurement_matrix, const float *measurement_noise,
                            const float *measurement, const float *predicted_measurement,
                            size_t measurement_dimension, float *workspace, size_t workspace_size);
void alg_kalman_internal_symmetrize(float *matrix, size_t dimension);

#endif /* ALG_KALMAN_INTERNAL_H */
