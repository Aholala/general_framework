#ifndef ALG_LQR_INTERNAL_H
#define ALG_LQR_INTERNAL_H

#include "alg_lqr.h"

bool alg_lqr_internal_is_finite_array(const float *values, size_t value_count);
bool alg_lqr_internal_has_nonnegative_diagonal(const float *matrix, size_t dimension);
void alg_lqr_internal_copy(float *destination, const float *source, size_t value_count);
void alg_lqr_internal_multiply(const float *left, size_t left_rows, size_t shared_dimension,
                               const float *right, size_t right_columns, float *output);
void alg_lqr_internal_multiply_left_transpose(const float *left, size_t left_rows,
                                              size_t left_columns, const float *right,
                                              size_t right_columns, float *output);
void alg_lqr_internal_symmetrize(float *matrix, size_t dimension);
alg_lqr_status_t alg_lqr_internal_invert(float *matrix, float *inverse, size_t dimension);
alg_lqr_status_t alg_lqr_internal_riccati_step(
    size_t state_dimension, size_t control_dimension, const float *state_matrix,
    const float *control_matrix, const float *state_weight, const float *control_weight,
    const float *cross_weight, const float *next_riccati, float *current_riccati,
    float *gain_matrix, float *workspace, size_t workspace_size);

#endif /* ALG_LQR_INTERNAL_H */
