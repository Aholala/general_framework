#ifndef ALG_LQR_INTERNAL_H
#define ALG_LQR_INTERNAL_H

#include "alg_lqr.h"

bool AlgLqrInternal_IsFiniteArray(const float *values, size_t value_count);
bool AlgLqrInternal_HasNonnegativeDiagonal(const float *matrix,
                                           size_t dimension);
void AlgLqrInternal_Copy(float *destination,
                         const float *source,
                         size_t value_count);
void AlgLqrInternal_Multiply(const float *left,
                             size_t left_rows,
                             size_t shared_dimension,
                             const float *right,
                             size_t right_columns,
                             float *output);
void AlgLqrInternal_MultiplyLeftTranspose(const float *left,
                                          size_t left_rows,
                                          size_t left_columns,
                                          const float *right,
                                          size_t right_columns,
                                          float *output);
void AlgLqrInternal_Symmetrize(float *matrix, size_t dimension);
AlgLqrStatus_t AlgLqrInternal_Invert(float *matrix,
                                     float *inverse,
                                     size_t dimension);
AlgLqrStatus_t AlgLqrInternal_RiccatiStep(
    size_t state_dimension,
    size_t control_dimension,
    const float *state_matrix,
    const float *control_matrix,
    const float *state_weight,
    const float *control_weight,
    const float *cross_weight,
    const float *next_riccati,
    float *current_riccati,
    float *gain_matrix,
    float *workspace,
    size_t workspace_size);

#endif /* ALG_LQR_INTERNAL_H */
