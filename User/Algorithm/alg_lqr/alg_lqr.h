#ifndef ALG_LQR_H
#define ALG_LQR_H

#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define ALG_LQR_RICCATI_WORKSPACE_SIZE(state_dimension, control_dimension)                         \
    ((4U * (state_dimension) * (state_dimension)) +                                                \
     (6U * (state_dimension) * (control_dimension)) +                                              \
     (3U * (control_dimension) * (control_dimension)))

#define ALG_LQR_FINITE_WORKSPACE_SIZE(state_dimension, control_dimension)                          \
    (((state_dimension) * (state_dimension)) +                                                     \
     ALG_LQR_RICCATI_WORKSPACE_SIZE((state_dimension), (control_dimension)))

#define ALG_LQR_DISCRETIZE_WORKSPACE_SIZE(state_dimension, control_dimension)                      \
    ((3U * (state_dimension) * (state_dimension)) + ((state_dimension) * (control_dimension)))

    typedef enum
    {
        ALG_LQR_STATUS_OK = 0,
        ALG_LQR_STATUS_INVALID_ARGUMENT,
        ALG_LQR_STATUS_OUT_OF_RANGE,
        ALG_LQR_STATUS_INSUFFICIENT_WORKSPACE,
        ALG_LQR_STATUS_NOT_INITIALIZED,
        ALG_LQR_STATUS_SINGULAR_MATRIX,
        ALG_LQR_STATUS_NOT_CONVERGED,
        ALG_LQR_STATUS_NUMERICAL_ERROR
    } alg_lqr_status_t;

    typedef struct
    {
        size_t state_dimension;
        size_t control_dimension;
        const float *gain_matrix;
        const float *control_min;
        const float *control_max;
    } alg_lqr_controller_config_t;

    typedef struct
    {
        alg_lqr_controller_config_t config;
        bool is_initialized;
    } alg_lqr_controller_t;

    alg_lqr_status_t alg_lqr_controller_init(alg_lqr_controller_t *me,
                                             const alg_lqr_controller_config_t *config);
    alg_lqr_status_t alg_lqr_controller_update(const alg_lqr_controller_t *me, const float *state,
                                               const float *reference_state,
                                               const float *equilibrium_control,
                                               const float *feedforward_control,
                                               float *control_output);

    typedef struct
    {
        size_t state_dimension;
        size_t control_dimension;
        const float *state_matrix;
        const float *control_matrix;
        const float *state_weight;
        const float *control_weight;
        const float *cross_weight;
        float tolerance;
        size_t maximum_iterations;
        float *workspace;
        size_t workspace_size;
    } alg_lqr_dare_config_t;

    alg_lqr_status_t alg_lqr_dare_solve(const alg_lqr_dare_config_t *config,
                                        float *riccati_solution, float *gain_matrix,
                                        size_t *completed_iterations);

    typedef struct
    {
        size_t state_dimension;
        size_t control_dimension;
        size_t horizon_length;
        const float *state_matrix;
        const float *control_matrix;
        const float *state_weight;
        const float *control_weight;
        const float *cross_weight;
        const float *terminal_state_weight;
        float *workspace;
        size_t workspace_size;
    } alg_lqr_finite_config_t;

    alg_lqr_status_t alg_lqr_finite_solve(const alg_lqr_finite_config_t *config,
                                          float *gain_sequence, float *initial_riccati_solution);

    alg_lqr_status_t alg_lqr_discretize_tustin(const float *continuous_state_matrix,
                                               const float *continuous_control_matrix,
                                               size_t state_dimension, size_t control_dimension,
                                               float delta_time_s, float *discrete_state_matrix,
                                               float *discrete_control_matrix, float *workspace,
                                               size_t workspace_size);

    alg_lqr_status_t alg_lqr_lqi_build_augmented_model(
        const float *state_matrix, const float *control_matrix, const float *output_matrix,
        size_t state_dimension, size_t control_dimension, size_t integral_dimension,
        float delta_time_s, float *augmented_state_matrix, float *augmented_control_matrix);

#ifdef __cplusplus
}
#endif

#endif /* ALG_LQR_H */
