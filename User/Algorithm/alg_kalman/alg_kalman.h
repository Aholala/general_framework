#ifndef ALG_KALMAN_H
#define ALG_KALMAN_H

#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define ALG_KALMAN_WORKSPACE_SIZE(state_dimension, measurement_dimension)                          \
    ((state_dimension) + (3U * (state_dimension) * (state_dimension)) +                            \
     (4U * (state_dimension) * (measurement_dimension)) + (2U * (measurement_dimension)) +         \
     (2U * (measurement_dimension) * (measurement_dimension)))

    typedef enum
    {
        ALG_KALMAN_STATUS_OK = 0,
        ALG_KALMAN_STATUS_INVALID_ARGUMENT,
        ALG_KALMAN_STATUS_OUT_OF_RANGE,
        ALG_KALMAN_STATUS_INSUFFICIENT_WORKSPACE,
        ALG_KALMAN_STATUS_NOT_INITIALIZED,
        ALG_KALMAN_STATUS_SINGULAR_MATRIX,
        ALG_KALMAN_STATUS_MODEL_ERROR,
        ALG_KALMAN_STATUS_NUMERICAL_ERROR
    } alg_kalman_status_t;

    typedef struct
    {
        float process_noise;
        float measurement_noise;
        float estimate;
        float covariance;
        float gain;
        bool is_initialized;
    } alg_kalman_scalar_t;

    alg_kalman_status_t alg_kalman_scalar_init(alg_kalman_scalar_t *me, float process_noise,
                                               float measurement_noise, float initial_estimate,
                                               float initial_covariance);
    alg_kalman_status_t alg_kalman_scalar_set_noise(alg_kalman_scalar_t *me, float process_noise,
                                                    float measurement_noise);
    alg_kalman_status_t alg_kalman_scalar_reset(alg_kalman_scalar_t *me, float initial_estimate,
                                                float initial_covariance);
    alg_kalman_status_t alg_kalman_scalar_predict(alg_kalman_scalar_t *me, float state_delta);
    alg_kalman_status_t alg_kalman_scalar_correct(alg_kalman_scalar_t *me, float measurement,
                                                  float *output);
    alg_kalman_status_t alg_kalman_scalar_update(alg_kalman_scalar_t *me, float measurement,
                                                 float *output);

    typedef struct
    {
        size_t state_dimension;
        size_t measurement_dimension;
        size_t control_dimension;
        float *state;
        float *covariance;
        const float *transition_matrix;
        const float *control_matrix;
        const float *process_noise;
        const float *measurement_matrix;
        const float *measurement_noise;
        float *workspace;
        size_t workspace_size;
    } alg_kalman_linear_config_t;

    typedef struct
    {
        alg_kalman_linear_config_t config;
        bool is_initialized;
    } alg_kalman_linear_t;

    alg_kalman_status_t alg_kalman_linear_init(alg_kalman_linear_t *me,
                                               const alg_kalman_linear_config_t *config);
    alg_kalman_status_t alg_kalman_linear_reset(alg_kalman_linear_t *me, const float *initial_state,
                                                const float *initial_covariance);
    alg_kalman_status_t alg_kalman_linear_predict(alg_kalman_linear_t *me,
                                                  const float *control_input);
    alg_kalman_status_t alg_kalman_linear_correct(alg_kalman_linear_t *me,
                                                  const float *measurement);
    const float *alg_kalman_linear_get_state(const alg_kalman_linear_t *me);
    const float *alg_kalman_linear_get_covariance(const alg_kalman_linear_t *me);

    typedef alg_kalman_status_t (*alg_kalman_state_function_t)(
        const float *state, size_t state_dimension, const float *control_input,
        size_t control_dimension, float delta_time_s, float *predicted_state, void *user_context);

    typedef alg_kalman_status_t (*alg_kalman_state_jacobian_function_t)(
        const float *state, size_t state_dimension, const float *control_input,
        size_t control_dimension, float delta_time_s, float *state_jacobian, void *user_context);

    typedef alg_kalman_status_t (*alg_kalman_measurement_function_t)(const float *state,
                                                                     size_t state_dimension,
                                                                     size_t measurement_dimension,
                                                                     float *predicted_measurement,
                                                                     void *user_context);

    typedef alg_kalman_status_t (*alg_kalman_measurement_jacobian_function_t)(
        const float *state, size_t state_dimension, size_t measurement_dimension,
        float *measurement_jacobian, void *user_context);

    typedef struct
    {
        size_t state_dimension;
        size_t measurement_dimension;
        size_t control_dimension;
        float *state;
        float *covariance;
        const float *process_noise;
        const float *measurement_noise;
        float *workspace;
        size_t workspace_size;
        alg_kalman_state_function_t state_function;
        alg_kalman_state_jacobian_function_t state_jacobian_function;
        alg_kalman_measurement_function_t measurement_function;
        alg_kalman_measurement_jacobian_function_t measurement_jacobian_function;
        void *user_context;
    } alg_kalman_extended_config_t;

    typedef struct
    {
        alg_kalman_extended_config_t config;
        bool is_initialized;
    } alg_kalman_extended_t;

    alg_kalman_status_t alg_kalman_extended_init(alg_kalman_extended_t *me,
                                                 const alg_kalman_extended_config_t *config);
    alg_kalman_status_t alg_kalman_extended_reset(alg_kalman_extended_t *me,
                                                  const float *initial_state,
                                                  const float *initial_covariance);
    alg_kalman_status_t alg_kalman_extended_predict(alg_kalman_extended_t *me,
                                                    const float *control_input, float delta_time_s);
    alg_kalman_status_t alg_kalman_extended_correct(alg_kalman_extended_t *me,
                                                    const float *measurement);
    const float *alg_kalman_extended_get_state(const alg_kalman_extended_t *me);
    const float *alg_kalman_extended_get_covariance(const alg_kalman_extended_t *me);

#ifdef __cplusplus
}
#endif

#endif /* ALG_KALMAN_H */
