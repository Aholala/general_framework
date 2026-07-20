#ifndef ALG_KALMAN_H
#define ALG_KALMAN_H

#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ALG_KALMAN_WORKSPACE_SIZE(state_dimension, measurement_dimension)       \
    ((state_dimension) +                                                       \
     (3U * (state_dimension) * (state_dimension)) +                            \
     (4U * (state_dimension) * (measurement_dimension)) +                      \
     (2U * (measurement_dimension)) +                                          \
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
} AlgKalmanStatus_t;

typedef struct
{
    float process_noise;
    float measurement_noise;
    float estimate;
    float covariance;
    float gain;
    bool is_initialized;
} AlgKalmanScalar_t;

AlgKalmanStatus_t AlgKalmanScalar_Init(AlgKalmanScalar_t *self,
                                       float process_noise,
                                       float measurement_noise,
                                       float initial_estimate,
                                       float initial_covariance);
AlgKalmanStatus_t AlgKalmanScalar_SetNoise(AlgKalmanScalar_t *self,
                                           float process_noise,
                                           float measurement_noise);
AlgKalmanStatus_t AlgKalmanScalar_Reset(AlgKalmanScalar_t *self,
                                        float initial_estimate,
                                        float initial_covariance);
AlgKalmanStatus_t AlgKalmanScalar_Predict(AlgKalmanScalar_t *self,
                                          float state_delta);
AlgKalmanStatus_t AlgKalmanScalar_Correct(AlgKalmanScalar_t *self,
                                          float measurement,
                                          float *output);
AlgKalmanStatus_t AlgKalmanScalar_Update(AlgKalmanScalar_t *self,
                                         float measurement,
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
} AlgKalmanLinearConfig_t;

typedef struct
{
    AlgKalmanLinearConfig_t config;
    bool is_initialized;
} AlgKalmanLinear_t;

AlgKalmanStatus_t AlgKalmanLinear_Init(AlgKalmanLinear_t *self,
                                       const AlgKalmanLinearConfig_t *config);
AlgKalmanStatus_t AlgKalmanLinear_Reset(AlgKalmanLinear_t *self,
                                        const float *initial_state,
                                        const float *initial_covariance);
AlgKalmanStatus_t AlgKalmanLinear_Predict(AlgKalmanLinear_t *self,
                                          const float *control_input);
AlgKalmanStatus_t AlgKalmanLinear_Correct(AlgKalmanLinear_t *self,
                                          const float *measurement);
const float *AlgKalmanLinear_GetState(const AlgKalmanLinear_t *self);
const float *AlgKalmanLinear_GetCovariance(const AlgKalmanLinear_t *self);

typedef AlgKalmanStatus_t (*AlgKalmanStateFunction_t)(
    const float *state,
    size_t state_dimension,
    const float *control_input,
    size_t control_dimension,
    float delta_time_s,
    float *predicted_state,
    void *user_context);

typedef AlgKalmanStatus_t (*AlgKalmanStateJacobianFunction_t)(
    const float *state,
    size_t state_dimension,
    const float *control_input,
    size_t control_dimension,
    float delta_time_s,
    float *state_jacobian,
    void *user_context);

typedef AlgKalmanStatus_t (*AlgKalmanMeasurementFunction_t)(
    const float *state,
    size_t state_dimension,
    size_t measurement_dimension,
    float *predicted_measurement,
    void *user_context);

typedef AlgKalmanStatus_t (*AlgKalmanMeasurementJacobianFunction_t)(
    const float *state,
    size_t state_dimension,
    size_t measurement_dimension,
    float *measurement_jacobian,
    void *user_context);

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
    AlgKalmanStateFunction_t state_function;
    AlgKalmanStateJacobianFunction_t state_jacobian_function;
    AlgKalmanMeasurementFunction_t measurement_function;
    AlgKalmanMeasurementJacobianFunction_t measurement_jacobian_function;
    void *user_context;
} AlgKalmanExtendedConfig_t;

typedef struct
{
    AlgKalmanExtendedConfig_t config;
    bool is_initialized;
} AlgKalmanExtended_t;

AlgKalmanStatus_t AlgKalmanExtended_Init(AlgKalmanExtended_t *self,
                                         const AlgKalmanExtendedConfig_t *config);
AlgKalmanStatus_t AlgKalmanExtended_Reset(AlgKalmanExtended_t *self,
                                          const float *initial_state,
                                          const float *initial_covariance);
AlgKalmanStatus_t AlgKalmanExtended_Predict(AlgKalmanExtended_t *self,
                                            const float *control_input,
                                            float delta_time_s);
AlgKalmanStatus_t AlgKalmanExtended_Correct(AlgKalmanExtended_t *self,
                                            const float *measurement);
const float *AlgKalmanExtended_GetState(const AlgKalmanExtended_t *self);
const float *AlgKalmanExtended_GetCovariance(const AlgKalmanExtended_t *self);

#ifdef __cplusplus
}
#endif

#endif /* ALG_KALMAN_H */
