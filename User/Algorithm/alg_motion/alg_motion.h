#ifndef ALG_MOTION_H
#define ALG_MOTION_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

    typedef enum
    {
        ALG_MOTION_STATUS_OK = 0,
        ALG_MOTION_STATUS_INVALID_ARGUMENT,
        ALG_MOTION_STATUS_NOT_INITIALIZED,
        ALG_MOTION_STATUS_OUT_OF_RANGE,
        ALG_MOTION_STATUS_NUMERICAL_ERROR
    } alg_motion_status_t;

    typedef struct
    {
        float rising_rate_per_s;
        float falling_rate_per_s;
        float output_min;
        float output_max;
    } alg_motion_rate_limiter_config_t;

    typedef struct
    {
        alg_motion_rate_limiter_config_t config;
        float output;
        bool is_initialized;
    } alg_motion_rate_limiter_t;

    typedef struct
    {
        float period;
        float previous_wrapped_value;
        float continuous_value;
        bool has_previous_value;
        bool is_initialized;
    } alg_motion_unwrapper_t;

    alg_motion_status_t
    alg_motion_rate_limiter_init(alg_motion_rate_limiter_t *const me,
                                 const alg_motion_rate_limiter_config_t *const config,
                                 float initial_output);
    alg_motion_status_t alg_motion_rate_limiter_reset(alg_motion_rate_limiter_t *const me,
                                                      float initial_output);
    alg_motion_status_t alg_motion_rate_limiter_update(alg_motion_rate_limiter_t *const me,
                                                       float target, float delta_time_s,
                                                       float *output);
    alg_motion_status_t alg_motion_unwrapper_init(alg_motion_unwrapper_t *const me, float period);
    alg_motion_status_t alg_motion_unwrapper_reset(alg_motion_unwrapper_t *const me,
                                                   float wrapped_value, float continuous_value);
    alg_motion_status_t alg_motion_unwrapper_update(alg_motion_unwrapper_t *const me,
                                                    float wrapped_value, float *continuous_value);

#ifdef __cplusplus
}
#endif

#endif /* ALG_MOTION_H */
