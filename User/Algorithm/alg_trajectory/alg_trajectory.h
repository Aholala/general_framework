#ifndef ALG_TRAJECTORY_H
#define ALG_TRAJECTORY_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

    typedef enum
    {
        ALG_TRAJECTORY_STATUS_OK = 0,
        ALG_TRAJECTORY_STATUS_FINISHED,
        ALG_TRAJECTORY_STATUS_INVALID_ARGUMENT,
        ALG_TRAJECTORY_STATUS_NOT_INITIALIZED,
        ALG_TRAJECTORY_STATUS_NUMERICAL_ERROR
    } alg_trajectory_status_t;

    typedef enum
    {
        ALG_TRAJECTORY_PROFILE_TRAPEZOIDAL = 0,
        ALG_TRAJECTORY_PROFILE_S_CURVE
    } alg_trajectory_profile_t;

    typedef enum
    {
        ALG_TRAJECTORY_TARGET_POSITION = 0,
        ALG_TRAJECTORY_TARGET_VELOCITY
    } alg_trajectory_target_type_t;

    typedef struct
    {
        float maximum_velocity_per_s;
        float maximum_acceleration_per_s2;
        float maximum_deceleration_per_s2;
        float maximum_jerk_per_s3;
        float position_tolerance;
        float velocity_tolerance_per_s;
    } alg_trajectory_config_t;

    typedef struct
    {
        float position;
        float velocity_per_s;
        float acceleration_per_s2;
    } alg_trajectory_state_t;

    typedef struct
    {
        alg_trajectory_config_t config;
        alg_trajectory_state_t state;
        float target_position;
        float target_velocity_per_s;
        alg_trajectory_profile_t profile;
        alg_trajectory_target_type_t target_type;
        bool is_finished;
        bool is_initialized;
    } alg_trajectory_t;

    alg_trajectory_status_t alg_trajectory_init(alg_trajectory_t *me,
                                                const alg_trajectory_config_t *config,
                                                alg_trajectory_profile_t profile,
                                                const alg_trajectory_state_t *initial_state);
    alg_trajectory_status_t alg_trajectory_reset(alg_trajectory_t *me,
                                                 const alg_trajectory_state_t *state);
    alg_trajectory_status_t alg_trajectory_set_position_target(alg_trajectory_t *me,
                                                               float target_position,
                                                               float terminal_velocity_per_s);
    alg_trajectory_status_t alg_trajectory_set_velocity_target(alg_trajectory_t *me,
                                                               float target_velocity_per_s);
    alg_trajectory_status_t alg_trajectory_set_profile(alg_trajectory_t *me,
                                                       alg_trajectory_profile_t profile);
    alg_trajectory_status_t alg_trajectory_update(alg_trajectory_t *me, float delta_time_s,
                                                  alg_trajectory_state_t *output_state);
    alg_trajectory_status_t alg_trajectory_get_state(const alg_trajectory_t *me,
                                                     alg_trajectory_state_t *state);
    bool alg_trajectory_is_finished(const alg_trajectory_t *me);
    float alg_trajectory_calculate_stopping_distance(float velocity_per_s,
                                                     float deceleration_per_s2);

#ifdef __cplusplus
}
#endif

#endif /* ALG_TRAJECTORY_H */
