#ifndef ALG_AXIS_CONTROLLER_H
#define ALG_AXIS_CONTROLLER_H

#include "alg_lqr.h"
#include "alg_pid.h"

#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct alg_axis_controller alg_axis_controller_t;

    typedef enum
    {
        ALG_AXIS_CONTROLLER_STATUS_OK = 0,
        ALG_AXIS_CONTROLLER_STATUS_INVALID_ARGUMENT,
        ALG_AXIS_CONTROLLER_STATUS_NOT_INITIALIZED,
        ALG_AXIS_CONTROLLER_STATUS_ALGORITHM_ERROR
    } alg_axis_controller_status_t;

    typedef struct
    {
        float target_position_rad;
        float target_velocity_rad_per_s;
        float measured_position_rad;
        float measured_velocity_rad_per_s;
        float actuator_feedforward;
        float delta_time_s;
    } alg_axis_controller_input_t;

    typedef struct
    {
        alg_axis_controller_status_t (*reset)(alg_axis_controller_t *me,
                                              float measured_position_rad,
                                              float measured_velocity_rad_per_s,
                                              float initial_output);
        alg_axis_controller_status_t (*update)(alg_axis_controller_t *me,
                                               const alg_axis_controller_input_t *input,
                                               float *control_output);
    } alg_axis_controller_ops_t;

    struct alg_axis_controller
    {
        const alg_axis_controller_ops_t *vptr;
        bool is_initialized;
    };

    typedef struct
    {
        alg_pid_cascade_config_t cascade_config;
    } alg_axis_pid_config_t;

    typedef struct
    {
        alg_axis_controller_t super;
        alg_pid_cascade_t cascade;
    } alg_axis_pid_t;

    typedef struct
    {
        const float *gain_matrix;
        float control_min;
        float control_max;
        float equilibrium_control;
    } alg_axis_lqr_config_t;

    typedef struct
    {
        alg_axis_controller_t super;
        alg_lqr_controller_t controller;
        float gain_matrix[2];
        float control_min;
        float control_max;
        float equilibrium_control;
    } alg_axis_lqr_t;

    alg_axis_controller_status_t alg_axis_pid_init(alg_axis_pid_t *me,
                                                   const alg_axis_pid_config_t *config);
    alg_axis_controller_status_t alg_axis_lqr_init(alg_axis_lqr_t *me,
                                                   const alg_axis_lqr_config_t *config);
    alg_axis_controller_t *alg_axis_pid_as_controller(alg_axis_pid_t *me);
    alg_axis_controller_t *alg_axis_lqr_as_controller(alg_axis_lqr_t *me);
    alg_axis_controller_status_t alg_axis_controller_reset(alg_axis_controller_t *me,
                                                           float measured_position_rad,
                                                           float measured_velocity_rad_per_s,
                                                           float initial_output);
    alg_axis_controller_status_t
    alg_axis_controller_update(alg_axis_controller_t *me, const alg_axis_controller_input_t *input,
                               float *control_output);

#ifdef __cplusplus
}
#endif

#endif /* ALG_AXIS_CONTROLLER_H */
