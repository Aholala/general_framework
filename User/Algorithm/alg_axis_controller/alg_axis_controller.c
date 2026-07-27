#include "alg_axis_controller.h"

#include <math.h>
#include <stddef.h>

#define ALG_AXIS_CONTROLLER_CONTAINER_OF(member_pointer, container_type, member_name)              \
    ((container_type *)((unsigned char *)(member_pointer) - offsetof(container_type, member_name)))

static bool alg_axis_controller_input_is_valid(const alg_axis_controller_input_t *input)
{
    return (input != NULL) && isfinite(input->target_position_rad) &&
           isfinite(input->target_velocity_rad_per_s) && isfinite(input->measured_position_rad) &&
           isfinite(input->measured_velocity_rad_per_s) && isfinite(input->actuator_feedforward) &&
           isfinite(input->delta_time_s) && (input->delta_time_s > 0.0F);
}

static alg_axis_controller_status_t
alg_axis_pid_reset_virtual(alg_axis_controller_t *controller_base, float measured_position_rad,
                           float measured_velocity_rad_per_s, float initial_output)
{
    alg_axis_pid_t *const me =
        ALG_AXIS_CONTROLLER_CONTAINER_OF(controller_base, alg_axis_pid_t, super);
    const alg_pid_status_t status = alg_pid_cascade_reset(
        &me->cascade, measured_position_rad, measured_velocity_rad_per_s, initial_output);
    return (status == ALG_PID_STATUS_OK) ? ALG_AXIS_CONTROLLER_STATUS_OK
                                         : ALG_AXIS_CONTROLLER_STATUS_ALGORITHM_ERROR;
}

static alg_axis_controller_status_t
alg_axis_pid_update_virtual(alg_axis_controller_t *controller_base,
                            const alg_axis_controller_input_t *input, float *control_output)
{
    alg_axis_pid_t *const me =
        ALG_AXIS_CONTROLLER_CONTAINER_OF(controller_base, alg_axis_pid_t, super);
    const alg_pid_cascade_input_t cascade_input = {
        .position_setpoint = input->target_position_rad,
        .position_measurement = input->measured_position_rad,
        .velocity_measurement = input->measured_velocity_rad_per_s,
        .velocity_feedforward = input->target_velocity_rad_per_s,
        .actuator_feedforward = input->actuator_feedforward,
        .delta_time_s = input->delta_time_s,
    };
    const alg_pid_status_t status =
        alg_pid_cascade_update(&me->cascade, &cascade_input, control_output);
    return (status == ALG_PID_STATUS_OK) ? ALG_AXIS_CONTROLLER_STATUS_OK
                                         : ALG_AXIS_CONTROLLER_STATUS_ALGORITHM_ERROR;
}

static alg_axis_controller_status_t
alg_axis_lqr_reset_virtual(alg_axis_controller_t *controller_base, float measured_position_rad,
                           float measured_velocity_rad_per_s, float initial_output)
{
    (void)controller_base;
    return (isfinite(measured_position_rad) && isfinite(measured_velocity_rad_per_s) &&
            isfinite(initial_output))
               ? ALG_AXIS_CONTROLLER_STATUS_OK
               : ALG_AXIS_CONTROLLER_STATUS_INVALID_ARGUMENT;
}

static alg_axis_controller_status_t
alg_axis_lqr_update_virtual(alg_axis_controller_t *controller_base,
                            const alg_axis_controller_input_t *input, float *control_output)
{
    alg_axis_lqr_t *const me =
        ALG_AXIS_CONTROLLER_CONTAINER_OF(controller_base, alg_axis_lqr_t, super);
    const float state[2] = {input->measured_position_rad, input->measured_velocity_rad_per_s};
    const float reference_state[2] = {input->target_position_rad, input->target_velocity_rad_per_s};
    const float feedforward_control[1] = {input->actuator_feedforward};
    const alg_lqr_status_t status =
        alg_lqr_controller_update(&me->controller, state, reference_state, &me->equilibrium_control,
                                  feedforward_control, control_output);
    return (status == ALG_LQR_STATUS_OK) ? ALG_AXIS_CONTROLLER_STATUS_OK
                                         : ALG_AXIS_CONTROLLER_STATUS_ALGORITHM_ERROR;
}

static const alg_axis_controller_ops_t s_alg_axis_pid_ops = {
    .reset = alg_axis_pid_reset_virtual,
    .update = alg_axis_pid_update_virtual,
};

static const alg_axis_controller_ops_t s_alg_axis_lqr_ops = {
    .reset = alg_axis_lqr_reset_virtual,
    .update = alg_axis_lqr_update_virtual,
};

alg_axis_controller_status_t alg_axis_pid_init(alg_axis_pid_t *me,
                                               const alg_axis_pid_config_t *config)
{
    if ((me == NULL) || (config == NULL))
    {
        return ALG_AXIS_CONTROLLER_STATUS_INVALID_ARGUMENT;
    }
    me->super.vptr = NULL;
    me->super.is_initialized = false;
    if (alg_pid_cascade_init(&me->cascade, &config->cascade_config) != ALG_PID_STATUS_OK)
    {
        return ALG_AXIS_CONTROLLER_STATUS_ALGORITHM_ERROR;
    }
    me->super.vptr = &s_alg_axis_pid_ops;
    me->super.is_initialized = true;
    return ALG_AXIS_CONTROLLER_STATUS_OK;
}

alg_axis_controller_status_t alg_axis_lqr_init(alg_axis_lqr_t *me,
                                               const alg_axis_lqr_config_t *config)
{
    alg_lqr_controller_config_t controller_config;

    if ((me == NULL) || (config == NULL) || (config->gain_matrix == NULL) ||
        !isfinite(config->gain_matrix[0]) || !isfinite(config->gain_matrix[1]) ||
        !isfinite(config->control_min) || !isfinite(config->control_max) ||
        !isfinite(config->equilibrium_control) || (config->control_min >= config->control_max))
    {
        return ALG_AXIS_CONTROLLER_STATUS_INVALID_ARGUMENT;
    }
    me->super.vptr = NULL;
    me->super.is_initialized = false;
    me->gain_matrix[0] = config->gain_matrix[0];
    me->gain_matrix[1] = config->gain_matrix[1];
    me->control_min = config->control_min;
    me->control_max = config->control_max;
    me->equilibrium_control = config->equilibrium_control;
    controller_config = (alg_lqr_controller_config_t){
        .state_dimension = 2U,
        .control_dimension = 1U,
        .gain_matrix = me->gain_matrix,
        .control_min = &me->control_min,
        .control_max = &me->control_max,
    };
    if (alg_lqr_controller_init(&me->controller, &controller_config) != ALG_LQR_STATUS_OK)
    {
        return ALG_AXIS_CONTROLLER_STATUS_ALGORITHM_ERROR;
    }
    me->super.vptr = &s_alg_axis_lqr_ops;
    me->super.is_initialized = true;
    return ALG_AXIS_CONTROLLER_STATUS_OK;
}

alg_axis_controller_t *alg_axis_pid_as_controller(alg_axis_pid_t *me)
{
    return (me != NULL) ? &me->super : NULL;
}

alg_axis_controller_t *alg_axis_lqr_as_controller(alg_axis_lqr_t *me)
{
    return (me != NULL) ? &me->super : NULL;
}

alg_axis_controller_status_t alg_axis_controller_reset(alg_axis_controller_t *me,
                                                       float measured_position_rad,
                                                       float measured_velocity_rad_per_s,
                                                       float initial_output)
{
    if ((me == NULL) || !me->is_initialized || (me->vptr == NULL) || (me->vptr->reset == NULL))
    {
        return (me == NULL) ? ALG_AXIS_CONTROLLER_STATUS_INVALID_ARGUMENT
                            : ALG_AXIS_CONTROLLER_STATUS_NOT_INITIALIZED;
    }
    return me->vptr->reset(me, measured_position_rad, measured_velocity_rad_per_s, initial_output);
}

alg_axis_controller_status_t alg_axis_controller_update(alg_axis_controller_t *me,
                                                        const alg_axis_controller_input_t *input,
                                                        float *control_output)
{
    if ((me == NULL) || (control_output == NULL) || !alg_axis_controller_input_is_valid(input))
    {
        return ALG_AXIS_CONTROLLER_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized || (me->vptr == NULL) || (me->vptr->update == NULL))
    {
        return ALG_AXIS_CONTROLLER_STATUS_NOT_INITIALIZED;
    }
    return me->vptr->update(me, input, control_output);
}
