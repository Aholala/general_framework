#include "module_shooter.h"

#include <math.h>
#include <stddef.h>
#include <stdlib.h>

static bool module_shooter_direction_is_valid(float direction_sign)
{
    return (direction_sign == 1.0F) || (direction_sign == -1.0F);
}

static bool module_shooter_jam_current_detected(
    const module_shooter_t *me,
    const module_motor_feedback_t *feeder_feedback)
{
    if (feeder_feedback->is_current_a_valid)
    {
        return fabsf(feeder_feedback->current_a) >=
               me->jam_current_threshold_a;
    }
    return abs(feeder_feedback->current_raw) >=
           abs(me->jam_current_threshold_raw);
}

static module_shooter_status_t module_shooter_set_motor_targets(
    module_shooter_t *me)
{
    const float left_velocity_rad_per_s =
        me->friction_enabled
            ? me->friction_target_velocity_rad_per_s *
                  me->left_friction_direction_sign
            : 0.0F;
    const float right_velocity_rad_per_s =
        me->friction_enabled
            ? me->friction_target_velocity_rad_per_s *
                  me->right_friction_direction_sign
            : 0.0F;

    if ((module_motor_set_target(me->left_friction_motor,
                                 left_velocity_rad_per_s) !=
         MODULE_MOTOR_STATUS_OK) ||
        (module_motor_set_target(me->right_friction_motor,
                                 right_velocity_rad_per_s) !=
         MODULE_MOTOR_STATUS_OK) ||
        (module_motor_set_target(me->feeder_motor,
                                 me->feeder_target_position_rad) !=
         MODULE_MOTOR_STATUS_OK))
    {
        return MODULE_SHOOTER_STATUS_MOTOR_ERROR;
    }
    return MODULE_SHOOTER_STATUS_OK;
}

static module_shooter_status_t module_shooter_update_motors(
    module_shooter_t *me, float delta_time_s)
{
    if ((module_motor_update(me->left_friction_motor, delta_time_s) !=
         MODULE_MOTOR_STATUS_OK) ||
        (module_motor_update(me->right_friction_motor, delta_time_s) !=
         MODULE_MOTOR_STATUS_OK) ||
        (module_motor_update(me->feeder_motor, delta_time_s) !=
         MODULE_MOTOR_STATUS_OK))
    {
        return MODULE_SHOOTER_STATUS_MOTOR_ERROR;
    }
    return MODULE_SHOOTER_STATUS_OK;
}

module_shooter_status_t module_shooter_init(
    module_shooter_t *me, const module_shooter_config_t *config)
{
    if ((me == NULL) || (config == NULL) ||
        (config->left_friction_motor == NULL) ||
        (config->right_friction_motor == NULL) ||
        (config->feeder_motor == NULL) ||
        !config->left_friction_motor->is_initialized ||
        !config->right_friction_motor->is_initialized ||
        !config->feeder_motor->is_initialized ||
        !module_shooter_direction_is_valid(
            config->left_friction_direction_sign) ||
        !module_shooter_direction_is_valid(
            config->right_friction_direction_sign) ||
        !module_shooter_direction_is_valid(
            config->feeder_direction_sign) ||
        !isfinite(config->feeder_step_rad) ||
        (config->feeder_step_rad <= 0.0F) ||
        !isfinite(config->feeder_position_tolerance_rad) ||
        (config->feeder_position_tolerance_rad < 0.0F) ||
        !isfinite(config->jam_velocity_threshold_rad_per_s) ||
        (config->jam_velocity_threshold_rad_per_s < 0.0F) ||
        !isfinite(config->jam_current_threshold_a) ||
        (config->jam_current_threshold_a < 0.0F) ||
        !isfinite(config->jam_confirmation_time_s) ||
        (config->jam_confirmation_time_s <= 0.0F) ||
        !isfinite(config->rollback_angle_rad) ||
        (config->rollback_angle_rad <= 0.0F) ||
        !isfinite(config->rollback_position_tolerance_rad) ||
        (config->rollback_position_tolerance_rad < 0.0F) ||
        (config->maximum_jam_retries == 0U) ||
        (config->maximum_pending_shots == 0U))
    {
        return MODULE_SHOOTER_STATUS_INVALID_ARGUMENT;
    }
    *me = (module_shooter_t){
        .left_friction_motor = config->left_friction_motor,
        .right_friction_motor = config->right_friction_motor,
        .feeder_motor = config->feeder_motor,
        .left_friction_direction_sign =
            config->left_friction_direction_sign,
        .right_friction_direction_sign =
            config->right_friction_direction_sign,
        .feeder_direction_sign = config->feeder_direction_sign,
        .feeder_step_rad = config->feeder_step_rad,
        .feeder_position_tolerance_rad =
            config->feeder_position_tolerance_rad,
        .jam_velocity_threshold_rad_per_s =
            config->jam_velocity_threshold_rad_per_s,
        .jam_current_threshold_a = config->jam_current_threshold_a,
        .jam_current_threshold_raw =
            config->jam_current_threshold_raw,
        .jam_confirmation_time_s =
            config->jam_confirmation_time_s,
        .rollback_angle_rad = config->rollback_angle_rad,
        .rollback_position_tolerance_rad =
            config->rollback_position_tolerance_rad,
        .maximum_pending_shots = config->maximum_pending_shots,
        .maximum_jam_retries = config->maximum_jam_retries,
        .state = MODULE_SHOOTER_STATE_DISABLED,
        .is_initialized = true,
    };
    return MODULE_SHOOTER_STATUS_OK;
}

module_shooter_status_t module_shooter_enable(module_shooter_t *me)
{
    const module_motor_feedback_t *feeder_feedback;

    if (me == NULL)
    {
        return MODULE_SHOOTER_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return MODULE_SHOOTER_STATUS_NOT_INITIALIZED;
    }
    if ((module_motor_enable(me->left_friction_motor) !=
         MODULE_MOTOR_STATUS_OK) ||
        (module_motor_enable(me->right_friction_motor) !=
         MODULE_MOTOR_STATUS_OK) ||
        (module_motor_enable(me->feeder_motor) !=
         MODULE_MOTOR_STATUS_OK))
    {
        (void)module_shooter_disable(me);
        return MODULE_SHOOTER_STATUS_MOTOR_ERROR;
    }
    feeder_feedback = module_motor_get_feedback(me->feeder_motor);
    if ((feeder_feedback == NULL) || !feeder_feedback->is_online)
    {
        (void)module_shooter_disable(me);
        return MODULE_SHOOTER_STATUS_NOT_READY;
    }
    me->feeder_target_position_rad = feeder_feedback->position_rad;
    me->state = MODULE_SHOOTER_STATE_READY;
    return MODULE_SHOOTER_STATUS_OK;
}

module_shooter_status_t module_shooter_disable(module_shooter_t *me)
{
    module_shooter_status_t status = MODULE_SHOOTER_STATUS_OK;

    if (me == NULL)
    {
        return MODULE_SHOOTER_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return MODULE_SHOOTER_STATUS_NOT_INITIALIZED;
    }
    if ((module_motor_disable(me->left_friction_motor) !=
         MODULE_MOTOR_STATUS_OK) ||
        (module_motor_disable(me->right_friction_motor) !=
         MODULE_MOTOR_STATUS_OK) ||
        (module_motor_disable(me->feeder_motor) != MODULE_MOTOR_STATUS_OK))
    {
        status = MODULE_SHOOTER_STATUS_MOTOR_ERROR;
    }
    me->pending_shots = 0U;
    me->friction_enabled = false;
    me->state = MODULE_SHOOTER_STATE_DISABLED;
    return status;
}

module_shooter_status_t module_shooter_set_friction(
    module_shooter_t *me, bool is_enabled,
    float target_velocity_rad_per_s)
{
    if ((me == NULL) || !isfinite(target_velocity_rad_per_s) ||
        (target_velocity_rad_per_s < 0.0F))
    {
        return MODULE_SHOOTER_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return MODULE_SHOOTER_STATUS_NOT_INITIALIZED;
    }
    me->friction_enabled = is_enabled;
    me->friction_target_velocity_rad_per_s =
        target_velocity_rad_per_s;
    return MODULE_SHOOTER_STATUS_OK;
}

module_shooter_status_t module_shooter_request_shots(
    module_shooter_t *me, uint16_t shot_count)
{
    if ((me == NULL) || (shot_count == 0U))
    {
        return MODULE_SHOOTER_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return MODULE_SHOOTER_STATUS_NOT_INITIALIZED;
    }
    if (me->state == MODULE_SHOOTER_STATE_FAULT)
    {
        return MODULE_SHOOTER_STATUS_FAULT;
    }
    if ((uint32_t)me->pending_shots + shot_count >
        me->maximum_pending_shots)
    {
        return MODULE_SHOOTER_STATUS_INVALID_ARGUMENT;
    }
    me->pending_shots = (uint16_t)(me->pending_shots + shot_count);
    return MODULE_SHOOTER_STATUS_OK;
}

module_shooter_status_t module_shooter_cancel_shots(
    module_shooter_t *me)
{
    if (me == NULL)
    {
        return MODULE_SHOOTER_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return MODULE_SHOOTER_STATUS_NOT_INITIALIZED;
    }
    me->pending_shots = 0U;
    return MODULE_SHOOTER_STATUS_OK;
}

module_shooter_status_t module_shooter_reset_fault(
    module_shooter_t *me)
{
    const module_motor_feedback_t *feeder_feedback;

    if (me == NULL)
    {
        return MODULE_SHOOTER_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return MODULE_SHOOTER_STATUS_NOT_INITIALIZED;
    }
    if (me->state != MODULE_SHOOTER_STATE_FAULT)
    {
        return MODULE_SHOOTER_STATUS_OK;
    }
    feeder_feedback = module_motor_get_feedback(me->feeder_motor);
    if ((feeder_feedback == NULL) || !feeder_feedback->is_online)
    {
        return MODULE_SHOOTER_STATUS_NOT_READY;
    }
    me->feeder_target_position_rad = feeder_feedback->position_rad;
    me->jam_retry_count = 0U;
    me->jam_elapsed_time_s = 0.0F;
    me->state = MODULE_SHOOTER_STATE_READY;
    return MODULE_SHOOTER_STATUS_OK;
}

module_shooter_status_t module_shooter_update(module_shooter_t *me,
                                               float delta_time_s)
{
    const module_motor_feedback_t *feeder_feedback;
    float position_error_rad;
    module_shooter_status_t status;

    if ((me == NULL) || !isfinite(delta_time_s) || (delta_time_s <= 0.0F))
    {
        return MODULE_SHOOTER_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return MODULE_SHOOTER_STATUS_NOT_INITIALIZED;
    }
    if (me->state == MODULE_SHOOTER_STATE_DISABLED)
    {
        return MODULE_SHOOTER_STATUS_NOT_READY;
    }
    if (me->state == MODULE_SHOOTER_STATE_FAULT)
    {
        return MODULE_SHOOTER_STATUS_FAULT;
    }
    feeder_feedback = module_motor_get_feedback(me->feeder_motor);
    if ((feeder_feedback == NULL) || !feeder_feedback->is_online)
    {
        return MODULE_SHOOTER_STATUS_NOT_READY;
    }

    position_error_rad =
        me->feeder_target_position_rad - feeder_feedback->position_rad;
    if ((me->state == MODULE_SHOOTER_STATE_READY) &&
        (me->pending_shots > 0U))
    {
        me->feeder_target_position_rad +=
            me->feeder_direction_sign * me->feeder_step_rad;
        me->jam_elapsed_time_s = 0.0F;
        me->state = MODULE_SHOOTER_STATE_FEEDING;
    }
    else if (me->state == MODULE_SHOOTER_STATE_FEEDING)
    {
        if (fabsf(position_error_rad) <=
            me->feeder_position_tolerance_rad)
        {
            --me->pending_shots;
            me->jam_retry_count = 0U;
            me->jam_elapsed_time_s = 0.0F;
            me->state = MODULE_SHOOTER_STATE_READY;
        }
        else if ((fabsf(feeder_feedback->velocity_rad_per_s) <=
                  me->jam_velocity_threshold_rad_per_s) &&
                 module_shooter_jam_current_detected(me, feeder_feedback))
        {
            me->jam_elapsed_time_s += delta_time_s;
            if (me->jam_elapsed_time_s >= me->jam_confirmation_time_s)
            {
                ++me->jam_retry_count;
                if (me->jam_retry_count > me->maximum_jam_retries)
                {
                    me->state = MODULE_SHOOTER_STATE_FAULT;
                    return MODULE_SHOOTER_STATUS_FAULT;
                }
                me->feeder_target_position_rad =
                    feeder_feedback->position_rad -
                    (me->feeder_direction_sign * me->rollback_angle_rad);
                me->jam_elapsed_time_s = 0.0F;
                me->state = MODULE_SHOOTER_STATE_ROLLBACK;
            }
        }
        else
        {
            me->jam_elapsed_time_s = 0.0F;
        }
    }
    else if ((me->state == MODULE_SHOOTER_STATE_ROLLBACK) &&
             (fabsf(position_error_rad) <=
              me->rollback_position_tolerance_rad))
    {
        me->feeder_target_position_rad =
            feeder_feedback->position_rad +
            (me->feeder_direction_sign * me->feeder_step_rad);
        me->jam_elapsed_time_s = 0.0F;
        me->state = MODULE_SHOOTER_STATE_FEEDING;
    }

    status = module_shooter_set_motor_targets(me);
    if (status != MODULE_SHOOTER_STATUS_OK)
    {
        return status;
    }
    return module_shooter_update_motors(me, delta_time_s);
}

module_shooter_state_t module_shooter_get_state(
    const module_shooter_t *me)
{
    return ((me != NULL) && me->is_initialized)
               ? me->state
               : MODULE_SHOOTER_STATE_DISABLED;
}

uint16_t module_shooter_get_pending_shots(
    const module_shooter_t *me)
{
    return ((me != NULL) && me->is_initialized)
               ? me->pending_shots
               : 0U;
}

uint8_t module_shooter_get_jam_retry_count(
    const module_shooter_t *me)
{
    return ((me != NULL) && me->is_initialized)
               ? me->jam_retry_count
               : 0U;
}
