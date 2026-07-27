#include "module_swerve.h"

#include <math.h>
#include <stddef.h>

static module_swerve_status_t module_swerve_read_steering_angle(const module_swerve_t *me,
                                                                float *steering_angle_rad)
{
    const module_motor_feedback_t *const steering_feedback =
        module_motor_get_feedback(me->steering_motor);

    if ((steering_feedback == NULL) || !steering_feedback->is_online)
    {
        return MODULE_SWERVE_STATUS_NOT_READY;
    }
    *steering_angle_rad = (steering_feedback->position_rad * me->steering_direction_sign) -
                          me->steering_zero_offset_rad;
    return MODULE_SWERVE_STATUS_OK;
}

module_swerve_status_t module_swerve_init(module_swerve_t *me, const module_swerve_config_t *config)
{
    if ((me == NULL) || (config == NULL) || (config->drive_motor == NULL) ||
        (config->steering_motor == NULL) || !config->drive_motor->is_initialized ||
        !config->steering_motor->is_initialized || !isfinite(config->wheel_radius_m) ||
        (config->wheel_radius_m <= 0.0F) || !isfinite(config->drive_reduction_ratio) ||
        (config->drive_reduction_ratio <= 0.0F) || !isfinite(config->steering_zero_offset_rad) ||
        ((config->drive_direction_sign != 1.0F) && (config->drive_direction_sign != -1.0F)) ||
        ((config->steering_direction_sign != 1.0F) && (config->steering_direction_sign != -1.0F)))
    {
        return MODULE_SWERVE_STATUS_INVALID_ARGUMENT;
    }
    *me = (module_swerve_t){
        .drive_motor = config->drive_motor,
        .steering_motor = config->steering_motor,
        .wheel_radius_m = config->wheel_radius_m,
        .drive_reduction_ratio = config->drive_reduction_ratio,
        .steering_zero_offset_rad = config->steering_zero_offset_rad,
        .drive_direction_sign = config->drive_direction_sign,
        .steering_direction_sign = config->steering_direction_sign,
        .is_initialized = true,
    };
    return MODULE_SWERVE_STATUS_OK;
}

module_swerve_status_t module_swerve_enable(module_swerve_t *me)
{
    if (me == NULL)
    {
        return MODULE_SWERVE_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return MODULE_SWERVE_STATUS_NOT_INITIALIZED;
    }
    if (module_motor_enable(me->drive_motor) != MODULE_MOTOR_STATUS_OK)
    {
        return MODULE_SWERVE_STATUS_MOTOR_ERROR;
    }
    if (module_motor_enable(me->steering_motor) != MODULE_MOTOR_STATUS_OK)
    {
        (void)module_motor_disable(me->drive_motor);
        return MODULE_SWERVE_STATUS_MOTOR_ERROR;
    }
    me->is_enabled = true;
    return MODULE_SWERVE_STATUS_OK;
}

module_swerve_status_t module_swerve_disable(module_swerve_t *me)
{
    module_motor_status_t drive_status;
    module_motor_status_t steering_status;

    if (me == NULL)
    {
        return MODULE_SWERVE_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return MODULE_SWERVE_STATUS_NOT_INITIALIZED;
    }
    drive_status = module_motor_disable(me->drive_motor);
    steering_status = module_motor_disable(me->steering_motor);
    me->is_enabled = false;
    return ((drive_status == MODULE_MOTOR_STATUS_OK) && (steering_status == MODULE_MOTOR_STATUS_OK))
               ? MODULE_SWERVE_STATUS_OK
               : MODULE_SWERVE_STATUS_MOTOR_ERROR;
}

module_swerve_status_t module_swerve_apply_target(module_swerve_t *me,
                                                  const alg_swerve_module_target_t *target,
                                                  float delta_time_s)
{
    alg_swerve_module_target_t optimized_target;
    float current_steering_angle_rad;
    float drive_velocity_rad_per_s;
    float steering_target_rad;
    module_swerve_status_t status;

    if ((me == NULL) || (target == NULL) || !isfinite(target->wheel_velocity_m_per_s) ||
        !isfinite(target->steering_angle_rad) || !isfinite(delta_time_s) || (delta_time_s <= 0.0F))
    {
        return MODULE_SWERVE_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return MODULE_SWERVE_STATUS_NOT_INITIALIZED;
    }
    if (!me->is_enabled)
    {
        return MODULE_SWERVE_STATUS_NOT_READY;
    }
    status = module_swerve_read_steering_angle(me, &current_steering_angle_rad);
    if (status != MODULE_SWERVE_STATUS_OK)
    {
        return status;
    }
    optimized_target = *target;
    if (alg_swerve_optimize_target(current_steering_angle_rad, &optimized_target) !=
        ALG_SWERVE_STATUS_OK)
    {
        return MODULE_SWERVE_STATUS_ALGORITHM_ERROR;
    }
    drive_velocity_rad_per_s = optimized_target.wheel_velocity_m_per_s / me->wheel_radius_m *
                               me->drive_reduction_ratio * me->drive_direction_sign;
    steering_target_rad = (optimized_target.steering_angle_rad + me->steering_zero_offset_rad) *
                          me->steering_direction_sign;
    if ((module_motor_set_target(me->drive_motor, drive_velocity_rad_per_s) !=
         MODULE_MOTOR_STATUS_OK) ||
        (module_motor_set_target(me->steering_motor, steering_target_rad) !=
         MODULE_MOTOR_STATUS_OK) ||
        (module_motor_update(me->drive_motor, delta_time_s) != MODULE_MOTOR_STATUS_OK) ||
        (module_motor_update(me->steering_motor, delta_time_s) != MODULE_MOTOR_STATUS_OK))
    {
        return MODULE_SWERVE_STATUS_MOTOR_ERROR;
    }
    return MODULE_SWERVE_STATUS_OK;
}

module_swerve_status_t module_swerve_get_steering_angle(const module_swerve_t *me,
                                                        float *steering_angle_rad)
{
    if ((me == NULL) || (steering_angle_rad == NULL))
    {
        return MODULE_SWERVE_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return MODULE_SWERVE_STATUS_NOT_INITIALIZED;
    }
    return module_swerve_read_steering_angle(me, steering_angle_rad);
}
