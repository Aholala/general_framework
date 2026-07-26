#include "module_m3508.h"

static module_dji_control_mode_t
module_m3508_map_control_mode(module_m3508_control_mode_t control_mode)
{
    static const module_dji_control_mode_t control_mode_map[] = {
        MODULE_DJI_CONTROL_DIRECT,
        MODULE_DJI_CONTROL_VELOCITY,
        MODULE_DJI_CONTROL_POSITION,
    };
    return control_mode_map[control_mode];
}

static module_motor_status_t
module_m3508_validate_control_mode(const module_m3508_t *const me,
                                   module_m3508_control_mode_t expected_control_mode)
{
    if (me == NULL)
    {
        return MODULE_MOTOR_STATUS_INVALID_ARGUMENT;
    }
    if (!me->super.super.is_initialized)
    {
        return MODULE_MOTOR_STATUS_NOT_INITIALIZED;
    }
    if (me->super.control_mode != module_m3508_map_control_mode(expected_control_mode))
    {
        return MODULE_MOTOR_STATUS_UNSUPPORTED;
    }
    return MODULE_MOTOR_STATUS_OK;
}

module_motor_status_t module_m3508_init(module_m3508_t *const me,
                                        const module_m3508_config_t *const config)
{
    module_dji_motor_config_t dji_motor_config;

    if ((me == NULL) || (config == NULL) || (config->control_mode > MODULE_M3508_CONTROL_POSITION))
    {
        return MODULE_MOTOR_STATUS_INVALID_ARGUMENT;
    }
    dji_motor_config = (module_dji_motor_config_t){
        .logical_name = config->logical_name,
        .registration_key = config->registration_key,
        .motor_bus = config->motor_bus,
        .motor_model = MODULE_DJI_MOTOR_M3508,
        .control_mode = module_m3508_map_control_mode(config->control_mode),
        .motor_identifier = config->motor_identifier,
        .direction_sign = config->direction_sign,
        .maximum_temperature_c = config->maximum_temperature_c,
        .current_scale_a_per_count = config->current_scale_a_per_count,
        .velocity_pid_config = config->velocity_pid_config,
        .position_pid_config = config->position_pid_config,
    };
    return module_dji_motor_init(&me->super, &dji_motor_config);
}

module_motor_status_t module_m3508_register(module_m3508_t *const me,
                                            module_motor_registry_t *const registry)
{
    return (me != NULL) ? module_dji_motor_register(&me->super, registry)
                        : MODULE_MOTOR_STATUS_INVALID_ARGUMENT;
}

module_motor_status_t module_m3508_unregister(
    module_m3508_t *const me,
    module_motor_registry_t *const registry)
{
    return (me != NULL)
               ? module_dji_motor_unregister(&me->super, registry)
               : MODULE_MOTOR_STATUS_INVALID_ARGUMENT;
}

module_motor_t *module_m3508_as_motor(module_m3508_t *const me)
{
    return (me != NULL) ? module_dji_motor_as_base(&me->super) : NULL;
}

module_dji_motor_t *module_m3508_as_dji_motor(module_m3508_t *const me)
{
    return (me != NULL) ? &me->super : NULL;
}

module_motor_status_t module_m3508_enable(module_m3508_t *const me)
{
    return module_motor_enable(module_m3508_as_motor(me));
}

module_motor_status_t module_m3508_disable(module_m3508_t *const me)
{
    return module_motor_disable(module_m3508_as_motor(me));
}

module_motor_status_t module_m3508_set_current_command_raw(module_m3508_t *const me,
                                                           int16_t current_command_raw)
{
    module_motor_status_t status =
        module_m3508_validate_control_mode(me, MODULE_M3508_CONTROL_CURRENT);
    if (status != MODULE_MOTOR_STATUS_OK)
    {
        return status;
    }
    return module_motor_set_target(module_m3508_as_motor(me), (float)current_command_raw);
}

module_motor_status_t module_m3508_set_velocity_rad_per_s(module_m3508_t *const me,
                                                          float velocity_rad_per_s)
{
    module_motor_status_t status =
        module_m3508_validate_control_mode(me, MODULE_M3508_CONTROL_VELOCITY);
    if (status != MODULE_MOTOR_STATUS_OK)
    {
        return status;
    }
    return module_motor_set_target(module_m3508_as_motor(me), velocity_rad_per_s);
}

module_motor_status_t module_m3508_set_position_rad(module_m3508_t *const me, float position_rad)
{
    module_motor_status_t status =
        module_m3508_validate_control_mode(me, MODULE_M3508_CONTROL_POSITION);
    if (status != MODULE_MOTOR_STATUS_OK)
    {
        return status;
    }
    return module_motor_set_target(module_m3508_as_motor(me), position_rad);
}

module_motor_status_t module_m3508_update(module_m3508_t *const me, float delta_time_s)
{
    return module_motor_update(module_m3508_as_motor(me), delta_time_s);
}

const module_motor_feedback_t *module_m3508_get_feedback(const module_m3508_t *const me)
{
    return (me != NULL) ? module_motor_get_feedback(&me->super.super) : NULL;
}

int16_t module_m3508_get_current_command_raw(const module_m3508_t *const me)
{
    return (me != NULL) ? module_dji_motor_get_command(&me->super) : 0;
}
