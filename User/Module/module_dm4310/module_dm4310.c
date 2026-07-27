#include "module_dm4310.h"

static module_dm_control_mode_t
module_dm4310_map_control_mode(module_dm4310_control_mode_t control_mode)
{
    static const module_dm_control_mode_t control_mode_map[] = {
        MODULE_DM_MODE_MIT,
        MODULE_DM_MODE_VELOCITY,
        MODULE_DM_MODE_POSITION_VELOCITY,
    };
    return control_mode_map[control_mode];
}

module_motor_status_t module_dm4310_init(module_dm4310_t *const me,
                                         const module_dm4310_config_t *const config)
{
    module_dm_motor_config_t dm_motor_config;

    if ((me == NULL) || (config == NULL) ||
        (config->control_mode > MODULE_DM4310_CONTROL_POSITION_VELOCITY))
    {
        return MODULE_MOTOR_STATUS_INVALID_ARGUMENT;
    }

    dm_motor_config = (module_dm_motor_config_t){
        .logical_name = config->logical_name,
        .registration_key = config->registration_key,
        .can = config->can,
        .control_mode = module_dm4310_map_control_mode(config->control_mode),
        .master_identifier = config->base_command_identifier,
        .feedback_identifier = config->feedback_identifier,
        .transmit_timeout_ms = config->transmit_timeout_ms,
        .limits = config->protocol_limits,
    };
    return module_dm_motor_init(&me->super, &dm_motor_config);
}

module_motor_status_t module_dm4310_register(module_dm4310_t *const me,
                                             module_motor_registry_t *const registry)
{
    return (me != NULL) ? module_dm_motor_register(&me->super, registry)
                        : MODULE_MOTOR_STATUS_INVALID_ARGUMENT;
}

module_motor_status_t module_dm4310_unregister(module_dm4310_t *const me,
                                               module_motor_registry_t *const registry)
{
    return (me != NULL) ? module_dm_motor_unregister(&me->super, registry)
                        : MODULE_MOTOR_STATUS_INVALID_ARGUMENT;
}

module_motor_t *module_dm4310_as_motor(module_dm4310_t *const me)
{
    return (me != NULL) ? module_dm_motor_as_base(&me->super) : NULL;
}

module_dm_motor_t *module_dm4310_as_dm_motor(module_dm4310_t *const me)
{
    return (me != NULL) ? &me->super : NULL;
}

module_motor_status_t module_dm4310_enable(module_dm4310_t *const me)
{
    return module_motor_enable(module_dm4310_as_motor(me));
}

module_motor_status_t module_dm4310_disable(module_dm4310_t *const me)
{
    return module_motor_disable(module_dm4310_as_motor(me));
}

module_motor_status_t module_dm4310_command_mit(module_dm4310_t *const me,
                                                const module_dm_mit_command_t *const command)
{
    return (me != NULL) ? module_dm_motor_command_mit(&me->super, command)
                        : MODULE_MOTOR_STATUS_INVALID_ARGUMENT;
}

module_motor_status_t module_dm4310_command_velocity(module_dm4310_t *const me,
                                                     float velocity_rad_per_s)
{
    return (me != NULL) ? module_dm_motor_command_velocity(&me->super, velocity_rad_per_s)
                        : MODULE_MOTOR_STATUS_INVALID_ARGUMENT;
}

module_motor_status_t module_dm4310_command_position_velocity(module_dm4310_t *const me,
                                                              float position_rad,
                                                              float velocity_rad_per_s)
{
    return (me != NULL) ? module_dm_motor_command_position_velocity(&me->super, position_rad,
                                                                    velocity_rad_per_s)
                        : MODULE_MOTOR_STATUS_INVALID_ARGUMENT;
}

module_motor_status_t module_dm4310_save_zero_position(module_dm4310_t *const me)
{
    if (me == NULL)
    {
        return MODULE_MOTOR_STATUS_INVALID_ARGUMENT;
    }
    if (me->super.super.state != MODULE_MOTOR_STATE_DISABLED)
    {
        return MODULE_MOTOR_STATUS_UNSUPPORTED;
    }
    return module_dm_motor_send_state_command(&me->super, MODULE_DM_COMMAND_SAVE_ZERO);
}

module_motor_status_t module_dm4310_clear_fault(module_dm4310_t *const me)
{
    return (me != NULL)
               ? module_dm_motor_send_state_command(&me->super, MODULE_DM_COMMAND_CLEAR_FAULT)
               : MODULE_MOTOR_STATUS_INVALID_ARGUMENT;
}

module_motor_status_t module_dm4310_handle_feedback(module_dm4310_t *const me,
                                                    const bsp_can_frame_t *const frame)
{
    return (me != NULL) ? module_dm_motor_handle_feedback(&me->super, frame)
                        : MODULE_MOTOR_STATUS_INVALID_ARGUMENT;
}

const module_motor_feedback_t *module_dm4310_get_feedback(const module_dm4310_t *const me)
{
    return (me != NULL) ? module_motor_get_feedback(&me->super.super) : NULL;
}

module_dm_fault_t module_dm4310_get_fault(const module_dm4310_t *const me)
{
    return (me != NULL) ? module_dm_motor_get_fault(&me->super)
                        : MODULE_DM_FAULT_COMMUNICATION_LOST;
}

float module_dm4310_get_mos_temperature_c(const module_dm4310_t *const me)
{
    return (me != NULL) ? module_dm_motor_get_mos_temperature_c(&me->super) : 0.0F;
}
