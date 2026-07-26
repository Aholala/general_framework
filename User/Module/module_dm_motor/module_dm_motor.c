#include "module_dm_motor.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

static module_dm_motor_t *module_dm_motor_get_device(module_motor_t *const motor_base)
{
    return MODULE_MOTOR_CONTAINER_OF(motor_base, module_dm_motor_t, super);
}

static bool module_dm_motor_is_within(float value, float minimum, float maximum)
{
    return isfinite(value) && (value >= minimum) && (value <= maximum);
}

static bool module_dm_motor_are_limits_valid(const module_dm_limits_t *const limits)
{
    const float values[] = {
        limits->position_min_rad,       limits->position_max_rad,
        limits->velocity_min_rad_per_s, limits->velocity_max_rad_per_s,
        limits->torque_min_nm,          limits->torque_max_nm,
        limits->proportional_gain_min,  limits->proportional_gain_max,
        limits->derivative_gain_min,    limits->derivative_gain_max,
    };
    size_t value_index;

    for (value_index = 0U; value_index < (sizeof(values) / sizeof(values[0])); ++value_index)
    {
        if (!isfinite(values[value_index]))
        {
            return false;
        }
    }
    return (limits->position_min_rad < limits->position_max_rad) &&
           (limits->velocity_min_rad_per_s < limits->velocity_max_rad_per_s) &&
           (limits->torque_min_nm < limits->torque_max_nm) &&
           (limits->proportional_gain_min < limits->proportional_gain_max) &&
           (limits->derivative_gain_min < limits->derivative_gain_max);
}

static bool module_dm_motor_is_identifier_valid(module_dm_control_mode_t control_mode,
                                                uint32_t master_identifier)
{
    const uint32_t identifier_offset =
        (control_mode == MODULE_DM_MODE_VELOCITY)
            ? 0x200U
            : ((control_mode == MODULE_DM_MODE_POSITION_VELOCITY) ? 0x100U : 0U);
    return master_identifier <= (0x7FFU - identifier_offset);
}

static uint32_t module_dm_motor_float_to_unsigned(float value, float minimum, float maximum,
                                                  uint8_t bit_count)
{
    const uint32_t maximum_integer = (1UL << bit_count) - 1UL;
    return (uint32_t)((value - minimum) * (float)maximum_integer / (maximum - minimum) + 0.5F);
}

static float module_dm_motor_unsigned_to_float(uint32_t value, float minimum, float maximum,
                                               uint8_t bit_count)
{
    const uint32_t maximum_integer = (1UL << bit_count) - 1UL;
    return (float)value * (maximum - minimum) / (float)maximum_integer + minimum;
}

static void module_dm_motor_encode_float_little_endian(float value, uint8_t output[4])
{
    uint32_t raw_value;
    (void)memcpy(&raw_value, &value, sizeof(raw_value));
    output[0] = (uint8_t)raw_value;
    output[1] = (uint8_t)(raw_value >> 8U);
    output[2] = (uint8_t)(raw_value >> 16U);
    output[3] = (uint8_t)(raw_value >> 24U);
}

static uint32_t module_dm_motor_get_mit_identifier(const module_dm_motor_t *const me)
{
    return me->master_identifier;
}

static uint32_t module_dm_motor_get_velocity_identifier(const module_dm_motor_t *const me)
{
    return me->master_identifier + 0x200U;
}

static uint32_t module_dm_motor_get_position_velocity_identifier(const module_dm_motor_t *const me)
{
    return me->master_identifier + 0x100U;
}

static module_motor_status_t module_dm_motor_encode_mit(module_dm_motor_t *const me,
                                                        uint8_t transmit_data[8])
{
    uint32_t position_raw;
    uint32_t velocity_raw;
    uint32_t proportional_gain_raw;
    uint32_t derivative_gain_raw;
    uint32_t torque_raw;
    const module_dm_mit_command_t *const command = &me->mit_command;

    if (!module_dm_motor_is_within(command->position_rad, me->limits.position_min_rad,
                                   me->limits.position_max_rad) ||
        !module_dm_motor_is_within(command->velocity_rad_per_s, me->limits.velocity_min_rad_per_s,
                                   me->limits.velocity_max_rad_per_s) ||
        !module_dm_motor_is_within(command->proportional_gain, me->limits.proportional_gain_min,
                                   me->limits.proportional_gain_max) ||
        !module_dm_motor_is_within(command->derivative_gain, me->limits.derivative_gain_min,
                                   me->limits.derivative_gain_max) ||
        !module_dm_motor_is_within(command->torque_nm, me->limits.torque_min_nm,
                                   me->limits.torque_max_nm))
    {
        return MODULE_MOTOR_STATUS_OUT_OF_RANGE;
    }

    position_raw = module_dm_motor_float_to_unsigned(
        command->position_rad, me->limits.position_min_rad, me->limits.position_max_rad, 16U);
    velocity_raw = module_dm_motor_float_to_unsigned(command->velocity_rad_per_s,
                                                     me->limits.velocity_min_rad_per_s,
                                                     me->limits.velocity_max_rad_per_s, 12U);
    proportional_gain_raw = module_dm_motor_float_to_unsigned(
        command->proportional_gain, me->limits.proportional_gain_min,
        me->limits.proportional_gain_max, 12U);
    derivative_gain_raw =
        module_dm_motor_float_to_unsigned(command->derivative_gain, me->limits.derivative_gain_min,
                                          me->limits.derivative_gain_max, 12U);
    torque_raw = module_dm_motor_float_to_unsigned(command->torque_nm, me->limits.torque_min_nm,
                                                   me->limits.torque_max_nm, 12U);

    transmit_data[0] = (uint8_t)(position_raw >> 8U);
    transmit_data[1] = (uint8_t)position_raw;
    transmit_data[2] = (uint8_t)(velocity_raw >> 4U);
    transmit_data[3] = (uint8_t)((velocity_raw << 4U) | (proportional_gain_raw >> 8U));
    transmit_data[4] = (uint8_t)proportional_gain_raw;
    transmit_data[5] = (uint8_t)(derivative_gain_raw >> 4U);
    transmit_data[6] = (uint8_t)((derivative_gain_raw << 4U) | (torque_raw >> 8U));
    transmit_data[7] = (uint8_t)torque_raw;
    return MODULE_MOTOR_STATUS_OK;
}

static module_motor_status_t module_dm_motor_encode_velocity(module_dm_motor_t *const me,
                                                             uint8_t transmit_data[8])
{
    if (!module_dm_motor_is_within(me->target_velocity_rad_per_s, me->limits.velocity_min_rad_per_s,
                                   me->limits.velocity_max_rad_per_s))
    {
        return MODULE_MOTOR_STATUS_OUT_OF_RANGE;
    }
    (void)memset(transmit_data, 0, 8U);
    module_dm_motor_encode_float_little_endian(me->target_velocity_rad_per_s, transmit_data);
    return MODULE_MOTOR_STATUS_OK;
}

static module_motor_status_t module_dm_motor_encode_position_velocity(module_dm_motor_t *const me,
                                                                      uint8_t transmit_data[8])
{
    if (!module_dm_motor_is_within(me->target_position_rad, me->limits.position_min_rad,
                                   me->limits.position_max_rad) ||
        !module_dm_motor_is_within(me->target_velocity_rad_per_s, me->limits.velocity_min_rad_per_s,
                                   me->limits.velocity_max_rad_per_s))
    {
        return MODULE_MOTOR_STATUS_OUT_OF_RANGE;
    }
    module_dm_motor_encode_float_little_endian(me->target_position_rad, &transmit_data[0]);
    module_dm_motor_encode_float_little_endian(me->target_velocity_rad_per_s, &transmit_data[4]);
    return MODULE_MOTOR_STATUS_OK;
}

static const module_dm_mode_ops_t s_module_dm_mit_ops = {
    .encode_command = module_dm_motor_encode_mit,
    .get_transmit_identifier = module_dm_motor_get_mit_identifier};
static const module_dm_mode_ops_t s_module_dm_velocity_ops = {
    .encode_command = module_dm_motor_encode_velocity,
    .get_transmit_identifier = module_dm_motor_get_velocity_identifier};
static const module_dm_mode_ops_t s_module_dm_position_velocity_ops = {
    .encode_command = module_dm_motor_encode_position_velocity,
    .get_transmit_identifier = module_dm_motor_get_position_velocity_identifier};

static module_motor_status_t module_dm_motor_transmit(module_dm_motor_t *const me,
                                                      const uint8_t transmit_data[8],
                                                      uint32_t transmit_identifier)
{
    const bsp_can_frame_t frame = {.identifier = transmit_identifier,
                                   .id_type = BSP_CAN_ID_STANDARD,
                                   .frame_type = BSP_CAN_FRAME_DATA,
                                   .data_length = 8U,
                                   .data = {transmit_data[0], transmit_data[1], transmit_data[2],
                                            transmit_data[3], transmit_data[4], transmit_data[5],
                                            transmit_data[6], transmit_data[7]}};
    return (bsp_can_transmit(me->can, &frame, me->transmit_timeout_ms) == BSP_STATUS_OK)
               ? MODULE_MOTOR_STATUS_OK
               : MODULE_MOTOR_STATUS_TRANSPORT_ERROR;
}

static module_motor_status_t module_dm_motor_enable_virtual(module_motor_t *const motor_base)
{
    module_dm_motor_t *const me = module_dm_motor_get_device(motor_base);
    module_motor_status_t status = module_dm_motor_send_state_command(me, MODULE_DM_COMMAND_ENABLE);
    if (status == MODULE_MOTOR_STATUS_OK)
    {
        motor_base->state = MODULE_MOTOR_STATE_ENABLED;
    }
    return status;
}

static module_motor_status_t module_dm_motor_disable_virtual(module_motor_t *const motor_base)
{
    module_dm_motor_t *const me = module_dm_motor_get_device(motor_base);
    module_motor_status_t status =
        module_dm_motor_send_state_command(me, MODULE_DM_COMMAND_DISABLE);
    if (status == MODULE_MOTOR_STATUS_OK)
    {
        motor_base->state = MODULE_MOTOR_STATE_DISABLED;
    }
    return status;
}

static module_motor_status_t module_dm_motor_set_target_virtual(module_motor_t *const motor_base,
                                                                float target_value)
{
    module_dm_motor_t *const me = module_dm_motor_get_device(motor_base);
    if (!isfinite(target_value))
    {
        return MODULE_MOTOR_STATUS_OUT_OF_RANGE;
    }
    if (me->control_mode == MODULE_DM_MODE_MIT)
    {
        me->mit_command.torque_nm = target_value;
    }
    else if (me->control_mode == MODULE_DM_MODE_VELOCITY)
    {
        me->target_velocity_rad_per_s = target_value;
    }
    else
    {
        me->target_position_rad = target_value;
    }
    return MODULE_MOTOR_STATUS_OK;
}

static module_motor_status_t module_dm_motor_update_virtual(module_motor_t *const motor_base,
                                                            float delta_time_s)
{
    module_dm_motor_t *const me = module_dm_motor_get_device(motor_base);
    uint8_t transmit_data[8];
    module_motor_status_t status;
    (void)delta_time_s;

    if (motor_base->state != MODULE_MOTOR_STATE_ENABLED)
    {
        return MODULE_MOTOR_STATUS_OK;
    }
    status = me->mode_vptr->encode_command(me, transmit_data);
    if (status != MODULE_MOTOR_STATUS_OK)
    {
        return status;
    }
    return module_dm_motor_transmit(me, transmit_data, me->mode_vptr->get_transmit_identifier(me));
}

static const module_motor_ops_t s_module_dm_motor_ops = {.enable = module_dm_motor_enable_virtual,
                                                         .disable = module_dm_motor_disable_virtual,
                                                         .set_target =
                                                             module_dm_motor_set_target_virtual,
                                                         .update = module_dm_motor_update_virtual};

module_motor_status_t module_dm_motor_init(module_dm_motor_t *const me,
                                           const module_dm_motor_config_t *const config)
{
    if ((me == NULL) || (config == NULL) || (config->logical_name == NULL) ||
        (config->can == NULL) || !bsp_device_is_initialized(&config->can->super) ||
        (config->control_mode > MODULE_DM_MODE_POSITION_VELOCITY) ||
        !module_dm_motor_is_identifier_valid(config->control_mode, config->master_identifier) ||
        (config->feedback_identifier > 0x7FFU) ||
        !module_dm_motor_are_limits_valid(&config->limits))
    {
        return MODULE_MOTOR_STATUS_INVALID_ARGUMENT;
    }
    me->mode_vptr = (config->control_mode == MODULE_DM_MODE_MIT)
                        ? &s_module_dm_mit_ops
                        : ((config->control_mode == MODULE_DM_MODE_VELOCITY)
                               ? &s_module_dm_velocity_ops
                               : &s_module_dm_position_velocity_ops);
    me->can = config->can;
    me->control_mode = config->control_mode;
    me->limits = config->limits;
    me->mit_command = (module_dm_mit_command_t){0};
    me->target_position_rad = 0.0F;
    me->target_velocity_rad_per_s = 0.0F;
    me->master_identifier = config->master_identifier;
    me->feedback_identifier = config->feedback_identifier;
    me->transmit_timeout_ms = config->transmit_timeout_ms;
    me->fault = MODULE_DM_FAULT_NONE;
    me->mos_temperature_c = 0.0F;
    return module_motor_init_base(&me->super, &s_module_dm_motor_ops, config->logical_name,
                                  config->registration_key);
}

module_motor_status_t module_dm_motor_register(module_dm_motor_t *const me,
                                               module_motor_registry_t *const registry)
{
    if ((me == NULL) || (registry == NULL))
    {
        return MODULE_MOTOR_STATUS_INVALID_ARGUMENT;
    }
    return module_motor_registry_register(registry, &me->super);
}

module_motor_status_t module_dm_motor_unregister(module_dm_motor_t *const me,
                                                 module_motor_registry_t *const registry)
{
    module_motor_status_t status;

    if ((me == NULL) || (registry == NULL))
    {
        return MODULE_MOTOR_STATUS_INVALID_ARGUMENT;
    }
    if (!me->super.is_registered)
    {
        return MODULE_MOTOR_STATUS_NOT_REGISTERED;
    }
    status = module_motor_disable(&me->super);
    if (status != MODULE_MOTOR_STATUS_OK)
    {
        return status;
    }
    return module_motor_registry_unregister(registry, &me->super);
}

module_motor_t *module_dm_motor_as_base(module_dm_motor_t *const me)
{
    return (me != NULL) ? &me->super : NULL;
}

module_motor_status_t module_dm_motor_send_state_command(module_dm_motor_t *const me,
                                                         module_dm_state_command_t command)
{
    uint8_t transmit_data[8] = {0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU};
    static const uint8_t command_codes[] = {0xFDU, 0xFCU, 0xFEU, 0xFBU};

    if ((me == NULL) || !me->super.is_registered)
    {
        return (me == NULL) ? MODULE_MOTOR_STATUS_INVALID_ARGUMENT
                            : MODULE_MOTOR_STATUS_NOT_REGISTERED;
    }
    if ((uint32_t)command >= (sizeof(command_codes) / sizeof(command_codes[0])))
    {
        return MODULE_MOTOR_STATUS_OUT_OF_RANGE;
    }
    transmit_data[7] = command_codes[command];
    return module_dm_motor_transmit(me, transmit_data, me->master_identifier);
}

module_motor_status_t module_dm_motor_command_mit(module_dm_motor_t *const me,
                                                  const module_dm_mit_command_t *const command)
{
    module_motor_status_t status = module_dm_motor_set_mit_target(me, command);
    return (status == MODULE_MOTOR_STATUS_OK) ? module_motor_update(&me->super, 1.0F) : status;
}

module_motor_status_t module_dm_motor_set_mit_target(module_dm_motor_t *const me,
                                                     const module_dm_mit_command_t *const command)
{
    if ((me == NULL) || (command == NULL))
    {
        return MODULE_MOTOR_STATUS_INVALID_ARGUMENT;
    }
    if (me->control_mode != MODULE_DM_MODE_MIT)
    {
        return MODULE_MOTOR_STATUS_UNSUPPORTED;
    }
    if (!module_dm_motor_is_within(command->position_rad, me->limits.position_min_rad,
                                   me->limits.position_max_rad) ||
        !module_dm_motor_is_within(command->velocity_rad_per_s, me->limits.velocity_min_rad_per_s,
                                   me->limits.velocity_max_rad_per_s) ||
        !module_dm_motor_is_within(command->proportional_gain, me->limits.proportional_gain_min,
                                   me->limits.proportional_gain_max) ||
        !module_dm_motor_is_within(command->derivative_gain, me->limits.derivative_gain_min,
                                   me->limits.derivative_gain_max) ||
        !module_dm_motor_is_within(command->torque_nm, me->limits.torque_min_nm,
                                   me->limits.torque_max_nm))
    {
        return MODULE_MOTOR_STATUS_OUT_OF_RANGE;
    }
    me->mit_command = *command;
    return MODULE_MOTOR_STATUS_OK;
}

module_motor_status_t module_dm_motor_command_velocity(module_dm_motor_t *const me,
                                                       float velocity_rad_per_s)
{
    module_motor_status_t status = module_dm_motor_set_velocity_target(me, velocity_rad_per_s);
    return (status == MODULE_MOTOR_STATUS_OK) ? module_motor_update(&me->super, 1.0F) : status;
}

module_motor_status_t module_dm_motor_set_velocity_target(module_dm_motor_t *const me,
                                                          float velocity_rad_per_s)
{
    if (me == NULL)
    {
        return MODULE_MOTOR_STATUS_INVALID_ARGUMENT;
    }
    if (me->control_mode != MODULE_DM_MODE_VELOCITY)
    {
        return MODULE_MOTOR_STATUS_UNSUPPORTED;
    }
    if (!module_dm_motor_is_within(velocity_rad_per_s, me->limits.velocity_min_rad_per_s,
                                   me->limits.velocity_max_rad_per_s))
    {
        return MODULE_MOTOR_STATUS_OUT_OF_RANGE;
    }
    me->target_velocity_rad_per_s = velocity_rad_per_s;
    return MODULE_MOTOR_STATUS_OK;
}

module_motor_status_t module_dm_motor_command_position_velocity(module_dm_motor_t *const me,
                                                                float position_rad,
                                                                float velocity_rad_per_s)
{
    module_motor_status_t status =
        module_dm_motor_set_position_velocity_target(me, position_rad, velocity_rad_per_s);
    return (status == MODULE_MOTOR_STATUS_OK) ? module_motor_update(&me->super, 1.0F) : status;
}

module_motor_status_t module_dm_motor_set_position_velocity_target(module_dm_motor_t *const me,
                                                                   float position_rad,
                                                                   float velocity_rad_per_s)
{
    if (me == NULL)
    {
        return MODULE_MOTOR_STATUS_INVALID_ARGUMENT;
    }
    if (me->control_mode != MODULE_DM_MODE_POSITION_VELOCITY)
    {
        return MODULE_MOTOR_STATUS_UNSUPPORTED;
    }
    if (!module_dm_motor_is_within(position_rad, me->limits.position_min_rad,
                                   me->limits.position_max_rad) ||
        !module_dm_motor_is_within(velocity_rad_per_s, me->limits.velocity_min_rad_per_s,
                                   me->limits.velocity_max_rad_per_s))
    {
        return MODULE_MOTOR_STATUS_OUT_OF_RANGE;
    }
    me->target_position_rad = position_rad;
    me->target_velocity_rad_per_s = velocity_rad_per_s;
    return MODULE_MOTOR_STATUS_OK;
}

module_motor_status_t module_dm_motor_handle_feedback(module_dm_motor_t *const me,
                                                      const bsp_can_frame_t *const frame)
{
    uint32_t position_raw;
    uint32_t velocity_raw;
    uint32_t torque_raw;
    uint8_t state_code;

    if ((me == NULL) || (frame == NULL) || !me->super.is_registered ||
        (frame->id_type != BSP_CAN_ID_STANDARD) || (frame->frame_type != BSP_CAN_FRAME_DATA) ||
        (frame->identifier != me->feedback_identifier) || (frame->data_length != 8U))
    {
        return MODULE_MOTOR_STATUS_INVALID_ARGUMENT;
    }
    state_code = (uint8_t)(frame->data[0] >> 4U);
    position_raw = ((uint32_t)frame->data[1] << 8U) | frame->data[2];
    velocity_raw = ((uint32_t)frame->data[3] << 4U) | (frame->data[4] >> 4U);
    torque_raw = ((uint32_t)(frame->data[4] & 0x0FU) << 8U) | frame->data[5];
    me->super.feedback.position_rad = module_dm_motor_unsigned_to_float(
        position_raw, me->limits.position_min_rad, me->limits.position_max_rad, 16U);
    me->super.feedback.velocity_rad_per_s = module_dm_motor_unsigned_to_float(
        velocity_raw, me->limits.velocity_min_rad_per_s, me->limits.velocity_max_rad_per_s, 12U);
    me->super.feedback.torque_nm = module_dm_motor_unsigned_to_float(
        torque_raw, me->limits.torque_min_nm, me->limits.torque_max_nm, 12U);
    me->mos_temperature_c = (float)frame->data[6];
    me->super.feedback.motor_temperature_c = (float)frame->data[7];
    (void)module_motor_notify_feedback(&me->super);
    me->fault = (state_code >= 8U) ? (module_dm_fault_t)state_code : MODULE_DM_FAULT_NONE;
    if (me->fault != MODULE_DM_FAULT_NONE)
    {
        me->super.state = MODULE_MOTOR_STATE_FAULT;
    }
    return MODULE_MOTOR_STATUS_OK;
}

module_dm_fault_t module_dm_motor_get_fault(const module_dm_motor_t *const me)
{
    return (me != NULL) ? me->fault : MODULE_DM_FAULT_COMMUNICATION_LOST;
}

float module_dm_motor_get_mos_temperature_c(const module_dm_motor_t *const me)
{
    return (me != NULL) ? me->mos_temperature_c : 0.0F;
}
