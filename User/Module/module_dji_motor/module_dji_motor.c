#include "module_dji_motor.h"

#include <math.h>
#include <stddef.h>

#define MODULE_DJI_ENCODER_COUNTS_PER_REVOLUTION (8192.0F)
#define MODULE_DJI_TWO_PI (6.28318530717958647692F)

static module_dji_motor_t *module_dji_motor_get_device(module_motor_t *const motor_base)
{
    return MODULE_MOTOR_CONTAINER_OF(motor_base, module_dji_motor_t, super);
}

static int16_t module_dji_motor_clamp_command(float command_value, int16_t command_limit)
{
    if (command_value > (float)command_limit)
    {
        command_value = (float)command_limit;
    }
    if (command_value < (float)-command_limit)
    {
        command_value = (float)-command_limit;
    }
    return (int16_t)command_value;
}

static module_motor_status_t module_dji_motor_enable_virtual(module_motor_t *const motor_base)
{
    motor_base->state = MODULE_MOTOR_STATE_ENABLED;
    return MODULE_MOTOR_STATUS_OK;
}

static module_motor_status_t module_dji_motor_disable_virtual(module_motor_t *const motor_base)
{
    module_dji_motor_t *const me = module_dji_motor_get_device(motor_base);
    me->command_value = 0;
    motor_base->state = MODULE_MOTOR_STATE_DISABLED;
    return MODULE_MOTOR_STATUS_OK;
}

static module_motor_status_t module_dji_motor_set_target_virtual(module_motor_t *const motor_base,
                                                                 float target_value)
{
    module_dji_motor_t *const me = module_dji_motor_get_device(motor_base);
    if (!isfinite(target_value))
    {
        return MODULE_MOTOR_STATUS_OUT_OF_RANGE;
    }
    me->target_value = target_value;
    return MODULE_MOTOR_STATUS_OK;
}

static module_motor_status_t module_dji_motor_update_virtual(module_motor_t *const motor_base,
                                                             float delta_time_s)
{
    module_dji_motor_t *const me = module_dji_motor_get_device(motor_base);
    float controller_output = 0.0F;

    if (motor_base->state != MODULE_MOTOR_STATE_ENABLED)
    {
        me->command_value = 0;
        return MODULE_MOTOR_STATUS_OK;
    }
    if (me->control_mode == MODULE_DJI_CONTROL_DIRECT)
    {
        controller_output = me->target_value;
    }
    else if (me->control_mode == MODULE_DJI_CONTROL_VELOCITY)
    {
        if (alg_pid_update(&me->velocity_controller, me->target_value,
                           motor_base->feedback.velocity_rad_per_s, delta_time_s,
                           &controller_output) != ALG_PID_STATUS_OK)
        {
            return MODULE_MOTOR_STATUS_OUT_OF_RANGE;
        }
    }
    else
    {
        const alg_pid_cascade_input_t controller_input = {
            .position_setpoint = me->target_value,
            .position_measurement = motor_base->feedback.position_rad,
            .velocity_measurement = motor_base->feedback.velocity_rad_per_s,
            .velocity_feedforward = 0.0F,
            .actuator_feedforward = 0.0F,
            .delta_time_s = delta_time_s};
        if (alg_pid_cascade_update(&me->position_controller, &controller_input,
                                   &controller_output) != ALG_PID_STATUS_OK)
        {
            return MODULE_MOTOR_STATUS_OUT_OF_RANGE;
        }
    }
    me->command_value = module_dji_motor_clamp_command(controller_output * me->direction_sign,
                                                       me->maximum_command_value);
    return MODULE_MOTOR_STATUS_OK;
}

static const module_motor_ops_t s_module_dji_motor_ops = {
    .enable = module_dji_motor_enable_virtual,
    .disable = module_dji_motor_disable_virtual,
    .set_target = module_dji_motor_set_target_virtual,
    .update = module_dji_motor_update_virtual};

static module_motor_status_t
module_dji_motor_get_protocol_mapping(const module_dji_motor_config_t *const config,
                                      uint32_t *const receive_identifier,
                                      uint8_t *const group_index, uint8_t *const group_slot)
{
    uint8_t zero_based_identifier;

    if ((config->motor_identifier == 0U) || (config->motor_identifier > 8U))
    {
        return MODULE_MOTOR_STATUS_OUT_OF_RANGE;
    }
    zero_based_identifier = (uint8_t)(config->motor_identifier - 1U);

    if (config->motor_model == MODULE_DJI_MOTOR_GM6020)
    {
        if (config->motor_identifier > 7U)
        {
            return MODULE_MOTOR_STATUS_OUT_OF_RANGE;
        }
        *receive_identifier = 0x204U + config->motor_identifier;
        *group_index = (config->motor_identifier <= 4U) ? 0U : 2U;
        *group_slot = (uint8_t)(zero_based_identifier % 4U);
    }
    else
    {
        *receive_identifier = 0x200U + config->motor_identifier;
        *group_index = (config->motor_identifier <= 4U) ? 1U : 0U;
        *group_slot = (uint8_t)(zero_based_identifier % 4U);
    }
    return MODULE_MOTOR_STATUS_OK;
}

module_motor_status_t module_dji_motor_bus_init(module_dji_motor_bus_t *const me,
                                                bsp_can_t *const can, uint32_t transmit_timeout_ms)
{
    size_t group_index;
    size_t slot_index;
    if ((me == NULL) || (can == NULL) || !bsp_device_is_initialized(&can->super))
    {
        return MODULE_MOTOR_STATUS_INVALID_ARGUMENT;
    }
    for (group_index = 0U; group_index < MODULE_DJI_MOTOR_GROUP_COUNT; ++group_index)
    {
        me->group_is_used[group_index] = false;
        for (slot_index = 0U; slot_index < MODULE_DJI_MOTOR_PER_GROUP; ++slot_index)
        {
            me->motor_slots[group_index][slot_index] = NULL;
        }
    }
    me->can = can;
    me->transmit_timeout_ms = transmit_timeout_ms;
    me->is_initialized = true;
    return MODULE_MOTOR_STATUS_OK;
}

module_motor_status_t module_dji_motor_init(module_dji_motor_t *const me,
                                            const module_dji_motor_config_t *const config)
{
    module_motor_status_t status;
    if ((me == NULL) || (config == NULL) || (config->logical_name == NULL) ||
        (config->motor_bus == NULL) || !config->motor_bus->is_initialized ||
        (config->motor_model > MODULE_DJI_MOTOR_GM6020) ||
        (config->control_mode > MODULE_DJI_CONTROL_POSITION) ||
        ((config->direction_sign != 1.0F) && (config->direction_sign != -1.0F)) ||
        !isfinite(config->maximum_temperature_c) || (config->maximum_temperature_c <= 0.0F) ||
        !isfinite(config->current_scale_a_per_count) || (config->current_scale_a_per_count < 0.0F))
    {
        return MODULE_MOTOR_STATUS_INVALID_ARGUMENT;
    }

    status = module_dji_motor_get_protocol_mapping(config, &me->receive_identifier,
                                                   &me->group_index, &me->group_slot);
    if (status != MODULE_MOTOR_STATUS_OK)
    {
        return status;
    }
    me->motor_bus = config->motor_bus;
    me->motor_model = config->motor_model;
    me->control_mode = config->control_mode;
    me->direction_sign = config->direction_sign;
    me->maximum_temperature_c = config->maximum_temperature_c;
    me->current_scale_a_per_count = config->current_scale_a_per_count;
    me->gear_ratio = (config->motor_model == MODULE_DJI_MOTOR_M2006)
                         ? 36.0F
                         : ((config->motor_model == MODULE_DJI_MOTOR_M3508) ? 19.0F : 1.0F);
    if (config->motor_model == MODULE_DJI_MOTOR_M2006)
    {
        me->maximum_command_value = 10000;
    }
    else if (config->motor_model == MODULE_DJI_MOTOR_M3508)
    {
        me->maximum_command_value = 16000;
    }
    else
    {
        me->maximum_command_value = 30000;
    }
    me->target_value = 0.0F;
    me->command_value = 0;
    me->previous_encoder_count = 0U;
    me->accumulated_encoder_count = 0;
    me->has_previous_encoder_count = false;

    if ((config->control_mode == MODULE_DJI_CONTROL_VELOCITY) &&
        (alg_pid_init(&me->velocity_controller, &config->velocity_pid_config) != ALG_PID_STATUS_OK))
    {
        return MODULE_MOTOR_STATUS_INVALID_ARGUMENT;
    }
    if ((config->control_mode == MODULE_DJI_CONTROL_POSITION) &&
        (alg_pid_cascade_init(&me->position_controller, &config->position_pid_config) !=
         ALG_PID_STATUS_OK))
    {
        return MODULE_MOTOR_STATUS_INVALID_ARGUMENT;
    }
    return module_motor_init_base(&me->super, &s_module_dji_motor_ops, config->logical_name,
                                  config->registration_key);
}

module_motor_status_t module_dji_motor_register(module_dji_motor_t *const me,
                                                module_motor_registry_t *const registry)
{
    module_motor_status_t status;
    size_t group_index;
    size_t slot_index;

    if ((me == NULL) || (registry == NULL) || !me->super.is_initialized)
    {
        return MODULE_MOTOR_STATUS_INVALID_ARGUMENT;
    }
    if (me->motor_bus->motor_slots[me->group_index][me->group_slot] != NULL)
    {
        return MODULE_MOTOR_STATUS_DUPLICATE_KEY;
    }
    for (group_index = 0U; group_index < MODULE_DJI_MOTOR_GROUP_COUNT; ++group_index)
    {
        for (slot_index = 0U; slot_index < MODULE_DJI_MOTOR_PER_GROUP; ++slot_index)
        {
            const module_dji_motor_t *const registered_motor =
                me->motor_bus->motor_slots[group_index][slot_index];
            if ((registered_motor != NULL) &&
                (registered_motor->receive_identifier == me->receive_identifier))
            {
                return MODULE_MOTOR_STATUS_DUPLICATE_KEY;
            }
        }
    }
    status = module_motor_registry_register(registry, &me->super);
    if (status == MODULE_MOTOR_STATUS_OK)
    {
        me->motor_bus->motor_slots[me->group_index][me->group_slot] = me;
        me->motor_bus->group_is_used[me->group_index] = true;
    }
    return status;
}

module_motor_status_t module_dji_motor_unregister(module_dji_motor_t *const me,
                                                  module_motor_registry_t *const registry)
{
    module_motor_status_t status;
    size_t slot_index;
    bool group_is_used = false;

    if ((me == NULL) || (registry == NULL))
    {
        return MODULE_MOTOR_STATUS_INVALID_ARGUMENT;
    }
    if (!me->super.is_registered || (me->motor_bus == NULL) || !me->motor_bus->is_initialized ||
        (me->motor_bus->motor_slots[me->group_index][me->group_slot] != me))
    {
        return MODULE_MOTOR_STATUS_NOT_REGISTERED;
    }
    status = module_motor_disable(&me->super);
    if (status != MODULE_MOTOR_STATUS_OK)
    {
        return status;
    }
    status = module_dji_motor_bus_flush(me->motor_bus);
    if (status != MODULE_MOTOR_STATUS_OK)
    {
        return status;
    }
    status = module_motor_registry_unregister(registry, &me->super);
    if (status != MODULE_MOTOR_STATUS_OK)
    {
        return status;
    }
    me->motor_bus->motor_slots[me->group_index][me->group_slot] = NULL;
    for (slot_index = 0U; slot_index < MODULE_DJI_MOTOR_PER_GROUP; ++slot_index)
    {
        if (me->motor_bus->motor_slots[me->group_index][slot_index] != NULL)
        {
            group_is_used = true;
            break;
        }
    }
    me->motor_bus->group_is_used[me->group_index] = group_is_used;
    return MODULE_MOTOR_STATUS_OK;
}

module_motor_t *module_dji_motor_as_base(module_dji_motor_t *const me)
{
    return (me != NULL) ? &me->super : NULL;
}

module_motor_status_t module_dji_motor_bus_handle_feedback(module_dji_motor_bus_t *const me,
                                                           const bsp_can_frame_t *const frame)
{
    size_t group_index;
    size_t slot_index;
    module_dji_motor_t *motor = NULL;
    uint16_t encoder_count;
    int32_t encoder_delta;
    int16_t speed_rpm;
    int16_t current_raw;

    if ((me == NULL) || (frame == NULL) || !me->is_initialized ||
        (frame->id_type != BSP_CAN_ID_STANDARD) || (frame->frame_type != BSP_CAN_FRAME_DATA) ||
        (frame->data_length != 8U))
    {
        return MODULE_MOTOR_STATUS_INVALID_ARGUMENT;
    }
    for (group_index = 0U; group_index < MODULE_DJI_MOTOR_GROUP_COUNT; ++group_index)
    {
        for (slot_index = 0U; slot_index < MODULE_DJI_MOTOR_PER_GROUP; ++slot_index)
        {
            if ((me->motor_slots[group_index][slot_index] != NULL) &&
                (me->motor_slots[group_index][slot_index]->receive_identifier == frame->identifier))
            {
                motor = me->motor_slots[group_index][slot_index];
                break;
            }
        }
    }
    if (motor == NULL)
    {
        return MODULE_MOTOR_STATUS_NOT_REGISTERED;
    }

    encoder_count = (uint16_t)(((uint16_t)frame->data[0] << 8U) | frame->data[1]);
    speed_rpm = (int16_t)(((uint16_t)frame->data[2] << 8U) | frame->data[3]);
    current_raw = (int16_t)(((uint16_t)frame->data[4] << 8U) | frame->data[5]);
    if (!motor->has_previous_encoder_count)
    {
        motor->previous_encoder_count = encoder_count;
        motor->has_previous_encoder_count = true;
    }
    encoder_delta = (int32_t)encoder_count - (int32_t)motor->previous_encoder_count;
    if (encoder_delta > 4096)
    {
        encoder_delta -= 8192;
    }
    else if (encoder_delta < -4096)
    {
        encoder_delta += 8192;
    }
    motor->previous_encoder_count = encoder_count;
    motor->accumulated_encoder_count += encoder_delta;

    motor->super.feedback.raw_position = encoder_count;
    motor->super.feedback.position_rad =
        motor->direction_sign * (float)motor->accumulated_encoder_count * MODULE_DJI_TWO_PI /
        (MODULE_DJI_ENCODER_COUNTS_PER_REVOLUTION * motor->gear_ratio);
    motor->super.feedback.velocity_rad_per_s =
        motor->direction_sign * (float)speed_rpm * MODULE_DJI_TWO_PI / (60.0F * motor->gear_ratio);
    motor->super.feedback.current_raw = current_raw;
    motor->super.feedback.is_current_a_valid = motor->current_scale_a_per_count > 0.0F;
    motor->super.feedback.current_a = motor->super.feedback.is_current_a_valid
                                          ? (float)current_raw * motor->current_scale_a_per_count
                                          : 0.0F;
    motor->super.feedback.motor_temperature_c = (float)frame->data[6];
    (void)module_motor_notify_feedback(&motor->super);
    if (motor->super.feedback.motor_temperature_c > motor->maximum_temperature_c)
    {
        motor->command_value = 0;
        motor->super.state = MODULE_MOTOR_STATE_FAULT;
    }
    return MODULE_MOTOR_STATUS_OK;
}

module_motor_status_t module_dji_motor_bus_flush(module_dji_motor_bus_t *const me)
{
    static const uint32_t group_identifiers[MODULE_DJI_MOTOR_GROUP_COUNT] = {0x1FFU, 0x200U,
                                                                             0x2FFU};
    size_t group_index;
    size_t slot_index;

    if ((me == NULL) || !me->is_initialized)
    {
        return MODULE_MOTOR_STATUS_NOT_INITIALIZED;
    }
    for (group_index = 0U; group_index < MODULE_DJI_MOTOR_GROUP_COUNT; ++group_index)
    {
        bsp_can_frame_t frame = {.identifier = group_identifiers[group_index],
                                 .id_type = BSP_CAN_ID_STANDARD,
                                 .frame_type = BSP_CAN_FRAME_DATA,
                                 .data_length = 8U,
                                 .data = {0U}};
        if (!me->group_is_used[group_index])
        {
            continue;
        }
        for (slot_index = 0U; slot_index < MODULE_DJI_MOTOR_PER_GROUP; ++slot_index)
        {
            int16_t command_value = 0;
            if (me->motor_slots[group_index][slot_index] != NULL)
            {
                command_value = me->motor_slots[group_index][slot_index]->command_value;
            }
            frame.data[slot_index * 2U] = (uint8_t)((uint16_t)command_value >> 8U);
            frame.data[slot_index * 2U + 1U] = (uint8_t)command_value;
        }
        if (bsp_can_transmit(me->can, &frame, me->transmit_timeout_ms) != BSP_STATUS_OK)
        {
            return MODULE_MOTOR_STATUS_TRANSPORT_ERROR;
        }
    }
    return MODULE_MOTOR_STATUS_OK;
}

int16_t module_dji_motor_get_command(const module_dji_motor_t *const me)
{
    return ((me != NULL) && me->super.is_initialized) ? me->command_value : 0;
}
