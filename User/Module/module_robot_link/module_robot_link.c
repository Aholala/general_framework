#include "module_robot_link.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define MODULE_ROBOT_LINK_SCALE (1000.0F)

static void module_robot_link_encode_int16(int16_t value, uint8_t output[2])
{
    const uint16_t unsigned_value = (uint16_t)value;
    output[0] = (uint8_t)unsigned_value;
    output[1] = (uint8_t)(unsigned_value >> 8U);
}

static int16_t module_robot_link_decode_int16(const uint8_t input[2])
{
    return (int16_t)((uint16_t)input[0] | ((uint16_t)input[1] << 8U));
}

static int16_t module_robot_link_encode_scaled(float value)
{
    float scaled_value = value * MODULE_ROBOT_LINK_SCALE;

    if (scaled_value > 32767.0F)
    {
        scaled_value = 32767.0F;
    }
    else if (scaled_value < -32768.0F)
    {
        scaled_value = -32768.0F;
    }
    return (int16_t)lrintf(scaled_value);
}

static float module_robot_link_decode_scaled(const uint8_t input[2])
{
    return (float)module_robot_link_decode_int16(input) / MODULE_ROBOT_LINK_SCALE;
}

static void module_robot_link_prepare_remote_transaction(module_robot_link_t *me, uint8_t sequence)
{
    if ((me->remote_receive_mask != 0U) && (me->remote_assembly_sequence == sequence))
    {
        return;
    }
    me->remote_staging = (module_dr16_data_t){0};
    me->remote_staging.valid_frame_count = me->remote_data.valid_frame_count;
    me->remote_staging.invalid_frame_count = me->remote_data.invalid_frame_count;
    me->remote_assembly_sequence = sequence;
    me->remote_receive_mask = 0U;
}

static void module_robot_link_prepare_gimbal_transaction(module_robot_link_t *me, uint8_t sequence)
{
    if ((me->gimbal_receive_mask != 0U) && (me->gimbal_assembly_sequence == sequence))
    {
        return;
    }
    me->gimbal_staging = (module_robot_link_gimbal_data_t){0};
    me->gimbal_assembly_sequence = sequence;
    me->gimbal_receive_mask = 0U;
}

static module_robot_link_status_t module_robot_link_transmit(module_robot_link_t *me,
                                                             module_robot_link_message_t message,
                                                             const uint8_t payload[6],
                                                             uint8_t flags, uint8_t sequence)
{
    bsp_can_frame_t frame = {
        .identifier = me->base_identifier + (uint32_t)message,
        .id_type = BSP_CAN_ID_STANDARD,
        .frame_type = BSP_CAN_FRAME_DATA,
        .data_length = 8U,
    };

    frame.data[0] = sequence;
    frame.data[1] = flags;
    (void)memcpy(&frame.data[2], payload, 6U);
    return (bsp_can_transmit(me->can, &frame, me->transmit_timeout_ms) == BSP_STATUS_OK)
               ? MODULE_ROBOT_LINK_STATUS_OK
               : MODULE_ROBOT_LINK_STATUS_TRANSPORT_ERROR;
}

static void module_robot_link_increment_elapsed(uint32_t *elapsed_time_ms, uint32_t increment_ms)
{
    if (*elapsed_time_ms > (UINT32_MAX - increment_ms))
    {
        *elapsed_time_ms = UINT32_MAX;
    }
    else
    {
        *elapsed_time_ms += increment_ms;
    }
}

module_robot_link_status_t module_robot_link_init(module_robot_link_t *me,
                                                  const module_robot_link_config_t *config)
{
    if ((me == NULL) || (config == NULL) || (config->can == NULL) ||
        !bsp_device_is_initialized(&config->can->super) ||
        (config->base_identifier > (0x7FFU - (uint32_t)MODULE_ROBOT_LINK_MESSAGE_COUNT)))
    {
        return MODULE_ROBOT_LINK_STATUS_INVALID_ARGUMENT;
    }
    (void)memset(me, 0, sizeof(*me));
    me->can = config->can;
    me->base_identifier = config->base_identifier;
    me->transmit_timeout_ms = config->transmit_timeout_ms;
    me->offline_timeout_ms = config->offline_timeout_ms;
    me->is_initialized = true;
    return MODULE_ROBOT_LINK_STATUS_OK;
}

module_robot_link_status_t module_robot_link_send_remote(module_robot_link_t *me,
                                                         const module_dr16_data_t *remote_data)
{
    uint8_t payload[6];
    uint8_t flags;
    uint8_t sequence;
    module_robot_link_status_t status;

    if ((me == NULL) || (remote_data == NULL))
    {
        return MODULE_ROBOT_LINK_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return MODULE_ROBOT_LINK_STATUS_NOT_INITIALIZED;
    }
    sequence = me->transmit_sequence++;
    module_robot_link_encode_int16(remote_data->channel[0], &payload[0]);
    module_robot_link_encode_int16(remote_data->channel[1], &payload[2]);
    module_robot_link_encode_int16(remote_data->channel[2], &payload[4]);
    status = module_robot_link_transmit(me, MODULE_ROBOT_LINK_MESSAGE_REMOTE_CHANNELS_PRIMARY,
                                        payload, 0U, sequence);
    if (status != MODULE_ROBOT_LINK_STATUS_OK)
    {
        return status;
    }
    module_robot_link_encode_int16(remote_data->channel[3], &payload[0]);
    module_robot_link_encode_int16(remote_data->dial, &payload[2]);
    module_robot_link_encode_int16((int16_t)remote_data->keyboard, &payload[4]);
    status = module_robot_link_transmit(me, MODULE_ROBOT_LINK_MESSAGE_REMOTE_CHANNELS_AUXILIARY,
                                        payload, 0U, sequence);
    if (status != MODULE_ROBOT_LINK_STATUS_OK)
    {
        return status;
    }
    module_robot_link_encode_int16(remote_data->mouse_x, &payload[0]);
    module_robot_link_encode_int16(remote_data->mouse_y, &payload[2]);
    module_robot_link_encode_int16(remote_data->mouse_z, &payload[4]);
    flags =
        (uint8_t)((uint8_t)remote_data->left_switch | ((uint8_t)remote_data->right_switch << 2U) |
                  (remote_data->mouse_left_pressed ? (1U << 4U) : 0U) |
                  (remote_data->mouse_right_pressed ? (1U << 5U) : 0U) |
                  (remote_data->is_online ? (1U << 7U) : 0U));
    return module_robot_link_transmit(me, MODULE_ROBOT_LINK_MESSAGE_REMOTE_INPUT, payload, flags,
                                      sequence);
}

module_robot_link_status_t
module_robot_link_send_gimbal(module_robot_link_t *me,
                              const module_robot_link_gimbal_data_t *gimbal_data)
{
    uint8_t payload[6];
    uint8_t flags;
    module_robot_link_status_t status;
    uint8_t sequence;

    if ((me == NULL) || (gimbal_data == NULL) || !isfinite(gimbal_data->yaw_rad) ||
        !isfinite(gimbal_data->pitch_rad) || !isfinite(gimbal_data->yaw_velocity_rad_per_s) ||
        !isfinite(gimbal_data->pitch_velocity_rad_per_s))
    {
        return MODULE_ROBOT_LINK_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return MODULE_ROBOT_LINK_STATUS_NOT_INITIALIZED;
    }
    sequence = me->transmit_sequence++;
    module_robot_link_encode_int16(module_robot_link_encode_scaled(gimbal_data->yaw_rad),
                                   &payload[0]);
    module_robot_link_encode_int16(module_robot_link_encode_scaled(gimbal_data->pitch_rad),
                                   &payload[2]);
    module_robot_link_encode_int16(
        module_robot_link_encode_scaled(gimbal_data->yaw_velocity_rad_per_s), &payload[4]);
    flags = (gimbal_data->imu_valid ? 1U : 0U) | (gimbal_data->motors_online ? 2U : 0U);
    status = module_robot_link_transmit(me, MODULE_ROBOT_LINK_MESSAGE_GIMBAL_PRIMARY, payload,
                                        flags, sequence);
    if (status != MODULE_ROBOT_LINK_STATUS_OK)
    {
        return status;
    }
    (void)memset(payload, 0, sizeof(payload));
    module_robot_link_encode_int16(
        module_robot_link_encode_scaled(gimbal_data->pitch_velocity_rad_per_s), &payload[0]);
    return module_robot_link_transmit(me, MODULE_ROBOT_LINK_MESSAGE_GIMBAL_AUXILIARY, payload,
                                      flags, sequence);
}

module_robot_link_status_t
module_robot_link_send_chassis(module_robot_link_t *me,
                               const module_robot_link_chassis_data_t *chassis_data)
{
    uint8_t payload[6];
    uint8_t flags;
    uint8_t sequence;

    if ((me == NULL) || (chassis_data == NULL) || !isfinite(chassis_data->velocity_x_m_per_s) ||
        !isfinite(chassis_data->velocity_y_m_per_s) ||
        !isfinite(chassis_data->angular_velocity_rad_per_s))
    {
        return MODULE_ROBOT_LINK_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return MODULE_ROBOT_LINK_STATUS_NOT_INITIALIZED;
    }
    sequence = me->transmit_sequence++;
    module_robot_link_encode_int16(
        module_robot_link_encode_scaled(chassis_data->velocity_x_m_per_s), &payload[0]);
    module_robot_link_encode_int16(
        module_robot_link_encode_scaled(chassis_data->velocity_y_m_per_s), &payload[2]);
    module_robot_link_encode_int16(
        module_robot_link_encode_scaled(chassis_data->angular_velocity_rad_per_s), &payload[4]);
    flags = (chassis_data->motors_online ? 1U : 0U) | (chassis_data->self_lock_active ? 2U : 0U);
    return module_robot_link_transmit(me, MODULE_ROBOT_LINK_MESSAGE_CHASSIS, payload, flags,
                                      sequence);
}

module_robot_link_status_t
module_robot_link_send_shooter(module_robot_link_t *me,
                               const module_robot_link_shooter_data_t *shooter_data)
{
    uint8_t payload[6] = {0U};
    uint8_t sequence;

    if ((me == NULL) || (shooter_data == NULL) ||
        !isfinite(shooter_data->friction_velocity_rad_per_s) ||
        !isfinite(shooter_data->feeder_position_rad))
    {
        return MODULE_ROBOT_LINK_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return MODULE_ROBOT_LINK_STATUS_NOT_INITIALIZED;
    }
    sequence = me->transmit_sequence++;
    module_robot_link_encode_int16(
        module_robot_link_encode_scaled(shooter_data->friction_velocity_rad_per_s), &payload[0]);
    module_robot_link_encode_int16(
        module_robot_link_encode_scaled(shooter_data->feeder_position_rad), &payload[2]);
    payload[4] = shooter_data->state;
    payload[5] = shooter_data->jam_retry_count;
    return module_robot_link_transmit(me, MODULE_ROBOT_LINK_MESSAGE_SHOOTER, payload, 0U, sequence);
}

module_robot_link_status_t module_robot_link_send_heartbeat(module_robot_link_t *me,
                                                            uint8_t board_role, uint32_t uptime_ms)
{
    uint8_t payload[6] = {
        board_role,
        0U,
        (uint8_t)uptime_ms,
        (uint8_t)(uptime_ms >> 8U),
        (uint8_t)(uptime_ms >> 16U),
        (uint8_t)(uptime_ms >> 24U),
    };
    uint8_t sequence;

    if (me == NULL)
    {
        return MODULE_ROBOT_LINK_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return MODULE_ROBOT_LINK_STATUS_NOT_INITIALIZED;
    }
    sequence = me->transmit_sequence++;
    return module_robot_link_transmit(me, MODULE_ROBOT_LINK_MESSAGE_HEARTBEAT, payload, 0U,
                                      sequence);
}

module_robot_link_status_t module_robot_link_handle_frame(module_robot_link_t *me,
                                                          const bsp_can_frame_t *frame)
{
    module_robot_link_message_t message;
    const uint8_t *payload;
    uint8_t sequence;

    if ((me == NULL) || (frame == NULL))
    {
        return MODULE_ROBOT_LINK_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return MODULE_ROBOT_LINK_STATUS_NOT_INITIALIZED;
    }
    if ((frame->id_type != BSP_CAN_ID_STANDARD) || (frame->frame_type != BSP_CAN_FRAME_DATA) ||
        (frame->data_length != 8U) || (frame->identifier < me->base_identifier) ||
        (frame->identifier >= (me->base_identifier + (uint32_t)MODULE_ROBOT_LINK_MESSAGE_COUNT)))
    {
        return MODULE_ROBOT_LINK_STATUS_INVALID_FRAME;
    }
    message = (module_robot_link_message_t)(frame->identifier - me->base_identifier);
    payload = &frame->data[2];
    sequence = frame->data[0];

    switch (message)
    {
    case MODULE_ROBOT_LINK_MESSAGE_REMOTE_CHANNELS_PRIMARY:
        module_robot_link_prepare_remote_transaction(me, sequence);
        me->remote_staging.channel[0] = module_robot_link_decode_int16(&payload[0]);
        me->remote_staging.channel[1] = module_robot_link_decode_int16(&payload[2]);
        me->remote_staging.channel[2] = module_robot_link_decode_int16(&payload[4]);
        me->remote_receive_mask |= 1U;
        break;
    case MODULE_ROBOT_LINK_MESSAGE_REMOTE_CHANNELS_AUXILIARY:
        module_robot_link_prepare_remote_transaction(me, sequence);
        me->remote_staging.channel[3] = module_robot_link_decode_int16(&payload[0]);
        me->remote_staging.dial = module_robot_link_decode_int16(&payload[2]);
        me->remote_staging.keyboard = (uint16_t)module_robot_link_decode_int16(&payload[4]);
        me->remote_receive_mask |= 2U;
        break;
    case MODULE_ROBOT_LINK_MESSAGE_REMOTE_INPUT:
        module_robot_link_prepare_remote_transaction(me, sequence);
        me->remote_staging.mouse_x = module_robot_link_decode_int16(&payload[0]);
        me->remote_staging.mouse_y = module_robot_link_decode_int16(&payload[2]);
        me->remote_staging.mouse_z = module_robot_link_decode_int16(&payload[4]);
        me->remote_staging.left_switch = (module_dr16_switch_t)(frame->data[1] & 0x03U);
        me->remote_staging.right_switch = (module_dr16_switch_t)((frame->data[1] >> 2U) & 0x03U);
        me->remote_staging.mouse_left_pressed = (frame->data[1] & (1U << 4U)) != 0U;
        me->remote_staging.mouse_right_pressed = (frame->data[1] & (1U << 5U)) != 0U;
        me->remote_staging.is_online = (frame->data[1] & (1U << 7U)) != 0U;
        me->remote_receive_mask |= 4U;
        break;
    case MODULE_ROBOT_LINK_MESSAGE_GIMBAL_PRIMARY:
        module_robot_link_prepare_gimbal_transaction(me, sequence);
        me->gimbal_staging.yaw_rad = module_robot_link_decode_scaled(&payload[0]);
        me->gimbal_staging.pitch_rad = module_robot_link_decode_scaled(&payload[2]);
        me->gimbal_staging.yaw_velocity_rad_per_s = module_robot_link_decode_scaled(&payload[4]);
        me->gimbal_staging.imu_valid = (frame->data[1] & 1U) != 0U;
        me->gimbal_staging.motors_online = (frame->data[1] & 2U) != 0U;
        me->gimbal_receive_mask |= 1U;
        break;
    case MODULE_ROBOT_LINK_MESSAGE_GIMBAL_AUXILIARY:
        module_robot_link_prepare_gimbal_transaction(me, sequence);
        me->gimbal_staging.pitch_velocity_rad_per_s = module_robot_link_decode_scaled(&payload[0]);
        me->gimbal_receive_mask |= 2U;
        break;
    case MODULE_ROBOT_LINK_MESSAGE_CHASSIS:
        me->chassis_data.velocity_x_m_per_s = module_robot_link_decode_scaled(&payload[0]);
        me->chassis_data.velocity_y_m_per_s = module_robot_link_decode_scaled(&payload[2]);
        me->chassis_data.angular_velocity_rad_per_s = module_robot_link_decode_scaled(&payload[4]);
        me->chassis_data.motors_online = (frame->data[1] & 1U) != 0U;
        me->chassis_data.self_lock_active = (frame->data[1] & 2U) != 0U;
        me->chassis_elapsed_time_ms = 0U;
        me->chassis_online = true;
        break;
    case MODULE_ROBOT_LINK_MESSAGE_SHOOTER:
        me->shooter_data.friction_velocity_rad_per_s = module_robot_link_decode_scaled(&payload[0]);
        me->shooter_data.feeder_position_rad = module_robot_link_decode_scaled(&payload[2]);
        me->shooter_data.state = payload[4];
        me->shooter_data.jam_retry_count = payload[5];
        me->shooter_elapsed_time_ms = 0U;
        me->shooter_online = true;
        break;
    case MODULE_ROBOT_LINK_MESSAGE_HEARTBEAT:
    case MODULE_ROBOT_LINK_MESSAGE_COUNT:
    default:
        break;
    }
    if (me->remote_receive_mask == 7U)
    {
        size_t channel_index;
        for (channel_index = 0U; channel_index < MODULE_DR16_CHANNEL_COUNT; ++channel_index)
        {
            me->remote_staging.normalized_channel[channel_index] =
                module_dr16_normalize_channel_value(me->remote_staging.channel[channel_index]);
        }
        me->remote_staging.normalized_dial =
            module_dr16_normalize_channel_value(me->remote_staging.dial);
        if (me->remote_staging.valid_frame_count != UINT32_MAX)
        {
            ++me->remote_staging.valid_frame_count;
        }
        me->remote_data = me->remote_staging;
        me->remote_elapsed_time_ms = 0U;
        me->remote_online = me->remote_data.is_online;
        me->remote_receive_mask = 0U;
    }
    if (me->gimbal_receive_mask == 3U)
    {
        me->gimbal_data = me->gimbal_staging;
        me->gimbal_elapsed_time_ms = 0U;
        me->gimbal_online = true;
        me->gimbal_receive_mask = 0U;
    }
    return MODULE_ROBOT_LINK_STATUS_OK;
}

const module_dr16_data_t *module_robot_link_get_remote(const module_robot_link_t *me)
{
    return ((me != NULL) && me->is_initialized && me->remote_online) ? &me->remote_data : NULL;
}

const module_robot_link_gimbal_data_t *module_robot_link_get_gimbal(const module_robot_link_t *me)
{
    return ((me != NULL) && me->is_initialized && me->gimbal_online) ? &me->gimbal_data : NULL;
}

const module_robot_link_chassis_data_t *module_robot_link_get_chassis(const module_robot_link_t *me)
{
    return ((me != NULL) && me->is_initialized && me->chassis_online) ? &me->chassis_data : NULL;
}

const module_robot_link_shooter_data_t *module_robot_link_get_shooter(const module_robot_link_t *me)
{
    return ((me != NULL) && me->is_initialized && me->shooter_online) ? &me->shooter_data : NULL;
}

void module_robot_link_update_time(module_robot_link_t *me, uint32_t elapsed_time_ms)
{
    if ((me == NULL) || !me->is_initialized)
    {
        return;
    }
    module_robot_link_increment_elapsed(&me->remote_elapsed_time_ms, elapsed_time_ms);
    module_robot_link_increment_elapsed(&me->gimbal_elapsed_time_ms, elapsed_time_ms);
    module_robot_link_increment_elapsed(&me->chassis_elapsed_time_ms, elapsed_time_ms);
    module_robot_link_increment_elapsed(&me->shooter_elapsed_time_ms, elapsed_time_ms);
    if (me->offline_timeout_ms > 0U)
    {
        me->remote_online =
            me->remote_online && (me->remote_elapsed_time_ms <= me->offline_timeout_ms);
        me->gimbal_online =
            me->gimbal_online && (me->gimbal_elapsed_time_ms <= me->offline_timeout_ms);
        me->chassis_online =
            me->chassis_online && (me->chassis_elapsed_time_ms <= me->offline_timeout_ms);
        me->shooter_online =
            me->shooter_online && (me->shooter_elapsed_time_ms <= me->offline_timeout_ms);
    }
}
