#include "module_referee_data.h"

#include <string.h>

static uint64_t module_referee_data_read_uint64(const uint8_t *data)
{
    uint64_t value = 0U;
    uint8_t byte_index;
    for (byte_index = 0U; byte_index < 8U; ++byte_index)
    {
        value |= (uint64_t)data[byte_index] << (8U * byte_index);
    }
    return value;
}

static uint32_t module_referee_data_command_bit(uint16_t command_id)
{
    switch (command_id)
    {
    case MODULE_REFEREE_COMMAND_GAME_STATUS:
        return 1UL << 0U;
    case MODULE_REFEREE_COMMAND_GAME_RESULT:
        return 1UL << 1U;
    case MODULE_REFEREE_COMMAND_ROBOT_HEALTH:
        return 1UL << 2U;
    case MODULE_REFEREE_COMMAND_EVENT_DATA:
        return 1UL << 3U;
    case MODULE_REFEREE_COMMAND_REFEREE_WARNING:
        return 1UL << 4U;
    case MODULE_REFEREE_COMMAND_ROBOT_STATUS:
        return 1UL << 5U;
    case MODULE_REFEREE_COMMAND_POWER_HEAT:
        return 1UL << 6U;
    case MODULE_REFEREE_COMMAND_ROBOT_POSITION:
        return 1UL << 7U;
    case MODULE_REFEREE_COMMAND_ROBOT_BUFF:
        return 1UL << 8U;
    case MODULE_REFEREE_COMMAND_ROBOT_HURT:
        return 1UL << 9U;
    case MODULE_REFEREE_COMMAND_SHOOT_DATA:
        return 1UL << 10U;
    case MODULE_REFEREE_COMMAND_PROJECTILE_ALLOWANCE:
        return 1UL << 11U;
    case MODULE_REFEREE_COMMAND_RFID_STATUS:
        return 1UL << 12U;
    default:
        return 0U;
    }
}

void module_referee_data_reset(module_referee_data_t *me)
{
    if (me != NULL)
    {
        *me = (module_referee_data_t){0};
    }
}

void module_referee_data_route_handler(uint16_t command_id, const uint8_t *payload,
                                       size_t payload_size, uint8_t sequence, void *user_context)
{
    module_referee_data_t *const me = (module_referee_data_t *)user_context;
    bool decoded = false;
    (void)sequence;
    if ((me == NULL) || ((payload == NULL) && (payload_size != 0U)))
    {
        return;
    }
    switch (command_id)
    {
    case MODULE_REFEREE_COMMAND_GAME_STATUS:
        if (payload_size >= 11U)
        {
            me->game_status.game_type = payload[0] & 0x0FU;
            me->game_status.game_progress = (payload[0] >> 4U) & 0x0FU;
            me->game_status.remaining_time_s = module_referee_read_uint16_le(&payload[1]);
            me->game_status.synchronization_timestamp_us =
                module_referee_data_read_uint64(&payload[3]);
            decoded = true;
        }
        break;
    case MODULE_REFEREE_COMMAND_GAME_RESULT:
        if (payload_size >= 1U)
        {
            me->game_result = payload[0];
            decoded = true;
        }
        break;
    case MODULE_REFEREE_COMMAND_ROBOT_HEALTH:
        if (payload_size >= 32U)
        {
            size_t robot_index;
            for (robot_index = 0U; robot_index < 16U; ++robot_index)
            {
                me->robot_health[robot_index] =
                    module_referee_read_uint16_le(&payload[2U * robot_index]);
            }
            decoded = true;
        }
        break;
    case MODULE_REFEREE_COMMAND_EVENT_DATA:
        if (payload_size >= 4U)
        {
            me->event_data = module_referee_read_uint32_le(payload);
            decoded = true;
        }
        break;
    case MODULE_REFEREE_COMMAND_REFEREE_WARNING:
        if (payload_size >= 2U)
        {
            me->warning_level = payload[0];
            me->warning_robot_id = payload[1];
            decoded = true;
        }
        break;
    case MODULE_REFEREE_COMMAND_ROBOT_STATUS:
        if (payload_size >= 13U)
        {
            me->robot_status.robot_id = payload[0];
            me->robot_status.robot_level = payload[1];
            me->robot_status.current_hp = module_referee_read_uint16_le(&payload[2]);
            me->robot_status.maximum_hp = module_referee_read_uint16_le(&payload[4]);
            me->robot_status.shooter_barrel_cooling_rate =
                module_referee_read_uint16_le(&payload[6]);
            me->robot_status.shooter_barrel_heat_limit = module_referee_read_uint16_le(&payload[8]);
            me->robot_status.chassis_power_limit_w = module_referee_read_uint16_le(&payload[10]);
            me->robot_status.gimbal_power_enabled = (payload[12] & 0x01U) != 0U;
            me->robot_status.chassis_power_enabled = (payload[12] & 0x02U) != 0U;
            me->robot_status.shooter_power_enabled = (payload[12] & 0x04U) != 0U;
            decoded = true;
        }
        break;
    case MODULE_REFEREE_COMMAND_POWER_HEAT:
        if (payload_size >= 16U)
        {
            me->power_heat.chassis_voltage_mv = module_referee_read_uint16_le(payload);
            me->power_heat.chassis_current_ma = module_referee_read_uint16_le(&payload[2]);
            me->power_heat.chassis_power_w = module_referee_read_float_le(&payload[4]);
            me->power_heat.chassis_buffer_energy_j = module_referee_read_uint16_le(&payload[8]);
            me->power_heat.shooter_17_mm_1_heat = module_referee_read_uint16_le(&payload[10]);
            me->power_heat.shooter_17_mm_2_heat = module_referee_read_uint16_le(&payload[12]);
            me->power_heat.shooter_42_mm_heat = module_referee_read_uint16_le(&payload[14]);
            decoded = true;
        }
        break;
    case MODULE_REFEREE_COMMAND_ROBOT_POSITION:
        if (payload_size >= 12U)
        {
            me->robot_position.position_x_m = module_referee_read_float_le(payload);
            me->robot_position.position_y_m = module_referee_read_float_le(&payload[4]);
            me->robot_position.yaw_rad = module_referee_read_float_le(&payload[8]);
            decoded = true;
        }
        break;
    case MODULE_REFEREE_COMMAND_ROBOT_HURT:
        if (payload_size >= 1U)
        {
            me->hurt.armor_id = payload[0] & 0x0FU;
            me->hurt.hurt_type = (payload[0] >> 4U) & 0x0FU;
            decoded = true;
        }
        break;
    case MODULE_REFEREE_COMMAND_SHOOT_DATA:
        if (payload_size >= 7U)
        {
            me->shoot_data.bullet_type = payload[0];
            me->shoot_data.shooter_number = payload[1];
            me->shoot_data.frequency_hz = payload[2];
            me->shoot_data.speed_m_per_s = module_referee_read_float_le(&payload[3]);
            decoded = true;
        }
        break;
    case MODULE_REFEREE_COMMAND_PROJECTILE_ALLOWANCE:
        if (payload_size >= 6U)
        {
            me->projectile_allowance.projectile_17_mm_remaining =
                module_referee_read_uint16_le(payload);
            me->projectile_allowance.projectile_42_mm_remaining =
                module_referee_read_uint16_le(&payload[2]);
            me->projectile_allowance.coin_remaining = module_referee_read_uint16_le(&payload[4]);
            decoded = true;
        }
        break;
    case MODULE_REFEREE_COMMAND_RFID_STATUS:
        if (payload_size >= 4U)
        {
            me->rfid_status = module_referee_read_uint32_le(payload);
            decoded = true;
        }
        break;
    default:
        return;
    }
    if (decoded)
    {
        me->update_mask |= module_referee_data_command_bit(command_id);
    }
    else
    {
        ++me->decode_error_count;
    }
}

bool module_referee_data_has_update(const module_referee_data_t *me, uint16_t command_id)
{
    const uint32_t command_bit = module_referee_data_command_bit(command_id);
    return (me != NULL) && (command_bit != 0U) && ((me->update_mask & command_bit) != 0U);
}

void module_referee_data_clear_updates(module_referee_data_t *me)
{
    if (me != NULL)
    {
        me->update_mask = 0U;
    }
}
