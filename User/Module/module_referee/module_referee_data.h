#ifndef MODULE_REFEREE_DATA_H
#define MODULE_REFEREE_DATA_H

#include "module_referee.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define MODULE_REFEREE_COMMAND_GAME_STATUS (0x0001U)
#define MODULE_REFEREE_COMMAND_GAME_RESULT (0x0002U)
#define MODULE_REFEREE_COMMAND_ROBOT_HEALTH (0x0003U)
#define MODULE_REFEREE_COMMAND_EVENT_DATA (0x0101U)
#define MODULE_REFEREE_COMMAND_REFEREE_WARNING (0x0104U)
#define MODULE_REFEREE_COMMAND_ROBOT_STATUS (0x0201U)
#define MODULE_REFEREE_COMMAND_POWER_HEAT (0x0202U)
#define MODULE_REFEREE_COMMAND_ROBOT_POSITION (0x0203U)
#define MODULE_REFEREE_COMMAND_ROBOT_BUFF (0x0204U)
#define MODULE_REFEREE_COMMAND_ROBOT_HURT (0x0206U)
#define MODULE_REFEREE_COMMAND_SHOOT_DATA (0x0207U)
#define MODULE_REFEREE_COMMAND_PROJECTILE_ALLOWANCE (0x0208U)
#define MODULE_REFEREE_COMMAND_RFID_STATUS (0x0209U)

    typedef struct
    {
        uint8_t game_type;
        uint8_t game_progress;
        uint16_t remaining_time_s;
        uint64_t synchronization_timestamp_us;
    } module_referee_game_status_t;

    typedef struct
    {
        uint8_t robot_id;
        uint8_t robot_level;
        uint16_t current_hp;
        uint16_t maximum_hp;
        uint16_t shooter_barrel_cooling_rate;
        uint16_t shooter_barrel_heat_limit;
        uint16_t chassis_power_limit_w;
        bool gimbal_power_enabled;
        bool chassis_power_enabled;
        bool shooter_power_enabled;
    } module_referee_robot_status_t;

    typedef struct
    {
        uint16_t chassis_voltage_mv;
        uint16_t chassis_current_ma;
        float chassis_power_w;
        uint16_t chassis_buffer_energy_j;
        uint16_t shooter_17_mm_1_heat;
        uint16_t shooter_17_mm_2_heat;
        uint16_t shooter_42_mm_heat;
    } module_referee_power_heat_t;

    typedef struct
    {
        float position_x_m;
        float position_y_m;
        float yaw_rad;
    } module_referee_robot_position_t;

    typedef struct
    {
        uint8_t armor_id;
        uint8_t hurt_type;
    } module_referee_hurt_t;

    typedef struct
    {
        uint8_t bullet_type;
        uint8_t shooter_number;
        uint8_t frequency_hz;
        float speed_m_per_s;
    } module_referee_shoot_data_t;

    typedef struct
    {
        uint16_t projectile_17_mm_remaining;
        uint16_t projectile_42_mm_remaining;
        uint16_t coin_remaining;
    } module_referee_projectile_allowance_t;

    typedef struct
    {
        module_referee_game_status_t game_status;
        module_referee_robot_status_t robot_status;
        module_referee_power_heat_t power_heat;
        module_referee_robot_position_t robot_position;
        module_referee_hurt_t hurt;
        module_referee_shoot_data_t shoot_data;
        module_referee_projectile_allowance_t projectile_allowance;
        uint32_t robot_health[16];
        uint32_t event_data;
        uint32_t rfid_status;
        uint32_t update_mask;
        uint32_t decode_error_count;
        uint8_t game_result;
        uint8_t warning_level;
        uint8_t warning_robot_id;
    } module_referee_data_t;

    void module_referee_data_reset(module_referee_data_t *me);
    void module_referee_data_route_handler(uint16_t command_id, const uint8_t *payload,
                                           size_t payload_size, uint8_t sequence,
                                           void *user_context);
    bool module_referee_data_has_update(const module_referee_data_t *me, uint16_t command_id);
    void module_referee_data_clear_updates(module_referee_data_t *me);

#ifdef __cplusplus
}
#endif

#endif
