#ifndef MODULE_ROBOT_LINK_H
#define MODULE_ROBOT_LINK_H

#include "bsp_can.h"
#include "module_dr16.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

    typedef enum
    {
        MODULE_ROBOT_LINK_STATUS_OK = 0,
        MODULE_ROBOT_LINK_STATUS_INVALID_ARGUMENT,
        MODULE_ROBOT_LINK_STATUS_NOT_INITIALIZED,
        MODULE_ROBOT_LINK_STATUS_TRANSPORT_ERROR,
        MODULE_ROBOT_LINK_STATUS_INVALID_FRAME
    } module_robot_link_status_t;

    typedef enum
    {
        MODULE_ROBOT_LINK_MESSAGE_REMOTE_CHANNELS_PRIMARY = 0,
        MODULE_ROBOT_LINK_MESSAGE_REMOTE_CHANNELS_AUXILIARY,
        MODULE_ROBOT_LINK_MESSAGE_REMOTE_INPUT,
        MODULE_ROBOT_LINK_MESSAGE_GIMBAL_PRIMARY,
        MODULE_ROBOT_LINK_MESSAGE_GIMBAL_AUXILIARY,
        MODULE_ROBOT_LINK_MESSAGE_CHASSIS,
        MODULE_ROBOT_LINK_MESSAGE_SHOOTER,
        MODULE_ROBOT_LINK_MESSAGE_HEARTBEAT,
        MODULE_ROBOT_LINK_MESSAGE_COUNT
    } module_robot_link_message_t;

    typedef struct
    {
        float yaw_rad;
        float pitch_rad;
        float yaw_velocity_rad_per_s;
        float pitch_velocity_rad_per_s;
        bool imu_valid;
        bool motors_online;
    } module_robot_link_gimbal_data_t;

    typedef struct
    {
        float velocity_x_m_per_s;
        float velocity_y_m_per_s;
        float angular_velocity_rad_per_s;
        bool motors_online;
        bool self_lock_active;
    } module_robot_link_chassis_data_t;

    typedef struct
    {
        float friction_velocity_rad_per_s;
        float feeder_position_rad;
        uint8_t state;
        uint8_t jam_retry_count;
    } module_robot_link_shooter_data_t;

    typedef struct
    {
        bsp_can_t *can;
        uint32_t base_identifier;
        uint32_t transmit_timeout_ms;
        uint32_t offline_timeout_ms;
    } module_robot_link_config_t;

    typedef struct
    {
        bsp_can_t *can;
        uint32_t base_identifier;
        uint32_t transmit_timeout_ms;
        uint32_t offline_timeout_ms;
        module_dr16_data_t remote_data;
        module_dr16_data_t remote_staging;
        module_robot_link_gimbal_data_t gimbal_data;
        module_robot_link_gimbal_data_t gimbal_staging;
        module_robot_link_chassis_data_t chassis_data;
        module_robot_link_shooter_data_t shooter_data;
        uint32_t remote_elapsed_time_ms;
        uint32_t gimbal_elapsed_time_ms;
        uint32_t chassis_elapsed_time_ms;
        uint32_t shooter_elapsed_time_ms;
        uint8_t transmit_sequence;
        uint8_t remote_receive_mask;
        uint8_t remote_assembly_sequence;
        uint8_t gimbal_receive_mask;
        uint8_t gimbal_assembly_sequence;
        bool remote_online;
        bool gimbal_online;
        bool chassis_online;
        bool shooter_online;
        bool is_initialized;
    } module_robot_link_t;

    module_robot_link_status_t module_robot_link_init(module_robot_link_t *me,
                                                      const module_robot_link_config_t *config);
    module_robot_link_status_t module_robot_link_send_remote(module_robot_link_t *me,
                                                             const module_dr16_data_t *remote_data);
    module_robot_link_status_t
    module_robot_link_send_gimbal(module_robot_link_t *me,
                                  const module_robot_link_gimbal_data_t *gimbal_data);
    module_robot_link_status_t
    module_robot_link_send_chassis(module_robot_link_t *me,
                                   const module_robot_link_chassis_data_t *chassis_data);
    module_robot_link_status_t
    module_robot_link_send_shooter(module_robot_link_t *me,
                                   const module_robot_link_shooter_data_t *shooter_data);
    module_robot_link_status_t module_robot_link_send_heartbeat(module_robot_link_t *me,
                                                                uint8_t board_role,
                                                                uint32_t uptime_ms);
    module_robot_link_status_t module_robot_link_handle_frame(module_robot_link_t *me,
                                                              const bsp_can_frame_t *frame);
    void module_robot_link_update_time(module_robot_link_t *me, uint32_t elapsed_time_ms);
    const module_dr16_data_t *module_robot_link_get_remote(const module_robot_link_t *me);
    const module_robot_link_gimbal_data_t *
    module_robot_link_get_gimbal(const module_robot_link_t *me);
    const module_robot_link_chassis_data_t *
    module_robot_link_get_chassis(const module_robot_link_t *me);
    const module_robot_link_shooter_data_t *
    module_robot_link_get_shooter(const module_robot_link_t *me);

#ifdef __cplusplus
}
#endif

#endif /* MODULE_ROBOT_LINK_H */
