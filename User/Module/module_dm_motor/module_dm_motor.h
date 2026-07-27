#ifndef MODULE_DM_MOTOR_H
#define MODULE_DM_MOTOR_H

#include "bsp_can.h"
#include "module_motor.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct module_dm_motor module_dm_motor_t;

    typedef enum
    {
        MODULE_DM_MODE_MIT = 0,
        MODULE_DM_MODE_VELOCITY,
        MODULE_DM_MODE_POSITION_VELOCITY
    } module_dm_control_mode_t;

    typedef enum
    {
        MODULE_DM_COMMAND_DISABLE = 0,
        MODULE_DM_COMMAND_ENABLE,
        MODULE_DM_COMMAND_SAVE_ZERO,
        MODULE_DM_COMMAND_CLEAR_FAULT
    } module_dm_state_command_t;

    typedef enum
    {
        MODULE_DM_FAULT_NONE = 0,
        MODULE_DM_FAULT_OVER_VOLTAGE = 8,
        MODULE_DM_FAULT_UNDER_VOLTAGE = 9,
        MODULE_DM_FAULT_OVER_CURRENT = 10,
        MODULE_DM_FAULT_MOS_OVER_TEMPERATURE = 11,
        MODULE_DM_FAULT_MOTOR_OVER_TEMPERATURE = 12,
        MODULE_DM_FAULT_COMMUNICATION_LOST = 13,
        MODULE_DM_FAULT_OVERLOAD = 14
    } module_dm_fault_t;

    typedef struct
    {
        float position_min_rad;
        float position_max_rad;
        float velocity_min_rad_per_s;
        float velocity_max_rad_per_s;
        float torque_min_nm;
        float torque_max_nm;
        float proportional_gain_min;
        float proportional_gain_max;
        float derivative_gain_min;
        float derivative_gain_max;
    } module_dm_limits_t;

    typedef struct
    {
        float position_rad;
        float velocity_rad_per_s;
        float proportional_gain;
        float derivative_gain;
        float torque_nm;
    } module_dm_mit_command_t;

    typedef struct
    {
        module_motor_status_t (*encode_command)(module_dm_motor_t *const me,
                                                uint8_t transmit_data[8]);
        uint32_t (*get_transmit_identifier)(const module_dm_motor_t *const me);
    } module_dm_mode_ops_t;

    typedef struct
    {
        const char *logical_name;
        uint32_t registration_key;
        bsp_can_t *can;
        module_dm_control_mode_t control_mode;
        uint32_t master_identifier;
        uint32_t feedback_identifier;
        uint32_t transmit_timeout_ms;
        module_dm_limits_t limits;
    } module_dm_motor_config_t;

    struct module_dm_motor
    {
        module_motor_t super;
        const module_dm_mode_ops_t *mode_vptr;
        bsp_can_t *can;
        module_dm_control_mode_t control_mode;
        module_dm_limits_t limits;
        module_dm_mit_command_t mit_command;
        float target_position_rad;
        float target_velocity_rad_per_s;
        uint32_t master_identifier;
        uint32_t feedback_identifier;
        uint32_t transmit_timeout_ms;
        module_dm_fault_t fault;
        float mos_temperature_c;
    };

    module_motor_status_t module_dm_motor_init(module_dm_motor_t *const me,
                                               const module_dm_motor_config_t *const config);
    module_motor_status_t module_dm_motor_register(module_dm_motor_t *const me,
                                                   module_motor_registry_t *const registry);
    module_motor_status_t module_dm_motor_unregister(module_dm_motor_t *const me,
                                                     module_motor_registry_t *const registry);
    module_motor_t *module_dm_motor_as_base(module_dm_motor_t *const me);
    module_motor_status_t module_dm_motor_send_state_command(module_dm_motor_t *const me,
                                                             module_dm_state_command_t command);
    module_motor_status_t module_dm_motor_command_mit(module_dm_motor_t *const me,
                                                      const module_dm_mit_command_t *const command);
    module_motor_status_t module_dm_motor_command_velocity(module_dm_motor_t *const me,
                                                           float velocity_rad_per_s);
    module_motor_status_t module_dm_motor_command_position_velocity(module_dm_motor_t *const me,
                                                                    float position_rad,
                                                                    float velocity_rad_per_s);
    module_motor_status_t
    module_dm_motor_set_mit_target(module_dm_motor_t *const me,
                                   const module_dm_mit_command_t *const command);
    module_motor_status_t module_dm_motor_set_velocity_target(module_dm_motor_t *const me,
                                                              float velocity_rad_per_s);
    module_motor_status_t module_dm_motor_set_position_velocity_target(module_dm_motor_t *const me,
                                                                       float position_rad,
                                                                       float velocity_rad_per_s);
    module_motor_status_t module_dm_motor_handle_feedback(module_dm_motor_t *const me,
                                                          const bsp_can_frame_t *const frame);
    module_dm_fault_t module_dm_motor_get_fault(const module_dm_motor_t *const me);
    float module_dm_motor_get_mos_temperature_c(const module_dm_motor_t *const me);

#ifdef __cplusplus
}
#endif

#endif /* MODULE_DM_MOTOR_H */
