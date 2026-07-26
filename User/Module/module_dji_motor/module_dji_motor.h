#ifndef MODULE_DJI_MOTOR_H
#define MODULE_DJI_MOTOR_H

#include "alg_pid.h"
#include "bsp_can.h"
#include "module_motor.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define MODULE_DJI_MOTOR_GROUP_COUNT (3U)
#define MODULE_DJI_MOTOR_PER_GROUP (4U)

    typedef struct module_dji_motor module_dji_motor_t;

    typedef enum
    {
        MODULE_DJI_MOTOR_M2006 = 0,
        MODULE_DJI_MOTOR_M3508,
        MODULE_DJI_MOTOR_GM6020
    } module_dji_motor_model_t;

    typedef enum
    {
        MODULE_DJI_CONTROL_DIRECT = 0,
        MODULE_DJI_CONTROL_VELOCITY,
        MODULE_DJI_CONTROL_POSITION
    } module_dji_control_mode_t;

    typedef struct
    {
        bsp_can_t *can;
        module_dji_motor_t *motor_slots[MODULE_DJI_MOTOR_GROUP_COUNT][MODULE_DJI_MOTOR_PER_GROUP];
        bool group_is_used[MODULE_DJI_MOTOR_GROUP_COUNT];
        uint32_t transmit_timeout_ms;
        bool is_initialized;
    } module_dji_motor_bus_t;

    typedef struct
    {
        const char *logical_name;
        uint32_t registration_key;
        module_dji_motor_bus_t *motor_bus;
        module_dji_motor_model_t motor_model;
        module_dji_control_mode_t control_mode;
        uint8_t motor_identifier;
        float direction_sign;
        float maximum_temperature_c;
        float current_scale_a_per_count;
        alg_pid_config_t velocity_pid_config;
        alg_pid_cascade_config_t position_pid_config;
    } module_dji_motor_config_t;

    struct module_dji_motor
    {
        module_motor_t super;
        module_dji_motor_bus_t *motor_bus;
        module_dji_motor_model_t motor_model;
        module_dji_control_mode_t control_mode;
        alg_pid_t velocity_controller;
        alg_pid_cascade_t position_controller;
        float target_value;
        float direction_sign;
        float gear_ratio;
        float maximum_temperature_c;
        float current_scale_a_per_count;
        int16_t command_value;
        int16_t maximum_command_value;
        uint16_t previous_encoder_count;
        int64_t accumulated_encoder_count;
        uint32_t receive_identifier;
        uint8_t group_index;
        uint8_t group_slot;
        bool has_previous_encoder_count;
    };

    module_motor_status_t module_dji_motor_bus_init(module_dji_motor_bus_t *const me,
                                                    bsp_can_t *const can,
                                                    uint32_t transmit_timeout_ms);
    module_motor_status_t module_dji_motor_init(module_dji_motor_t *const me,
                                                const module_dji_motor_config_t *const config);
    module_motor_status_t module_dji_motor_register(module_dji_motor_t *const me,
                                                    module_motor_registry_t *const registry);
    module_motor_status_t module_dji_motor_unregister(
        module_dji_motor_t *const me,
        module_motor_registry_t *const registry);
    module_motor_t *module_dji_motor_as_base(module_dji_motor_t *const me);
    module_motor_status_t module_dji_motor_bus_handle_feedback(module_dji_motor_bus_t *const me,
                                                               const bsp_can_frame_t *const frame);
    module_motor_status_t module_dji_motor_bus_flush(module_dji_motor_bus_t *const me);
    int16_t module_dji_motor_get_command(const module_dji_motor_t *const me);

#ifdef __cplusplus
}
#endif

#endif /* MODULE_DJI_MOTOR_H */
