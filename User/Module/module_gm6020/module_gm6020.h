#ifndef MODULE_GM6020_H
#define MODULE_GM6020_H

#include "module_dji_motor.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct module_gm6020 module_gm6020_t;

    typedef enum
    {
        MODULE_GM6020_CONTROL_VOLTAGE = 0,
        MODULE_GM6020_CONTROL_VELOCITY,
        MODULE_GM6020_CONTROL_POSITION
    } module_gm6020_control_mode_t;

    typedef struct
    {
        const char *logical_name;
        uint32_t registration_key;
        module_dji_motor_bus_t *motor_bus;
        module_gm6020_control_mode_t control_mode;
        uint8_t motor_identifier;
        float direction_sign;
        float maximum_temperature_c;
        float current_scale_a_per_count;
        alg_pid_config_t velocity_pid_config;
        alg_pid_cascade_config_t position_pid_config;
    } module_gm6020_config_t;

    struct module_gm6020
    {
        module_dji_motor_t super;
    };

    module_motor_status_t module_gm6020_init(module_gm6020_t *const me,
                                             const module_gm6020_config_t *const config);
    module_motor_status_t module_gm6020_register(module_gm6020_t *const me,
                                                 module_motor_registry_t *const registry);
    module_motor_status_t module_gm6020_unregister(module_gm6020_t *const me,
                                                   module_motor_registry_t *const registry);
    module_motor_t *module_gm6020_as_motor(module_gm6020_t *const me);
    module_dji_motor_t *module_gm6020_as_dji_motor(module_gm6020_t *const me);
    module_motor_status_t module_gm6020_enable(module_gm6020_t *const me);
    module_motor_status_t module_gm6020_disable(module_gm6020_t *const me);
    module_motor_status_t module_gm6020_set_voltage_command_raw(module_gm6020_t *const me,
                                                                int16_t voltage_command_raw);
    module_motor_status_t module_gm6020_set_velocity_rad_per_s(module_gm6020_t *const me,
                                                               float velocity_rad_per_s);
    module_motor_status_t module_gm6020_set_position_rad(module_gm6020_t *const me,
                                                         float position_rad);
    module_motor_status_t module_gm6020_update(module_gm6020_t *const me, float delta_time_s);
    const module_motor_feedback_t *module_gm6020_get_feedback(const module_gm6020_t *const me);
    int16_t module_gm6020_get_voltage_command_raw(const module_gm6020_t *const me);

#ifdef __cplusplus
}
#endif

#endif /* MODULE_GM6020_H */
