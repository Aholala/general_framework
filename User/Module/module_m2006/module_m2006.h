#ifndef MODULE_M2006_H
#define MODULE_M2006_H

#include "module_dji_motor.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct module_m2006 module_m2006_t;

    typedef enum
    {
        MODULE_M2006_CONTROL_CURRENT = 0,
        MODULE_M2006_CONTROL_VELOCITY,
        MODULE_M2006_CONTROL_POSITION
    } module_m2006_control_mode_t;

    typedef struct
    {
        const char *logical_name;
        uint32_t registration_key;
        module_dji_motor_bus_t *motor_bus;
        module_m2006_control_mode_t control_mode;
        uint8_t motor_identifier;
        float direction_sign;
        float maximum_temperature_c;
        float current_scale_a_per_count;
        alg_pid_config_t velocity_pid_config;
        alg_pid_cascade_config_t position_pid_config;
    } module_m2006_config_t;

    struct module_m2006
    {
        module_dji_motor_t super;
    };

    module_motor_status_t module_m2006_init(module_m2006_t *const me,
                                            const module_m2006_config_t *const config);
    module_motor_status_t module_m2006_register(module_m2006_t *const me,
                                                module_motor_registry_t *const registry);
    module_motor_status_t module_m2006_unregister(
        module_m2006_t *const me,
        module_motor_registry_t *const registry);
    module_motor_t *module_m2006_as_motor(module_m2006_t *const me);
    module_dji_motor_t *module_m2006_as_dji_motor(module_m2006_t *const me);
    module_motor_status_t module_m2006_enable(module_m2006_t *const me);
    module_motor_status_t module_m2006_disable(module_m2006_t *const me);
    module_motor_status_t module_m2006_set_current_command_raw(module_m2006_t *const me,
                                                               int16_t current_command_raw);
    module_motor_status_t module_m2006_set_velocity_rad_per_s(module_m2006_t *const me,
                                                              float velocity_rad_per_s);
    module_motor_status_t module_m2006_set_position_rad(module_m2006_t *const me,
                                                        float position_rad);
    module_motor_status_t module_m2006_update(module_m2006_t *const me, float delta_time_s);
    const module_motor_feedback_t *module_m2006_get_feedback(const module_m2006_t *const me);
    int16_t module_m2006_get_current_command_raw(const module_m2006_t *const me);

#ifdef __cplusplus
}
#endif

#endif /* MODULE_M2006_H */
