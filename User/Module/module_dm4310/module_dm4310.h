#ifndef MODULE_DM4310_H
#define MODULE_DM4310_H

#include "module_dm_motor.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct module_dm4310 module_dm4310_t;

    typedef enum
    {
        MODULE_DM4310_CONTROL_MIT = 0,
        MODULE_DM4310_CONTROL_VELOCITY,
        MODULE_DM4310_CONTROL_POSITION_VELOCITY
    } module_dm4310_control_mode_t;

    typedef struct
    {
        const char *logical_name;
        uint32_t registration_key;
        bsp_can_t *can;
        module_dm4310_control_mode_t control_mode;
        uint32_t base_command_identifier;
        uint32_t feedback_identifier;
        uint32_t transmit_timeout_ms;
        module_dm_limits_t protocol_limits;
    } module_dm4310_config_t;

    struct module_dm4310
    {
        module_dm_motor_t super;
    };

    module_motor_status_t module_dm4310_init(module_dm4310_t *const me,
                                             const module_dm4310_config_t *const config);
    module_motor_status_t module_dm4310_register(module_dm4310_t *const me,
                                                 module_motor_registry_t *const registry);
    module_motor_status_t module_dm4310_unregister(
        module_dm4310_t *const me,
        module_motor_registry_t *const registry);
    module_motor_t *module_dm4310_as_motor(module_dm4310_t *const me);
    module_dm_motor_t *module_dm4310_as_dm_motor(module_dm4310_t *const me);
    module_motor_status_t module_dm4310_enable(module_dm4310_t *const me);
    module_motor_status_t module_dm4310_disable(module_dm4310_t *const me);
    module_motor_status_t module_dm4310_command_mit(module_dm4310_t *const me,
                                                    const module_dm_mit_command_t *const command);
    module_motor_status_t module_dm4310_command_velocity(module_dm4310_t *const me,
                                                         float velocity_rad_per_s);
    module_motor_status_t module_dm4310_command_position_velocity(module_dm4310_t *const me,
                                                                  float position_rad,
                                                                  float velocity_rad_per_s);
    module_motor_status_t module_dm4310_save_zero_position(module_dm4310_t *const me);
    module_motor_status_t module_dm4310_clear_fault(module_dm4310_t *const me);
    module_motor_status_t module_dm4310_handle_feedback(module_dm4310_t *const me,
                                                        const bsp_can_frame_t *const frame);
    const module_motor_feedback_t *module_dm4310_get_feedback(const module_dm4310_t *const me);
    module_dm_fault_t module_dm4310_get_fault(const module_dm4310_t *const me);
    float module_dm4310_get_mos_temperature_c(const module_dm4310_t *const me);

#ifdef __cplusplus
}
#endif

#endif /* MODULE_DM4310_H */
