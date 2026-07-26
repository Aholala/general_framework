#ifndef MODULE_MOTOR_H
#define MODULE_MOTOR_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define MODULE_MOTOR_CONTAINER_OF(member_pointer, parent_type, member_name)                        \
    ((parent_type *)((uint8_t *)(member_pointer) - offsetof(parent_type, member_name)))

    typedef struct module_motor module_motor_t;
    typedef struct module_motor_registry module_motor_registry_t;

    typedef enum
    {
        MODULE_MOTOR_STATUS_OK = 0,
        MODULE_MOTOR_STATUS_INVALID_ARGUMENT,
        MODULE_MOTOR_STATUS_NOT_INITIALIZED,
        MODULE_MOTOR_STATUS_NOT_REGISTERED,
        MODULE_MOTOR_STATUS_ALREADY_REGISTERED,
        MODULE_MOTOR_STATUS_DUPLICATE_KEY,
        MODULE_MOTOR_STATUS_NO_RESOURCE,
        MODULE_MOTOR_STATUS_OUT_OF_RANGE,
        MODULE_MOTOR_STATUS_UNSUPPORTED,
        MODULE_MOTOR_STATUS_TRANSPORT_ERROR,
        MODULE_MOTOR_STATUS_FEEDBACK_UNAVAILABLE
    } module_motor_status_t;

    typedef enum
    {
        MODULE_MOTOR_STATE_DISABLED = 0,
        MODULE_MOTOR_STATE_ENABLED,
        MODULE_MOTOR_STATE_FAULT
    } module_motor_state_t;

    typedef struct
    {
        float position_rad;
        float velocity_rad_per_s;
        float torque_nm;
        float current_a;
        float motor_temperature_c;
        int16_t current_raw;
        uint32_t raw_position;
        uint32_t update_count;
        uint32_t elapsed_time_since_update_ms;
        bool is_current_a_valid;
        bool is_online;
    } module_motor_feedback_t;

    typedef struct
    {
        module_motor_status_t (*enable)(module_motor_t *const me);
        module_motor_status_t (*disable)(module_motor_t *const me);
        module_motor_status_t (*set_target)(module_motor_t *const me, float target_value);
        module_motor_status_t (*update)(module_motor_t *const me, float delta_time_s);
    } module_motor_ops_t;

    struct module_motor
    {
        const module_motor_ops_t *vptr;
        const char *logical_name;
        uint32_t registration_key;
        size_t registry_index;
        module_motor_state_t state;
        module_motor_feedback_t feedback;
        uint32_t feedback_timeout_ms;
        bool is_initialized;
        bool is_registered;
    };

    struct module_motor_registry
    {
        module_motor_t **motor_storage;
        size_t motor_capacity;
        size_t motor_count;
        bool is_initialized;
    };

    module_motor_status_t module_motor_init_base(module_motor_t *const me,
                                                 const module_motor_ops_t *const vptr,
                                                 const char *const logical_name,
                                                 uint32_t registration_key);
    module_motor_status_t module_motor_registry_init(module_motor_registry_t *const me,
                                                     module_motor_t **const motor_storage,
                                                     size_t motor_capacity);
    module_motor_status_t module_motor_registry_register(module_motor_registry_t *const me,
                                                         module_motor_t *const motor);
    module_motor_status_t module_motor_registry_unregister(module_motor_registry_t *const me,
                                                           module_motor_t *const motor);
    module_motor_t *module_motor_registry_find(const module_motor_registry_t *const me,
                                               uint32_t registration_key);
    size_t module_motor_registry_get_count(const module_motor_registry_t *const me);
    module_motor_status_t module_motor_enable(module_motor_t *const me);
    module_motor_status_t module_motor_disable(module_motor_t *const me);
    module_motor_status_t module_motor_clear_fault(module_motor_t *const me);
    module_motor_status_t module_motor_set_target(module_motor_t *const me, float target_value);
    module_motor_status_t module_motor_update(module_motor_t *const me, float delta_time_s);
    module_motor_status_t module_motor_set_feedback_timeout(module_motor_t *const me,
                                                            uint32_t feedback_timeout_ms);
    module_motor_status_t module_motor_update_feedback_time(module_motor_t *const me,
                                                            uint32_t elapsed_time_ms);
    module_motor_status_t module_motor_notify_feedback(module_motor_t *const me);
    const module_motor_feedback_t *module_motor_get_feedback(const module_motor_t *const me);

#ifdef __cplusplus
}
#endif

#endif /* MODULE_MOTOR_H */
