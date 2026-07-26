#ifndef MODULE_MOTOR_HEALTH_H
#define MODULE_MOTOR_HEALTH_H

#include "module_motor.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define MODULE_MOTOR_HEALTH_REASON_NONE (0U)
#define MODULE_MOTOR_HEALTH_REASON_NOT_REGISTERED (1U << 0)
#define MODULE_MOTOR_HEALTH_REASON_OFFLINE (1U << 1)
#define MODULE_MOTOR_HEALTH_REASON_MOTOR_FAULT (1U << 2)
#define MODULE_MOTOR_HEALTH_REASON_NOT_ENABLED (1U << 3)
#define MODULE_MOTOR_HEALTH_REASON_OVER_TEMPERATURE (1U << 4)

    typedef enum
    {
        MODULE_MOTOR_HEALTH_STATUS_OK = 0,
        MODULE_MOTOR_HEALTH_STATUS_DEGRADED,
        MODULE_MOTOR_HEALTH_STATUS_INVALID_ARGUMENT,
        MODULE_MOTOR_HEALTH_STATUS_NOT_INITIALIZED,
        MODULE_MOTOR_HEALTH_STATUS_MOTOR_ERROR
    } module_motor_health_status_t;

    typedef struct
    {
        uint32_t reason_mask;
        uint32_t fault_elapsed_time_ms;
        uint32_t recovery_elapsed_time_ms;
        bool is_available;
    } module_motor_health_state_t;

    typedef struct
    {
        module_motor_t *const *motors;
        size_t motor_count;
        module_motor_health_state_t *state_storage;
        const float *maximum_temperature_c;
        uint32_t fault_confirmation_time_ms;
        uint32_t recovery_confirmation_time_ms;
        bool require_enabled_state;
        bool manage_feedback_time;
    } module_motor_health_config_t;

    typedef struct
    {
        module_motor_t *const *motors;
        size_t motor_count;
        module_motor_health_state_t *states;
        const float *maximum_temperature_c;
        uint32_t fault_confirmation_time_ms;
        uint32_t recovery_confirmation_time_ms;
        bool require_enabled_state;
        bool manage_feedback_time;
        bool is_initialized;
    } module_motor_health_t;

    module_motor_health_status_t module_motor_health_init(
        module_motor_health_t *me,
        const module_motor_health_config_t *config);
    module_motor_health_status_t module_motor_health_update(
        module_motor_health_t *me, uint32_t elapsed_time_ms);
    module_motor_health_status_t module_motor_health_get_availability(
        const module_motor_health_t *me, bool *motor_is_available,
        size_t output_capacity);
    const module_motor_health_state_t *module_motor_health_get_state(
        const module_motor_health_t *me, size_t motor_index);

#ifdef __cplusplus
}
#endif

#endif /* MODULE_MOTOR_HEALTH_H */
