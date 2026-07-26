#ifndef MODULE_SHOOTER_H
#define MODULE_SHOOTER_H

#include "module_motor.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

    typedef enum
    {
        MODULE_SHOOTER_STATUS_OK = 0,
        MODULE_SHOOTER_STATUS_INVALID_ARGUMENT,
        MODULE_SHOOTER_STATUS_NOT_INITIALIZED,
        MODULE_SHOOTER_STATUS_NOT_READY,
        MODULE_SHOOTER_STATUS_MOTOR_ERROR,
        MODULE_SHOOTER_STATUS_FAULT
    } module_shooter_status_t;

    typedef enum
    {
        MODULE_SHOOTER_STATE_DISABLED = 0,
        MODULE_SHOOTER_STATE_READY,
        MODULE_SHOOTER_STATE_FEEDING,
        MODULE_SHOOTER_STATE_ROLLBACK,
        MODULE_SHOOTER_STATE_FAULT
    } module_shooter_state_t;

    typedef struct
    {
        module_motor_t *left_friction_motor;
        module_motor_t *right_friction_motor;
        module_motor_t *feeder_motor;
        float left_friction_direction_sign;
        float right_friction_direction_sign;
        float feeder_direction_sign;
        float feeder_step_rad;
        float feeder_position_tolerance_rad;
        float jam_velocity_threshold_rad_per_s;
        float jam_current_threshold_a;
        int16_t jam_current_threshold_raw;
        float jam_confirmation_time_s;
        float rollback_angle_rad;
        float rollback_position_tolerance_rad;
        uint8_t maximum_jam_retries;
        uint16_t maximum_pending_shots;
    } module_shooter_config_t;

    typedef struct
    {
        module_motor_t *left_friction_motor;
        module_motor_t *right_friction_motor;
        module_motor_t *feeder_motor;
        float left_friction_direction_sign;
        float right_friction_direction_sign;
        float feeder_direction_sign;
        float feeder_step_rad;
        float feeder_position_tolerance_rad;
        float jam_velocity_threshold_rad_per_s;
        float jam_current_threshold_a;
        int16_t jam_current_threshold_raw;
        float jam_confirmation_time_s;
        float rollback_angle_rad;
        float rollback_position_tolerance_rad;
        float friction_target_velocity_rad_per_s;
        float feeder_target_position_rad;
        float jam_elapsed_time_s;
        uint16_t pending_shots;
        uint16_t maximum_pending_shots;
        uint8_t jam_retry_count;
        uint8_t maximum_jam_retries;
        module_shooter_state_t state;
        bool friction_enabled;
        bool is_initialized;
    } module_shooter_t;

    module_shooter_status_t module_shooter_init(
        module_shooter_t *me, const module_shooter_config_t *config);
    module_shooter_status_t module_shooter_enable(module_shooter_t *me);
    module_shooter_status_t module_shooter_disable(module_shooter_t *me);
    module_shooter_status_t module_shooter_set_friction(
        module_shooter_t *me, bool is_enabled,
        float target_velocity_rad_per_s);
    module_shooter_status_t module_shooter_request_shots(
        module_shooter_t *me, uint16_t shot_count);
    module_shooter_status_t module_shooter_cancel_shots(
        module_shooter_t *me);
    module_shooter_status_t module_shooter_reset_fault(
        module_shooter_t *me);
    module_shooter_status_t module_shooter_update(module_shooter_t *me,
                                                   float delta_time_s);
    module_shooter_state_t module_shooter_get_state(
        const module_shooter_t *me);
    uint16_t module_shooter_get_pending_shots(
        const module_shooter_t *me);
    uint8_t module_shooter_get_jam_retry_count(
        const module_shooter_t *me);

#ifdef __cplusplus
}
#endif

#endif /* MODULE_SHOOTER_H */
