#ifndef MODULE_SWERVE_H
#define MODULE_SWERVE_H

#include "alg_swerve.h"
#include "module_motor.h"

#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

    typedef enum
    {
        MODULE_SWERVE_STATUS_OK = 0,
        MODULE_SWERVE_STATUS_INVALID_ARGUMENT,
        MODULE_SWERVE_STATUS_NOT_INITIALIZED,
        MODULE_SWERVE_STATUS_NOT_READY,
        MODULE_SWERVE_STATUS_ALGORITHM_ERROR,
        MODULE_SWERVE_STATUS_MOTOR_ERROR
    } module_swerve_status_t;

    typedef struct
    {
        module_motor_t *drive_motor;
        module_motor_t *steering_motor;
        float wheel_radius_m;
        float drive_reduction_ratio;
        float steering_zero_offset_rad;
        float drive_direction_sign;
        float steering_direction_sign;
    } module_swerve_config_t;

    typedef struct
    {
        module_motor_t *drive_motor;
        module_motor_t *steering_motor;
        float wheel_radius_m;
        float drive_reduction_ratio;
        float steering_zero_offset_rad;
        float drive_direction_sign;
        float steering_direction_sign;
        bool is_enabled;
        bool is_initialized;
    } module_swerve_t;

    module_swerve_status_t module_swerve_init(
        module_swerve_t *me, const module_swerve_config_t *config);
    module_swerve_status_t module_swerve_enable(module_swerve_t *me);
    module_swerve_status_t module_swerve_disable(module_swerve_t *me);
    module_swerve_status_t module_swerve_apply_target(
        module_swerve_t *me, const alg_swerve_module_target_t *target,
        float delta_time_s);
    module_swerve_status_t module_swerve_get_steering_angle(
        const module_swerve_t *me, float *steering_angle_rad);

#ifdef __cplusplus
}
#endif

#endif /* MODULE_SWERVE_H */
