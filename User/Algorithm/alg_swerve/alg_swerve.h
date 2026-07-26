#ifndef ALG_SWERVE_H
#define ALG_SWERVE_H

#include "alg_chassis.h"

#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C"
{
#endif

    typedef enum
    {
        ALG_SWERVE_STATUS_OK = 0,
        ALG_SWERVE_STATUS_DEGRADED,
        ALG_SWERVE_STATUS_INVALID_ARGUMENT,
        ALG_SWERVE_STATUS_NOT_INITIALIZED
    } alg_swerve_status_t;

    typedef struct
    {
        float position_x_m;
        float position_y_m;
    } alg_swerve_module_geometry_t;

    typedef struct
    {
        float velocity_x_m_per_s;
        float velocity_y_m_per_s;
        float angular_velocity_rad_per_s;
        float reference_heading_rad;
        float center_of_rotation_x_m;
        float center_of_rotation_y_m;
        bool command_is_reference_relative;
    } alg_swerve_command_t;

    typedef struct
    {
        float wheel_velocity_m_per_s;
        float steering_angle_rad;
    } alg_swerve_module_target_t;

    typedef enum
    {
        ALG_SWERVE_MODULE_FRONT_LEFT = 0,
        ALG_SWERVE_MODULE_FRONT_RIGHT,
        ALG_SWERVE_MODULE_REAR_LEFT,
        ALG_SWERVE_MODULE_REAR_RIGHT,
        ALG_SWERVE_RECTANGULAR_MODULE_COUNT
    } alg_swerve_rectangular_module_index_t;

    typedef struct
    {
        const alg_swerve_module_geometry_t *module_geometry;
        size_t module_count;
        float maximum_wheel_velocity_m_per_s;
        bool is_initialized;
    } alg_swerve_t;

    alg_swerve_status_t alg_swerve_init(
        alg_swerve_t *me, const alg_swerve_module_geometry_t *module_geometry,
        size_t module_count, float maximum_wheel_velocity_m_per_s);
    alg_swerve_status_t alg_swerve_configure_rectangular_layout(
        alg_swerve_module_geometry_t
            module_geometry[ALG_SWERVE_RECTANGULAR_MODULE_COUNT],
        float half_wheelbase_m, float half_track_width_m);
    alg_swerve_status_t alg_swerve_calculate(
        const alg_swerve_t *me, const alg_swerve_command_t *command,
        alg_swerve_module_target_t *module_targets, size_t target_capacity);
    alg_swerve_status_t alg_swerve_calculate_with_availability(
        const alg_swerve_t *me, const alg_swerve_command_t *command,
        const bool *module_is_available,
        alg_swerve_module_target_t *module_targets,
        size_t target_capacity);
    alg_chassis_status_t alg_swerve_forward(
        const alg_swerve_t *me,
        const alg_swerve_module_target_t *measured_module_states,
        const bool *module_is_available,
        const float *odometry_weights, uint8_t known_component_mask,
        const alg_chassis_velocity_t *known_velocity,
        alg_chassis_constraint_t *constraint_workspace,
        size_t workspace_capacity, alg_chassis_solution_t *solution);
    alg_swerve_status_t alg_swerve_optimize_target(
        float current_steering_angle_rad,
        alg_swerve_module_target_t *module_target);
    alg_swerve_status_t alg_swerve_calculate_self_lock(
        const alg_swerve_t *me, alg_swerve_module_target_t *module_targets,
        size_t target_capacity);
    float alg_swerve_wrap_angle_rad(float angle_rad);

#ifdef __cplusplus
}
#endif

#endif /* ALG_SWERVE_H */
