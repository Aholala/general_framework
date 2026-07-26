#ifndef ALG_CHASSIS_H
#define ALG_CHASSIS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define ALG_CHASSIS_COMPONENT_VELOCITY_X (1U << 0)
#define ALG_CHASSIS_COMPONENT_VELOCITY_Y (1U << 1)
#define ALG_CHASSIS_COMPONENT_ANGULAR_VELOCITY (1U << 2)
#define ALG_CHASSIS_COMPONENT_ALL                                                                  \
    (ALG_CHASSIS_COMPONENT_VELOCITY_X | ALG_CHASSIS_COMPONENT_VELOCITY_Y |                         \
     ALG_CHASSIS_COMPONENT_ANGULAR_VELOCITY)

    typedef enum
    {
        ALG_CHASSIS_STATUS_OK = 0,
        ALG_CHASSIS_STATUS_DEGRADED,
        ALG_CHASSIS_STATUS_INVALID_ARGUMENT,
        ALG_CHASSIS_STATUS_NOT_INITIALIZED,
        ALG_CHASSIS_STATUS_UNDERDETERMINED,
        ALG_CHASSIS_STATUS_SINGULAR,
        ALG_CHASSIS_STATUS_NUMERICAL_ERROR
    } alg_chassis_status_t;

    typedef struct
    {
        float velocity_x_m_per_s;
        float velocity_y_m_per_s;
        float angular_velocity_rad_per_s;
    } alg_chassis_velocity_t;

    typedef struct
    {
        float position_x_m;
        float position_y_m;
        float heading_rad;
    } alg_chassis_pose_t;

    typedef enum
    {
        ALG_CHASSIS_INTEGRATION_EULER = 0,
        ALG_CHASSIS_INTEGRATION_MIDPOINT,
        ALG_CHASSIS_INTEGRATION_EXACT
    } alg_chassis_integration_method_t;

    typedef struct
    {
        float velocity_x_coefficient;
        float velocity_y_coefficient;
        float angular_velocity_coefficient_m;
        float measured_velocity_m_per_s;
        float weight;
        bool is_available;
    } alg_chassis_constraint_t;

    typedef struct
    {
        alg_chassis_velocity_t velocity;
        float residual_root_mean_square_m_per_s;
        size_t used_constraint_count;
        size_t unknown_component_count;
        bool is_degraded;
    } alg_chassis_solution_t;

    alg_chassis_status_t alg_chassis_solve_velocity(const alg_chassis_constraint_t *constraints,
                                                    size_t constraint_count,
                                                    uint8_t known_component_mask,
                                                    const alg_chassis_velocity_t *known_velocity,
                                                    size_t nominal_constraint_count,
                                                    alg_chassis_solution_t *solution);
    alg_chassis_status_t alg_chassis_calculate_constraint_residuals(
        const alg_chassis_constraint_t *constraints, size_t constraint_count,
        const alg_chassis_velocity_t *velocity, float *residuals_m_per_s, size_t residual_capacity);
    alg_chassis_status_t
    alg_chassis_transform_reference_to_body(const alg_chassis_velocity_t *reference_velocity,
                                            float reference_heading_rad,
                                            alg_chassis_velocity_t *body_velocity);
    alg_chassis_status_t alg_chassis_convert_center_velocity_to_origin(
        const alg_chassis_velocity_t *center_velocity, float center_of_rotation_x_m,
        float center_of_rotation_y_m, alg_chassis_velocity_t *origin_velocity);
    alg_chassis_status_t alg_chassis_scale_wheel_velocities(float *wheel_velocities,
                                                            const bool *wheel_is_available,
                                                            size_t wheel_count,
                                                            float maximum_absolute_velocity,
                                                            float *applied_scale);
    alg_chassis_status_t
    alg_chassis_integrate_odometry(alg_chassis_pose_t *me,
                                   const alg_chassis_velocity_t *body_velocity, float delta_time_s,
                                   alg_chassis_integration_method_t integration_method);

#ifdef __cplusplus
}
#endif

#endif /* ALG_CHASSIS_H */
