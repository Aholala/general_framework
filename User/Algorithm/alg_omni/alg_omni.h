#ifndef ALG_OMNI_H
#define ALG_OMNI_H

#include "alg_chassis.h"

#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define ALG_OMNI_THREE_WHEEL_COUNT (3U)
#define ALG_OMNI_FOUR_WHEEL_COUNT (4U)

    typedef struct
    {
        float position_x_m;
        float position_y_m;
        float drive_direction_rad;
        float wheel_radius_m;
        float direction_sign;
        float odometry_weight;
    } alg_omni_wheel_config_t;

    typedef struct
    {
        const alg_omni_wheel_config_t *wheel_configs;
        size_t wheel_count;
        float maximum_wheel_angular_velocity_rad_per_s;
        bool is_initialized;
    } alg_omni_t;

    alg_chassis_status_t alg_omni_init(alg_omni_t *me, const alg_omni_wheel_config_t *wheel_configs,
                                       size_t wheel_count,
                                       float maximum_wheel_angular_velocity_rad_per_s);
    alg_chassis_status_t
    alg_omni_configure_tangential_layout(alg_omni_wheel_config_t *wheel_configs, size_t wheel_count,
                                         float center_to_wheel_distance_m, float wheel_radius_m,
                                         float first_wheel_position_angle_rad,
                                         float tangential_direction_sign,
                                         const float *wheel_direction_signs, float odometry_weight);
    alg_chassis_status_t alg_omni_inverse(const alg_omni_t *me,
                                          const alg_chassis_velocity_t *chassis_velocity,
                                          const bool *wheel_is_available,
                                          float *wheel_angular_velocities_rad_per_s,
                                          size_t output_capacity, float *applied_scale);
    alg_chassis_status_t alg_omni_inverse_with_center_of_rotation(
        const alg_omni_t *me, const alg_chassis_velocity_t *center_velocity,
        float center_of_rotation_x_m, float center_of_rotation_y_m, const bool *wheel_is_available,
        float *wheel_angular_velocities_rad_per_s, size_t output_capacity, float *applied_scale);
    alg_chassis_status_t
    alg_omni_forward(const alg_omni_t *me, const float *wheel_angular_velocities_rad_per_s,
                     const bool *wheel_is_available, uint8_t known_component_mask,
                     const alg_chassis_velocity_t *known_velocity,
                     alg_chassis_constraint_t *constraint_workspace, size_t workspace_capacity,
                     alg_chassis_solution_t *solution);

#ifdef __cplusplus
}
#endif

#endif /* ALG_OMNI_H */
