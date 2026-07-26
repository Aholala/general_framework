#ifndef ALG_DIFFERENTIAL_H
#define ALG_DIFFERENTIAL_H

#include "alg_chassis.h"

#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define ALG_DIFFERENTIAL_TWO_WHEEL_COUNT (2U)
#define ALG_DIFFERENTIAL_FOUR_WHEEL_COUNT (4U)

    typedef enum
    {
        ALG_DIFFERENTIAL_SIDE_LEFT = 0,
        ALG_DIFFERENTIAL_SIDE_RIGHT
    } alg_differential_side_t;

    typedef struct
    {
        alg_differential_side_t side;
        float wheel_radius_m;
        float direction_sign;
        float odometry_weight;
    } alg_differential_wheel_config_t;

    typedef struct
    {
        const alg_differential_wheel_config_t *wheel_configs;
        size_t wheel_count;
        float track_width_m;
        float maximum_wheel_angular_velocity_rad_per_s;
        bool is_initialized;
    } alg_differential_t;

    alg_chassis_status_t alg_differential_init(alg_differential_t *me,
                                               const alg_differential_wheel_config_t *wheel_configs,
                                               size_t wheel_count, float track_width_m,
                                               float maximum_wheel_angular_velocity_rad_per_s);
    alg_chassis_status_t alg_differential_inverse(const alg_differential_t *me,
                                                  const alg_chassis_velocity_t *chassis_velocity,
                                                  const bool *wheel_is_available,
                                                  float *wheel_angular_velocities_rad_per_s,
                                                  size_t output_capacity, float *applied_scale);
    alg_chassis_status_t alg_differential_inverse_with_lateral_center_of_rotation(
        const alg_differential_t *me, const alg_chassis_velocity_t *center_velocity,
        float center_of_rotation_y_m, const bool *wheel_is_available,
        float *wheel_angular_velocities_rad_per_s, size_t output_capacity, float *applied_scale);
    alg_chassis_status_t alg_differential_forward(const alg_differential_t *me,
                                                  const float *wheel_angular_velocities_rad_per_s,
                                                  const bool *wheel_is_available,
                                                  uint8_t known_component_mask,
                                                  const alg_chassis_velocity_t *known_velocity,
                                                  alg_chassis_constraint_t *constraint_workspace,
                                                  size_t workspace_capacity,
                                                  alg_chassis_solution_t *solution);

#ifdef __cplusplus
}
#endif

#endif /* ALG_DIFFERENTIAL_H */
