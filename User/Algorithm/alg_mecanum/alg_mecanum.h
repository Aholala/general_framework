#ifndef ALG_MECANUM_H
#define ALG_MECANUM_H

#include "alg_chassis.h"

#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define ALG_MECANUM_WHEEL_COUNT (4U)

    typedef enum
    {
        ALG_MECANUM_ROLLER_X = 0,
        ALG_MECANUM_ROLLER_O
    } alg_mecanum_roller_arrangement_t;

    typedef enum
    {
        ALG_MECANUM_WHEEL_FRONT_LEFT = 0,
        ALG_MECANUM_WHEEL_FRONT_RIGHT,
        ALG_MECANUM_WHEEL_REAR_LEFT,
        ALG_MECANUM_WHEEL_REAR_RIGHT
    } alg_mecanum_wheel_index_t;

    typedef struct
    {
        float wheel_radius_m;
        float half_wheelbase_m;
        float half_track_width_m;
        float direction_sign[ALG_MECANUM_WHEEL_COUNT];
        float odometry_weight[ALG_MECANUM_WHEEL_COUNT];
        float maximum_wheel_angular_velocity_rad_per_s;
        alg_mecanum_roller_arrangement_t roller_arrangement;
    } alg_mecanum_config_t;

    typedef struct
    {
        alg_mecanum_config_t config;
        float lateral_coefficient[ALG_MECANUM_WHEEL_COUNT];
        float angular_coefficient_m[ALG_MECANUM_WHEEL_COUNT];
        bool is_initialized;
    } alg_mecanum_t;

    alg_chassis_status_t alg_mecanum_init(alg_mecanum_t *me, const alg_mecanum_config_t *config);
    alg_chassis_status_t
    alg_mecanum_inverse(const alg_mecanum_t *me, const alg_chassis_velocity_t *chassis_velocity,
                        const bool wheel_is_available[ALG_MECANUM_WHEEL_COUNT],
                        float wheel_angular_velocities_rad_per_s[ALG_MECANUM_WHEEL_COUNT],
                        float *applied_scale);
    alg_chassis_status_t alg_mecanum_inverse_with_center_of_rotation(
        const alg_mecanum_t *me, const alg_chassis_velocity_t *center_velocity,
        float center_of_rotation_x_m, float center_of_rotation_y_m,
        const bool wheel_is_available[ALG_MECANUM_WHEEL_COUNT],
        float wheel_angular_velocities_rad_per_s[ALG_MECANUM_WHEEL_COUNT], float *applied_scale);
    alg_chassis_status_t
    alg_mecanum_forward(const alg_mecanum_t *me,
                        const float wheel_angular_velocities_rad_per_s[ALG_MECANUM_WHEEL_COUNT],
                        const bool wheel_is_available[ALG_MECANUM_WHEEL_COUNT],
                        uint8_t known_component_mask, const alg_chassis_velocity_t *known_velocity,
                        alg_chassis_solution_t *solution);

#ifdef __cplusplus
}
#endif

#endif /* ALG_MECANUM_H */
