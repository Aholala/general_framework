#ifndef ALG_ACKERMANN_H
#define ALG_ACKERMANN_H

#include "alg_chassis.h"

#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define ALG_ACKERMANN_WHEEL_COUNT (4U)
#define ALG_ACKERMANN_CONSTRAINT_COUNT (8U)

    typedef enum
    {
        ALG_ACKERMANN_WHEEL_FRONT_LEFT = 0,
        ALG_ACKERMANN_WHEEL_FRONT_RIGHT,
        ALG_ACKERMANN_WHEEL_REAR_LEFT,
        ALG_ACKERMANN_WHEEL_REAR_RIGHT
    } alg_ackermann_wheel_index_t;

    typedef struct
    {
        float wheelbase_m;
        float track_width_m;
        float wheel_radius_m[ALG_ACKERMANN_WHEEL_COUNT];
        float direction_sign[ALG_ACKERMANN_WHEEL_COUNT];
        float odometry_weight[ALG_ACKERMANN_WHEEL_COUNT];
        float lateral_constraint_weight;
        float maximum_steering_angle_rad;
        float maximum_wheel_angular_velocity_rad_per_s;
    } alg_ackermann_config_t;

    typedef struct
    {
        float wheel_angular_velocity_rad_per_s;
        float steering_angle_rad;
    } alg_ackermann_wheel_state_t;

    typedef struct
    {
        alg_ackermann_config_t config;
        float wheel_position_x_m[ALG_ACKERMANN_WHEEL_COUNT];
        float wheel_position_y_m[ALG_ACKERMANN_WHEEL_COUNT];
        bool is_initialized;
    } alg_ackermann_t;

    alg_chassis_status_t alg_ackermann_init(alg_ackermann_t *me,
                                            const alg_ackermann_config_t *config);
    alg_chassis_status_t
    alg_ackermann_inverse(const alg_ackermann_t *me, const alg_chassis_velocity_t *chassis_velocity,
                          const bool wheel_is_available[ALG_ACKERMANN_WHEEL_COUNT],
                          alg_ackermann_wheel_state_t wheel_states[ALG_ACKERMANN_WHEEL_COUNT],
                          float *applied_scale);
    alg_chassis_status_t alg_ackermann_forward(
        const alg_ackermann_t *me,
        const alg_ackermann_wheel_state_t wheel_states[ALG_ACKERMANN_WHEEL_COUNT],
        const bool wheel_is_available[ALG_ACKERMANN_WHEEL_COUNT], uint8_t known_component_mask,
        const alg_chassis_velocity_t *known_velocity,
        alg_chassis_constraint_t constraint_workspace[ALG_ACKERMANN_CONSTRAINT_COUNT],
        alg_chassis_solution_t *solution);

#ifdef __cplusplus
}
#endif

#endif /* ALG_ACKERMANN_H */
