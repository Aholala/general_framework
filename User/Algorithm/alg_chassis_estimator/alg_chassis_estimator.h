#ifndef ALG_CHASSIS_ESTIMATOR_H
#define ALG_CHASSIS_ESTIMATOR_H

#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C"
{
#endif

    typedef enum
    {
        ALG_CHASSIS_ESTIMATOR_STATUS_OK = 0,
        ALG_CHASSIS_ESTIMATOR_STATUS_DEGRADED,
        ALG_CHASSIS_ESTIMATOR_STATUS_INVALID_ARGUMENT,
        ALG_CHASSIS_ESTIMATOR_STATUS_NOT_INITIALIZED,
        ALG_CHASSIS_ESTIMATOR_STATUS_SINGULAR
    } alg_chassis_estimator_status_t;

    typedef struct
    {
        float position_x_m;
        float position_y_m;
        float direction_x;
        float direction_y;
    } alg_chassis_estimator_wheel_t;

    typedef struct
    {
        float linear_velocity_m_per_s;
        float weight;
        bool is_valid;
    } alg_chassis_estimator_measurement_t;

    typedef struct
    {
        float velocity_x_m_per_s;
        float velocity_y_m_per_s;
        float angular_velocity_rad_per_s;
        float position_x_m;
        float position_y_m;
        float heading_rad;
        float residual_rms_m_per_s;
        size_t valid_wheel_count;
    } alg_chassis_estimator_state_t;

    typedef struct
    {
        const alg_chassis_estimator_wheel_t *wheels;
        size_t wheel_count;
        float velocity_filter_time_constant_s;
        alg_chassis_estimator_state_t state;
        bool is_initialized;
    } alg_chassis_estimator_t;

    alg_chassis_estimator_status_t
    alg_chassis_estimator_init(alg_chassis_estimator_t *me,
                               const alg_chassis_estimator_wheel_t *wheels, size_t wheel_count,
                               float velocity_filter_time_constant_s);
    alg_chassis_estimator_status_t alg_chassis_estimator_update(
        alg_chassis_estimator_t *me, const alg_chassis_estimator_measurement_t *measurements,
        float external_yaw_rate_rad_per_s, bool use_external_yaw_rate, float delta_time_s);
    alg_chassis_estimator_status_t alg_chassis_estimator_reset_pose(alg_chassis_estimator_t *me,
                                                                    float position_x_m,
                                                                    float position_y_m,
                                                                    float heading_rad);
    const alg_chassis_estimator_state_t *
    alg_chassis_estimator_get_state(const alg_chassis_estimator_t *me);

#ifdef __cplusplus
}
#endif

#endif
