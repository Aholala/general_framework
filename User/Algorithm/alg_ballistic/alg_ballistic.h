#ifndef ALG_BALLISTIC_H
#define ALG_BALLISTIC_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

    typedef enum
    {
        ALG_BALLISTIC_STATUS_OK = 0,
        ALG_BALLISTIC_STATUS_INVALID_ARGUMENT,
        ALG_BALLISTIC_STATUS_NOT_INITIALIZED,
        ALG_BALLISTIC_STATUS_NO_SOLUTION
    } alg_ballistic_status_t;

    typedef struct
    {
        float gravity_m_per_s2;
        float drag_coefficient_per_m;
        float integration_step_s;
        float maximum_flight_time_s;
        float pitch_min_rad;
        float pitch_max_rad;
        unsigned int maximum_iterations;
        float height_tolerance_m;
    } alg_ballistic_config_t;

    typedef struct
    {
        float horizontal_distance_m;
        float vertical_distance_m;
        float target_velocity_x_m_per_s;
        float target_velocity_y_m_per_s;
        float target_velocity_z_m_per_s;
        float projectile_speed_m_per_s;
        float system_delay_s;
    } alg_ballistic_target_t;

    typedef struct
    {
        float pitch_rad;
        float yaw_lead_rad;
        float flight_time_s;
        float predicted_drop_m;
    } alg_ballistic_solution_t;

    typedef struct
    {
        alg_ballistic_config_t config;
        bool is_initialized;
    } alg_ballistic_t;

    alg_ballistic_status_t alg_ballistic_init(alg_ballistic_t *me,
                                              const alg_ballistic_config_t *config);
    alg_ballistic_status_t alg_ballistic_solve(const alg_ballistic_t *me,
                                               const alg_ballistic_target_t *target,
                                               alg_ballistic_solution_t *solution);

#ifdef __cplusplus
}
#endif

#endif
