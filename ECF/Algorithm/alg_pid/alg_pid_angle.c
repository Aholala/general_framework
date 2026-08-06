/**
 * @file alg_pid_angle.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 角度串级 PID 封装实现
 * @version 1.0
 * @date 2026-08-01
 * @copyright Copyright (c) 2026
 */

#include "alg_pid.h"

#include <stddef.h>

/**
 * @brief 初始化角度串级 PID
 */
alg_pid_status_t alg_pid_angle_init(alg_pid_angle_t *me, const alg_pid_angle_config_t *config)
{
    if ((me == NULL) || (config == NULL))
    {
        return ALG_PID_STATUS_INVALID_ARGUMENT;
    }

    return alg_pid_cascade_init(&me->cascade, &config->cascade_config);
}

/**
 * @brief 重置角度串级 PID
 */
alg_pid_status_t alg_pid_angle_reset(alg_pid_angle_t *me, float measured_position_rad,
                                     float measured_velocity_rad_per_s, float initial_output)
{
    if (me == NULL)
    {
        return ALG_PID_STATUS_INVALID_ARGUMENT;
    }

    return alg_pid_cascade_reset(&me->cascade, measured_position_rad,
                                 measured_velocity_rad_per_s, initial_output);
}

/**
 * @brief 更新角度串级 PID
 */
alg_pid_status_t alg_pid_angle_update(alg_pid_angle_t *me, const alg_pid_angle_input_t *input,
                                      float *control_output)
{
    alg_pid_cascade_input_t cascade_input;

    if ((me == NULL) || (input == NULL) || (control_output == NULL))
    {
        return ALG_PID_STATUS_INVALID_ARGUMENT;
    }

    cascade_input = (alg_pid_cascade_input_t){
        .position_setpoint = input->target_position_rad,
        .position_measurement = input->measured_position_rad,
        .velocity_measurement = input->measured_velocity_rad_per_s,
        .velocity_feedforward = input->target_velocity_rad_per_s,
        .actuator_feedforward = input->actuator_feedforward,
        .delta_time_s = input->delta_time_s,
    };

    return alg_pid_cascade_update(&me->cascade, &cascade_input, control_output);
}

/**
 * @brief 获取角速度设定值
 */
float alg_pid_angle_get_velocity_setpoint(const alg_pid_angle_t *me)
{
    if (me == NULL)
    {
        return 0.0F;
    }

    return alg_pid_cascade_get_velocity_setpoint(&me->cascade);
}
