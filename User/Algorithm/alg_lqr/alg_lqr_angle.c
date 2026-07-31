/**
 * @file alg_lqr_angle.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 二维角度 LQR 封装实现
 * @version 1.0
 * @date 2026-08-01
 * @copyright Copyright (c) 2026
 */

#include "alg_lqr.h"

#include <math.h>
#include <stddef.h>

/**
 * @brief 校验角度 LQR 输入
 */
static bool alg_lqr_angle_input_is_valid(const alg_lqr_angle_input_t *input)
{
    return (input != NULL) && isfinite(input->target_position_rad) &&
           isfinite(input->target_velocity_rad_per_s) &&
           isfinite(input->measured_position_rad) &&
           isfinite(input->measured_velocity_rad_per_s) &&
           isfinite(input->actuator_feedforward) && isfinite(input->delta_time_s) &&
           (input->delta_time_s > 0.0F);
}

/**
 * @brief 初始化二维角度 LQR
 */
alg_lqr_status_t alg_lqr_angle_init(alg_lqr_angle_t *me, const alg_lqr_angle_config_t *config)
{
    alg_lqr_controller_config_t controller_config;

    if ((me == NULL) || (config == NULL) || (config->gain_matrix == NULL))
    {
        return ALG_LQR_STATUS_INVALID_ARGUMENT;
    }
    if (!isfinite(config->gain_matrix[0]) || !isfinite(config->gain_matrix[1]) ||
        !isfinite(config->control_min) || !isfinite(config->control_max) ||
        !isfinite(config->equilibrium_control) || (config->control_min >= config->control_max))
    {
        return ALG_LQR_STATUS_OUT_OF_RANGE;
    }

    me->gain_matrix[0] = config->gain_matrix[0];
    me->gain_matrix[1] = config->gain_matrix[1];
    me->control_min = config->control_min;
    me->control_max = config->control_max;
    me->equilibrium_control = config->equilibrium_control;

    controller_config = (alg_lqr_controller_config_t){
        .state_dimension = 2U,
        .control_dimension = 1U,
        .gain_matrix = me->gain_matrix,
        .control_min = &me->control_min,
        .control_max = &me->control_max,
    };

    return alg_lqr_controller_init(&me->controller, &controller_config);
}

/**
 * @brief 校验当前状态并重置二维角度 LQR
 */
alg_lqr_status_t alg_lqr_angle_reset(alg_lqr_angle_t *me, float measured_position_rad,
                                     float measured_velocity_rad_per_s, float initial_output)
{
    if (me == NULL)
    {
        return ALG_LQR_STATUS_INVALID_ARGUMENT;
    }
    if (!me->controller.is_initialized)
    {
        return ALG_LQR_STATUS_NOT_INITIALIZED;
    }
    if (!isfinite(measured_position_rad) || !isfinite(measured_velocity_rad_per_s) ||
        !isfinite(initial_output))
    {
        return ALG_LQR_STATUS_OUT_OF_RANGE;
    }

    return ALG_LQR_STATUS_OK;
}

/**
 * @brief 更新二维角度 LQR
 */
alg_lqr_status_t alg_lqr_angle_update(const alg_lqr_angle_t *me,
                                      const alg_lqr_angle_input_t *input,
                                      float *control_output)
{
    float state[2];
    float reference_state[2];
    float feedforward_control[1];

    if ((me == NULL) || (control_output == NULL) || !alg_lqr_angle_input_is_valid(input))
    {
        return ALG_LQR_STATUS_INVALID_ARGUMENT;
    }
    if (!me->controller.is_initialized)
    {
        return ALG_LQR_STATUS_NOT_INITIALIZED;
    }

    state[0] = input->measured_position_rad;
    state[1] = input->measured_velocity_rad_per_s;
    reference_state[0] = input->target_position_rad;
    reference_state[1] = input->target_velocity_rad_per_s;
    feedforward_control[0] = input->actuator_feedforward;

    return alg_lqr_controller_update(&me->controller, state, reference_state,
                                     &me->equilibrium_control, feedforward_control,
                                     control_output);
}
