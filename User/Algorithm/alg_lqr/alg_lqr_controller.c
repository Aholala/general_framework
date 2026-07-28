/**
 * @file alg_lqr_controller.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief LQR 控制器实现（初始化与更新）
 * @version 1.0
 * @date 2026-07-28
 * @copyright Copyright (c) 2026
 *
 * @note 实现简单高效的控制器，仅包含矩阵-向量乘法和限幅。
 *       所有输入数组均检查有限性。
 */

#include "alg_lqr_internal.h"
#include <math.h>
#include <stddef.h>

/**
 * @brief 初始化 LQR 控制器
 * @param me     控制器对象
 * @param config 配置
 * @return 执行状态
 */
alg_lqr_status_t alg_lqr_controller_init(alg_lqr_controller_t *me,
                                         const alg_lqr_controller_config_t *config)
{
    size_t control_index;

    // ---- 参数检查 ----
    if ((me == NULL) || (config == NULL) || (config->gain_matrix == NULL))
        return ALG_LQR_STATUS_INVALID_ARGUMENT;

    me->is_initialized = false;
    if ((config->state_dimension == 0U) || (config->control_dimension == 0U))
        return ALG_LQR_STATUS_OUT_OF_RANGE;

    // 限幅指针必须同时 NULL 或同时非 NULL
    if ((config->control_min == NULL) != (config->control_max == NULL))
        return ALG_LQR_STATUS_INVALID_ARGUMENT;

    // 检查增益矩阵有限性
    if (!alg_lqr_internal_is_finite_array(config->gain_matrix,
                                          config->control_dimension * config->state_dimension))
        return ALG_LQR_STATUS_OUT_OF_RANGE;

    // 检查限幅有效性
    if (config->control_min != NULL)
    {
        for (control_index = 0U; control_index < config->control_dimension; ++control_index)
        {
            if (!isfinite(config->control_min[control_index]) ||
                !isfinite(config->control_max[control_index]) ||
                (config->control_min[control_index] >= config->control_max[control_index]))
                return ALG_LQR_STATUS_OUT_OF_RANGE;
        }
    }

    me->config = *config;
    me->is_initialized = true;
    return ALG_LQR_STATUS_OK;
}

/**
 * @brief 根据状态计算控制量
 * @param me                控制器对象
 * @param state             当前状态
 * @param reference_state   参考状态（可为 NULL）
 * @param equilibrium_control 平衡输入（可为 NULL）
 * @param feedforward_control 前馈（可为 NULL）
 * @param control_output    输出控制
 * @return 执行状态
 */
alg_lqr_status_t alg_lqr_controller_update(const alg_lqr_controller_t *me, const float *state,
                                           const float *reference_state,
                                           const float *equilibrium_control,
                                           const float *feedforward_control, float *control_output)
{
    size_t control_index;
    size_t state_index;
    float state_error;
    float output;

    // ---- 参数检查 ----
    if ((me == NULL) || (state == NULL) || (control_output == NULL))
        return ALG_LQR_STATUS_INVALID_ARGUMENT;
    if (!me->is_initialized)
        return ALG_LQR_STATUS_NOT_INITIALIZED;

    // 输入数组有限性检查
    if (!alg_lqr_internal_is_finite_array(state, me->config.state_dimension) ||
        ((reference_state != NULL) &&
         !alg_lqr_internal_is_finite_array(reference_state, me->config.state_dimension)) ||
        ((equilibrium_control != NULL) &&
         !alg_lqr_internal_is_finite_array(equilibrium_control, me->config.control_dimension)) ||
        ((feedforward_control != NULL) &&
         !alg_lqr_internal_is_finite_array(feedforward_control, me->config.control_dimension)))
        return ALG_LQR_STATUS_OUT_OF_RANGE;

    // ---- 计算每个控制量 ----
    for (control_index = 0U; control_index < me->config.control_dimension; ++control_index)
    {
        // 基础：平衡 + 前馈
        output = (equilibrium_control != NULL) ? equilibrium_control[control_index] : 0.0F;
        if (feedforward_control != NULL)
            output += feedforward_control[control_index];

        // 状态反馈：u -= K * (x - x_ref)
        for (state_index = 0U; state_index < me->config.state_dimension; ++state_index)
        {
            state_error = state[state_index] -
                          ((reference_state != NULL) ? reference_state[state_index] : 0.0F);
            output -=
                me->config.gain_matrix[(control_index * me->config.state_dimension) + state_index] *
                state_error;
        }

        // 检查数值有效性
        if (!isfinite(output))
            return ALG_LQR_STATUS_NUMERICAL_ERROR;

        // 限幅
        if (me->config.control_min != NULL)
        {
            if (output < me->config.control_min[control_index])
                output = me->config.control_min[control_index];
            else if (output > me->config.control_max[control_index])
                output = me->config.control_max[control_index];
        }
        control_output[control_index] = output;
    }
    return ALG_LQR_STATUS_OK;
}