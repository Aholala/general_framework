/**
 * @file alg_pid_schedule.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 增益调度 PID 控制器实现
 * @version 1.0
 * @date 2026-07-28
 * @copyright Copyright (c) 2026
 *
 * @note 根据工作点线性插值 Kp、Ki、Kd。
 *       增益表必须按工作点严格递增，超出范围时使用最近端点。
 */

#include "alg_pid_internal.h"
#include <math.h>
#include <stddef.h>

/**
 * @brief 验证增益表合法性
 */
static alg_pid_status_t
alg_pid_gain_schedule_validate_points(const alg_pid_gain_point_t *gain_points,
                                      size_t gain_point_count)
{
    size_t i;

    if (gain_points == NULL) {
        return ALG_PID_STATUS_INVALID_ARGUMENT;
}
    if (gain_point_count == 0U) {
        return ALG_PID_STATUS_OUT_OF_RANGE;
}

    for (i = 0U; i < gain_point_count; ++i)
    {
        if (!isfinite(gain_points[i].operating_point) ||
            !isfinite(gain_points[i].proportional_gain) ||
            !isfinite(gain_points[i].integral_gain) || !isfinite(gain_points[i].derivative_gain)) {
            return ALG_PID_STATUS_OUT_OF_RANGE;
}
        if ((i > 0U) && (gain_points[i].operating_point <= gain_points[i - 1U].operating_point)) {
            return ALG_PID_STATUS_OUT_OF_RANGE;
}
    }
    return ALG_PID_STATUS_OK;
}

/**
 * @brief 线性插值增益
 */
static void alg_pid_gain_schedule_interpolate(const alg_pid_gain_schedule_t *me,
                                              float operating_point, float *kp, float *ki,
                                              float *kd)
{
    const alg_pid_gain_point_t *lower;
    const alg_pid_gain_point_t *upper;
    size_t i;
    float factor;

    // 边界处理
    if ((me->gain_point_count == 1U) || (operating_point <= me->gain_points[0].operating_point))
    {
        lower = &me->gain_points[0];
        upper = lower;
    }
    else if (operating_point >= me->gain_points[me->gain_point_count - 1U].operating_point)
    {
        lower = &me->gain_points[me->gain_point_count - 1U];
        upper = lower;
    }
    else
    {
        // 查找所在区间
        lower = &me->gain_points[0];
        upper = &me->gain_points[1];
        for (i = 1U; i < me->gain_point_count; ++i)
        {
            if (operating_point <= me->gain_points[i].operating_point)
            {
                lower = &me->gain_points[i - 1U];
                upper = &me->gain_points[i];
                break;
            }
        }
    }

    // 计算插值因子
    factor = 0.0F;
    if (upper != lower) {
        factor = (operating_point - lower->operating_point) /
                 (upper->operating_point - lower->operating_point);
}

    *kp = lower->proportional_gain + factor * (upper->proportional_gain - lower->proportional_gain);
    *ki = lower->integral_gain + factor * (upper->integral_gain - lower->integral_gain);
    *kd = lower->derivative_gain + factor * (upper->derivative_gain - lower->derivative_gain);
}

/**
 * @brief 初始化增益调度 PID
 */
alg_pid_status_t alg_pid_gain_schedule_init(alg_pid_gain_schedule_t *me,
                                            const alg_pid_config_t *base_config,
                                            const alg_pid_gain_point_t *gain_points,
                                            size_t gain_point_count)
{
    alg_pid_status_t status;

    if (me == NULL) {
        return ALG_PID_STATUS_INVALID_ARGUMENT;
}

    me->is_initialized = false;
    status = alg_pid_gain_schedule_validate_points(gain_points, gain_point_count);
    if (status != ALG_PID_STATUS_OK) {
        return status;
}

    status = alg_pid_init(&me->controller, base_config);
    if (status != ALG_PID_STATUS_OK) {
        return status;
}

    me->gain_points = gain_points;
    me->gain_point_count = gain_point_count;
    me->is_initialized = true;
    return ALG_PID_STATUS_OK;
}

/**
 * @brief 增益调度更新
 */
alg_pid_status_t alg_pid_gain_schedule_update(alg_pid_gain_schedule_t *me, float operating_point,
                                              const alg_pid_input_t *input, float *output)
{
    if ((me == NULL) || (input == NULL) || (output == NULL)) {
        return ALG_PID_STATUS_INVALID_ARGUMENT;
}
    if (!me->is_initialized) {
        return ALG_PID_STATUS_NOT_INITIALIZED;
}
    if (!isfinite(operating_point)) {
        return ALG_PID_STATUS_OUT_OF_RANGE;
}

    // 插值得到当前增益
    alg_pid_gain_schedule_interpolate(me, operating_point, &me->controller.config.proportional_gain,
                                      &me->controller.config.integral_gain,
                                      &me->controller.config.derivative_gain);
    return alg_pid_update_advanced(&me->controller, input, output);
}

/**
 * @brief 重置增益调度 PID
 */
alg_pid_status_t alg_pid_gain_schedule_reset(alg_pid_gain_schedule_t *me, float measurement,
                                             float initial_output)
{
    if (me == NULL) {
        return ALG_PID_STATUS_INVALID_ARGUMENT;
}
    if (!me->is_initialized) {
        return ALG_PID_STATUS_NOT_INITIALIZED;
}
    return alg_pid_reset(&me->controller, measurement, initial_output);
}