/**
 * @file alg_pid_cascade.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 位置—速度串级 PID 控制器实现
 * @version 1.0
 * @date 2026-07-28
 * @copyright Copyright (c) 2026
 *
 * @note 外环（位置环）输出作为内环（速度环）的设定值。
 *       位置环可按降频运行，速度环每周期执行。
 */

#include "alg_pid_internal.h"
#include <math.h>
#include <stddef.h>

/**
 * @brief 初始化串级 PID
 */
alg_pid_status_t alg_pid_cascade_init(alg_pid_cascade_t *me, const alg_pid_cascade_config_t *config)
{
    alg_pid_status_t status;

    if ((me == NULL) || (config == NULL)) {
        return ALG_PID_STATUS_INVALID_ARGUMENT;
}

    me->is_initialized = false;
    if ((config->position_loop_divider == 0U) || !isfinite(config->velocity_setpoint_min) ||
        !isfinite(config->velocity_setpoint_max) ||
        (config->velocity_setpoint_min >= config->velocity_setpoint_max)) {
        return ALG_PID_STATUS_OUT_OF_RANGE;
}

    status = alg_pid_position_init(&me->position_controller, &config->position_config);
    if (status != ALG_PID_STATUS_OK) {
        return status;
}

    status = alg_pid_velocity_init(&me->velocity_controller, &config->velocity_config);
    if (status != ALG_PID_STATUS_OK) {
        return status;
}

    me->position_loop_divider = config->position_loop_divider;
    me->position_loop_counter = config->position_loop_divider - 1U;
    me->position_elapsed_time_s = 0.0F;
    me->velocity_setpoint_min = config->velocity_setpoint_min;
    me->velocity_setpoint_max = config->velocity_setpoint_max;
    me->velocity_setpoint = 0.0F;
    me->is_initialized = true;
    return ALG_PID_STATUS_OK;
}

/**
 * @brief 重置串级 PID
 */
alg_pid_status_t alg_pid_cascade_reset(alg_pid_cascade_t *me, float position_measurement,
                                       float velocity_measurement, float initial_output)
{
    alg_pid_status_t status;

    if (me == NULL) {
        return ALG_PID_STATUS_INVALID_ARGUMENT;
}
    if (!me->is_initialized) {
        return ALG_PID_STATUS_NOT_INITIALIZED;
}
    if (!isfinite(position_measurement) || !isfinite(velocity_measurement) ||
        !isfinite(initial_output)) {
        return ALG_PID_STATUS_OUT_OF_RANGE;
}

    status = alg_pid_reset(&me->position_controller, position_measurement, 0.0F);
    if (status != ALG_PID_STATUS_OK) {
        return status;
}

    status = alg_pid_reset(&me->velocity_controller, velocity_measurement, initial_output);
    if (status != ALG_PID_STATUS_OK) {
        return status;
}

    me->position_loop_counter = me->position_loop_divider - 1U;
    me->position_elapsed_time_s = 0.0F;
    me->velocity_setpoint = 0.0F;
    return ALG_PID_STATUS_OK;
}

/**
 * @brief 串级 PID 更新
 */
alg_pid_status_t alg_pid_cascade_update(alg_pid_cascade_t *me, const alg_pid_cascade_input_t *input,
                                        float *output)
{
    alg_pid_input_t pos_input, vel_input;
    alg_pid_status_t status;
    float pos_output;

    // ---- 参数检查 ----
    if ((me == NULL) || (input == NULL) || (output == NULL)) {
        return ALG_PID_STATUS_INVALID_ARGUMENT;
}
    if (!me->is_initialized) {
        return ALG_PID_STATUS_NOT_INITIALIZED;
}
    if (!isfinite(input->position_setpoint) || !isfinite(input->position_measurement) ||
        !isfinite(input->velocity_measurement) || !isfinite(input->velocity_feedforward) ||
        !isfinite(input->actuator_feedforward) || !isfinite(input->delta_time_s) ||
        (input->delta_time_s <= 0.0F)) {
        return ALG_PID_STATUS_OUT_OF_RANGE;
}

    // ---- 累积时间和计数器 ----
    me->position_elapsed_time_s += input->delta_time_s;
    ++me->position_loop_counter;

    // ---- 位置环更新（降频） ----
    if (me->position_loop_counter >= me->position_loop_divider)
    {
        pos_input = (alg_pid_input_t){.setpoint = input->position_setpoint,
                                      .measurement = input->position_measurement,
                                      .setpoint_rate_per_s = 0.0F,
                                      .setpoint_acceleration_per_s2 = 0.0F,
                                      .additional_feedforward = input->velocity_feedforward,
                                      .delta_time_s = me->position_elapsed_time_s};
        status = alg_pid_update_advanced(&me->position_controller, &pos_input, &pos_output);
        if (status != ALG_PID_STATUS_OK) {
            return status;
}

        // 速度设定值限幅
        me->velocity_setpoint = alg_pid_internal_clamp(pos_output, me->velocity_setpoint_min,
                                                       me->velocity_setpoint_max);
        me->position_loop_counter = 0U;
        me->position_elapsed_time_s = 0.0F;
    }

    // ---- 速度环更新（每周期） ----
    vel_input = (alg_pid_input_t){.setpoint = me->velocity_setpoint,
                                  .measurement = input->velocity_measurement,
                                  .setpoint_rate_per_s = 0.0F,
                                  .setpoint_acceleration_per_s2 = 0.0F,
                                  .additional_feedforward = input->actuator_feedforward,
                                  .delta_time_s = input->delta_time_s};
    return alg_pid_update_advanced(&me->velocity_controller, &vel_input, output);
}

/**
 * @brief 获取当前速度环设定值
 */
float alg_pid_cascade_get_velocity_setpoint(const alg_pid_cascade_t *me)
{
    return ((me != NULL) && me->is_initialized) ? me->velocity_setpoint : 0.0F;
}