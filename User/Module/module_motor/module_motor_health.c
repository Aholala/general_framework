/**
 * @file module_motor_health.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 多电机健康聚合器实现
 * @version 1.0
 * @date 2026-07-28
 * @copyright Copyright (c) 2026
 *
 * @note 将注册、在线、故障、使能和温度状态转换为稳定的可用性数组。
 *       支持过流、编码器突跳、跟踪误差、堵转、饱和和总线错误等扩展诊断。
 *       所有阈值数组按电机索引提供，不需要的检查传 NULL 即可关闭。
 */

#include "module_motor_health.h"

#include <math.h>   // fabsf, isfinite
#include <stddef.h> // NULL

/**
 * @brief 安全累加时间（防溢出）
 * @param accumulated_time_ms 已累积时间
 * @param elapsed_time_ms 要累加的时间
 * @return 累加后的时间（饱和到 UINT32_MAX）
 */
static uint32_t module_motor_health_add_time(uint32_t accumulated_time_ms, uint32_t elapsed_time_ms)
{
    return (elapsed_time_ms > (UINT32_MAX - accumulated_time_ms))
               ? UINT32_MAX
               : accumulated_time_ms + elapsed_time_ms;
}

/**
 * @brief 评估单个电机的健康原因
 * @param me 健康模块对象
 * @param motor_index 电机索引
 * @param elapsed_time_ms 距上次评估的时间（毫秒）
 * @return 原因位掩码
 * @note 检查：注册状态、离线、故障、使能、过温、过流、编码器突跳、
 *       跟踪误差、堵转、输出饱和、总线错误
 */
static uint32_t module_motor_health_evaluate_reason(const module_motor_health_t *me,
                                                    size_t motor_index, uint32_t elapsed_time_ms)
{
    const module_motor_t *const motor = me->motors[motor_index];
    module_motor_health_state_t *const state = &me->states[motor_index];
    const module_motor_feedback_t *feedback;
    module_motor_health_observation_t observation = {0};
    uint32_t reason_mask = MODULE_MOTOR_HEALTH_REASON_NONE;

    /* -------- 1. 检查注册状态 -------- */
    if (!motor->is_registered)
    {
        return MODULE_MOTOR_HEALTH_REASON_NOT_REGISTERED;
    }

    /* -------- 2. 获取反馈 -------- */
    feedback = module_motor_get_feedback(motor);

    /* -------- 3. 检查反馈在线状态 -------- */
    if ((feedback == NULL) || !feedback->is_online)
    {
        reason_mask |= MODULE_MOTOR_HEALTH_REASON_OFFLINE;
    }

    /* -------- 4. 检查电机故障状态 -------- */
    if (motor->state == MODULE_MOTOR_STATE_FAULT)
    {
        reason_mask |= MODULE_MOTOR_HEALTH_REASON_MOTOR_FAULT;
    }

    /* -------- 5. 检查使能状态（可选） -------- */
    if (me->require_enabled_state && (motor->state != MODULE_MOTOR_STATE_ENABLED))
    {
        reason_mask |= MODULE_MOTOR_HEALTH_REASON_NOT_ENABLED;
    }

    /* -------- 6. 检查过温 -------- */
    if ((feedback != NULL) && (me->maximum_temperature_c != NULL) &&
        (feedback->motor_temperature_c > me->maximum_temperature_c[motor_index]))
    {
        reason_mask |= MODULE_MOTOR_HEALTH_REASON_OVER_TEMPERATURE;
    }

    /* -------- 7. 检查过流 -------- */
    if ((feedback != NULL) && feedback->is_current_a_valid && (me->maximum_current_a != NULL) &&
        (fabsf(feedback->current_a) > me->maximum_current_a[motor_index]))
    {
        reason_mask |= MODULE_MOTOR_HEALTH_REASON_OVER_CURRENT;
    }

    /* -------- 8. 检查编码器突跳 -------- */
    if ((feedback != NULL) && state->has_previous_sample && (me->maximum_encoder_step != NULL))
    {
        // 计算编码器变化量（绝对值）
        uint32_t position_delta = (feedback->raw_position >= state->previous_raw_position)
                                      ? feedback->raw_position - state->previous_raw_position
                                      : state->previous_raw_position - feedback->raw_position;

        // 若配置了模数，选择最短路径（处理回绕）
        if ((me->encoder_modulus != NULL) && (me->encoder_modulus[motor_index] > 0U) &&
            (position_delta < me->encoder_modulus[motor_index]))
        {
            const uint32_t wrapped_delta = me->encoder_modulus[motor_index] - position_delta;
            position_delta = (wrapped_delta < position_delta) ? wrapped_delta : position_delta;
        }

        if (position_delta > me->maximum_encoder_step[motor_index])
        {
            reason_mask |= MODULE_MOTOR_HEALTH_REASON_ENCODER_JUMP;
        }
    }

    /* -------- 9. 扩展诊断（通过 observer 回调） -------- */
    if ((me->observer != NULL) && me->observer(motor, &observation, me->observer_user_context) &&
        observation.is_valid)
    {
        /* 9a. 跟踪误差超限 */
        if ((me->maximum_tracking_error != NULL) &&
            (fabsf(observation.tracking_error) > me->maximum_tracking_error[motor_index]))
        {
            reason_mask |= MODULE_MOTOR_HEALTH_REASON_TRACKING_ERROR;
        }

        /* 9b. 堵转检测：电流大且速度接近零 */
        if ((feedback != NULL) && feedback->is_current_a_valid && (me->stall_current_a != NULL) &&
            (me->stall_velocity_rad_per_s != NULL) &&
            (fabsf(feedback->current_a) >= me->stall_current_a[motor_index]) &&
            (fabsf(feedback->velocity_rad_per_s) <= me->stall_velocity_rad_per_s[motor_index]))
        {
            state->stall_elapsed_time_ms =
                module_motor_health_add_time(state->stall_elapsed_time_ms, elapsed_time_ms);
            if (state->stall_elapsed_time_ms >= me->stall_confirmation_time_ms)
            {
                reason_mask |= MODULE_MOTOR_HEALTH_REASON_STALL;
            }
        }
        else
        {
            state->stall_elapsed_time_ms = 0U; // 非堵转状态，重置计时
        }

        /* 9c. 输出饱和检测 */
        if ((observation.output_limit > 0.0F) &&
            (fabsf(observation.commanded_effort) >=
             observation.output_limit * me->output_saturation_ratio))
        {
            state->saturation_elapsed_time_ms =
                module_motor_health_add_time(state->saturation_elapsed_time_ms, elapsed_time_ms);
            if (state->saturation_elapsed_time_ms >= me->saturation_confirmation_time_ms)
            {
                reason_mask |= MODULE_MOTOR_HEALTH_REASON_OUTPUT_SATURATED;
            }
        }
        else
        {
            state->saturation_elapsed_time_ms = 0U; // 未饱和，重置计时
        }

        /* 9d. 总线错误检测 */
        if (state->has_previous_sample &&
            (observation.bus_error_count != state->previous_bus_error_count))
        {
            reason_mask |= MODULE_MOTOR_HEALTH_REASON_BUS_ERROR;
        }
        state->previous_bus_error_count = observation.bus_error_count;
    }

    /* -------- 10. 保存历史数据（用于下一轮评估） -------- */
    if (feedback != NULL)
    {
        state->previous_raw_position = feedback->raw_position;
        state->has_previous_sample = true;
    }

    return reason_mask;
}

/**
 * @brief 初始化健康模块
 * @param me 健康模块对象
 * @param config 配置参数
 * @return 执行状态
 */
module_motor_health_status_t module_motor_health_init(module_motor_health_t *me,
                                                      const module_motor_health_config_t *config)
{
    size_t motor_index;

    /* -------- 参数校验 -------- */
    if ((me == NULL) || (config == NULL) || (config->motors == NULL) ||
        (config->motor_count == 0U) || (config->state_storage == NULL))
    {
        return MODULE_MOTOR_HEALTH_STATUS_INVALID_ARGUMENT;
    }

    // 检查输出饱和比例有效性
    if (!isfinite(config->output_saturation_ratio) || (config->output_saturation_ratio < 0.0F) ||
        (config->output_saturation_ratio > 1.0F))
    {
        return MODULE_MOTOR_HEALTH_STATUS_INVALID_ARGUMENT;
    }

    me->is_initialized = false;

    /* -------- 验证电机和阈值数组 -------- */
    for (motor_index = 0U; motor_index < config->motor_count; ++motor_index)
    {
        // 检查电机对象有效性
        if ((config->motors[motor_index] == NULL) || !config->motors[motor_index]->is_initialized ||
            ((config->maximum_temperature_c != NULL) &&
             (!isfinite(config->maximum_temperature_c[motor_index]) ||
              (config->maximum_temperature_c[motor_index] <= 0.0F))))
        {
            return MODULE_MOTOR_HEALTH_STATUS_INVALID_ARGUMENT;
        }
        // 初始化状态存储
        config->state_storage[motor_index] = (module_motor_health_state_t){
            .reason_mask = MODULE_MOTOR_HEALTH_REASON_NOT_REGISTERED,
        };
    }

    /* -------- 保存配置到对象 -------- */
    me->motors = config->motors;
    me->motor_count = config->motor_count;
    me->states = config->state_storage;
    me->maximum_temperature_c = config->maximum_temperature_c;
    me->maximum_current_a = config->maximum_current_a;
    me->maximum_encoder_step = config->maximum_encoder_step;
    me->encoder_modulus = config->encoder_modulus;
    me->maximum_tracking_error = config->maximum_tracking_error;
    me->stall_current_a = config->stall_current_a;
    me->stall_velocity_rad_per_s = config->stall_velocity_rad_per_s;
    me->output_saturation_ratio = config->output_saturation_ratio;
    me->stall_confirmation_time_ms = config->stall_confirmation_time_ms;
    me->saturation_confirmation_time_ms = config->saturation_confirmation_time_ms;
    me->observer = config->observer;
    me->observer_user_context = config->observer_user_context;
    me->fault_confirmation_time_ms = config->fault_confirmation_time_ms;
    me->recovery_confirmation_time_ms = config->recovery_confirmation_time_ms;
    me->require_enabled_state = config->require_enabled_state;
    me->manage_feedback_time = config->manage_feedback_time;
    me->is_initialized = true;

    return MODULE_MOTOR_HEALTH_STATUS_OK;
}

/**
 * @brief 周期更新健康状态
 * @param me 健康模块对象
 * @param elapsed_time_ms 距上次更新的时间（毫秒）
 * @return OK=所有电机可用，DEGRADED=部分不可用
 */
module_motor_health_status_t module_motor_health_update(module_motor_health_t *me,
                                                        uint32_t elapsed_time_ms)
{
    size_t motor_index;
    size_t available_motor_count = 0U;

    /* -------- 状态检查 -------- */
    if (me == NULL)
    {
        return MODULE_MOTOR_HEALTH_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return MODULE_MOTOR_HEALTH_STATUS_NOT_INITIALIZED;
    }

    /* -------- 遍历所有电机 -------- */
    for (motor_index = 0U; motor_index < me->motor_count; ++motor_index)
    {
        module_motor_health_state_t *const health_state = &me->states[motor_index];
        uint32_t reason_mask;

        /* -------- 1. 更新反馈超时（若由本模块管理） -------- */
        if (me->manage_feedback_time &&
            (module_motor_update_feedback_time(me->motors[motor_index], elapsed_time_ms) !=
             MODULE_MOTOR_STATUS_OK))
        {
            return MODULE_MOTOR_HEALTH_STATUS_MOTOR_ERROR;
        }

        /* -------- 2. 评估健康原因 -------- */
        reason_mask = module_motor_health_evaluate_reason(me, motor_index, elapsed_time_ms);

        /* -------- 3. 更新故障/恢复状态 -------- */
        if (reason_mask == MODULE_MOTOR_HEALTH_REASON_NONE)
        {
            // 无故障：累积恢复时间
            health_state->fault_elapsed_time_ms = 0U;
            health_state->recovery_elapsed_time_ms = module_motor_health_add_time(
                health_state->recovery_elapsed_time_ms, elapsed_time_ms);
            if (health_state->recovery_elapsed_time_ms >= me->recovery_confirmation_time_ms)
            {
                health_state->is_available = true;
                health_state->reason_mask = MODULE_MOTOR_HEALTH_REASON_NONE;
            }
        }
        else
        {
            // 有故障：累积故障时间
            health_state->recovery_elapsed_time_ms = 0U;
            health_state->reason_mask = reason_mask;
            health_state->fault_elapsed_time_ms =
                module_motor_health_add_time(health_state->fault_elapsed_time_ms, elapsed_time_ms);
            if (health_state->fault_elapsed_time_ms >= me->fault_confirmation_time_ms)
            {
                health_state->is_available = false;
            }
        }

        /* -------- 4. 统计可用电机数量 -------- */
        if (health_state->is_available)
        {
            ++available_motor_count;
        }
    }

    /* -------- 5. 返回聚合状态 -------- */
    return (available_motor_count == me->motor_count) ? MODULE_MOTOR_HEALTH_STATUS_OK
                                                      : MODULE_MOTOR_HEALTH_STATUS_DEGRADED;
}

/**
 * @brief 获取所有电机的可用性状态
 * @param me 健康模块对象
 * @param motor_is_available 输出可用性数组（调用者分配）
 * @param output_capacity 输出数组容量
 * @return OK=所有可用，DEGRADED=部分不可用
 */
module_motor_health_status_t module_motor_health_get_availability(const module_motor_health_t *me,
                                                                  bool *motor_is_available,
                                                                  size_t output_capacity)
{
    size_t motor_index;
    size_t available_motor_count = 0U;

    /* -------- 参数校验 -------- */
    if ((me == NULL) || (motor_is_available == NULL))
    {
        return MODULE_MOTOR_HEALTH_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return MODULE_MOTOR_HEALTH_STATUS_NOT_INITIALIZED;
    }
    if (output_capacity < me->motor_count)
    {
        return MODULE_MOTOR_HEALTH_STATUS_INVALID_ARGUMENT;
    }

    /* -------- 填充可用性数组 -------- */
    for (motor_index = 0U; motor_index < me->motor_count; ++motor_index)
    {
        motor_is_available[motor_index] = me->states[motor_index].is_available;
        if (motor_is_available[motor_index])
        {
            ++available_motor_count;
        }
    }

    return (available_motor_count == me->motor_count) ? MODULE_MOTOR_HEALTH_STATUS_OK
                                                      : MODULE_MOTOR_HEALTH_STATUS_DEGRADED;
}

/**
 * @brief 获取指定电机的健康状态
 * @param me 健康模块对象
 * @param motor_index 电机索引
 * @return 健康状态指针，若索引无效则返回 NULL
 */
const module_motor_health_state_t *module_motor_health_get_state(const module_motor_health_t *me,
                                                                 size_t motor_index)
{
    return ((me != NULL) && me->is_initialized && (motor_index < me->motor_count))
               ? &me->states[motor_index]
               : NULL;
}