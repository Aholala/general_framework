/**
 * @file module_dm_motor_bus.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 达妙电机总线管理实现
 * @version 1.0
 * @date 2026-07-28
 * @copyright Copyright (c) 2026
 *
 * @note 提供反馈 ID 路由、重复 ID 检查和轮询发送预算。
 *       maximum_transmits_per_cycle 用来限制一个控制周期内的独立 CAN 帧数量。
 */

#include "module_dm_motor_bus.h"

#include <stddef.h> // NULL, size_t

/**
 * @brief 初始化达妙电机总线
 * @param me 总线对象
 * @param can CAN BSP 基类
 * @param motor_storage 电机存储数组（调用者分配）
 * @param motor_capacity 数组容量
 * @param maximum_transmits_per_cycle 每周期最大发送帧数
 * @return 执行状态
 */
module_motor_status_t module_dm_motor_bus_init(module_dm_motor_bus_t *me, bsp_can_t *can,
                                               module_dm_motor_t **motor_storage,
                                               size_t motor_capacity,
                                               size_t maximum_transmits_per_cycle)
{
    size_t motor_index;

    // ---- 参数校验 ----
    if ((me == NULL) || (can == NULL) || !bsp_device_is_initialized(&can->super) ||
        (motor_storage == NULL) || (motor_capacity == 0U) || (maximum_transmits_per_cycle == 0U))
    {
        return MODULE_MOTOR_STATUS_INVALID_ARGUMENT;
    }

    // ---- 清空电机存储 ----
    for (motor_index = 0U; motor_index < motor_capacity; ++motor_index)
    {
        motor_storage[motor_index] = NULL;
    }

    // ---- 初始化总线 ----
    *me = (module_dm_motor_bus_t){
        .can = can,
        .motor_storage = motor_storage,
        .motor_capacity = motor_capacity,
        .maximum_transmits_per_cycle = maximum_transmits_per_cycle,
        .is_initialized = true,
    };
    return MODULE_MOTOR_STATUS_OK;
}

/**
 * @brief 注册电机到总线
 * @param me 总线对象
 * @param motor 电机对象
 * @return 执行状态
 * @note 检查重复的反馈 ID 和发送 ID
 */
module_motor_status_t module_dm_motor_bus_register(module_dm_motor_bus_t *me,
                                                   module_dm_motor_t *motor)
{
    size_t motor_index;

    // ---- 参数校验 ----
    if ((me == NULL) || (motor == NULL) || !me->is_initialized || !motor->super.is_initialized ||
        !motor->super.is_registered || (motor->can != me->can))
    {
        return MODULE_MOTOR_STATUS_INVALID_ARGUMENT;
    }

    // ---- 检查重复 ----
    for (motor_index = 0U; motor_index < me->motor_count; ++motor_index)
    {
        // 反馈 CAN ID 可以共用，但反馈 D0 中的电机 ID 低 4 位必须可区分。
        if ((me->motor_storage[motor_index] == motor) ||
            ((me->motor_storage[motor_index]->feedback_identifier == motor->feedback_identifier) &&
             ((me->motor_storage[motor_index]->master_identifier & 0x0FU) ==
              (motor->master_identifier & 0x0FU))) ||
            (me->motor_storage[motor_index]->mode_vptr->get_transmit_identifier(
                 me->motor_storage[motor_index]) ==
             motor->mode_vptr->get_transmit_identifier(motor)))
        {
            return MODULE_MOTOR_STATUS_DUPLICATE_KEY;
        }
    }

    // ---- 检查容量 ----
    if (me->motor_count >= me->motor_capacity)
    {
        return MODULE_MOTOR_STATUS_NO_RESOURCE;
    }

    // ---- 注册 ----
    me->motor_storage[me->motor_count] = motor;
    ++me->motor_count;
    return MODULE_MOTOR_STATUS_OK;
}

/**
 * @brief 从总线注销电机
 * @param me 总线对象
 * @param motor 电机对象
 * @return 执行状态
 */
module_motor_status_t module_dm_motor_bus_unregister(module_dm_motor_bus_t *me,
                                                     module_dm_motor_t *motor)
{
    size_t motor_index;

    if ((me == NULL) || (motor == NULL) || !me->is_initialized)
    {
        return MODULE_MOTOR_STATUS_INVALID_ARGUMENT;
    }

    // ---- 查找电机 ----
    for (motor_index = 0U; motor_index < me->motor_count; ++motor_index)
    {
        if (me->motor_storage[motor_index] == motor)
        {
            size_t move_index;
            // 后续电机前移
            for (move_index = motor_index; move_index + 1U < me->motor_count; ++move_index)
            {
                me->motor_storage[move_index] = me->motor_storage[move_index + 1U];
            }
            --me->motor_count;
            me->motor_storage[me->motor_count] = NULL;

            // 调整轮询索引
            if (me->next_transmit_index >= me->motor_count)
            {
                me->next_transmit_index = 0U;
            }
            return MODULE_MOTOR_STATUS_OK;
        }
    }
    return MODULE_MOTOR_STATUS_NOT_REGISTERED;
}

/**
 * @brief 处理 CAN 反馈帧（路由到对应的电机）
 * @param me 总线对象
 * @param frame CAN 帧
 * @return 执行状态
 */
module_motor_status_t module_dm_motor_bus_handle_feedback(module_dm_motor_bus_t *me,
                                                          const bsp_can_frame_t *frame)
{
    size_t motor_index;

    if ((me == NULL) || (frame == NULL) || !me->is_initialized)
    {
        return MODULE_MOTOR_STATUS_INVALID_ARGUMENT;
    }

    // ---- 查找匹配的电机 ----
    for (motor_index = 0U; motor_index < me->motor_count; ++motor_index)
    {
        if ((me->motor_storage[motor_index]->feedback_identifier == frame->identifier) &&
            (frame->data_length == 8U) &&
            ((frame->data[0] & 0x0FU) ==
             (uint8_t)(me->motor_storage[motor_index]->master_identifier & 0x0FU)))
        {
            const module_motor_status_t status =
                module_dm_motor_handle_feedback(me->motor_storage[motor_index], frame);
            if (status == MODULE_MOTOR_STATUS_OK)
            {
                ++me->routed_frame_count; // 成功路由计数
            }
            return status;
        }
    }

    // ---- 未找到匹配 ----
    ++me->unknown_frame_count;
    return MODULE_MOTOR_STATUS_FEEDBACK_UNAVAILABLE;
}

/**
 * @brief 更新总线（轮询发送电机命令）
 * @param me 总线对象
 * @param delta_time_s 时间步长（秒）
 * @return 执行状态
 * @note 每周期最多发送 maximum_transmits_per_cycle 帧
 */
module_motor_status_t module_dm_motor_bus_update(module_dm_motor_bus_t *me, float delta_time_s)
{
    size_t transmit_count;
    bool had_error = false;

    // ---- 参数校验 ----
    if ((me == NULL) || !me->is_initialized || (delta_time_s <= 0.0F))
    {
        return MODULE_MOTOR_STATUS_INVALID_ARGUMENT;
    }
    if (me->motor_count == 0U)
    {
        return MODULE_MOTOR_STATUS_OK;
    }

    // ---- 轮询发送 ----
    for (transmit_count = 0U;
         (transmit_count < me->maximum_transmits_per_cycle) && (transmit_count < me->motor_count);
         ++transmit_count)
    {
        module_dm_motor_t *const motor = me->motor_storage[me->next_transmit_index];
        const module_motor_status_t status = module_motor_update(&motor->super, delta_time_s);

        // 更新轮询索引（循环）
        me->next_transmit_index = (me->next_transmit_index + 1U) % me->motor_count;

        if (status != MODULE_MOTOR_STATUS_OK)
        {
            ++me->transmit_error_count;
            had_error = true;
        }
    }

    return had_error ? MODULE_MOTOR_STATUS_TRANSPORT_ERROR : MODULE_MOTOR_STATUS_OK;
}
