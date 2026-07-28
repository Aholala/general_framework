/**
 * @file module_shooter.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief RoboMaster 发射机构状态机实现
 * @version 1.0
 * @date 2026-07-28
 * @copyright Copyright (c) 2026
 *
 * @note 组合左右摩擦轮和拨弹电机，提供状态机管理（DISABLED → READY → FEEDING → ROLLBACK → FAULT）
 *       支持堵转检测、自动回退和有限重试。
 */

#include "module_shooter.h"

#include <math.h>   // fabsf, isfinite
#include <stddef.h> // NULL
#include <stdlib.h> // abs

/**
 * @brief 检查方向符号是否合法（+1 或 -1）
 * @param direction_sign 方向符号
 * @return true 合法
 */
static bool module_shooter_direction_is_valid(float direction_sign)
{
    return (direction_sign == 1.0F) || (direction_sign == -1.0F);
}

/**
 * @brief 检测拨弹电机是否发生堵转（基于电流）
 * @param me 发射机构对象
 * @param feeder_feedback 拨弹电机反馈
 * @return true=检测到堵转
 * @note 优先使用安培阈值，若无效则使用原始值阈值
 */
static bool module_shooter_jam_current_detected(const module_shooter_t *me,
                                                const module_motor_feedback_t *feeder_feedback)
{
    if (feeder_feedback->is_current_a_valid)
    {
        // 电流安培值有效，使用安培阈值
        return fabsf(feeder_feedback->current_a) >= me->jam_current_threshold_a;
    }
    // 否则使用原始电流阈值
    return abs(feeder_feedback->current_raw) >= abs(me->jam_current_threshold_raw);
}

/**
 * @brief 设置三个电机的目标值
 * @param me 发射机构对象
 * @return 执行状态
 * @note 摩擦轮目标根据 friction_enabled 和方向符号计算
 *       拨弹目标为 feeder_target_position_rad
 */
static module_shooter_status_t module_shooter_set_motor_targets(module_shooter_t *me)
{
    const float left_velocity_rad_per_s =
        me->friction_enabled
            ? me->friction_target_velocity_rad_per_s * me->left_friction_direction_sign
            : 0.0F;
    const float right_velocity_rad_per_s =
        me->friction_enabled
            ? me->friction_target_velocity_rad_per_s * me->right_friction_direction_sign
            : 0.0F;

    if ((module_motor_set_target(me->left_friction_motor, left_velocity_rad_per_s) !=
         MODULE_MOTOR_STATUS_OK) ||
        (module_motor_set_target(me->right_friction_motor, right_velocity_rad_per_s) !=
         MODULE_MOTOR_STATUS_OK) ||
        (module_motor_set_target(me->feeder_motor, me->feeder_target_position_rad) !=
         MODULE_MOTOR_STATUS_OK))
    {
        return MODULE_SHOOTER_STATUS_MOTOR_ERROR;
    }
    return MODULE_SHOOTER_STATUS_OK;
}

/**
 * @brief 更新三个电机的控制周期
 * @param me 发射机构对象
 * @param delta_time_s 时间步长（秒）
 * @return 执行状态
 */
static module_shooter_status_t module_shooter_update_motors(module_shooter_t *me,
                                                            float delta_time_s)
{
    if ((module_motor_update(me->left_friction_motor, delta_time_s) != MODULE_MOTOR_STATUS_OK) ||
        (module_motor_update(me->right_friction_motor, delta_time_s) != MODULE_MOTOR_STATUS_OK) ||
        (module_motor_update(me->feeder_motor, delta_time_s) != MODULE_MOTOR_STATUS_OK))
    {
        return MODULE_SHOOTER_STATUS_MOTOR_ERROR;
    }
    return MODULE_SHOOTER_STATUS_OK;
}

/* ======================== 公共 API ======================== */

/**
 * @brief 初始化发射机构
 * @param me 发射机构对象
 * @param config 配置参数
 * @return 执行状态
 */
module_shooter_status_t module_shooter_init(module_shooter_t *me,
                                            const module_shooter_config_t *config)
{
    // ---- 参数校验 ----
    if ((me == NULL) || (config == NULL) || (config->left_friction_motor == NULL) ||
        (config->right_friction_motor == NULL) || (config->feeder_motor == NULL) ||
        !config->left_friction_motor->is_initialized ||
        !config->right_friction_motor->is_initialized || !config->feeder_motor->is_initialized ||
        !module_shooter_direction_is_valid(config->left_friction_direction_sign) ||
        !module_shooter_direction_is_valid(config->right_friction_direction_sign) ||
        !module_shooter_direction_is_valid(config->feeder_direction_sign) ||
        !isfinite(config->feeder_step_rad) || (config->feeder_step_rad <= 0.0F) ||
        !isfinite(config->feeder_position_tolerance_rad) ||
        (config->feeder_position_tolerance_rad < 0.0F) ||
        !isfinite(config->jam_velocity_threshold_rad_per_s) ||
        (config->jam_velocity_threshold_rad_per_s < 0.0F) ||
        !isfinite(config->jam_current_threshold_a) || (config->jam_current_threshold_a < 0.0F) ||
        !isfinite(config->jam_confirmation_time_s) || (config->jam_confirmation_time_s <= 0.0F) ||
        !isfinite(config->rollback_angle_rad) || (config->rollback_angle_rad <= 0.0F) ||
        !isfinite(config->rollback_position_tolerance_rad) ||
        (config->rollback_position_tolerance_rad < 0.0F) || (config->maximum_jam_retries == 0U) ||
        (config->maximum_pending_shots == 0U))
    {
        return MODULE_SHOOTER_STATUS_INVALID_ARGUMENT;
    }

    // ---- 初始化对象 ----
    *me = (module_shooter_t){
        .left_friction_motor = config->left_friction_motor,
        .right_friction_motor = config->right_friction_motor,
        .feeder_motor = config->feeder_motor,
        .left_friction_direction_sign = config->left_friction_direction_sign,
        .right_friction_direction_sign = config->right_friction_direction_sign,
        .feeder_direction_sign = config->feeder_direction_sign,
        .feeder_step_rad = config->feeder_step_rad,
        .feeder_position_tolerance_rad = config->feeder_position_tolerance_rad,
        .jam_velocity_threshold_rad_per_s = config->jam_velocity_threshold_rad_per_s,
        .jam_current_threshold_a = config->jam_current_threshold_a,
        .jam_current_threshold_raw = config->jam_current_threshold_raw,
        .jam_confirmation_time_s = config->jam_confirmation_time_s,
        .rollback_angle_rad = config->rollback_angle_rad,
        .rollback_position_tolerance_rad = config->rollback_position_tolerance_rad,
        .maximum_pending_shots = config->maximum_pending_shots,
        .maximum_jam_retries = config->maximum_jam_retries,
        .state = MODULE_SHOOTER_STATE_DISABLED,
        .is_initialized = true,
    };
    return MODULE_SHOOTER_STATUS_OK;
}

/**
 * @brief 使能发射机构
 * @param me 发射机构对象
 * @return 执行状态
 * @note 使能三个电机，读取拨弹当前位置作为目标，状态转入 READY
 */
module_shooter_status_t module_shooter_enable(module_shooter_t *me)
{
    const module_motor_feedback_t *feeder_feedback;

    if (me == NULL)
    {
        return MODULE_SHOOTER_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return MODULE_SHOOTER_STATUS_NOT_INITIALIZED;
    }

    // 使能三个电机
    if ((module_motor_enable(me->left_friction_motor) != MODULE_MOTOR_STATUS_OK) ||
        (module_motor_enable(me->right_friction_motor) != MODULE_MOTOR_STATUS_OK) ||
        (module_motor_enable(me->feeder_motor) != MODULE_MOTOR_STATUS_OK))
    {
        (void)module_shooter_disable(me); // 回滚：禁用所有电机
        return MODULE_SHOOTER_STATUS_MOTOR_ERROR;
    }

    // 读取拨弹电机当前位置，作为初始目标
    feeder_feedback = module_motor_get_feedback(me->feeder_motor);
    if ((feeder_feedback == NULL) || !feeder_feedback->is_online)
    {
        (void)module_shooter_disable(me);
        return MODULE_SHOOTER_STATUS_NOT_READY;
    }
    me->feeder_target_position_rad = feeder_feedback->position_rad;
    me->state = MODULE_SHOOTER_STATE_READY;
    return MODULE_SHOOTER_STATUS_OK;
}

/**
 * @brief 禁用发射机构
 * @param me 发射机构对象
 * @return 执行状态
 * @note 禁用三个电机，清空待发队列，关闭摩擦轮
 */
module_shooter_status_t module_shooter_disable(module_shooter_t *me)
{
    module_shooter_status_t status = MODULE_SHOOTER_STATUS_OK;

    if (me == NULL)
    {
        return MODULE_SHOOTER_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return MODULE_SHOOTER_STATUS_NOT_INITIALIZED;
    }

    // 禁用三个电机
    if ((module_motor_disable(me->left_friction_motor) != MODULE_MOTOR_STATUS_OK) ||
        (module_motor_disable(me->right_friction_motor) != MODULE_MOTOR_STATUS_OK) ||
        (module_motor_disable(me->feeder_motor) != MODULE_MOTOR_STATUS_OK))
    {
        status = MODULE_SHOOTER_STATUS_MOTOR_ERROR;
    }

    // 清空状态
    me->pending_shots = 0U;
    me->friction_enabled = false;
    me->state = MODULE_SHOOTER_STATE_DISABLED;
    return status;
}

/**
 * @brief 设置摩擦轮使能状态和目标速度
 * @param me 发射机构对象
 * @param is_enabled true=使能，false=停止
 * @param target_velocity_rad_per_s 目标速度（rad/s，非负）
 * @return 执行状态
 */
module_shooter_status_t module_shooter_set_friction(module_shooter_t *me, bool is_enabled,
                                                    float target_velocity_rad_per_s)
{
    if ((me == NULL) || !isfinite(target_velocity_rad_per_s) || (target_velocity_rad_per_s < 0.0F))
    {
        return MODULE_SHOOTER_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return MODULE_SHOOTER_STATUS_NOT_INITIALIZED;
    }
    me->friction_enabled = is_enabled;
    me->friction_target_velocity_rad_per_s = target_velocity_rad_per_s;
    return MODULE_SHOOTER_STATUS_OK;
}

/**
 * @brief 请求发射指定数量的弹丸
 * @param me 发射机构对象
 * @param shot_count 请求发射数量
 * @return 执行状态
 * @note 累加到待发队列，但不超过 maximum_pending_shots
 */
module_shooter_status_t module_shooter_request_shots(module_shooter_t *me, uint16_t shot_count)
{
    if ((me == NULL) || (shot_count == 0U))
    {
        return MODULE_SHOOTER_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return MODULE_SHOOTER_STATUS_NOT_INITIALIZED;
    }
    if (me->state == MODULE_SHOOTER_STATE_FAULT)
    {
        return MODULE_SHOOTER_STATUS_FAULT;
    }
    // 检查是否超出最大待发量
    if ((uint32_t)me->pending_shots + shot_count > me->maximum_pending_shots)
    {
        return MODULE_SHOOTER_STATUS_INVALID_ARGUMENT;
    }
    me->pending_shots = (uint16_t)(me->pending_shots + shot_count);
    return MODULE_SHOOTER_STATUS_OK;
}

/**
 * @brief 取消所有待发射请求
 * @param me 发射机构对象
 * @return 执行状态
 */
module_shooter_status_t module_shooter_cancel_shots(module_shooter_t *me)
{
    if (me == NULL)
    {
        return MODULE_SHOOTER_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return MODULE_SHOOTER_STATUS_NOT_INITIALIZED;
    }
    me->pending_shots = 0U;
    return MODULE_SHOOTER_STATUS_OK;
}

/**
 * @brief 清除故障状态
 * @param me 发射机构对象
 * @return 执行状态
 * @note 需要拨弹电机在线且反馈有效，将当前实际位置设为目标
 */
module_shooter_status_t module_shooter_reset_fault(module_shooter_t *me)
{
    const module_motor_feedback_t *feeder_feedback;

    if (me == NULL)
    {
        return MODULE_SHOOTER_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return MODULE_SHOOTER_STATUS_NOT_INITIALIZED;
    }
    if (me->state != MODULE_SHOOTER_STATE_FAULT)
    {
        return MODULE_SHOOTER_STATUS_OK;
    }

    // 检查拨弹电机在线
    feeder_feedback = module_motor_get_feedback(me->feeder_motor);
    if ((feeder_feedback == NULL) || !feeder_feedback->is_online)
    {
        return MODULE_SHOOTER_STATUS_NOT_READY;
    }

    // 重置状态：目标位置对齐当前位置
    me->feeder_target_position_rad = feeder_feedback->position_rad;
    me->jam_retry_count = 0U;
    me->jam_elapsed_time_s = 0.0F;
    me->state = MODULE_SHOOTER_STATE_READY;
    return MODULE_SHOOTER_STATUS_OK;
}

/**
 * @brief 周期更新状态机
 * @param me 发射机构对象
 * @param delta_time_s 时间步长（秒）
 * @return 执行状态
 * @note 核心逻辑：处理待发请求、推进送弹、检测堵转、执行回退、故障判断
 */
module_shooter_status_t module_shooter_update(module_shooter_t *me, float delta_time_s)
{
    const module_motor_feedback_t *feeder_feedback;
    float position_error_rad;
    module_shooter_status_t status;

    // ---- 参数校验 ----
    if ((me == NULL) || !isfinite(delta_time_s) || (delta_time_s <= 0.0F))
    {
        return MODULE_SHOOTER_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return MODULE_SHOOTER_STATUS_NOT_INITIALIZED;
    }

    // ---- 状态检查 ----
    if (me->state == MODULE_SHOOTER_STATE_DISABLED)
    {
        return MODULE_SHOOTER_STATUS_NOT_READY;
    }
    if (me->state == MODULE_SHOOTER_STATE_FAULT)
    {
        return MODULE_SHOOTER_STATUS_FAULT;
    }

    // ---- 获取拨弹电机反馈 ----
    feeder_feedback = module_motor_get_feedback(me->feeder_motor);
    if ((feeder_feedback == NULL) || !feeder_feedback->is_online)
    {
        return MODULE_SHOOTER_STATUS_NOT_READY;
    }

    // ---- 计算位置误差（目标 - 实际） ----
    position_error_rad = me->feeder_target_position_rad - feeder_feedback->position_rad;

    // ---- 状态机逻辑 ----

    /* 状态：READY 且有待发请求 -> 进入 FEEDING */
    if ((me->state == MODULE_SHOOTER_STATE_READY) && (me->pending_shots > 0U))
    {
        // 目标位置增加一步（方向由 feeder_direction_sign 决定）
        me->feeder_target_position_rad += me->feeder_direction_sign * me->feeder_step_rad;
        me->jam_elapsed_time_s = 0.0F;
        me->state = MODULE_SHOOTER_STATE_FEEDING;
    }
    /* 状态：FEEDING -> 等待到位或检测堵转 */
    else if (me->state == MODULE_SHOOTER_STATE_FEEDING)
    {
        // 已到位（误差在容差内）
        if (fabsf(position_error_rad) <= me->feeder_position_tolerance_rad)
        {
            --me->pending_shots;      // 成功发射一发
            me->jam_retry_count = 0U; // 重置重试计数
            me->jam_elapsed_time_s = 0.0F;
            me->state = MODULE_SHOOTER_STATE_READY;
        }
        // 堵转检测：速度低于阈值 && 电流超过阈值
        else if ((fabsf(feeder_feedback->velocity_rad_per_s) <=
                  me->jam_velocity_threshold_rad_per_s) &&
                 module_shooter_jam_current_detected(me, feeder_feedback))
        {
            me->jam_elapsed_time_s += delta_time_s;
            // 达到堵转确认时间
            if (me->jam_elapsed_time_s >= me->jam_confirmation_time_s)
            {
                ++me->jam_retry_count;
                // 超过最大重试 -> 故障
                if (me->jam_retry_count > me->maximum_jam_retries)
                {
                    me->state = MODULE_SHOOTER_STATE_FAULT;
                    return MODULE_SHOOTER_STATUS_FAULT;
                }
                // 执行回退：目标位置后退 rollback_angle_rad
                me->feeder_target_position_rad =
                    feeder_feedback->position_rad -
                    (me->feeder_direction_sign * me->rollback_angle_rad);
                me->jam_elapsed_time_s = 0.0F;
                me->state = MODULE_SHOOTER_STATE_ROLLBACK;
            }
        }
        else
        {
            // 非堵转状态，重置堵转计时
            me->jam_elapsed_time_s = 0.0F;
        }
    }
    /* 状态：ROLLBACK -> 等待回退到位，然后重新进入 FEEDING */
    else if ((me->state == MODULE_SHOOTER_STATE_ROLLBACK) &&
             (fabsf(position_error_rad) <= me->rollback_position_tolerance_rad))
    {
        // 回退到位，再次尝试前进一步
        me->feeder_target_position_rad =
            feeder_feedback->position_rad + (me->feeder_direction_sign * me->feeder_step_rad);
        me->jam_elapsed_time_s = 0.0F;
        me->state = MODULE_SHOOTER_STATE_FEEDING;
    }

    // ---- 设置电机目标 ----
    status = module_shooter_set_motor_targets(me);
    if (status != MODULE_SHOOTER_STATUS_OK)
    {
        return status;
    }

    // ---- 更新电机控制周期 ----
    return module_shooter_update_motors(me, delta_time_s);
}

/**
 * @brief 获取当前状态
 * @param me 发射机构对象
 * @return 当前状态
 */
module_shooter_state_t module_shooter_get_state(const module_shooter_t *me)
{
    return ((me != NULL) && me->is_initialized) ? me->state : MODULE_SHOOTER_STATE_DISABLED;
}

/**
 * @brief 获取待发弹量
 * @param me 发射机构对象
 * @return 待发弹量
 */
uint16_t module_shooter_get_pending_shots(const module_shooter_t *me)
{
    return ((me != NULL) && me->is_initialized) ? me->pending_shots : 0U;
}

/**
 * @brief 获取当前堵重重试次数
 * @param me 发射机构对象
 * @return 重试次数
 */
uint8_t module_shooter_get_jam_retry_count(const module_shooter_t *me)
{
    return ((me != NULL) && me->is_initialized) ? me->jam_retry_count : 0U;
}