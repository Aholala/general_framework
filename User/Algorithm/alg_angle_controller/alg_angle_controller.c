/**
 * @file alg_angle_controller.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 角度控制器多态接口实现
 * @version 1.0
 * @date 2026-07-28
 * @copyright Copyright (c) 2026
 *
 * @note 包含 PID 和 LQR 两种角度控制器的多态封装。
 *       基类提供统一的 reset/update 接口，派生类转发到底层算法。
 */

#include "alg_angle_controller.h"

#include <math.h>
#include <stddef.h>

/**
 * @brief container_of 宏，用于从基类指针获取派生对象
 * @param member_pointer 成员指针
 * @param container_type 容器类型
 * @param member_name 成员名称
 * @return 容器对象指针
 */
#define ALG_ANGLE_CONTROLLER_CONTAINER_OF(member_pointer, container_type, member_name) \
    ((container_type *)((unsigned char *)(member_pointer) - offsetof(container_type, member_name)))

/**
 * @brief 校验控制器输入是否有效
 * @param input 输入指针
 * @return true=有效
 * @note 检查所有字段是否为有限数，且 delta_time_s > 0
 */
static bool alg_angle_controller_input_is_valid(const alg_angle_controller_input_t *input)
{
    return (input != NULL) && isfinite(input->target_position_rad) &&
           isfinite(input->target_velocity_rad_per_s) && isfinite(input->measured_position_rad) &&
           isfinite(input->measured_velocity_rad_per_s) && isfinite(input->actuator_feedforward) &&
           isfinite(input->delta_time_s) && (input->delta_time_s > 0.0F);
}

/* ======================== PID 虚函数实现 ======================== */

/**
 * @brief PID 控制器重置（虚函数实现）
 * @param controller_base 基类指针
 * @param measured_position_rad 当前测量位置
 * @param measured_velocity_rad_per_s 当前测量速度
 * @param initial_output 初始输出值
 * @return 执行状态
 * @note 转发到 alg_pid_cascade_reset
 */
static alg_angle_controller_status_t
alg_angle_pid_reset_virtual(alg_angle_controller_t *controller_base,
                           float measured_position_rad,
                           float measured_velocity_rad_per_s,
                           float initial_output)
{
    // 从基类指针获取派生对象
    alg_angle_pid_t *const me =
        ALG_ANGLE_CONTROLLER_CONTAINER_OF(controller_base, alg_angle_pid_t, super);
    // 调用串级 PID 重置
    const alg_pid_status_t status = alg_pid_cascade_reset(
        &me->cascade, measured_position_rad, measured_velocity_rad_per_s, initial_output);
    return (status == ALG_PID_STATUS_OK) ? ALG_ANGLE_CONTROLLER_STATUS_OK
                                         : ALG_ANGLE_CONTROLLER_STATUS_ALGORITHM_ERROR;
}

/**
 * @brief PID 控制器更新（虚函数实现）
 * @param controller_base 基类指针
 * @param input 控制器输入
 * @param control_output 输出控制量
 * @return 执行状态
 * @note 转发到 alg_pid_cascade_update
 */
static alg_angle_controller_status_t
alg_angle_pid_update_virtual(alg_angle_controller_t *controller_base,
                            const alg_angle_controller_input_t *input, float *control_output)
{
    // 从基类指针获取派生对象
    alg_angle_pid_t *const me =
        ALG_ANGLE_CONTROLLER_CONTAINER_OF(controller_base, alg_angle_pid_t, super);

    // 构建串级 PID 输入
    const alg_pid_cascade_input_t cascade_input = {
        .position_setpoint = input->target_position_rad,
        .position_measurement = input->measured_position_rad,
        .velocity_measurement = input->measured_velocity_rad_per_s,
        .velocity_feedforward = input->target_velocity_rad_per_s,
        .actuator_feedforward = input->actuator_feedforward,
        .delta_time_s = input->delta_time_s,
    };

    // 调用串级 PID 更新
    const alg_pid_status_t status =
        alg_pid_cascade_update(&me->cascade, &cascade_input, control_output);
    return (status == ALG_PID_STATUS_OK) ? ALG_ANGLE_CONTROLLER_STATUS_OK
                                         : ALG_ANGLE_CONTROLLER_STATUS_ALGORITHM_ERROR;
}

/* ======================== LQR 虚函数实现 ======================== */

/**
 * @brief LQR 控制器重置（虚函数实现）
 * @param controller_base 基类指针（未使用）
 * @param measured_position_rad 当前测量位置
 * @param measured_velocity_rad_per_s 当前测量速度
 * @param initial_output 初始输出值
 * @return 执行状态
 * @note LQR 控制器无内部状态，只需校验参数有效性
 */
static alg_angle_controller_status_t
alg_angle_lqr_reset_virtual(alg_angle_controller_t *controller_base, float measured_position_rad,
                           float measured_velocity_rad_per_s, float initial_output)
{
    (void)controller_base; // LQR 无内部状态，不需要使用基类指针
    // 只需校验参数有效性
    return (isfinite(measured_position_rad) && isfinite(measured_velocity_rad_per_s) &&
            isfinite(initial_output))
               ? ALG_ANGLE_CONTROLLER_STATUS_OK
               : ALG_ANGLE_CONTROLLER_STATUS_INVALID_ARGUMENT;
}

/**
 * @brief LQR 控制器更新（虚函数实现）
 * @param controller_base 基类指针
 * @param input 控制器输入
 * @param control_output 输出控制量
 * @return 执行状态
 * @note 转发到 alg_lqr_controller_update
 */
static alg_angle_controller_status_t
alg_angle_lqr_update_virtual(alg_angle_controller_t *controller_base,
                            const alg_angle_controller_input_t *input,
                            float *control_output)
{
    // 从基类指针获取派生对象
    alg_angle_lqr_t *const me =
        ALG_ANGLE_CONTROLLER_CONTAINER_OF(controller_base, alg_angle_lqr_t, super);

    // 构建状态向量：[位置, 速度]
    const float state[2] = {input->measured_position_rad, input->measured_velocity_rad_per_s};
    // 构建参考状态：[目标位置, 目标速度]
    const float reference_state[2] = {input->target_position_rad, input->target_velocity_rad_per_s};
    // 前馈向量
    const float feedforward_control[1] = {input->actuator_feedforward};

    // 调用 LQR 控制器更新
    const alg_lqr_status_t status =
        alg_lqr_controller_update(&me->controller, state, reference_state, &me->equilibrium_control,
                                  feedforward_control, control_output);
    return (status == ALG_LQR_STATUS_OK) ? ALG_ANGLE_CONTROLLER_STATUS_OK
                                         : ALG_ANGLE_CONTROLLER_STATUS_ALGORITHM_ERROR;
}

/* ======================== 虚表定义 ======================== */

/** PID 控制器虚表（静态常量） */
static const alg_angle_controller_ops_t s_alg_angle_pid_ops = {
    .reset = alg_angle_pid_reset_virtual,
    .update = alg_angle_pid_update_virtual,
};

/** LQR 控制器虚表（静态常量） */
static const alg_angle_controller_ops_t s_alg_angle_lqr_ops = {
    .reset = alg_angle_lqr_reset_virtual,
    .update = alg_angle_lqr_update_virtual,
};

/* ======================== 构造函数 ======================== */

/**
 * @brief 初始化 PID 角度控制器
 * @param me PID 控制器对象
 * @param config 配置参数
 * @return 执行状态
 */
alg_angle_controller_status_t alg_angle_pid_init(alg_angle_pid_t *me,
                                               const alg_angle_pid_config_t *config)
{
    // 参数校验
    if ((me == NULL) || (config == NULL))
    {
        return ALG_ANGLE_CONTROLLER_STATUS_INVALID_ARGUMENT;
    }

    // 先标记为未初始化，避免中途失败留下半成品
    me->super.vptr = NULL;
    me->super.is_initialized = false;

    // 初始化串级 PID
    if (alg_pid_cascade_init(&me->cascade, &config->cascade_config) != ALG_PID_STATUS_OK)
    {
        return ALG_ANGLE_CONTROLLER_STATUS_ALGORITHM_ERROR;
    }

    // 绑定虚表并标记为已初始化
    me->super.vptr = &s_alg_angle_pid_ops;
    me->super.is_initialized = true;
    return ALG_ANGLE_CONTROLLER_STATUS_OK;
}

/**
 * @brief 初始化 LQR 角度控制器
 * @param me LQR 控制器对象
 * @param config 配置参数
 * @return 执行状态
 */
alg_angle_controller_status_t alg_angle_lqr_init(alg_angle_lqr_t *me,
                                               const alg_angle_lqr_config_t *config)
{
    alg_lqr_controller_config_t controller_config;

    // ---- 参数校验 ----
    if ((me == NULL) || (config == NULL) || (config->gain_matrix == NULL) ||
        !isfinite(config->gain_matrix[0]) || !isfinite(config->gain_matrix[1]) ||
        !isfinite(config->control_min) || !isfinite(config->control_max) ||
        !isfinite(config->equilibrium_control) || (config->control_min >= config->control_max))
    {
        return ALG_ANGLE_CONTROLLER_STATUS_INVALID_ARGUMENT;
    }

    // 先标记为未初始化
    me->super.vptr = NULL;
    me->super.is_initialized = false;

    // 复制配置到对象
    me->gain_matrix[0] = config->gain_matrix[0];
    me->gain_matrix[1] = config->gain_matrix[1];
    me->control_min = config->control_min;
    me->control_max = config->control_max;
    me->equilibrium_control = config->equilibrium_control;

    // 构建 LQR 控制器配置
    controller_config = (alg_lqr_controller_config_t){
        .state_dimension = 2U,   // 状态维度：[位置, 速度]
        .control_dimension = 1U, // 控制维度：单一输出
        .gain_matrix = me->gain_matrix,
        .control_min = &me->control_min,
        .control_max = &me->control_max,
    };

    // 初始化 LQR 控制器
    if (alg_lqr_controller_init(&me->controller, &controller_config) != ALG_LQR_STATUS_OK)
    {
        return ALG_ANGLE_CONTROLLER_STATUS_ALGORITHM_ERROR;
    }

    // 绑定虚表并标记为已初始化
    me->super.vptr = &s_alg_angle_lqr_ops;
    me->super.is_initialized = true;
    return ALG_ANGLE_CONTROLLER_STATUS_OK;
}

/* ======================== 向上转型（类型转换） ======================== */

/**
 * @brief 将 PID 控制器转为基类指针
 * @param me PID 控制器对象
 * @return 基类指针
 */
alg_angle_controller_t *alg_angle_pid_as_controller(alg_angle_pid_t *me)
{
    return (me != NULL) ? &me->super : NULL;
}

/**
 * @brief 将 LQR 控制器转为基类指针
 * @param me LQR 控制器对象
 * @return 基类指针
 */
alg_angle_controller_t *alg_angle_lqr_as_controller(alg_angle_lqr_t *me)
{
    return (me != NULL) ? &me->super : NULL;
}

/* ======================== 多态公共接口 ======================== */

/**
 * @brief 重置控制器内部状态（多态）
 * @param me 控制器基类指针
 * @param measured_position_rad 当前测量位置
 * @param measured_velocity_rad_per_s 当前测量速度
 * @param initial_output 初始输出值
 * @return 执行状态
 */
alg_angle_controller_status_t alg_angle_controller_reset(alg_angle_controller_t *me,
                                                       float measured_position_rad,
                                                       float measured_velocity_rad_per_s,
                                                       float initial_output)
{
    // ---- 参数校验 ----
    if ((me == NULL) || !me->is_initialized || (me->vptr == NULL) || (me->vptr->reset == NULL))
    {
        return (me == NULL) ? ALG_ANGLE_CONTROLLER_STATUS_INVALID_ARGUMENT
                            : ALG_ANGLE_CONTROLLER_STATUS_NOT_INITIALIZED;
    }
    // 通过虚表调用派生类的 reset 实现
    return me->vptr->reset(me, measured_position_rad, measured_velocity_rad_per_s, initial_output);
}

/**
 * @brief 更新控制器输出（多态）
 * @param me 控制器基类指针
 * @param input 控制器输入
 * @param control_output 输出控制量
 * @return 执行状态
 */
alg_angle_controller_status_t alg_angle_controller_update(alg_angle_controller_t *me,
                                                        const alg_angle_controller_input_t *input,
                                                        float *control_output)
{
    // ---- 参数校验 ----
    if ((me == NULL) || (control_output == NULL) || !alg_angle_controller_input_is_valid(input))
    {
        return ALG_ANGLE_CONTROLLER_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized || (me->vptr == NULL) || (me->vptr->update == NULL))
    {
        return ALG_ANGLE_CONTROLLER_STATUS_NOT_INITIALIZED;
    }
    // 通过虚表调用派生类的 update 实现
    return me->vptr->update(me, input, control_output);
}