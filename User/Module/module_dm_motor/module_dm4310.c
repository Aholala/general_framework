/**
 * @file module_dm4310.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 达妙 DM4310 电机专用派生模块实现
 * @version 1.0
 * @date 2026-07-28
 * @copyright Copyright (c) 2026
 *
 * @note 继承自 module_dm_motor_t，提供型号专用接口。
 *       所有功能通过转发到 module_dm_motor 实现，不增加额外逻辑。
 *       控制模式枚举映射、协议范围配置和 CAN ID 偏移由本模块处理。
 */

#include "module_dm4310.h"

MODULE_MOTOR_STATIC_ASSERT_SUPER_FIRST(module_dm4310_t);

/**
 * @brief 将 DM4310 控制模式映射到达妙通用控制模式
 * @param control_mode DM4310 控制模式
 * @return 对应的 module_dm_control_mode_t
 * @note 两种枚举完全对应，但独立命名以避免混淆
 */
static module_dm_control_mode_t
module_dm4310_map_control_mode(module_dm4310_control_mode_t control_mode)
{
    static const module_dm_control_mode_t control_mode_map[] = {
        MODULE_DM_MODE_MIT,               // MODULE_DM4310_CONTROL_MIT
        MODULE_DM_MODE_VELOCITY,          // MODULE_DM4310_CONTROL_VELOCITY
        MODULE_DM_MODE_POSITION_VELOCITY, // MODULE_DM4310_CONTROL_POSITION_VELOCITY
        MODULE_DM_MODE_FORCE_POSITION,    // MODULE_DM4310_CONTROL_FORCE_POSITION
    };
    return control_mode_map[control_mode];
}

/* ======================== 初始化与注册 ======================== */

/**
 * @brief 初始化 DM4310 电机
 * @param me 电机对象
 * @param config 配置参数
 * @return 执行状态
 * @note 构建 module_dm_motor_config_t 并调用基类初始化
 */
module_motor_status_t module_dm4310_init(module_dm4310_t *const me,
                                         const module_dm4310_config_t *const config)
{
    module_dm_motor_config_t dm_motor_config;

    // 参数校验
    if ((me == NULL) || (config == NULL) ||
        (config->control_mode > MODULE_DM4310_CONTROL_FORCE_POSITION))
    {
        return MODULE_MOTOR_STATUS_INVALID_ARGUMENT;
    }

    // 构建达妙电机配置
    dm_motor_config = (module_dm_motor_config_t){
        .logical_name = config->logical_name,
        .registration_key = config->registration_key,
        .can = config->can,
        .control_mode = module_dm4310_map_control_mode(config->control_mode),
        .master_identifier = config->base_command_identifier,
        .feedback_identifier = config->feedback_identifier,
        .transmit_timeout_ms = config->transmit_timeout_ms,
        .limits = config->protocol_limits, // 协议范围直接透传
    };
    // 调用达妙电机基类初始化
    return module_dm_motor_init(&me->super, &dm_motor_config);
}

/**
 * @brief 注册电机到电机注册表
 * @param me 电机对象
 * @param registry 电机注册表
 * @return 执行状态
 */
module_motor_status_t module_dm4310_register(module_dm4310_t *const me,
                                             module_motor_registry_t *const registry)
{
    return (me != NULL) ? module_dm_motor_register(&me->super, registry)
                        : MODULE_MOTOR_STATUS_INVALID_ARGUMENT;
}

/**
 * @brief 从电机注册表注销电机
 */
module_motor_status_t module_dm4310_unregister(module_dm4310_t *const me,
                                               module_motor_registry_t *const registry)
{
    return (me != NULL) ? module_dm_motor_unregister(&me->super, registry)
                        : MODULE_MOTOR_STATUS_INVALID_ARGUMENT;
}

/* ======================== 类型转换 ======================== */

/**
 * @brief 向上转型为 module_motor_t
 * @param me 电机对象
 * @return 基类指针
 */
module_motor_t *module_dm4310_as_motor(module_dm4310_t *const me)
{
    return (me != NULL) ? module_dm_motor_as_base(&me->super) : NULL;
}

/**
 * @brief 向上转型为 module_dm_motor_t
 * @param me 电机对象
 * @return 达妙电机基类指针
 */
module_dm_motor_t *module_dm4310_as_dm_motor(module_dm4310_t *const me)
{
    return (me != NULL) ? &me->super : NULL;
}

/* ======================== 控制接口 ======================== */

/**
 * @brief 使能电机
 */
module_motor_status_t module_dm4310_enable(module_dm4310_t *const me)
{
    return module_motor_enable(module_dm4310_as_motor(me));
}

/**
 * @brief 禁用电机
 */
module_motor_status_t module_dm4310_disable(module_dm4310_t *const me)
{
    return module_motor_disable(module_dm4310_as_motor(me));
}

/**
 * @brief 执行 MIT 模式命令
 */
module_motor_status_t module_dm4310_command_mit(module_dm4310_t *const me,
                                                const module_dm_mit_command_t *const command)
{
    return (me != NULL) ? module_dm_motor_command_mit(&me->super, command)
                        : MODULE_MOTOR_STATUS_INVALID_ARGUMENT;
}

/**
 * @brief 执行速度模式命令
 */
module_motor_status_t module_dm4310_command_velocity(module_dm4310_t *const me,
                                                     float velocity_rad_per_s)
{
    return (me != NULL) ? module_dm_motor_command_velocity(&me->super, velocity_rad_per_s)
                        : MODULE_MOTOR_STATUS_INVALID_ARGUMENT;
}

/**
 * @brief 执行位置速度模式命令
 */
module_motor_status_t module_dm4310_command_position_velocity(module_dm4310_t *const me,
                                                              float position_rad,
                                                              float velocity_rad_per_s)
{
    return (me != NULL) ? module_dm_motor_command_position_velocity(&me->super, position_rad,
                                                                    velocity_rad_per_s)
                        : MODULE_MOTOR_STATUS_INVALID_ARGUMENT;
}

/**
 * @brief 执行力位混合模式命令
 */
module_motor_status_t
module_dm4310_command_force_position(module_dm4310_t *const me,
                                     const module_dm_force_position_command_t *const command)
{
    return (me != NULL) ? module_dm_motor_command_force_position(&me->super, command)
                        : MODULE_MOTOR_STATUS_INVALID_ARGUMENT;
}

/* ======================== 特殊命令 ======================== */

/**
 * @brief 将当前输出轴位置设为零位
 * @param me 电机对象
 * @return 执行状态
 * @note 仅允许在 DISABLED 状态下调用！
 *       该命令会改变输出轴位置参考，只能在明确的标定流程中执行。
 */
module_motor_status_t module_dm4310_set_zero_position(module_dm4310_t *const me)
{
    if (me == NULL)
    {
        return MODULE_MOTOR_STATUS_INVALID_ARGUMENT;
    }
    // 安全检查：必须在 DISABLED 状态
    if (me->super.super.state != MODULE_MOTOR_STATE_DISABLED)
    {
        return MODULE_MOTOR_STATUS_UNSUPPORTED;
    }
    return module_dm_motor_send_state_command(&me->super, MODULE_DM_COMMAND_SET_ZERO);
}

/**
 * @brief 兼容旧接口：将当前输出轴位置设为零位
 */
module_motor_status_t module_dm4310_save_zero_position(module_dm4310_t *const me)
{
    return module_dm4310_set_zero_position(me);
}

/**
 * @brief 清除驱动器故障
 */
module_motor_status_t module_dm4310_clear_fault(module_dm4310_t *const me)
{
    return (me != NULL)
               ? module_dm_motor_send_state_command(&me->super, MODULE_DM_COMMAND_CLEAR_FAULT)
               : MODULE_MOTOR_STATUS_INVALID_ARGUMENT;
}

/* ======================== 反馈与状态 ======================== */

/**
 * @brief 处理 CAN 反馈帧
 */
module_motor_status_t module_dm4310_handle_feedback(module_dm4310_t *const me,
                                                    const bsp_can_frame_t *const frame)
{
    return (me != NULL) ? module_dm_motor_handle_feedback(&me->super, frame)
                        : MODULE_MOTOR_STATUS_INVALID_ARGUMENT;
}

/**
 * @brief 获取电机反馈数据
 */
const module_motor_feedback_t *module_dm4310_get_feedback(const module_dm4310_t *const me)
{
    return (me != NULL) ? module_motor_get_feedback(&me->super.super) : NULL;
}

/**
 * @brief 获取驱动器故障码
 */
module_dm_fault_t module_dm4310_get_fault(const module_dm4310_t *const me)
{
    return (me != NULL) ? module_dm_motor_get_fault(&me->super)
                        : MODULE_DM_FAULT_COMMUNICATION_LOST;
}

/**
 * @brief 获取 MOS 管温度
 */
float module_dm4310_get_mos_temperature_c(const module_dm4310_t *const me)
{
    return (me != NULL) ? module_dm_motor_get_mos_temperature_c(&me->super) : 0.0F;
}
