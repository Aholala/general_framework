/**
 * @file module_m2006.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 大疆 M2006 电机（C610 电调）专用派生模块实现
 * @version 1.0
 * @date 2026-07-28
 * @copyright Copyright (c) 2026
 *
 * @note 固定电机型号为 M2006，使用 C610 电调的命令范围 [-10000, 10000]，
 *       默认减速比 36.0F，防止调用者误配成 M3508 或 GM6020。
 *       所有功能通过转发到 module_dji_motor 实现。
 */

#include "module_m2006.h"

MODULE_MOTOR_STATIC_ASSERT_SUPER_FIRST(module_m2006_t);

/**
 * @brief 将 M2006 控制模式映射到大疆通用控制模式
 * @param control_mode M2006 控制模式
 * @return 对应的 module_dji_control_mode_t
 * @note CURRENT → DIRECT，VELOCITY → VELOCITY，POSITION → POSITION
 */
static module_dji_control_mode_t
module_m2006_map_control_mode(module_m2006_control_mode_t control_mode)
{
    static const module_dji_control_mode_t control_mode_map[] = {
        MODULE_DJI_CONTROL_DIRECT,   // MODULE_M2006_CONTROL_CURRENT
        MODULE_DJI_CONTROL_VELOCITY, // MODULE_M2006_CONTROL_VELOCITY
        MODULE_DJI_CONTROL_POSITION, // MODULE_M2006_CONTROL_POSITION
    };
    return control_mode_map[control_mode];
}

/**
 * @brief 校验 M2006 的控制模式是否匹配
 * @param me 电机对象
 * @param expected_control_mode 期望的控制模式
 * @return 执行状态
 * @note 若控制模式不匹配，返回 UNSUPPORTED，防止电流误当速度或位置
 */
static module_motor_status_t
module_m2006_validate_control_mode(const module_m2006_t *const me,
                                   module_m2006_control_mode_t expected_control_mode)
{
    if (me == NULL)
    {
        return MODULE_MOTOR_STATUS_INVALID_ARGUMENT;
    }
    if (!me->super.super.is_initialized)
    {
        return MODULE_MOTOR_STATUS_NOT_INITIALIZED;
    }
    // 检查当前配置的控制模式是否与期望一致
    if (me->super.control_mode != module_m2006_map_control_mode(expected_control_mode))
    {
        return MODULE_MOTOR_STATUS_UNSUPPORTED;
    }
    return MODULE_MOTOR_STATUS_OK;
}

/* ======================== 初始化与注册 ======================== */

/**
 * @brief 初始化 M2006 电机
 * @param me 电机对象
 * @param config 配置参数
 * @return 执行状态
 * @note 构建 module_dji_motor_config_t，固定 motor_model = MODULE_DJI_MOTOR_M2006
 *       M2006 默认减速比为 36.0F（由 module_dji_motor 内部自动设置）
 */
module_motor_status_t module_m2006_init(module_m2006_t *const me,
                                        const module_m2006_config_t *const config)
{
    module_dji_motor_config_t dji_motor_config;

    // 参数校验
    if ((me == NULL) || (config == NULL) || (config->control_mode > MODULE_M2006_CONTROL_POSITION))
    {
        return MODULE_MOTOR_STATUS_INVALID_ARGUMENT;
    }

    // 构建大疆电机配置，固定型号为 M2006
    dji_motor_config = (module_dji_motor_config_t){
        .logical_name = config->logical_name,
        .registration_key = config->registration_key,
        .motor_bus = config->motor_bus,
        .motor_model = MODULE_DJI_MOTOR_M2006, // 固定为 M2006
        .control_mode = module_m2006_map_control_mode(config->control_mode),
        .motor_identifier = config->motor_identifier,
        .direction_sign = config->direction_sign,
        .maximum_temperature_c = config->maximum_temperature_c,
        .current_scale_a_per_count = config->current_scale_a_per_count,
        .velocity_pid_config = config->velocity_pid_config,
        .position_pid_config = config->position_pid_config,
    };
    // 调用大疆电机基类初始化
    return module_dji_motor_init(&me->super, &dji_motor_config);
}

/**
 * @brief 注册电机到电机注册表
 */
module_motor_status_t module_m2006_register(module_m2006_t *const me,
                                            module_motor_registry_t *const registry)
{
    return (me != NULL) ? module_dji_motor_register(&me->super, registry)
                        : MODULE_MOTOR_STATUS_INVALID_ARGUMENT;
}

/**
 * @brief 从电机注册表注销电机
 */
module_motor_status_t module_m2006_unregister(module_m2006_t *const me,
                                              module_motor_registry_t *const registry)
{
    return (me != NULL) ? module_dji_motor_unregister(&me->super, registry)
                        : MODULE_MOTOR_STATUS_INVALID_ARGUMENT;
}

/* ======================== 类型转换 ======================== */

/**
 * @brief 向上转型为 module_motor_t
 */
module_motor_t *module_m2006_as_motor(module_m2006_t *const me)
{
    return (me != NULL) ? module_dji_motor_as_base(&me->super) : NULL;
}

/**
 * @brief 向上转型为 module_dji_motor_t
 */
module_dji_motor_t *module_m2006_as_dji_motor(module_m2006_t *const me)
{
    return (me != NULL) ? &me->super : NULL;
}

/* ======================== 控制接口 ======================== */

/**
 * @brief 使能电机
 */
module_motor_status_t module_m2006_enable(module_m2006_t *const me)
{
    return module_motor_enable(module_m2006_as_motor(me));
}

/**
 * @brief 禁用电机
 */
module_motor_status_t module_m2006_disable(module_m2006_t *const me)
{
    return module_motor_disable(module_m2006_as_motor(me));
}

/**
 * @brief 设置原始电流命令（电流模式）
 * @param me 电机对象
 * @param current_command_raw 电流命令（-10000~10000）
 * @return 执行状态
 * @note 仅当控制模式为 CURRENT 时有效，否则返回 UNSUPPORTED
 *       C610 电调电流命令范围 [-10000, 10000]
 */
module_motor_status_t module_m2006_set_current_command_raw(module_m2006_t *const me,
                                                           int16_t current_command_raw)
{
    module_motor_status_t status =
        module_m2006_validate_control_mode(me, MODULE_M2006_CONTROL_CURRENT);
    if (status != MODULE_MOTOR_STATUS_OK)
    {
        return status;
    }
    // DIRECT 模式的目标值直接作为命令值（原始电流）
    return module_motor_set_target(module_m2006_as_motor(me), (float)current_command_raw);
}

/**
 * @brief 设置速度目标（速度模式）
 * @param me 电机对象
 * @param velocity_rad_per_s 速度目标（rad/s）
 * @return 执行状态
 * @note 仅当控制模式为 VELOCITY 时有效，否则返回 UNSUPPORTED
 */
module_motor_status_t module_m2006_set_velocity_rad_per_s(module_m2006_t *const me,
                                                          float velocity_rad_per_s)
{
    module_motor_status_t status =
        module_m2006_validate_control_mode(me, MODULE_M2006_CONTROL_VELOCITY);
    if (status != MODULE_MOTOR_STATUS_OK)
    {
        return status;
    }
    return module_motor_set_target(module_m2006_as_motor(me), velocity_rad_per_s);
}

/**
 * @brief 设置位置目标（位置模式）
 * @param me 电机对象
 * @param position_rad 位置目标（弧度）
 * @return 执行状态
 * @note 仅当控制模式为 POSITION 时有效，否则返回 UNSUPPORTED
 */
module_motor_status_t module_m2006_set_position_rad(module_m2006_t *const me, float position_rad)
{
    module_motor_status_t status =
        module_m2006_validate_control_mode(me, MODULE_M2006_CONTROL_POSITION);
    if (status != MODULE_MOTOR_STATUS_OK)
    {
        return status;
    }
    return module_motor_set_target(module_m2006_as_motor(me), position_rad);
}

/**
 * @brief 周期更新电机
 * @param me 电机对象
 * @param delta_time_s 时间步长（秒）
 * @return 执行状态
 */
module_motor_status_t module_m2006_update(module_m2006_t *const me, float delta_time_s)
{
    return module_motor_update(module_m2006_as_motor(me), delta_time_s);
}

/* ======================== 反馈与状态 ======================== */

/**
 * @brief 获取电机反馈数据
 * @param me 电机对象
 * @return 反馈数据指针，未初始化或离线返回 NULL
 */
const module_motor_feedback_t *module_m2006_get_feedback(const module_m2006_t *const me)
{
    return (me != NULL) ? module_motor_get_feedback(&me->super.super) : NULL;
}

/**
 * @brief 获取当前原始电流命令值
 * @param me 电机对象
 * @return 当前命令值（-10000~10000）
 */
int16_t module_m2006_get_current_command_raw(const module_m2006_t *const me)
{
    return (me != NULL) ? module_dji_motor_get_command(&me->super) : 0;
}
