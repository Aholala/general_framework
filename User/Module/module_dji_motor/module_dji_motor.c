/**
 * @file module_dji_motor.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 大疆 M2006、M3508 和 GM6020 电机 CAN 协议驱动实现
 * @version 1.0
 * @date 2026-07-28
 * @copyright Copyright (c) 2026
 *
 * @note 负责总线注册、反馈解码、编码器多圈累计、控制模式和分组电流帧发送。
 *       具体型号派生模块只补充型号参数与专用语义。
 */

#include "module_dji_motor.h"

#include <math.h>   // isfinite
#include <stddef.h> // NULL

MODULE_MOTOR_STATIC_ASSERT_SUPER_FIRST(module_dji_motor_t);

/** @brief 编码器每圈计数（13 位编码器，0~8191） */
#define MODULE_DJI_ENCODER_COUNTS_PER_REVOLUTION (8192.0F)
/** @brief 2π 常量 */
#define MODULE_DJI_TWO_PI (6.28318530717958647692F)

/**
 * @brief 从 module_motor_t 基类获取派生 module_dji_motor_t 对象
 * @param motor_base module_motor_t 基类指针
 * @return 派生对象指针
 */
static module_dji_motor_t *module_dji_motor_get_device(module_motor_t *const motor_base)
{
    return MODULE_MOTOR_CONTAINER_OF(motor_base, module_dji_motor_t, super);
}

/**
 * @brief 将浮点命令钳位到 int16 范围
 * @param command_value 浮点命令值
 * @param command_limit 命令限幅值（正数）
 * @return 钳位后的 int16 命令值
 */
static int16_t module_dji_motor_clamp_command(float command_value, int16_t command_limit)
{
    if (command_value > (float)command_limit)
    {
        command_value = (float)command_limit;
    }
    if (command_value < (float)-command_limit)
    {
        command_value = (float)-command_limit;
    }
    return (int16_t)command_value;
}

/* ======================== 虚函数实现（module_motor_ops_t） ======================== */

/**
 * @brief 使能电机（虚函数实现）
 * @param motor_base 基类指针
 * @return 执行状态
 */
static module_motor_status_t module_dji_motor_enable_virtual(module_motor_t *const motor_base)
{
    module_dji_motor_t *const me = module_dji_motor_get_device(motor_base);
    module_motor_status_t status;

    if (me->control_mode >= MODULE_DJI_CONTROL_CURRENT)
    {
        status = module_motor_pid_reset(&me->current_pid, motor_base->feedback.current_a, 0.0F);
        if (status != MODULE_MOTOR_STATUS_OK)
        {
            return status;
        }
    }
    if (me->control_mode >= MODULE_DJI_CONTROL_VELOCITY)
    {
        status = module_motor_pid_reset(&me->velocity_pid, motor_base->feedback.velocity_rad_per_s,
                                        motor_base->feedback.current_a);
        if (status != MODULE_MOTOR_STATUS_OK)
        {
            return status;
        }
    }
    if (me->control_mode >= MODULE_DJI_CONTROL_ANGLE)
    {
        status = module_motor_pid_reset(&me->angle_pid, motor_base->feedback.position_rad,
                                        motor_base->feedback.velocity_rad_per_s);
        if (status != MODULE_MOTOR_STATUS_OK)
        {
            return status;
        }
    }

    motor_base->state = MODULE_MOTOR_STATE_ENABLED;
    return MODULE_MOTOR_STATUS_OK;
}

/**
 * @brief 禁用电机（虚函数实现）
 * @param motor_base 基类指针
 * @return 执行状态
 */
static module_motor_status_t module_dji_motor_disable_virtual(module_motor_t *const motor_base)
{
    module_dji_motor_t *const me = module_dji_motor_get_device(motor_base);
    me->command_value = 0; // 清空命令
    motor_base->state = MODULE_MOTOR_STATE_DISABLED;
    return MODULE_MOTOR_STATUS_OK;
}

static module_motor_status_t
module_dji_motor_can_clear_fault_virtual(const module_motor_t *const motor_base)
{
    const module_dji_motor_t *const me =
        MODULE_MOTOR_CONTAINER_OF_CONST(motor_base, module_dji_motor_t, super);
    return (motor_base->feedback.motor_temperature_c <= me->maximum_temperature_c)
               ? MODULE_MOTOR_STATUS_OK
               : MODULE_MOTOR_STATUS_FEEDBACK_UNAVAILABLE;
}

/**
 * @brief 设置目标值（虚函数实现）
 * @param motor_base 基类指针
 * @param target_value 目标值（含义取决于控制模式）
 * @return 执行状态
 */
static module_motor_status_t module_dji_motor_set_target_virtual(module_motor_t *const motor_base,
                                                                 float target_value)
{
    module_dji_motor_t *const me = module_dji_motor_get_device(motor_base);
    if (!isfinite(target_value))
    {
        return MODULE_MOTOR_STATUS_OUT_OF_RANGE;
    }
    if (me->control_mode == MODULE_DJI_CONTROL_DIRECT)
    {
        me->direct_command_value = target_value;
    }
    else if (me->control_mode == MODULE_DJI_CONTROL_CURRENT)
    {
        me->target_current_a = target_value;
    }
    else if (me->control_mode == MODULE_DJI_CONTROL_VELOCITY)
    {
        me->target_velocity_rad_per_s = target_value;
    }
    else
    {
        me->target_angle_rad = target_value;
    }
    return MODULE_MOTOR_STATUS_OK;
}

/**
 * @brief 更新电机（虚函数实现）
 *        根据控制模式计算 command_value
 * @param motor_base 基类指针
 * @param delta_time_s 时间步长（秒）
 * @return 执行状态
 */
static module_motor_status_t module_dji_motor_update_virtual(module_motor_t *const motor_base,
                                                             float delta_time_s)
{
    module_dji_motor_t *const me = module_dji_motor_get_device(motor_base);
    float current_setpoint_a;
    float velocity_setpoint_rad_per_s;
    float command_output = 0.0F;
    module_motor_status_t status;

    // 禁用状态：命令清零
    if (motor_base->state != MODULE_MOTOR_STATE_ENABLED)
    {
        me->command_value = 0;
        return MODULE_MOTOR_STATUS_OK;
    }

    // ---- 根据控制模式计算控制器输出 ----
    if (me->control_mode == MODULE_DJI_CONTROL_DIRECT)
    {
        // 直通模式：目标值直接作为控制器输出
        command_output = me->direct_command_value;
    }
    else
    {
        if (!motor_base->feedback.is_online || !motor_base->feedback.is_current_a_valid)
        {
            return MODULE_MOTOR_STATUS_FEEDBACK_UNAVAILABLE;
        }

        current_setpoint_a = me->target_current_a;
        if (me->control_mode >= MODULE_DJI_CONTROL_VELOCITY)
        {
            velocity_setpoint_rad_per_s = me->target_velocity_rad_per_s;
            if (me->control_mode == MODULE_DJI_CONTROL_ANGLE)
            {
                status = module_motor_pid_update(&me->angle_pid, me->target_angle_rad,
                                                 motor_base->feedback.position_rad, delta_time_s,
                                                 &velocity_setpoint_rad_per_s);
                if (status != MODULE_MOTOR_STATUS_OK)
                {
                    return status;
                }
            }

            status = module_motor_pid_update(&me->velocity_pid, velocity_setpoint_rad_per_s,
                                             motor_base->feedback.velocity_rad_per_s, delta_time_s,
                                             &current_setpoint_a);
            if (status != MODULE_MOTOR_STATUS_OK)
            {
                return status;
            }
        }

        status =
            module_motor_pid_update(&me->current_pid, current_setpoint_a,
                                    motor_base->feedback.current_a, delta_time_s, &command_output);
        if (status != MODULE_MOTOR_STATUS_OK)
        {
            return status;
        }
    }

    // 钳位并应用方向符号
    me->command_value = module_dji_motor_clamp_command(command_output * me->direction_sign,
                                                       me->maximum_command_value);
    return MODULE_MOTOR_STATUS_OK;
}

/** 电机虚表（module_motor_ops_t） */
static const module_motor_ops_t s_module_dji_motor_ops = {
    .enable = module_dji_motor_enable_virtual,
    .disable = module_dji_motor_disable_virtual,
    .can_clear_fault = module_dji_motor_can_clear_fault_virtual,
    .set_target = module_dji_motor_set_target_virtual,
    .update = module_dji_motor_update_virtual};

/* ======================== 内部工具函数 ======================== */

/**
 * @brief 根据电机型号和标识符计算 CAN 接收 ID、组索引和槽位
 * @param config 电机配置
 * @param receive_identifier 输出：CAN 接收标识符
 * @param group_index 输出：组索引（0,1,2）
 * @param group_slot 输出：组内槽位（0~3）
 * @return 执行状态
 */
static module_motor_status_t
module_dji_motor_get_protocol_mapping(const module_dji_motor_config_t *const config,
                                      uint32_t *const receive_identifier,
                                      uint8_t *const group_index, uint8_t *const group_slot)
{
    uint8_t zero_based_identifier;

    // 校验电机标识符：1~8
    if ((config->motor_identifier == 0U) || (config->motor_identifier > 8U))
    {
        return MODULE_MOTOR_STATUS_OUT_OF_RANGE;
    }
    zero_based_identifier = (uint8_t)(config->motor_identifier - 1U);

    if (config->motor_model == MODULE_DJI_MOTOR_GM6020)
    {
        // GM6020：电压模式用 0x1FF/0x2FF，电流及级联模式用 0x1FE/0x2FE。
        if (config->motor_identifier > 7U)
        {
            return MODULE_MOTOR_STATUS_OUT_OF_RANGE;
        }
        *receive_identifier = 0x204U + config->motor_identifier;
        if (config->control_mode == MODULE_DJI_CONTROL_DIRECT)
        {
            *group_index = (config->motor_identifier <= 4U) ? 0U : 2U;
        }
        else
        {
            *group_index = (config->motor_identifier <= 4U) ? 3U : 4U;
        }
        *group_slot = (uint8_t)(zero_based_identifier % 4U);
    }
    else
    {
        // M2006/M3508：ID 1~8，接收 ID 0x201~0x208，组 1（ID 1~4）或组 0（ID 5~8）
        *receive_identifier = 0x200U + config->motor_identifier;
        *group_index = (config->motor_identifier <= 4U) ? 1U : 0U;
        *group_slot = (uint8_t)(zero_based_identifier % 4U);
    }
    return MODULE_MOTOR_STATUS_OK;
}

/* ======================== 总线 API ======================== */

/**
 * @brief 初始化 DJI 电机总线
 * @param me 总线对象
 * @param can CAN BSP 基类指针（已初始化）
 * @param transmit_timeout_ms CAN 发送超时（毫秒）
 * @return 执行状态
 */
module_motor_status_t module_dji_motor_bus_init(module_dji_motor_bus_t *const me,
                                                bsp_can_t *const can, uint32_t transmit_timeout_ms)
{
    size_t group_index;
    size_t slot_index;

    if ((me == NULL) || (can == NULL) || !bsp_device_is_initialized(&can->super))
    {
        return MODULE_MOTOR_STATUS_INVALID_ARGUMENT;
    }

    // 清空所有槽位
    for (group_index = 0U; group_index < MODULE_DJI_MOTOR_GROUP_COUNT; ++group_index)
    {
        me->group_is_used[group_index] = false;
        for (slot_index = 0U; slot_index < MODULE_DJI_MOTOR_PER_GROUP; ++slot_index)
        {
            me->motor_slots[group_index][slot_index] = NULL;
        }
    }

    me->can = can;
    me->transmit_timeout_ms = transmit_timeout_ms;
    me->is_initialized = true;
    return MODULE_MOTOR_STATUS_OK;
}

/**
 * @brief 初始化 DJI 电机实例
 * @param me 电机对象
 * @param config 配置参数
 * @return 执行状态
 */
module_motor_status_t module_dji_motor_init(module_dji_motor_t *const me,
                                            const module_dji_motor_config_t *const config)
{
    module_motor_status_t status;

    // ---- 参数校验 ----
    if ((me == NULL) || (config == NULL) || (config->motor_name == NULL) ||
        (config->motor_bus == NULL) || !config->motor_bus->is_initialized ||
        (config->motor_model > MODULE_DJI_MOTOR_GM6020) ||
        (config->control_mode > MODULE_DJI_CONTROL_ANGLE) ||
        ((config->direction_sign != 1.0F) && (config->direction_sign != -1.0F)) ||
        !isfinite(config->maximum_temperature_c) || (config->maximum_temperature_c <= 0.0F) ||
        !isfinite(config->current_scale_a_per_count) ||
        (config->position_reference > MODULE_DJI_POSITION_ENCODER_ABSOLUTE) ||
        (config->encoder_zero_count >= (uint16_t)MODULE_DJI_ENCODER_COUNTS_PER_REVOLUTION) ||
        !isfinite(config->position_offset_rad) ||
        ((config->control_mode != MODULE_DJI_CONTROL_DIRECT) &&
         (config->current_scale_a_per_count <= 0.0F)))
    {
        return MODULE_MOTOR_STATUS_INVALID_ARGUMENT;
    }

    // ---- 计算协议映射 ----
    status = module_dji_motor_get_protocol_mapping(config, &me->receive_identifier,
                                                   &me->group_index, &me->group_slot);
    if (status != MODULE_MOTOR_STATUS_OK)
    {
        return status;
    }

    // ---- 保存配置 ----
    me->motor_bus = config->motor_bus;
    me->motor_model = config->motor_model;
    me->control_mode = config->control_mode;
    me->motor_identifier = config->motor_identifier;
    me->direction_sign = config->direction_sign;
    me->maximum_temperature_c = config->maximum_temperature_c;
    me->current_scale_a_per_count = config->current_scale_a_per_count;
    me->position_reference = config->position_reference;
    me->encoder_zero_count = config->encoder_zero_count;
    me->position_offset_rad = config->position_offset_rad;

    // 减速比：M2006=36，M3508=3591/187（手册精确值），GM6020=1
    me->gear_ratio =
        (config->motor_model == MODULE_DJI_MOTOR_M2006)
            ? 36.0F
            : ((config->motor_model == MODULE_DJI_MOTOR_M3508) ? (3591.0F / 187.0F) : 1.0F);

    // 手册量程：M2006/C610=10000，M3508/C620=16384；
    // GM6020 电压模式=25000，电流及其级联模式=16384。
    if (config->motor_model == MODULE_DJI_MOTOR_M2006)
    {
        me->maximum_command_value = 10000;
    }
    else if (config->motor_model == MODULE_DJI_MOTOR_M3508)
    {
        me->maximum_command_value = 16384;
    }
    else
    {
        me->maximum_command_value =
            (config->control_mode == MODULE_DJI_CONTROL_DIRECT) ? 25000 : 16384;
    }

    // ---- 初始化状态 ----
    me->direct_command_value = 0.0F;
    me->target_current_a = 0.0F;
    me->target_velocity_rad_per_s = 0.0F;
    me->target_angle_rad = 0.0F;
    me->current_pid = (module_motor_pid_t){0};
    me->velocity_pid = (module_motor_pid_t){0};
    me->angle_pid = (module_motor_pid_t){0};
    me->command_value = 0;
    me->previous_encoder_count = 0U;
    me->accumulated_encoder_count = 0;
    me->has_previous_encoder_count = false;

    // ---- 按控制链初始化电流、速度和角度 PID ----
    if ((config->control_mode >= MODULE_DJI_CONTROL_CURRENT) &&
        (module_motor_pid_init(&me->current_pid, &config->current_pid_config) !=
         MODULE_MOTOR_STATUS_OK))
    {
        return MODULE_MOTOR_STATUS_INVALID_ARGUMENT;
    }
    if ((config->control_mode >= MODULE_DJI_CONTROL_VELOCITY) &&
        (module_motor_pid_init(&me->velocity_pid, &config->velocity_pid_config) !=
         MODULE_MOTOR_STATUS_OK))
    {
        return MODULE_MOTOR_STATUS_INVALID_ARGUMENT;
    }
    if ((config->control_mode == MODULE_DJI_CONTROL_ANGLE) &&
        (module_motor_pid_init(&me->angle_pid, &config->angle_pid_config) !=
         MODULE_MOTOR_STATUS_OK))
    {
        return MODULE_MOTOR_STATUS_INVALID_ARGUMENT;
    }

    // ---- 初始化基类 ----
    return module_motor_init_base(&me->super, &s_module_dji_motor_ops, config->motor_name,
                                  config->registration_key, config->motor_identifier);
}

/**
 * @brief 注册电机到总线槽位和电机注册表
 * @param me 电机对象
 * @param registry 电机注册表
 * @return 执行状态
 */
module_motor_status_t module_dji_motor_register(module_dji_motor_t *const me,
                                                module_motor_registry_t *const registry)
{
    module_motor_status_t status;
    size_t group_index;
    size_t slot_index;

    // ---- 参数校验 ----
    if ((me == NULL) || (registry == NULL) || !me->super.is_initialized)
    {
        return MODULE_MOTOR_STATUS_INVALID_ARGUMENT;
    }

    // 检查目标槽位是否已被占用
    if (me->motor_bus->motor_slots[me->group_index][me->group_slot] != NULL)
    {
        return MODULE_MOTOR_STATUS_DUPLICATE_KEY;
    }

    // 检查是否有其他电机使用相同的接收 ID
    for (group_index = 0U; group_index < MODULE_DJI_MOTOR_GROUP_COUNT; ++group_index)
    {
        for (slot_index = 0U; slot_index < MODULE_DJI_MOTOR_PER_GROUP; ++slot_index)
        {
            const module_dji_motor_t *const registered_motor =
                me->motor_bus->motor_slots[group_index][slot_index];
            if ((registered_motor != NULL) &&
                (registered_motor->receive_identifier == me->receive_identifier))
            {
                return MODULE_MOTOR_STATUS_DUPLICATE_KEY;
            }
        }
    }

    // ---- 注册到电机注册表 ----
    status = module_motor_registry_register(registry, &me->super);
    if (status == MODULE_MOTOR_STATUS_OK)
    {
        // 占用总线槽位
        me->motor_bus->motor_slots[me->group_index][me->group_slot] = me;
        me->motor_bus->group_is_used[me->group_index] = true;
    }
    return status;
}

/**
 * @brief 从总线槽位和电机注册表注销电机
 * @param me 电机对象
 * @param registry 电机注册表
 * @return 执行状态
 */
module_motor_status_t module_dji_motor_unregister(module_dji_motor_t *const me,
                                                  module_motor_registry_t *const registry)
{
    module_motor_status_t status;
    size_t slot_index;
    bool group_is_used = false;

    // ---- 参数校验 ----
    if ((me == NULL) || (registry == NULL))
    {
        return MODULE_MOTOR_STATUS_INVALID_ARGUMENT;
    }
    if (!me->super.is_registered || (me->motor_bus == NULL) || !me->motor_bus->is_initialized ||
        (me->motor_bus->motor_slots[me->group_index][me->group_slot] != me))
    {
        return MODULE_MOTOR_STATUS_NOT_REGISTERED;
    }

    // ---- 禁用电机 ----
    status = module_motor_disable(&me->super);
    if (status != MODULE_MOTOR_STATUS_OK)
    {
        return status;
    }

    // ---- 刷新总线（发送零命令） ----
    status = module_dji_motor_bus_flush(me->motor_bus);
    if (status != MODULE_MOTOR_STATUS_OK)
    {
        return status;
    }

    // ---- 从注册表注销 ----
    status = module_motor_registry_unregister(registry, &me->super);
    if (status != MODULE_MOTOR_STATUS_OK)
    {
        return status;
    }

    // ---- 释放总线槽位 ----
    me->motor_bus->motor_slots[me->group_index][me->group_slot] = NULL;

    // 检查组是否还有其他电机
    for (slot_index = 0U; slot_index < MODULE_DJI_MOTOR_PER_GROUP; ++slot_index)
    {
        if (me->motor_bus->motor_slots[me->group_index][slot_index] != NULL)
        {
            group_is_used = true;
            break;
        }
    }
    me->motor_bus->group_is_used[me->group_index] = group_is_used;

    return MODULE_MOTOR_STATUS_OK;
}

/**
 * @brief 将 module_dji_motor_t 向上转型为 module_motor_t
 * @param me 派生对象指针
 * @return 基类指针
 */
module_motor_t *module_dji_motor_as_base(module_dji_motor_t *const me)
{
    return (me != NULL) ? &me->super : NULL;
}

const module_motor_pid_t *module_dji_motor_get_current_pid(const module_dji_motor_t *const me)
{
    return ((me != NULL) && me->current_pid.is_initialized) ? &me->current_pid : NULL;
}

const module_motor_pid_t *module_dji_motor_get_velocity_pid(const module_dji_motor_t *const me)
{
    return ((me != NULL) && me->velocity_pid.is_initialized) ? &me->velocity_pid : NULL;
}

const module_motor_pid_t *module_dji_motor_get_angle_pid(const module_dji_motor_t *const me)
{
    return ((me != NULL) && me->angle_pid.is_initialized) ? &me->angle_pid : NULL;
}

/**
 * @brief 处理 CAN 反馈帧
 * @param me 总线对象
 * @param frame CAN 帧
 * @return 执行状态
 * @note 解码编码器、速度、电流、温度，处理 13 位编码器回绕，
 *       累积多圈位置，更新反馈数据
 */
module_motor_status_t module_dji_motor_bus_handle_feedback(module_dji_motor_bus_t *const me,
                                                           const bsp_can_frame_t *const frame)
{
    size_t group_index;
    size_t slot_index;
    module_dji_motor_t *motor = NULL;
    uint16_t encoder_count;
    int32_t encoder_delta;
    int16_t speed_rpm;
    int16_t current_raw;

    // ---- 参数校验 ----
    if ((me == NULL) || (frame == NULL) || !me->is_initialized ||
        (frame->id_type != BSP_CAN_ID_STANDARD) || (frame->frame_type != BSP_CAN_FRAME_DATA) ||
        (frame->data_length != 8U))
    {
        return MODULE_MOTOR_STATUS_INVALID_ARGUMENT;
    }

    // ---- 查找匹配的电机 ----
    for (group_index = 0U; group_index < MODULE_DJI_MOTOR_GROUP_COUNT; ++group_index)
    {
        for (slot_index = 0U; slot_index < MODULE_DJI_MOTOR_PER_GROUP; ++slot_index)
        {
            if ((me->motor_slots[group_index][slot_index] != NULL) &&
                (me->motor_slots[group_index][slot_index]->receive_identifier == frame->identifier))
            {
                motor = me->motor_slots[group_index][slot_index];
                break;
            }
        }
    }
    if (motor == NULL)
    {
        return MODULE_MOTOR_STATUS_NOT_REGISTERED;
    }

    // ---- 解码反馈数据 ----
    // 编码器：13 位（0~8191），高字节在前
    encoder_count = (uint16_t)(((uint16_t)frame->data[0] << 8U) | frame->data[1]);
    // 速度：RPM（有符号）
    speed_rpm = (int16_t)(((uint16_t)frame->data[2] << 8U) | frame->data[3]);
    // 电流：原始值（有符号）
    current_raw = (int16_t)(((uint16_t)frame->data[4] << 8U) | frame->data[5]);

    // ---- 处理编码器回绕（13 位，范围 0~8191） ----
    if (!motor->has_previous_encoder_count)
    {
        motor->previous_encoder_count = encoder_count;
        motor->has_previous_encoder_count = true;
        if (motor->position_reference == MODULE_DJI_POSITION_ENCODER_ABSOLUTE)
        {
            motor->accumulated_encoder_count =
                (int32_t)encoder_count - (int32_t)motor->encoder_zero_count;
            if (motor->accumulated_encoder_count > 4096)
            {
                motor->accumulated_encoder_count -= 8192;
            }
            else if (motor->accumulated_encoder_count < -4096)
            {
                motor->accumulated_encoder_count += 8192;
            }
        }
    }
    // 计算增量，处理回绕：超过半圈（4096）视为回绕
    encoder_delta = (int32_t)encoder_count - (int32_t)motor->previous_encoder_count;
    if (encoder_delta > 4096)
    {
        encoder_delta -= 8192; // 正向回绕（从接近 8191 到 0）
    }
    else if (encoder_delta < -4096)
    {
        encoder_delta += 8192; // 负向回绕（从 0 到接近 8191）
    }
    motor->previous_encoder_count = encoder_count;
    motor->accumulated_encoder_count += encoder_delta;

    // ---- 更新反馈数据 ----
    // 原始编码器值
    motor->super.feedback.raw_position = encoder_count;

    // 位置（弧度）：累积编码器值 × 2π / (8192 × 减速比)，应用方向符号
    motor->super.feedback.position_rad =
        motor->direction_sign * (float)motor->accumulated_encoder_count * MODULE_DJI_TWO_PI /
            (MODULE_DJI_ENCODER_COUNTS_PER_REVOLUTION * motor->gear_ratio) +
        motor->position_offset_rad;

    // 速度（rad/s）：RPM × 2π / 60 / 减速比，应用方向符号
    motor->super.feedback.velocity_rad_per_s =
        motor->direction_sign * (float)speed_rpm * MODULE_DJI_TWO_PI / (60.0F * motor->gear_ratio);

    // 电流
    motor->super.feedback.current_raw = current_raw;
    motor->super.feedback.is_current_a_valid = motor->current_scale_a_per_count > 0.0F;
    motor->super.feedback.current_a =
        motor->super.feedback.is_current_a_valid
            ? (float)current_raw * motor->current_scale_a_per_count * motor->direction_sign
            : 0.0F;

    // 温度
    motor->super.feedback.motor_temperature_c = (float)frame->data[6];

    // ---- 通知反馈更新 ----
    (void)module_motor_notify_feedback(&motor->super);

    // ---- 过温保护 ----
    if (motor->super.feedback.motor_temperature_c > motor->maximum_temperature_c)
    {
        motor->command_value = 0;
        motor->super.state = MODULE_MOTOR_STATE_FAULT;
    }

    return MODULE_MOTOR_STATUS_OK;
}

/**
 * @brief 刷新总线：将各槽位命令打包成 CAN 帧发送
 * @param me 总线对象
 * @return 执行状态
 * @note 五个发送组：0x1FF、0x200、0x2FF（电压/通用组），0x1FE、0x2FE（GM6020 电流组）
 *       每组四个电机，每个电机 2 字节命令
 */
module_motor_status_t module_dji_motor_bus_flush(module_dji_motor_bus_t *const me)
{
    static const uint32_t group_identifiers[MODULE_DJI_MOTOR_GROUP_COUNT] = {0x1FFU, 0x200U, 0x2FFU,
                                                                             0x1FEU, 0x2FEU};
    size_t group_index;
    size_t slot_index;
    bool had_transport_error = false;

    if ((me == NULL) || !me->is_initialized)
    {
        return MODULE_MOTOR_STATUS_NOT_INITIALIZED;
    }

    for (group_index = 0U; group_index < MODULE_DJI_MOTOR_GROUP_COUNT; ++group_index)
    {
        bsp_can_frame_t frame = {.identifier = group_identifiers[group_index],
                                 .id_type = BSP_CAN_ID_STANDARD,
                                 .frame_type = BSP_CAN_FRAME_DATA,
                                 .data_length = 8U,
                                 .data = {0U}};

        // 如果该组无电机，跳过
        if (!me->group_is_used[group_index])
        {
            continue;
        }

        // 打包四个槽位的命令（每个槽位 2 字节，高字节在前）
        for (slot_index = 0U; slot_index < MODULE_DJI_MOTOR_PER_GROUP; ++slot_index)
        {
            int16_t command_value = 0;
            if (me->motor_slots[group_index][slot_index] != NULL)
            {
                command_value = me->motor_slots[group_index][slot_index]->command_value;
            }
            // 大端序：高字节在前
            frame.data[slot_index * 2U] = (uint8_t)((uint16_t)command_value >> 8U);
            frame.data[slot_index * 2U + 1U] = (uint8_t)command_value;
        }

        // 发送 CAN 帧
        if (bsp_can_transmit(me->can, &frame, me->transmit_timeout_ms) != BSP_STATUS_OK)
        {
            had_transport_error = true;
        }
    }
    return had_transport_error ? MODULE_MOTOR_STATUS_TRANSPORT_ERROR : MODULE_MOTOR_STATUS_OK;
}

/**
 * @brief 获取当前命令值（用于调试）
 * @param me 电机对象
 * @return 当前命令值（int16）
 */
int16_t module_dji_motor_get_command(const module_dji_motor_t *const me)
{
    return ((me != NULL) && me->super.is_initialized) ? me->command_value : 0;
}

module_motor_status_t module_dji_motor_reset_position(module_dji_motor_t *const me,
                                                      float position_rad)
{
    float encoder_position_rad;

    if ((me == NULL) || !isfinite(position_rad))
    {
        return MODULE_MOTOR_STATUS_INVALID_ARGUMENT;
    }
    if (!me->super.is_initialized)
    {
        return MODULE_MOTOR_STATUS_NOT_INITIALIZED;
    }
    if (!me->has_previous_encoder_count)
    {
        return MODULE_MOTOR_STATUS_FEEDBACK_UNAVAILABLE;
    }

    encoder_position_rad = me->direction_sign * (float)me->accumulated_encoder_count *
                           MODULE_DJI_TWO_PI /
                           (MODULE_DJI_ENCODER_COUNTS_PER_REVOLUTION * me->gear_ratio);
    me->position_offset_rad = position_rad - encoder_position_rad;
    me->super.feedback.position_rad = position_rad;
    me->target_angle_rad = position_rad;
    return MODULE_MOTOR_STATUS_OK;
}
