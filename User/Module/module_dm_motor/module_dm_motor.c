/**
 * @file module_dm_motor.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 达妙电机 CAN 协议驱动实现
 * @version 1.0
 * @date 2026-07-28
 * @copyright Copyright (c) 2026
 *
 * @note 支持 MIT、位置速度、速度和力位混合四种控制模式，通过 mode_vptr 多态实现。
 *       浮点量按 limits 范围量化到协议字段，需与固件协议一致。
 */

#include "module_dm_motor.h"

#include <math.h>   // isfinite
#include <stddef.h> // NULL
#include <string.h> // memcpy, memset

MODULE_MOTOR_STATIC_ASSERT_SUPER_FIRST(module_dm_motor_t);

/**
 * @brief 从 module_motor_t 基类获取派生 module_dm_motor_t 对象
 */
static module_dm_motor_t *module_dm_motor_get_device(module_motor_t *const motor_base)
{
    return MODULE_MOTOR_CONTAINER_OF(motor_base, module_dm_motor_t, super);
}

/**
 * @brief 检查值是否在范围内（有限且 >= min <= max）
 */
static bool module_dm_motor_is_within(float value, float minimum, float maximum)
{
    return isfinite(value) && (value >= minimum) && (value <= maximum);
}

/**
 * @brief 检查限制参数是否合法
 * @param limits 限制参数
 * @return true=合法
 * @note 检查所有字段有限，且 min < max
 */
static bool module_dm_motor_are_limits_valid(const module_dm_limits_t *const limits)
{
    const float values[] = {
        limits->position_min_rad,       limits->position_max_rad,
        limits->velocity_min_rad_per_s, limits->velocity_max_rad_per_s,
        limits->torque_min_nm,          limits->torque_max_nm,
        limits->proportional_gain_min,  limits->proportional_gain_max,
        limits->derivative_gain_min,    limits->derivative_gain_max,
    };
    size_t value_index;

    // 检查所有值是否有限
    for (value_index = 0U; value_index < (sizeof(values) / sizeof(values[0])); ++value_index)
    {
        if (!isfinite(values[value_index]))
        {
            return false;
        }
    }
    // 检查 min < max
    return (limits->position_min_rad < limits->position_max_rad) &&
           (limits->velocity_min_rad_per_s < limits->velocity_max_rad_per_s) &&
           (limits->torque_min_nm < limits->torque_max_nm) &&
           (limits->proportional_gain_min < limits->proportional_gain_max) &&
           (limits->derivative_gain_min < limits->derivative_gain_max);
}

/**
 * @brief 检查主机标识符是否合法
 * @param control_mode 控制模式
 * @param master_identifier 主机标识符
 * @return true=合法
 * @note 不同模式有不同的 CAN ID 偏移，需确保不超过 0x7FF
 */
static bool module_dm_motor_is_identifier_valid(module_dm_control_mode_t control_mode,
                                                uint32_t master_identifier)
{
    const uint32_t identifier_offset =
        (control_mode == MODULE_DM_MODE_VELOCITY)
            ? 0x200U
            : ((control_mode == MODULE_DM_MODE_POSITION_VELOCITY)
                   ? 0x100U
                   : ((control_mode == MODULE_DM_MODE_FORCE_POSITION) ? 0x300U : 0U));
    return master_identifier <= (0x7FFU - identifier_offset);
}

/**
 * @brief 将浮点值量化到无符号整数
 * @param value 浮点值
 * @param minimum 最小值
 * @param maximum 最大值
 * @param bit_count 位宽（8/12/16）
 * @return 量化后的无符号整数（四舍五入）
 */
static uint32_t module_dm_motor_float_to_unsigned(float value, float minimum, float maximum,
                                                  uint8_t bit_count)
{
    const uint32_t maximum_integer = (1UL << bit_count) - 1UL;
    return (uint32_t)((value - minimum) * (float)maximum_integer / (maximum - minimum) + 0.5F);
}

/**
 * @brief 将无符号整数反量化为浮点值
 * @param value 无符号整数
 * @param minimum 最小值
 * @param maximum 最大值
 * @param bit_count 位宽
 * @return 浮点值
 */
static float module_dm_motor_unsigned_to_float(uint32_t value, float minimum, float maximum,
                                               uint8_t bit_count)
{
    const uint32_t maximum_integer = (1UL << bit_count) - 1UL;
    return (float)value * (maximum - minimum) / (float)maximum_integer + minimum;
}

/**
 * @brief 将浮点数以小端序编码到 4 字节缓冲区
 * @param value 浮点值
 * @param output 输出缓冲区（4 字节）
 */
static void module_dm_motor_encode_float_little_endian(float value, uint8_t output[4])
{
    uint32_t raw_value;
    (void)memcpy(&raw_value, &value, sizeof(raw_value)); // 二进制复制
    output[0] = (uint8_t)raw_value;
    output[1] = (uint8_t)(raw_value >> 8U);
    output[2] = (uint8_t)(raw_value >> 16U);
    output[3] = (uint8_t)(raw_value >> 24U);
}

/** @brief 将 uint32 以小端序编码到 4 字节缓冲区 */
static void module_dm_motor_encode_u32_little_endian(uint32_t value, uint8_t output[4])
{
    output[0] = (uint8_t)value;
    output[1] = (uint8_t)(value >> 8U);
    output[2] = (uint8_t)(value >> 16U);
    output[3] = (uint8_t)(value >> 24U);
}

/** @brief 从小端序 4 字节解码 uint32 */
static uint32_t module_dm_motor_decode_u32_little_endian(const uint8_t input[4])
{
    return (uint32_t)input[0] | ((uint32_t)input[1] << 8U) |
           ((uint32_t)input[2] << 16U) | ((uint32_t)input[3] << 24U);
}

/**
 * @brief 获取 MIT 模式的 CAN 发送 ID
 * @param me 电机对象
 * @return CAN ID
 */
static uint32_t module_dm_motor_get_mit_identifier(const module_dm_motor_t *const me)
{
    return me->master_identifier;
}

/**
 * @brief 获取速度模式的 CAN 发送 ID（偏移 0x200）
 */
static uint32_t module_dm_motor_get_velocity_identifier(const module_dm_motor_t *const me)
{
    return me->master_identifier + 0x200U;
}

/**
 * @brief 获取位置速度模式的 CAN 发送 ID（偏移 0x100）
 */
static uint32_t module_dm_motor_get_position_velocity_identifier(const module_dm_motor_t *const me)
{
    return me->master_identifier + 0x100U;
}

/**
 * @brief 获取力位混合模式的 CAN 发送 ID（偏移 0x300）
 */
static uint32_t module_dm_motor_get_force_position_identifier(const module_dm_motor_t *const me)
{
    return me->master_identifier + 0x300U;
}

/* ======================== 模式编码函数 ======================== */

/**
 * @brief MIT 模式编码函数
 * @param me 电机对象
 * @param transmit_data 输出 8 字节 CAN 数据
 * @return 执行状态
 * @note 协议格式：
 *       字节 0-1：位置（16 位）
 *       字节 2-3：速度（12 位）+ Kp（12 位）混合
 *       字节 4-5：Kp（续）+ Kd（12 位）混合
 *       字节 6-7：Kd（续）+ 扭矩（12 位）混合
 */
static module_motor_status_t module_dm_motor_encode_mit(module_dm_motor_t *const me,
                                                        uint8_t transmit_data[8])
{
    uint32_t position_raw;
    uint32_t velocity_raw;
    uint32_t proportional_gain_raw;
    uint32_t derivative_gain_raw;
    uint32_t torque_raw;
    const module_dm_mit_command_t *const command = &me->mit_command;

    // ---- 检查所有字段是否在限制范围内 ----
    if (!module_dm_motor_is_within(command->position_rad, me->limits.position_min_rad,
                                   me->limits.position_max_rad) ||
        !module_dm_motor_is_within(command->velocity_rad_per_s, me->limits.velocity_min_rad_per_s,
                                   me->limits.velocity_max_rad_per_s) ||
        !module_dm_motor_is_within(command->proportional_gain, me->limits.proportional_gain_min,
                                   me->limits.proportional_gain_max) ||
        !module_dm_motor_is_within(command->derivative_gain, me->limits.derivative_gain_min,
                                   me->limits.derivative_gain_max) ||
        !module_dm_motor_is_within(command->torque_nm, me->limits.torque_min_nm,
                                   me->limits.torque_max_nm))
    {
        return MODULE_MOTOR_STATUS_OUT_OF_RANGE;
    }

    // ---- 量化各字段 ----
    position_raw = module_dm_motor_float_to_unsigned(
        command->position_rad, me->limits.position_min_rad, me->limits.position_max_rad, 16U);
    velocity_raw = module_dm_motor_float_to_unsigned(command->velocity_rad_per_s,
                                                     me->limits.velocity_min_rad_per_s,
                                                     me->limits.velocity_max_rad_per_s, 12U);
    proportional_gain_raw = module_dm_motor_float_to_unsigned(
        command->proportional_gain, me->limits.proportional_gain_min,
        me->limits.proportional_gain_max, 12U);
    derivative_gain_raw =
        module_dm_motor_float_to_unsigned(command->derivative_gain, me->limits.derivative_gain_min,
                                          me->limits.derivative_gain_max, 12U);
    torque_raw = module_dm_motor_float_to_unsigned(command->torque_nm, me->limits.torque_min_nm,
                                                   me->limits.torque_max_nm, 12U);

    // ---- 打包（紧凑位域） ----
    transmit_data[0] = (uint8_t)(position_raw >> 8U); // 位置高字节
    transmit_data[1] = (uint8_t)position_raw;         // 位置低字节
    transmit_data[2] = (uint8_t)(velocity_raw >> 4U); // 速度高 8 位
    transmit_data[3] =
        (uint8_t)((velocity_raw << 4U) | (proportional_gain_raw >> 8U)); // 速度低4位 + Kp高4位
    transmit_data[4] = (uint8_t)proportional_gain_raw;                   // Kp 低 8 位
    transmit_data[5] = (uint8_t)(derivative_gain_raw >> 4U);             // Kd 高 8 位
    transmit_data[6] =
        (uint8_t)((derivative_gain_raw << 4U) | (torque_raw >> 8U)); // Kd低4位 + 扭矩高4位
    transmit_data[7] = (uint8_t)torque_raw;                          // 扭矩低 8 位
    return MODULE_MOTOR_STATUS_OK;
}

/**
 * @brief 速度模式编码函数
 * @param me 电机对象
 * @param transmit_data 输出 8 字节 CAN 数据
 * @return 执行状态
 * @note 前 4 字节为 float 速度值（小端序），后 4 字节为零
 */
static module_motor_status_t module_dm_motor_encode_velocity(module_dm_motor_t *const me,
                                                             uint8_t transmit_data[8])
{
    if (!module_dm_motor_is_within(me->target_velocity_rad_per_s, me->limits.velocity_min_rad_per_s,
                                   me->limits.velocity_max_rad_per_s))
    {
        return MODULE_MOTOR_STATUS_OUT_OF_RANGE;
    }
    (void)memset(transmit_data, 0, 8U);
    module_dm_motor_encode_float_little_endian(me->target_velocity_rad_per_s, transmit_data);
    return MODULE_MOTOR_STATUS_OK;
}

/**
 * @brief 位置速度模式编码函数
 * @param me 电机对象
 * @param transmit_data 输出 8 字节 CAN 数据
 * @return 执行状态
 * @note 前 4 字节为位置（float），后 4 字节为速度（float）
 */
static module_motor_status_t module_dm_motor_encode_position_velocity(module_dm_motor_t *const me,
                                                                      uint8_t transmit_data[8])
{
    if (!module_dm_motor_is_within(me->target_position_rad, me->limits.position_min_rad,
                                   me->limits.position_max_rad) ||
        !module_dm_motor_is_within(me->target_velocity_rad_per_s, me->limits.velocity_min_rad_per_s,
                                   me->limits.velocity_max_rad_per_s))
    {
        return MODULE_MOTOR_STATUS_OUT_OF_RANGE;
    }
    module_dm_motor_encode_float_little_endian(me->target_position_rad, &transmit_data[0]);
    module_dm_motor_encode_float_little_endian(me->target_velocity_rad_per_s, &transmit_data[4]);
    return MODULE_MOTOR_STATUS_OK;
}

/**
 * @brief 力位混合模式编码函数
 * @note D0-D3: p_des(float LE)，D4-D5: v_des*100(uint16 LE)，
 *       D6-D7: i_des*10000(uint16 LE)
 */
static module_motor_status_t module_dm_motor_encode_force_position(module_dm_motor_t *const me,
                                                                   uint8_t transmit_data[8])
{
    const module_dm_force_position_command_t *const command = &me->force_position_command;
    uint16_t velocity_limit_raw;
    uint16_t current_limit_raw;

    if (!module_dm_motor_is_within(command->position_rad, me->limits.position_min_rad,
                                   me->limits.position_max_rad) ||
        !module_dm_motor_is_within(command->velocity_limit_rad_per_s, 0.0F, 100.0F) ||
        !module_dm_motor_is_within(command->current_limit_per_unit, 0.0F, 1.0F))
    {
        return MODULE_MOTOR_STATUS_OUT_OF_RANGE;
    }

    velocity_limit_raw = (uint16_t)(command->velocity_limit_rad_per_s * 100.0F + 0.5F);
    current_limit_raw = (uint16_t)(command->current_limit_per_unit * 10000.0F + 0.5F);
    module_dm_motor_encode_float_little_endian(command->position_rad, &transmit_data[0]);
    transmit_data[4] = (uint8_t)velocity_limit_raw;
    transmit_data[5] = (uint8_t)(velocity_limit_raw >> 8U);
    transmit_data[6] = (uint8_t)current_limit_raw;
    transmit_data[7] = (uint8_t)(current_limit_raw >> 8U);
    return MODULE_MOTOR_STATUS_OK;
}

/* ======================== 模式操作虚表 ======================== */

static const module_dm_mode_ops_t s_module_dm_mit_ops = {
    .encode_command = module_dm_motor_encode_mit,
    .get_transmit_identifier = module_dm_motor_get_mit_identifier,
    .transmit_data_length = 8U};
static const module_dm_mode_ops_t s_module_dm_velocity_ops = {
    .encode_command = module_dm_motor_encode_velocity,
    .get_transmit_identifier = module_dm_motor_get_velocity_identifier,
    .transmit_data_length = 4U};
static const module_dm_mode_ops_t s_module_dm_position_velocity_ops = {
    .encode_command = module_dm_motor_encode_position_velocity,
    .get_transmit_identifier = module_dm_motor_get_position_velocity_identifier,
    .transmit_data_length = 8U};
static const module_dm_mode_ops_t s_module_dm_force_position_ops = {
    .encode_command = module_dm_motor_encode_force_position,
    .get_transmit_identifier = module_dm_motor_get_force_position_identifier,
    .transmit_data_length = 8U};

/* ======================== 内部传输函数 ======================== */

/**
 * @brief 发送 CAN 帧
 * @param me 电机对象
 * @param transmit_data 8 字节数据
 * @param transmit_identifier CAN ID
 * @return 执行状态
 */
static module_motor_status_t module_dm_motor_transmit(module_dm_motor_t *const me,
                                                      const uint8_t transmit_data[8],
                                                      uint32_t transmit_identifier,
                                                      uint8_t transmit_data_length)
{
    const bsp_can_frame_t frame = {.identifier = transmit_identifier,
                                   .id_type = BSP_CAN_ID_STANDARD,
                                   .frame_type = BSP_CAN_FRAME_DATA,
                                   .data_length = transmit_data_length,
                                   .data = {transmit_data[0], transmit_data[1], transmit_data[2],
                                            transmit_data[3], transmit_data[4], transmit_data[5],
                                            transmit_data[6], transmit_data[7]}};
    return (bsp_can_transmit(me->can, &frame, me->transmit_timeout_ms) == BSP_STATUS_OK)
               ? MODULE_MOTOR_STATUS_OK
               : MODULE_MOTOR_STATUS_TRANSPORT_ERROR;
}

/* ======================== 虚函数实现（module_motor_ops_t） ======================== */

/**
 * @brief 使能电机（虚函数实现）
 * @param motor_base 基类指针
 * @return 执行状态
 */
static module_motor_status_t module_dm_motor_enable_virtual(module_motor_t *const motor_base)
{
    module_dm_motor_t *const me = module_dm_motor_get_device(motor_base);
    module_motor_status_t status = module_dm_motor_send_state_command(me, MODULE_DM_COMMAND_ENABLE);
    if (status == MODULE_MOTOR_STATUS_OK)
    {
        motor_base->state = MODULE_MOTOR_STATE_ENABLED;
    }
    return status;
}

/**
 * @brief 禁用电机（虚函数实现）
 */
static module_motor_status_t module_dm_motor_disable_virtual(module_motor_t *const motor_base)
{
    module_dm_motor_t *const me = module_dm_motor_get_device(motor_base);
    module_motor_status_t status =
        module_dm_motor_send_state_command(me, MODULE_DM_COMMAND_DISABLE);
    if (status == MODULE_MOTOR_STATUS_OK)
    {
        motor_base->state = MODULE_MOTOR_STATE_DISABLED;
    }
    return status;
}

/**
 * @brief 设置目标值（虚函数实现）
 * @param motor_base 基类指针
 * @param target_value 目标值（含义取决于控制模式）
 * @return 执行状态
 */
static module_motor_status_t module_dm_motor_set_target_virtual(module_motor_t *const motor_base,
                                                                float target_value)
{
    module_dm_motor_t *const me = module_dm_motor_get_device(motor_base);
    if (!isfinite(target_value))
    {
        return MODULE_MOTOR_STATUS_OUT_OF_RANGE;
    }
    // 不同模式将 target_value 存入不同字段
    if (me->control_mode == MODULE_DM_MODE_MIT)
    {
        me->mit_command.torque_nm = target_value; // MIT 模式 target 作为前馈扭矩
    }
    else if (me->control_mode == MODULE_DM_MODE_VELOCITY)
    {
        me->target_velocity_rad_per_s = target_value;
    }
    else if (me->control_mode == MODULE_DM_MODE_POSITION_VELOCITY)
    {
        me->target_position_rad = target_value;
    }
    else
    {
        me->force_position_command.position_rad = target_value;
    }
    return MODULE_MOTOR_STATUS_OK;
}

/**
 * @brief 更新电机（虚函数实现）
 * @param motor_base 基类指针
 * @param delta_time_s 时间步长（未使用）
 * @return 执行状态
 */
static module_motor_status_t module_dm_motor_update_virtual(module_motor_t *const motor_base,
                                                            float delta_time_s)
{
    module_dm_motor_t *const me = module_dm_motor_get_device(motor_base);
    uint8_t transmit_data[8];
    module_motor_status_t status;
    (void)delta_time_s; // 达妙协议不需要时间步长

    if (motor_base->state != MODULE_MOTOR_STATE_ENABLED)
    {
        return MODULE_MOTOR_STATUS_OK;
    }
    // 通过模式虚表编码命令
    status = me->mode_vptr->encode_command(me, transmit_data);
    if (status != MODULE_MOTOR_STATUS_OK)
    {
        return status;
    }
    // 发送 CAN 帧
    return module_dm_motor_transmit(me, transmit_data, me->mode_vptr->get_transmit_identifier(me),
                                    me->mode_vptr->transmit_data_length);
}

/** 电机虚表 */
static const module_motor_ops_t s_module_dm_motor_ops = {.enable = module_dm_motor_enable_virtual,
                                                         .disable = module_dm_motor_disable_virtual,
                                                         .set_target =
                                                             module_dm_motor_set_target_virtual,
                                                         .update = module_dm_motor_update_virtual};

/* ======================== 公共 API ======================== */

/**
 * @brief 初始化达妙电机
 */
module_motor_status_t module_dm_motor_init(module_dm_motor_t *const me,
                                           const module_dm_motor_config_t *const config)
{
    // ---- 参数校验 ----
    if ((me == NULL) || (config == NULL) || (config->motor_name == NULL) ||
        (config->can == NULL) || !bsp_device_is_initialized(&config->can->super) ||
        (config->control_mode > MODULE_DM_MODE_FORCE_POSITION) ||
        !module_dm_motor_is_identifier_valid(config->control_mode, config->master_identifier) ||
        (config->feedback_identifier > 0x7FFU) ||
        !module_dm_motor_are_limits_valid(&config->limits))
    {
        return MODULE_MOTOR_STATUS_INVALID_ARGUMENT;
    }

    // ---- 选择模式操作表 ----
    me->mode_vptr = (config->control_mode == MODULE_DM_MODE_MIT)
                        ? &s_module_dm_mit_ops
                        : ((config->control_mode == MODULE_DM_MODE_VELOCITY)
                               ? &s_module_dm_velocity_ops
                               : ((config->control_mode == MODULE_DM_MODE_POSITION_VELOCITY)
                                      ? &s_module_dm_position_velocity_ops
                                      : &s_module_dm_force_position_ops));

    // ---- 保存配置 ----
    me->can = config->can;
    me->control_mode = config->control_mode;
    me->limits = config->limits;
    me->mit_command = (module_dm_mit_command_t){0};
    me->target_position_rad = 0.0F;
    me->target_velocity_rad_per_s = 0.0F;
    me->force_position_command = (module_dm_force_position_command_t){0};
    me->master_identifier = config->master_identifier;
    me->feedback_identifier = config->feedback_identifier;
    me->transmit_timeout_ms = config->transmit_timeout_ms;
    me->fault = MODULE_DM_FAULT_NONE;
    me->mos_temperature_c = 0.0F;
    me->requested_communication_timeout_counts = 0U;
    me->confirmed_communication_timeout_counts = 0U;
    me->communication_timeout_is_confirmed = false;
    me->parameter_response = (module_dm_parameter_response_t){0};

    // ---- 初始化基类 ----
    return module_motor_init_base(&me->super, &s_module_dm_motor_ops, config->motor_name,
                                  config->registration_key, config->master_identifier);
}

/**
 * @brief 注册电机到电机注册表
 */
module_motor_status_t module_dm_motor_register(module_dm_motor_t *const me,
                                               module_motor_registry_t *const registry)
{
    if ((me == NULL) || (registry == NULL))
    {
        return MODULE_MOTOR_STATUS_INVALID_ARGUMENT;
    }
    return module_motor_registry_register(registry, &me->super);
}

/**
 * @brief 从电机注册表注销电机
 */
module_motor_status_t module_dm_motor_unregister(module_dm_motor_t *const me,
                                                 module_motor_registry_t *const registry)
{
    module_motor_status_t status;

    if ((me == NULL) || (registry == NULL))
    {
        return MODULE_MOTOR_STATUS_INVALID_ARGUMENT;
    }
    if (!me->super.is_registered)
    {
        return MODULE_MOTOR_STATUS_NOT_REGISTERED;
    }
    // 先禁用电机
    status = module_motor_disable(&me->super);
    if (status != MODULE_MOTOR_STATUS_OK)
    {
        return status;
    }
    return module_motor_registry_unregister(registry, &me->super);
}

/**
 * @brief 向上转型为 module_motor_t
 */
module_motor_t *module_dm_motor_as_base(module_dm_motor_t *const me)
{
    return (me != NULL) ? &me->super : NULL;
}

/**
 * @brief 发送状态命令
 * @param me 电机对象
 * @param command 状态命令
 * @return 执行状态
 * @note 命令帧格式：前 7 字节为 0xFF，第 8 字节为命令码
 *       ENABLE=0xFC, DISABLE=0xFD, SET_ZERO=0xFE, CLEAR_FAULT=0xFB
 */
module_motor_status_t module_dm_motor_send_state_command(module_dm_motor_t *const me,
                                                         module_dm_state_command_t command)
{
    uint8_t transmit_data[8] = {0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU};
    static const uint8_t command_codes[] = {0xFDU, 0xFCU, 0xFEU, 0xFBU};

    // 参数校验
    if ((me == NULL) || !me->super.is_registered)
    {
        return (me == NULL) ? MODULE_MOTOR_STATUS_INVALID_ARGUMENT
                            : MODULE_MOTOR_STATUS_NOT_REGISTERED;
    }
    if ((uint32_t)command >= (sizeof(command_codes) / sizeof(command_codes[0])))
    {
        return MODULE_MOTOR_STATUS_OUT_OF_RANGE;
    }
    transmit_data[7] = command_codes[command]; // 命令码放入最后一个字节
    return module_dm_motor_transmit(me, transmit_data,
                                    me->mode_vptr->get_transmit_identifier(me), 8U);
}

/* ======================== 参数协议 ======================== */

/** @brief 构造参数服务帧的 CANID_L/CANID_H 公共字段 */
static void module_dm_motor_prepare_parameter_frame(const module_dm_motor_t *const me,
                                                    uint8_t operation, uint8_t register_address,
                                                    uint8_t data[8])
{
    (void)memset(data, 0, 8U);
    data[0] = (uint8_t)me->master_identifier;
    data[1] = (uint8_t)(me->master_identifier >> 8U);
    data[2] = operation;
    data[3] = register_address;
}

module_motor_status_t module_dm_motor_read_parameter(module_dm_motor_t *const me,
                                                     uint8_t register_address)
{
    uint8_t transmit_data[8];

    if ((me == NULL) || !me->super.is_registered)
    {
        return (me == NULL) ? MODULE_MOTOR_STATUS_INVALID_ARGUMENT
                            : MODULE_MOTOR_STATUS_NOT_REGISTERED;
    }
    module_dm_motor_prepare_parameter_frame(
        me, (uint8_t)MODULE_DM_PARAMETER_OPERATION_READ, register_address, transmit_data);
    return module_dm_motor_transmit(me, transmit_data, 0x7FFU, 4U);
}

module_motor_status_t module_dm_motor_write_parameter_u32(module_dm_motor_t *const me,
                                                          uint8_t register_address, uint32_t value)
{
    uint8_t transmit_data[8];

    if ((me == NULL) || !me->super.is_registered)
    {
        return (me == NULL) ? MODULE_MOTOR_STATUS_INVALID_ARGUMENT
                            : MODULE_MOTOR_STATUS_NOT_REGISTERED;
    }
    if (me->super.state != MODULE_MOTOR_STATE_DISABLED)
    {
        return MODULE_MOTOR_STATUS_UNSUPPORTED;
    }
    module_dm_motor_prepare_parameter_frame(
        me, (uint8_t)MODULE_DM_PARAMETER_OPERATION_WRITE, register_address, transmit_data);
    module_dm_motor_encode_u32_little_endian(value, &transmit_data[4]);
    return module_dm_motor_transmit(me, transmit_data, 0x7FFU, 8U);
}

module_motor_status_t module_dm_motor_write_parameter_float(module_dm_motor_t *const me,
                                                            uint8_t register_address, float value)
{
    uint8_t transmit_data[8];

    if ((me == NULL) || !isfinite(value) || !me->super.is_registered)
    {
        if ((me != NULL) && !me->super.is_registered)
        {
            return MODULE_MOTOR_STATUS_NOT_REGISTERED;
        }
        return MODULE_MOTOR_STATUS_INVALID_ARGUMENT;
    }
    if (me->super.state != MODULE_MOTOR_STATE_DISABLED)
    {
        return MODULE_MOTOR_STATUS_UNSUPPORTED;
    }
    module_dm_motor_prepare_parameter_frame(
        me, (uint8_t)MODULE_DM_PARAMETER_OPERATION_WRITE, register_address, transmit_data);
    module_dm_motor_encode_float_little_endian(value, &transmit_data[4]);
    return module_dm_motor_transmit(me, transmit_data, 0x7FFU, 8U);
}

module_motor_status_t module_dm_motor_save_parameters(module_dm_motor_t *const me)
{
    uint8_t transmit_data[8];

    if ((me == NULL) || !me->super.is_registered)
    {
        return (me == NULL) ? MODULE_MOTOR_STATUS_INVALID_ARGUMENT
                            : MODULE_MOTOR_STATUS_NOT_REGISTERED;
    }
    if (me->super.state != MODULE_MOTOR_STATE_DISABLED)
    {
        return MODULE_MOTOR_STATUS_UNSUPPORTED;
    }
    module_dm_motor_prepare_parameter_frame(
        me, (uint8_t)MODULE_DM_PARAMETER_OPERATION_SAVE, 0x01U, transmit_data);
    return module_dm_motor_transmit(me, transmit_data, 0x7FFU, 4U);
}

module_motor_status_t
module_dm_motor_set_communication_timeout(module_dm_motor_t *const me, uint32_t timeout_counts)
{
    module_motor_status_t status = module_dm_motor_write_parameter_u32(
        me, (uint8_t)MODULE_DM_REGISTER_COMMUNICATION_TIMEOUT, timeout_counts);
    if (status == MODULE_MOTOR_STATUS_OK)
    {
        me->requested_communication_timeout_counts = timeout_counts;
        me->communication_timeout_is_confirmed = false;
    }
    return status;
}

const module_dm_parameter_response_t *
module_dm_motor_get_parameter_response(const module_dm_motor_t *const me)
{
    return ((me != NULL) && me->parameter_response.is_valid) ? &me->parameter_response : NULL;
}

/**
 * @brief 立即执行 MIT 命令
 */
module_motor_status_t module_dm_motor_command_mit(module_dm_motor_t *const me,
                                                  const module_dm_mit_command_t *const command)
{
    module_motor_status_t status = module_dm_motor_set_mit_target(me, command);
    return (status == MODULE_MOTOR_STATUS_OK) ? module_motor_update(&me->super, 1.0F) : status;
}

/**
 * @brief 设置 MIT 目标（由统一 update 调度）
 */
module_motor_status_t module_dm_motor_set_mit_target(module_dm_motor_t *const me,
                                                     const module_dm_mit_command_t *const command)
{
    if ((me == NULL) || (command == NULL))
    {
        return MODULE_MOTOR_STATUS_INVALID_ARGUMENT;
    }
    if (me->control_mode != MODULE_DM_MODE_MIT)
    {
        return MODULE_MOTOR_STATUS_UNSUPPORTED;
    }
    // 检查所有字段是否在限制范围内
    if (!module_dm_motor_is_within(command->position_rad, me->limits.position_min_rad,
                                   me->limits.position_max_rad) ||
        !module_dm_motor_is_within(command->velocity_rad_per_s, me->limits.velocity_min_rad_per_s,
                                   me->limits.velocity_max_rad_per_s) ||
        !module_dm_motor_is_within(command->proportional_gain, me->limits.proportional_gain_min,
                                   me->limits.proportional_gain_max) ||
        !module_dm_motor_is_within(command->derivative_gain, me->limits.derivative_gain_min,
                                   me->limits.derivative_gain_max) ||
        !module_dm_motor_is_within(command->torque_nm, me->limits.torque_min_nm,
                                   me->limits.torque_max_nm))
    {
        return MODULE_MOTOR_STATUS_OUT_OF_RANGE;
    }
    me->mit_command = *command;
    return MODULE_MOTOR_STATUS_OK;
}

/**
 * @brief 立即执行速度命令
 */
module_motor_status_t module_dm_motor_command_velocity(module_dm_motor_t *const me,
                                                       float velocity_rad_per_s)
{
    module_motor_status_t status = module_dm_motor_set_velocity_target(me, velocity_rad_per_s);
    return (status == MODULE_MOTOR_STATUS_OK) ? module_motor_update(&me->super, 1.0F) : status;
}

/**
 * @brief 设置速度目标
 */
module_motor_status_t module_dm_motor_set_velocity_target(module_dm_motor_t *const me,
                                                          float velocity_rad_per_s)
{
    if (me == NULL)
    {
        return MODULE_MOTOR_STATUS_INVALID_ARGUMENT;
    }
    if (me->control_mode != MODULE_DM_MODE_VELOCITY)
    {
        return MODULE_MOTOR_STATUS_UNSUPPORTED;
    }
    if (!module_dm_motor_is_within(velocity_rad_per_s, me->limits.velocity_min_rad_per_s,
                                   me->limits.velocity_max_rad_per_s))
    {
        return MODULE_MOTOR_STATUS_OUT_OF_RANGE;
    }
    me->target_velocity_rad_per_s = velocity_rad_per_s;
    return MODULE_MOTOR_STATUS_OK;
}

/**
 * @brief 立即执行位置速度命令
 */
module_motor_status_t module_dm_motor_command_position_velocity(module_dm_motor_t *const me,
                                                                float position_rad,
                                                                float velocity_rad_per_s)
{
    module_motor_status_t status =
        module_dm_motor_set_position_velocity_target(me, position_rad, velocity_rad_per_s);
    return (status == MODULE_MOTOR_STATUS_OK) ? module_motor_update(&me->super, 1.0F) : status;
}

/**
 * @brief 设置位置速度目标
 */
module_motor_status_t module_dm_motor_set_position_velocity_target(module_dm_motor_t *const me,
                                                                   float position_rad,
                                                                   float velocity_rad_per_s)
{
    if (me == NULL)
    {
        return MODULE_MOTOR_STATUS_INVALID_ARGUMENT;
    }
    if (me->control_mode != MODULE_DM_MODE_POSITION_VELOCITY)
    {
        return MODULE_MOTOR_STATUS_UNSUPPORTED;
    }
    if (!module_dm_motor_is_within(position_rad, me->limits.position_min_rad,
                                   me->limits.position_max_rad) ||
        !module_dm_motor_is_within(velocity_rad_per_s, me->limits.velocity_min_rad_per_s,
                                   me->limits.velocity_max_rad_per_s))
    {
        return MODULE_MOTOR_STATUS_OUT_OF_RANGE;
    }
    me->target_position_rad = position_rad;
    me->target_velocity_rad_per_s = velocity_rad_per_s;
    return MODULE_MOTOR_STATUS_OK;
}

/**
 * @brief 立即执行力位混合模式命令
 */
module_motor_status_t
module_dm_motor_command_force_position(module_dm_motor_t *const me,
                                       const module_dm_force_position_command_t *const command)
{
    const module_motor_status_t status = module_dm_motor_set_force_position_target(me, command);
    return (status == MODULE_MOTOR_STATUS_OK) ? module_motor_update(&me->super, 1.0F) : status;
}

/**
 * @brief 设置力位混合模式目标
 */
module_motor_status_t
module_dm_motor_set_force_position_target(module_dm_motor_t *const me,
                                          const module_dm_force_position_command_t *const command)
{
    if ((me == NULL) || (command == NULL))
    {
        return MODULE_MOTOR_STATUS_INVALID_ARGUMENT;
    }
    if (me->control_mode != MODULE_DM_MODE_FORCE_POSITION)
    {
        return MODULE_MOTOR_STATUS_UNSUPPORTED;
    }
    if (!module_dm_motor_is_within(command->position_rad, me->limits.position_min_rad,
                                   me->limits.position_max_rad) ||
        !module_dm_motor_is_within(command->velocity_limit_rad_per_s, 0.0F, 100.0F) ||
        !module_dm_motor_is_within(command->current_limit_per_unit, 0.0F, 1.0F))
    {
        return MODULE_MOTOR_STATUS_OUT_OF_RANGE;
    }
    me->force_position_command = *command;
    return MODULE_MOTOR_STATUS_OK;
}

/**
 * @brief 处理 CAN 反馈帧
 * @param me 电机对象
 * @param frame CAN 帧
 * @return 执行状态
 * @note 解码位置（16位）、速度（12位）、扭矩（12位）、MOS温度、电机温度、故障码
 */
module_motor_status_t module_dm_motor_handle_feedback(module_dm_motor_t *const me,
                                                      const bsp_can_frame_t *const frame)
{
    uint32_t position_raw;
    uint32_t velocity_raw;
    uint32_t torque_raw;
    uint8_t state_code;

    // ---- 公共参数校验 ----
    if ((me == NULL) || (frame == NULL) || !me->super.is_registered ||
        (frame->id_type != BSP_CAN_ID_STANDARD) || (frame->frame_type != BSP_CAN_FRAME_DATA) ||
        (frame->identifier != me->feedback_identifier))
    {
        return MODULE_MOTOR_STATUS_INVALID_ARGUMENT;
    }

    // ---- 参数读/写/保存响应 ----
    if ((frame->data_length >= 4U) &&
        (frame->data[0] == (uint8_t)me->master_identifier) &&
        (frame->data[1] == (uint8_t)(me->master_identifier >> 8U)) &&
        ((frame->data[2] == (uint8_t)MODULE_DM_PARAMETER_OPERATION_READ) ||
         (frame->data[2] == (uint8_t)MODULE_DM_PARAMETER_OPERATION_WRITE) ||
         (frame->data[2] == (uint8_t)MODULE_DM_PARAMETER_OPERATION_SAVE)))
    {
        uint32_t raw_value = 0U;
        float float_value = 0.0F;

        if ((frame->data[2] != (uint8_t)MODULE_DM_PARAMETER_OPERATION_SAVE) &&
            (frame->data_length != 8U))
        {
            return MODULE_MOTOR_STATUS_INVALID_ARGUMENT;
        }
        if (frame->data_length == 8U)
        {
            raw_value = module_dm_motor_decode_u32_little_endian(&frame->data[4]);
            (void)memcpy(&float_value, &raw_value, sizeof(float_value));
        }
        me->parameter_response.operation = (module_dm_parameter_operation_t)frame->data[2];
        me->parameter_response.register_address = frame->data[3];
        me->parameter_response.raw_value = raw_value;
        me->parameter_response.float_value = float_value;
        me->parameter_response.is_valid = true;

        if ((frame->data[2] != (uint8_t)MODULE_DM_PARAMETER_OPERATION_SAVE) &&
            (frame->data[3] == (uint8_t)MODULE_DM_REGISTER_COMMUNICATION_TIMEOUT))
        {
            me->confirmed_communication_timeout_counts = raw_value;
            me->communication_timeout_is_confirmed = true;
        }
        return MODULE_MOTOR_STATUS_OK;
    }

    // ---- 运动反馈校验 ----
    if ((frame->data_length != 8U) ||
        ((frame->data[0] & 0x0FU) != (uint8_t)(me->master_identifier & 0x0FU)))
    {
        return MODULE_MOTOR_STATUS_INVALID_ARGUMENT;
    }

    // ---- 解码反馈 ----
    state_code = (uint8_t)(frame->data[0] >> 4U);                             // 状态码（高 4 位）
    position_raw = ((uint32_t)frame->data[1] << 8U) | frame->data[2];         // 位置（16 位）
    velocity_raw = ((uint32_t)frame->data[3] << 4U) | (frame->data[4] >> 4U); // 速度（12 位）
    torque_raw = ((uint32_t)(frame->data[4] & 0x0FU) << 8U) | frame->data[5]; // 扭矩（12 位）

    // ---- 反量化为浮点值 ----
    me->super.feedback.position_rad = module_dm_motor_unsigned_to_float(
        position_raw, me->limits.position_min_rad, me->limits.position_max_rad, 16U);
    me->super.feedback.velocity_rad_per_s = module_dm_motor_unsigned_to_float(
        velocity_raw, me->limits.velocity_min_rad_per_s, me->limits.velocity_max_rad_per_s, 12U);
    me->super.feedback.torque_nm = module_dm_motor_unsigned_to_float(
        torque_raw, me->limits.torque_min_nm, me->limits.torque_max_nm, 12U);

    // ---- 温度 ----
    me->mos_temperature_c = (float)frame->data[6];                  // MOS 管温度
    me->super.feedback.motor_temperature_c = (float)frame->data[7]; // 电机温度

    // ---- 通知反馈更新 ----
    (void)module_motor_notify_feedback(&me->super);

    // ---- 故障检测 ----
    me->fault = (state_code >= 8U) ? (module_dm_fault_t)state_code : MODULE_DM_FAULT_NONE;
    if (me->fault != MODULE_DM_FAULT_NONE)
    {
        me->super.state = MODULE_MOTOR_STATE_FAULT;
    }
    else if (state_code == 1U)
    {
        me->super.state = MODULE_MOTOR_STATE_ENABLED;
    }
    else
    {
        me->super.state = MODULE_MOTOR_STATE_DISABLED;
    }
    return MODULE_MOTOR_STATUS_OK;
}

/**
 * @brief 获取当前故障码
 */
module_dm_fault_t module_dm_motor_get_fault(const module_dm_motor_t *const me)
{
    return (me != NULL) ? me->fault : MODULE_DM_FAULT_COMMUNICATION_LOST;
}

/**
 * @brief 获取 MOS 管温度
 */
float module_dm_motor_get_mos_temperature_c(const module_dm_motor_t *const me)
{
    return (me != NULL) ? me->mos_temperature_c : 0.0F;
}
