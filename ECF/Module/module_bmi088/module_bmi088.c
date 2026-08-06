/**
 * @file module_bmi088.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief BMI088 六轴 IMU 传感器模块实现
 * @version 1.0
 * @date 2026-07-28
 * @copyright Copyright (c) 2026
 *
 * @note 通过 SPI 读写 BMI088 寄存器，实现加速度计和陀螺仪的数据读取、
 *       配置、自检和陀螺仪零偏标定。
 */

#include "module_bmi088.h"
#include "module_bmi088_registers.h"

#include <math.h>   // INFINITY, isfinite
#include <stddef.h> // NULL
#include <string.h> // memcpy

MODULE_STATIC_ASSERT_SUPER_FIRST(module_bmi088_t);

/* ======================== 内部常量 ======================== */

/** 标准重力加速度（m/s²） */
#define MODULE_BMI088_STANDARD_GRAVITY (9.80665F)
/** 角度转弧度系数（°/s → rad/s） */
#define MODULE_BMI088_DEGREES_TO_RADIANS (0.01745329251994329577F)
/** SPI 单次传输最大字节数（10 字节） */
#define MODULE_BMI088_MAX_TRANSFER_SIZE (10U)

/* ======================== 辅助结构体 ======================== */

/**
 * @brief 寄存器配置表项（用于批量初始化）
 */
typedef struct
{
    uint8_t register_address; // 寄存器地址
    uint8_t register_value;   // 要写入的值
} module_bmi088_register_config_t;

/* ======================== 内部函数 ======================== */

/**
 * @brief 执行 SPI 全双工交换（带片选控制）
 * @param me BMI088 对象
 * @param sensor 选择传感器（加速度计或陀螺仪）
 * @param transmit_data 发送数据指针
 * @param receive_data 接收数据指针
 * @param data_size 数据大小（字节）
 * @return 执行状态
 */
static module_bmi088_status_t module_bmi088_exchange(module_bmi088_t *const me,
                                                     module_bmi088_sensor_t sensor,
                                                     const uint8_t *transmit_data,
                                                     uint8_t *receive_data, size_t data_size)
{
    bsp_status_t status;
    // 选中片选
    me->set_chip_select(me->user_context, sensor, true);
    // 执行 SPI 全双工交换（阻塞模式）
    status = bsp_spi_exchange(me->spi, transmit_data, receive_data, data_size,
                              BSP_TRANSFER_MODE_BLOCKING, me->transfer_timeout_ms);
    // 取消片选
    me->set_chip_select(me->user_context, sensor, false);
    return (status == BSP_STATUS_OK) ? MODULE_BMI088_STATUS_OK
                                     : MODULE_BMI088_STATUS_TRANSPORT_ERROR;
}

/**
 * @brief 写单个寄存器（不校验）
 * @param me BMI088 对象
 * @param sensor 选择传感器
 * @param register_address 寄存器地址
 * @param register_value 要写入的值
 * @return 执行状态
 */
static module_bmi088_status_t module_bmi088_write_register(module_bmi088_t *const me,
                                                           module_bmi088_sensor_t sensor,
                                                           uint8_t register_address,
                                                           uint8_t register_value)
{
    // SPI 写操作：发送地址+值（地址不含读位）
    const uint8_t transmit_data[2] = {register_address, register_value};
    uint8_t receive_data[2] = {0U};
    return module_bmi088_exchange(me, sensor, transmit_data, receive_data, sizeof(transmit_data));
}

/**
 * @brief 读取多个寄存器
 * @param me BMI088 对象
 * @param sensor 选择传感器
 * @param register_address 起始寄存器地址
 * @param receive_data 输出缓冲区
 * @param data_size 读取字节数
 * @return 执行状态
 * @note 加速度计读操作需要 2 字节协议开销（命令+填充），陀螺仪为 1 字节。
 */
static module_bmi088_status_t module_bmi088_read_registers(module_bmi088_t *const me,
                                                           module_bmi088_sensor_t sensor,
                                                           uint8_t register_address,
                                                           uint8_t *receive_data, size_t data_size)
{
    uint8_t transmit_buffer[MODULE_BMI088_MAX_TRANSFER_SIZE] = {0U};
    uint8_t receive_buffer[MODULE_BMI088_MAX_TRANSFER_SIZE] = {0U};
    // 协议开销：加速度计读操作需要额外发送一个 0x00 填充字节
    size_t protocol_overhead = (sensor == MODULE_BMI088_SENSOR_ACCEL) ? 2U : 1U;

    // 参数校验
    if ((receive_data == NULL) || (data_size == 0U) ||
        ((data_size + protocol_overhead) > MODULE_BMI088_MAX_TRANSFER_SIZE))
    {
        return MODULE_BMI088_STATUS_INVALID_ARGUMENT;
    }
    // 构建发送命令：寄存器地址 | 读位（0x80）
    transmit_buffer[0] = register_address | MODULE_BMI088_SPI_READ_BIT;
    // 执行 SPI 交换
    if (module_bmi088_exchange(me, sensor, transmit_buffer, receive_buffer,
                               data_size + protocol_overhead) != MODULE_BMI088_STATUS_OK)
    {
        return MODULE_BMI088_STATUS_TRANSPORT_ERROR;
    }
    // 跳过协议开销，拷贝有效数据
    (void)memcpy(receive_data, &receive_buffer[protocol_overhead], data_size);
    return MODULE_BMI088_STATUS_OK;
}

/**
 * @brief 写寄存器并回读校验
 * @param me BMI088 对象
 * @param sensor 选择传感器
 * @param register_address 寄存器地址
 * @param register_value 要写入的值
 * @return 执行状态，若回读不一致则返回 REGISTER_VERIFY_FAILED
 */
static module_bmi088_status_t module_bmi088_write_and_verify(module_bmi088_t *const me,
                                                             module_bmi088_sensor_t sensor,
                                                             uint8_t register_address,
                                                             uint8_t register_value)
{
    uint8_t read_value;
    module_bmi088_status_t status =
        module_bmi088_write_register(me, sensor, register_address, register_value);
    if (status != MODULE_BMI088_STATUS_OK)
    {
        return status;
    }
    // 等待寄存器更新
    me->delay_ms(me->user_context, 1U);
    status = module_bmi088_read_registers(me, sensor, register_address, &read_value, 1U);
    if (status != MODULE_BMI088_STATUS_OK)
    {
        return status;
    }
    return (read_value == register_value) ? MODULE_BMI088_STATUS_OK
                                          : MODULE_BMI088_STATUS_REGISTER_VERIFY_FAILED;
}

/**
 * @brief 获取加速度换算因子（LSB → m/s²）
 * @param acceleration_range 量程
 * @return 换算因子（浮点数）
 */
static float module_bmi088_get_acceleration_scale(module_bmi088_accel_range_t acceleration_range)
{
    static const float range_g[] = {3.0F, 6.0F, 12.0F, 24.0F};
    // 16 位有符号数最大值为 32768，换算为 G，再乘标准重力
    return range_g[acceleration_range] * MODULE_BMI088_STANDARD_GRAVITY / 32768.0F;
}

/**
 * @brief 获取角速度换算因子（LSB → rad/s）
 * @param angular_velocity_range 量程
 * @return 换算因子（浮点数）
 */
static float
module_bmi088_get_angular_velocity_scale(module_bmi088_gyro_range_t angular_velocity_range)
{
    static const float range_dps[] = {2000.0F, 1000.0F, 500.0F, 250.0F, 125.0F};
    // 16 位有符号数最大值为 32768，换算为 rad/s
    return range_dps[angular_velocity_range] * MODULE_BMI088_DEGREES_TO_RADIANS / 32768.0F;
}

/**
 * @brief 从 2 字节小端数据解码 int16
 * @param data 2 字节数组（小端序）
 * @return 解码后的有符号 16 位整数
 */
static int16_t module_bmi088_decode_int16(const uint8_t data[2])
{
    return (int16_t)(((uint16_t)data[1] << 8U) | data[0]);
}

/**
 * @brief 校验轴映射表合法性
 * @param axis_map 三个轴映射表
 * @return 执行状态
 * @note 检查 source_axis 是否唯一且 0~2，direction_sign 是否为 ±1
 */
static module_bmi088_status_t
module_bmi088_validate_axis_map(const module_bmi088_axis_map_t axis_map[3])
{
    bool source_is_used[3] = {false, false, false}; // 标记每个原始轴是否已被使用
    size_t axis_index;
    for (axis_index = 0U; axis_index < 3U; ++axis_index)
    {
        // 检查 source_axis 范围
        if ((axis_map[axis_index].source_axis > 2U) ||
            // 检查方向符号
            ((axis_map[axis_index].direction_sign != 1.0F) &&
             (axis_map[axis_index].direction_sign != -1.0F)) ||
            // 检查是否重复使用原始轴
            source_is_used[axis_map[axis_index].source_axis])
        {
            return MODULE_BMI088_STATUS_INVALID_ARGUMENT;
        }
        source_is_used[axis_map[axis_index].source_axis] = true;
    }
    return MODULE_BMI088_STATUS_OK;
}

/* ======================== module_device 回调函数 ======================== */

/**
 * @brief 设备启动回调（空操作）
 */
static module_device_status_t module_bmi088_device_start(module_device_t *const device_base)
{
    (void)device_base;
    return MODULE_DEVICE_STATUS_OK;
}

/**
 * @brief 设备停止回调（空操作）
 */
static module_device_status_t module_bmi088_device_stop(module_device_t *const device_base)
{
    (void)device_base;
    return MODULE_DEVICE_STATUS_OK;
}

/**
 * @brief 设备更新回调（调用 module_bmi088_read）
 */
static module_device_status_t module_bmi088_device_update(module_device_t *const device_base,
                                                          uint32_t elapsed_time_ms)
{
    module_bmi088_t *const me = MODULE_CONTAINER_OF(device_base, module_bmi088_t, super);
    (void)elapsed_time_ms;
    return (module_bmi088_read(me) == MODULE_BMI088_STATUS_OK)
               ? MODULE_DEVICE_STATUS_OK
               : MODULE_DEVICE_STATUS_OPERATION_FAILED;
}

/** BMI088 的设备操作表 */
static const module_device_ops_t s_module_bmi088_device_ops = {
    .start = module_bmi088_device_start,
    .stop = module_bmi088_device_stop,
    .update = module_bmi088_device_update,
};

/* ======================== 内部配置函数 ======================== */

/**
 * @brief 执行 BMI088 寄存器配置（写寄存器并回读校验）
 * @param me BMI088 对象
 * @param acceleration_range 加速度量程
 * @param angular_velocity_range 陀螺仪量程
 * @return 执行状态
 */
static module_bmi088_status_t
module_bmi088_configure_internal(module_bmi088_t *const me,
                                 module_bmi088_accel_range_t acceleration_range,
                                 module_bmi088_gyro_range_t angular_velocity_range)
{
    // 量程枚举值到寄存器值的映射表
    static const uint8_t acceleration_range_values[] = {0x00U, 0x01U, 0x02U, 0x03U};
    static const uint8_t angular_velocity_range_values[] = {0x00U, 0x01U, 0x02U, 0x03U, 0x04U};

    // 参数校验
    if ((me == NULL) || (acceleration_range > MODULE_BMI088_ACCEL_RANGE_24G) ||
        (angular_velocity_range > MODULE_BMI088_GYRO_RANGE_125_DPS))
    {
        return MODULE_BMI088_STATUS_INVALID_ARGUMENT;
    }

    // 加速度计配置表
    const module_bmi088_register_config_t acceleration_config[] = {
        {MODULE_BMI088_ACCEL_POWER_CONTROL_REGISTER, MODULE_BMI088_ACCEL_POWER_ON},      // 上电
        {MODULE_BMI088_ACCEL_POWER_CONFIG_REGISTER, MODULE_BMI088_ACCEL_ACTIVE_MODE},    // 普通模式
        {MODULE_BMI088_ACCEL_CONFIG_REGISTER, MODULE_BMI088_ACCEL_CONFIG_NORMAL_800_HZ}, // 800Hz
        {MODULE_BMI088_ACCEL_RANGE_REGISTER, acceleration_range_values[acceleration_range]}, // 量程
        {MODULE_BMI088_ACCEL_INTERRUPT_IO_REGISTER,
         MODULE_BMI088_ACCEL_INTERRUPT_OUTPUT_ENABLE}, // 中断输出
        {MODULE_BMI088_ACCEL_INTERRUPT_MAP_REGISTER,
         MODULE_BMI088_ACCEL_INTERRUPT_MAP_DATA_READY} // 数据就绪映射
    };

    // 陀螺仪配置表
    const module_bmi088_register_config_t gyroscope_config[] = {
        {MODULE_BMI088_GYRO_RANGE_REGISTER,
         angular_velocity_range_values[angular_velocity_range]}, // 量程
        {MODULE_BMI088_GYRO_BANDWIDTH_REGISTER,
         MODULE_BMI088_GYRO_BANDWIDTH_2000_230_HZ},                          // 2000Hz/230Hz
        {MODULE_BMI088_GYRO_POWER_REGISTER, MODULE_BMI088_GYRO_NORMAL_MODE}, // 普通模式
        {MODULE_BMI088_GYRO_INTERRUPT_CONTROL_REGISTER,
         MODULE_BMI088_GYRO_DATA_READY_ENABLE}, // 数据就绪中断
        {MODULE_BMI088_GYRO_INTERRUPT_IO_REGISTER,
         MODULE_BMI088_GYRO_INTERRUPT_PUSH_PULL_ACTIVE_LOW}, // 中断 IO
        {MODULE_BMI088_GYRO_INTERRUPT_MAP_REGISTER,
         MODULE_BMI088_GYRO_INTERRUPT_MAP_DATA_READY_INT3} // 映射到 INT3
    };

    size_t register_index;
    module_bmi088_status_t status;

    // 配置加速度计
    for (register_index = 0U;
         register_index < (sizeof(acceleration_config) / sizeof(acceleration_config[0]));
         ++register_index)
    {
        status = module_bmi088_write_and_verify(
            me, MODULE_BMI088_SENSOR_ACCEL, acceleration_config[register_index].register_address,
            acceleration_config[register_index].register_value);
        if (status != MODULE_BMI088_STATUS_OK)
        {
            return status;
        }
    }

    // 配置陀螺仪
    for (register_index = 0U;
         register_index < (sizeof(gyroscope_config) / sizeof(gyroscope_config[0]));
         ++register_index)
    {
        status = module_bmi088_write_and_verify(me, MODULE_BMI088_SENSOR_GYRO,
                                                gyroscope_config[register_index].register_address,
                                                gyroscope_config[register_index].register_value);
        if (status != MODULE_BMI088_STATUS_OK)
        {
            return status;
        }
    }

    // 保存量程和换算因子
    me->acceleration_range = acceleration_range;
    me->angular_velocity_range = angular_velocity_range;
    me->acceleration_scale_m_per_s2 = module_bmi088_get_acceleration_scale(acceleration_range);
    me->angular_velocity_scale_rad_per_s =
        module_bmi088_get_angular_velocity_scale(angular_velocity_range);
    return MODULE_BMI088_STATUS_OK;
}

/* ======================== 公共 API ======================== */

/**
 * @brief 重新配置量程
 */
module_bmi088_status_t module_bmi088_configure(module_bmi088_t *const me,
                                               module_bmi088_accel_range_t acceleration_range,
                                               module_bmi088_gyro_range_t angular_velocity_range)
{
    if (me == NULL)
    {
        return MODULE_BMI088_STATUS_INVALID_ARGUMENT;
    }
    if (!module_device_is_initialized(&me->super))
    {
        return MODULE_BMI088_STATUS_NOT_INITIALIZED;
    }
    return module_bmi088_configure_internal(me, acceleration_range, angular_velocity_range);
}

/**
 * @brief 初始化 BMI088
 */
module_bmi088_status_t module_bmi088_init(module_bmi088_t *const me,
                                          const module_bmi088_config_t *const config)
{
    uint8_t chip_identifier;
    size_t axis_index;
    module_bmi088_status_t status;

    /* -------- 参数校验 -------- */
    if ((me == NULL) || (config == NULL) || (config->spi == NULL) ||
        !bsp_device_is_initialized(&config->spi->super) || (config->set_chip_select == NULL) ||
        (config->delay_ms == NULL) ||
        (module_bmi088_validate_axis_map(config->axis_map) != MODULE_BMI088_STATUS_OK))
    {
        return MODULE_BMI088_STATUS_INVALID_ARGUMENT;
    }

    /* -------- 初始化基类 -------- */
    if (module_device_init_base(&me->super, &s_module_bmi088_device_ops,
                                (config->logical_name != NULL) ? config->logical_name : "bmi088",
                                config->registration_key) != MODULE_DEVICE_STATUS_OK)
    {
        return MODULE_BMI088_STATUS_INVALID_ARGUMENT;
    }

    /* -------- 复制配置到对象 -------- */
    me->spi = config->spi;
    me->set_chip_select = config->set_chip_select;
    me->delay_ms = config->delay_ms;
    me->get_time_us = config->get_time_us;
    me->user_context = config->user_context;
    me->transfer_timeout_ms = config->transfer_timeout_ms;
    me->raw_data = (module_bmi088_raw_data_t){0};
    me->data = (module_bmi088_process_data_t){0};
    me->has_timestamp = false;
    for (axis_index = 0U; axis_index < 3U; ++axis_index)
    {
        me->axis_map[axis_index] = config->axis_map[axis_index];
        me->angular_velocity_bias_rad_per_s[axis_index] = 0.0F;
    }

    /* -------- 硬件复位 -------- */
    // 取消片选（确保初始状态）
    me->set_chip_select(me->user_context, MODULE_BMI088_SENSOR_ACCEL, false);
    me->set_chip_select(me->user_context, MODULE_BMI088_SENSOR_GYRO, false);

    // 软复位两个传感器
    (void)module_bmi088_write_register(me, MODULE_BMI088_SENSOR_ACCEL,
                                       MODULE_BMI088_ACCEL_SOFT_RESET_REGISTER,
                                       MODULE_BMI088_ACCEL_SOFT_RESET_COMMAND);
    (void)module_bmi088_write_register(me, MODULE_BMI088_SENSOR_GYRO,
                                       MODULE_BMI088_GYRO_SOFT_RESET_REGISTER,
                                       MODULE_BMI088_GYRO_SOFT_RESET_COMMAND);
    me->delay_ms(me->user_context, 80U); // 等待复位完成

    /* -------- 验证 Chip ID -------- */
    status = module_bmi088_read_registers(
        me, MODULE_BMI088_SENSOR_ACCEL, MODULE_BMI088_ACCEL_CHIP_ID_REGISTER, &chip_identifier, 1U);
    if ((status != MODULE_BMI088_STATUS_OK) ||
        (chip_identifier != MODULE_BMI088_ACCEL_CHIP_ID_VALUE))
    {
        module_device_abort_init(&me->super);
        return MODULE_BMI088_STATUS_ACCEL_NOT_FOUND;
    }

    status = module_bmi088_read_registers(
        me, MODULE_BMI088_SENSOR_GYRO, MODULE_BMI088_GYRO_CHIP_ID_REGISTER, &chip_identifier, 1U);
    if ((status != MODULE_BMI088_STATUS_OK) ||
        (chip_identifier != MODULE_BMI088_GYRO_CHIP_ID_VALUE))
    {
        module_device_abort_init(&me->super);
        return MODULE_BMI088_STATUS_GYRO_NOT_FOUND;
    }

    /* -------- 配置寄存器 -------- */
    status = module_bmi088_configure_internal(me, config->acceleration_range,
                                              config->angular_velocity_range);
    if (status != MODULE_BMI088_STATUS_OK)
    {
        module_device_abort_init(&me->super);
        return status;
    }

    /* -------- 完成初始化 -------- */
    if (module_device_complete_init(&me->super) != MODULE_DEVICE_STATUS_OK)
    {
        module_device_abort_init(&me->super);
        return MODULE_BMI088_STATUS_INVALID_ARGUMENT;
    }
    return MODULE_BMI088_STATUS_OK;
}

/**
 * @brief 读取一次传感器数据
 */
module_bmi088_status_t module_bmi088_read(module_bmi088_t *const me)
{
    uint8_t acceleration_data[6];       // 加速度原始数据（3 轴 × 2 字节）
    uint8_t angular_velocity_data[6];   // 角速度原始数据（3 轴 × 2 字节）
    uint8_t temperature_data[2];        // 温度原始数据（2 字节）
    int16_t sensor_acceleration[3];     // 解码后的加速度原始值
    int16_t sensor_angular_velocity[3]; // 解码后的角速度原始值
    int16_t temperature_raw;            // 解码后的温度原始值（10 位有符号）
    size_t axis_index;

    // 状态检查
    if ((me == NULL) || !module_device_is_initialized(&me->super))
    {
        return (me == NULL) ? MODULE_BMI088_STATUS_INVALID_ARGUMENT
                            : MODULE_BMI088_STATUS_NOT_INITIALIZED;
    }

    // 读取三个数据块（加速度、角速度、温度）
    if ((module_bmi088_read_registers(me, MODULE_BMI088_SENSOR_ACCEL,
                                      MODULE_BMI088_ACCEL_DATA_REGISTER, acceleration_data,
                                      6U) != MODULE_BMI088_STATUS_OK) ||
        (module_bmi088_read_registers(me, MODULE_BMI088_SENSOR_GYRO,
                                      MODULE_BMI088_GYRO_DATA_REGISTER, angular_velocity_data,
                                      6U) != MODULE_BMI088_STATUS_OK) ||
        (module_bmi088_read_registers(me, MODULE_BMI088_SENSOR_ACCEL,
                                      MODULE_BMI088_ACCEL_TEMPERATURE_REGISTER, temperature_data,
                                      2U) != MODULE_BMI088_STATUS_OK))
    {
        me->data.is_valid = false;
        ++me->data.failed_sample_count;
        return MODULE_BMI088_STATUS_TRANSPORT_ERROR;
    }

    // 解码原始值
    for (axis_index = 0U; axis_index < 3U; ++axis_index)
    {
        sensor_acceleration[axis_index] =
            module_bmi088_decode_int16(&acceleration_data[axis_index * 2U]);
        sensor_angular_velocity[axis_index] =
            module_bmi088_decode_int16(&angular_velocity_data[axis_index * 2U]);
    }

    // 应用轴映射和量程换算
    for (axis_index = 0U; axis_index < 3U; ++axis_index)
    {
        const uint8_t source_axis = me->axis_map[axis_index].source_axis;
        const float direction_sign = me->axis_map[axis_index].direction_sign;
        // 保存原始值（方便调试）
        me->raw_data.acceleration[axis_index] = sensor_acceleration[source_axis];
        me->raw_data.angular_velocity[axis_index] = sensor_angular_velocity[source_axis];
        // 计算物理量（并扣除零偏）
        me->data.acceleration_m_per_s2[axis_index] = direction_sign *
                                                     (float)sensor_acceleration[source_axis] *
                                                     me->acceleration_scale_m_per_s2;
        me->data.angular_velocity_rad_per_s[axis_index] =
            direction_sign * (float)sensor_angular_velocity[source_axis] *
                me->angular_velocity_scale_rad_per_s -
            me->angular_velocity_bias_rad_per_s[axis_index];
    }

    // 解码温度（10 位有符号，左移 3 位 + 低 5 位）
    temperature_raw =
        (int16_t)(((uint16_t)temperature_data[0] << 3U) | (temperature_data[1] >> 5U));
    if (temperature_raw > 1023) // 负温度
    {
        temperature_raw -= 2048;
    }
    me->raw_data.temperature = temperature_raw;
    me->data.temperature_c = (float)temperature_raw * 0.125F + 23.0F;

    // 记录时间戳和采样间隔
    if (me->get_time_us != NULL)
    {
        const uint32_t timestamp_us = me->get_time_us(me->user_context);
        me->data.sample_interval_us =
            me->has_timestamp ? (timestamp_us - me->data.timestamp_us) : 0U;
        me->data.timestamp_us = timestamp_us;
        me->has_timestamp = true;
    }

    // 更新统计
    ++me->data.sample_count;
    me->data.is_valid = true;
    return MODULE_BMI088_STATUS_OK;
}

/**
 * @brief 执行加速度计和陀螺仪自检
 */
module_bmi088_status_t module_bmi088_run_self_test(module_bmi088_t *const me)
{
    uint8_t positive_data[6];      // 正方向自检读数
    uint8_t negative_data[6];      // 负方向自检读数
    int32_t acceleration_delta[3]; // 正负读数差值
    uint8_t self_test_status = 0U;
    uint32_t poll_count;
    size_t axis_index;
    module_bmi088_status_t status;
    // 保存当前量程，以便自检完成后恢复
    const module_bmi088_accel_range_t saved_acceleration_range =
        (me != NULL) ? me->acceleration_range : MODULE_BMI088_ACCEL_RANGE_6G;
    const module_bmi088_gyro_range_t saved_angular_velocity_range =
        (me != NULL) ? me->angular_velocity_range : MODULE_BMI088_GYRO_RANGE_2000_DPS;

    if ((me == NULL) || !module_device_is_initialized(&me->super))
    {
        return (me == NULL) ? MODULE_BMI088_STATUS_INVALID_ARGUMENT
                            : MODULE_BMI088_STATUS_NOT_INITIALIZED;
    }

    /* -------- 加速度计自检 -------- */
    // 配置加速度计为自检模式（量程 ±24G，ODR 800Hz）
    status = module_bmi088_write_and_verify(me, MODULE_BMI088_SENSOR_ACCEL,
                                            MODULE_BMI088_ACCEL_CONFIG_REGISTER, 0xACU);
    if (status == MODULE_BMI088_STATUS_OK)
    {
        status = module_bmi088_write_and_verify(me, MODULE_BMI088_SENSOR_ACCEL,
                                                MODULE_BMI088_ACCEL_RANGE_REGISTER, 0x03U);
    }
    if (status == MODULE_BMI088_STATUS_OK)
    {
        status = module_bmi088_write_register(me, MODULE_BMI088_SENSOR_ACCEL,
                                              MODULE_BMI088_ACCEL_SELF_TEST_REGISTER,
                                              MODULE_BMI088_ACCEL_SELF_TEST_POSITIVE);
    }
    me->delay_ms(me->user_context, 50U);

    // 读取正方向数据
    if (status == MODULE_BMI088_STATUS_OK)
    {
        status = module_bmi088_read_registers(me, MODULE_BMI088_SENSOR_ACCEL,
                                              MODULE_BMI088_ACCEL_DATA_REGISTER, positive_data, 6U);
    }
    if (status == MODULE_BMI088_STATUS_OK)
    {
        status = module_bmi088_write_register(me, MODULE_BMI088_SENSOR_ACCEL,
                                              MODULE_BMI088_ACCEL_SELF_TEST_REGISTER,
                                              MODULE_BMI088_ACCEL_SELF_TEST_NEGATIVE);
    }
    me->delay_ms(me->user_context, 50U);

    // 读取负方向数据
    if (status == MODULE_BMI088_STATUS_OK)
    {
        status = module_bmi088_read_registers(me, MODULE_BMI088_SENSOR_ACCEL,
                                              MODULE_BMI088_ACCEL_DATA_REGISTER, negative_data, 6U);
    }
    // 关闭自检
    (void)module_bmi088_write_register(me, MODULE_BMI088_SENSOR_ACCEL,
                                       MODULE_BMI088_ACCEL_SELF_TEST_REGISTER,
                                       MODULE_BMI088_ACCEL_SELF_TEST_OFF);

    // 判定加速度计自检结果
    if (status == MODULE_BMI088_STATUS_OK)
    {
        for (axis_index = 0U; axis_index < 3U; ++axis_index)
        {
            int16_t pos = module_bmi088_decode_int16(&positive_data[axis_index * 2U]);
            int16_t neg = module_bmi088_decode_int16(&negative_data[axis_index * 2U]);
            acceleration_delta[axis_index] = (int32_t)pos - (int32_t)neg;
            if (acceleration_delta[axis_index] < 0)
            {
                acceleration_delta[axis_index] = -acceleration_delta[axis_index];
            }
        }
        // BMI088 规格要求：X/Y 轴差值 >= 1365，Z 轴 >= 683
        if ((acceleration_delta[0] < 1365) || (acceleration_delta[1] < 1365) ||
            (acceleration_delta[2] < 683))
        {
            status = MODULE_BMI088_STATUS_SELF_TEST_FAILED;
        }
    }

    // 若加速度计自检失败，恢复配置并返回
    if (status != MODULE_BMI088_STATUS_OK)
    {
        (void)module_bmi088_configure(me, saved_acceleration_range, saved_angular_velocity_range);
        return status;
    }

    /* -------- 陀螺仪自检 -------- */
    status = module_bmi088_write_register(me, MODULE_BMI088_SENSOR_GYRO,
                                          MODULE_BMI088_GYRO_SELF_TEST_REGISTER,
                                          MODULE_BMI088_GYRO_SELF_TEST_TRIGGER);
    if (status != MODULE_BMI088_STATUS_OK)
    {
        return status;
    }

    // 轮询等待自检完成
    for (poll_count = 0U; poll_count < 20U; ++poll_count)
    {
        me->delay_ms(me->user_context, 5U);
        status = module_bmi088_read_registers(me, MODULE_BMI088_SENSOR_GYRO,
                                              MODULE_BMI088_GYRO_SELF_TEST_REGISTER,
                                              &self_test_status, 1U);
        if ((status == MODULE_BMI088_STATUS_OK) &&
            ((self_test_status & MODULE_BMI088_GYRO_SELF_TEST_READY) != 0U))
        {
            break;
        }
    }

    // 判定陀螺仪自检结果
    if (((self_test_status & MODULE_BMI088_GYRO_SELF_TEST_READY) == 0U) ||
        ((self_test_status & MODULE_BMI088_GYRO_SELF_TEST_FAILED) != 0U))
    {
        (void)module_bmi088_configure(me, saved_acceleration_range, saved_angular_velocity_range);
        return MODULE_BMI088_STATUS_SELF_TEST_FAILED;
    }

    // 恢复原量程配置
    return module_bmi088_configure(me, saved_acceleration_range, saved_angular_velocity_range);
}

/**
 * @brief 陀螺仪零偏标定
 */
module_bmi088_status_t module_bmi088_calibrate_gyroscope(module_bmi088_t *const me,
                                                         uint32_t sample_count,
                                                         uint32_t sample_interval_ms,
                                                         float maximum_stationary_deviation)
{
    float sum[3] = {0.0F, 0.0F, 0.0F};                    // 累加值
    float minimum[3] = {INFINITY, INFINITY, INFINITY};    // 最小值
    float maximum[3] = {-INFINITY, -INFINITY, -INFINITY}; // 最大值
    uint32_t sample_index;
    size_t axis_index;

    // 参数校验
    if ((me == NULL) || !module_device_is_initialized(&me->super) || (sample_count == 0U) ||
        (maximum_stationary_deviation <= 0.0F))
    {
        return MODULE_BMI088_STATUS_INVALID_ARGUMENT;
    }

    // 连续采样
    for (sample_index = 0U; sample_index < sample_count; ++sample_index)
    {
        if (module_bmi088_read(me) != MODULE_BMI088_STATUS_OK)
        {
            return MODULE_BMI088_STATUS_TRANSPORT_ERROR;
        }
        for (axis_index = 0U; axis_index < 3U; ++axis_index)
        {
            // 计算当前采样值的物理量（不含零偏）
            const float unbiased_value = me->axis_map[axis_index].direction_sign *
                                         (float)me->raw_data.angular_velocity[axis_index] *
                                         me->angular_velocity_scale_rad_per_s;
            sum[axis_index] += unbiased_value;
            // 更新极值
            if (unbiased_value < minimum[axis_index])
            {
                minimum[axis_index] = unbiased_value;
            }
            if (unbiased_value > maximum[axis_index])
            {
                maximum[axis_index] = unbiased_value;
            }
        }
        me->delay_ms(me->user_context, sample_interval_ms);
    }

    // 检查运动量是否超过阈值
    for (axis_index = 0U; axis_index < 3U; ++axis_index)
    {
        if ((maximum[axis_index] - minimum[axis_index]) > maximum_stationary_deviation)
        {
            return MODULE_BMI088_STATUS_CALIBRATION_MOTION;
        }
    }

    // 计算平均值作为零偏
    for (axis_index = 0U; axis_index < 3U; ++axis_index)
    {
        me->angular_velocity_bias_rad_per_s[axis_index] = sum[axis_index] / (float)sample_count;
    }
    return MODULE_BMI088_STATUS_OK;
}

/**
 * @brief 手动设置陀螺仪零偏
 */
module_bmi088_status_t
module_bmi088_set_gyroscope_bias(module_bmi088_t *const me,
                                 const float angular_velocity_bias_rad_per_s[3])
{
    size_t axis_index;
    if ((me == NULL) || (angular_velocity_bias_rad_per_s == NULL) ||
        !module_device_is_initialized(&me->super))
    {
        return MODULE_BMI088_STATUS_INVALID_ARGUMENT;
    }
    for (axis_index = 0U; axis_index < 3U; ++axis_index)
    {
        if (!isfinite(angular_velocity_bias_rad_per_s[axis_index]))
        {
            return MODULE_BMI088_STATUS_OUT_OF_RANGE;
        }
        me->angular_velocity_bias_rad_per_s[axis_index] =
            angular_velocity_bias_rad_per_s[axis_index];
    }
    return MODULE_BMI088_STATUS_OK;
}

/**
 * @brief 获取物理量数据指针
 */
const module_bmi088_process_data_t *module_bmi088_get_data(const module_bmi088_t *const me)
{
    return ((me != NULL) && module_device_is_initialized(&me->super)) ? &me->data : NULL;
}

/**
 * @brief 获取原始数据指针
 */
const module_bmi088_raw_data_t *module_bmi088_get_raw_data(const module_bmi088_t *const me)
{
    return ((me != NULL) && module_device_is_initialized(&me->super)) ? &me->raw_data : NULL;
}

/**
 * @brief 获取 module_device_t 基类指针
 */
module_device_t *module_bmi088_as_device(module_bmi088_t *const me)
{
    return (me != NULL) ? &me->super : NULL;
}
