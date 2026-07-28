/**
 * @file module_nrf24l01.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief nRF24L01(+) 2.4GHz 收发器驱动实现
 * @version 1.0
 * @date 2026-07-28
 * @copyright Copyright (c) 2026
 *
 * @note 基于 bsp_spi_t、CE GPIO、CSN GPIO 和注入的微秒延时。
 *       支持地址、管道、速率、功率、自动应答、自动重发、发送轮询和接收。
 */

#include "module_nrf24l01.h"

#include <stddef.h> // NULL
#include <string.h> // memcpy, memset

/* ======================== nRF24L01 命令宏 ======================== */

/** @brief 读寄存器命令（低 5 位为寄存器地址） */
#define MODULE_NRF24L01_COMMAND_READ_REGISTER (0x00U)
/** @brief 写寄存器命令（低 5 位为寄存器地址） */
#define MODULE_NRF24L01_COMMAND_WRITE_REGISTER (0x20U)
/** @brief 读载荷命令（从接收 FIFO 读取） */
#define MODULE_NRF24L01_COMMAND_READ_PAYLOAD (0x61U)
/** @brief 写载荷命令（写入发送 FIFO） */
#define MODULE_NRF24L01_COMMAND_WRITE_PAYLOAD (0xA0U)
/** @brief 清空发送 FIFO */
#define MODULE_NRF24L01_COMMAND_FLUSH_TRANSMIT (0xE1U)
/** @brief 清空接收 FIFO */
#define MODULE_NRF24L01_COMMAND_FLUSH_RECEIVE (0xE2U)
/** @brief 空操作（用于读取状态寄存器） */
#define MODULE_NRF24L01_COMMAND_NOP (0xFFU)

/* ======================== nRF24L01 寄存器地址宏 ======================== */

#define MODULE_NRF24L01_REGISTER_CONFIG (0x00U)                       // 配置寄存器
#define MODULE_NRF24L01_REGISTER_ENABLE_AUTO_ACK (0x01U)              // 自动应答使能
#define MODULE_NRF24L01_REGISTER_ENABLE_RECEIVE_ADDRESS (0x02U)       // 接收地址使能
#define MODULE_NRF24L01_REGISTER_SETUP_ADDRESS_WIDTH (0x03U)          // 地址宽度设置
#define MODULE_NRF24L01_REGISTER_SETUP_RETRANSMIT (0x04U)             // 自动重发设置
#define MODULE_NRF24L01_REGISTER_RADIO_FREQUENCY_CHANNEL (0x05U)      // 频道设置
#define MODULE_NRF24L01_REGISTER_RADIO_FREQUENCY_SETUP (0x06U)        // RF 设置（速率/功率）
#define MODULE_NRF24L01_REGISTER_STATUS (0x07U)                       // 状态寄存器
#define MODULE_NRF24L01_REGISTER_OBSERVE_TRANSMIT (0x08U)             // 发送观察寄存器
#define MODULE_NRF24L01_REGISTER_RECEIVE_ADDRESS_PIPE_0 (0x0AU)       // 管道0 接收地址
#define MODULE_NRF24L01_REGISTER_TRANSMIT_ADDRESS (0x10U)             // 发送地址
#define MODULE_NRF24L01_REGISTER_RECEIVE_PAYLOAD_WIDTH_PIPE_0 (0x11U) // 管道0 载荷宽度

/* ======================== 配置寄存器位定义 ======================== */

/** @brief 使能 CRC */
#define MODULE_NRF24L01_CONFIG_ENABLE_CRC (1U << 3)
/** @brief CRC 为 2 字节（否则 1 字节） */
#define MODULE_NRF24L01_CONFIG_CRC_TWO_BYTES (1U << 2)
/** @brief 上电（PWR_UP） */
#define MODULE_NRF24L01_CONFIG_POWER_UP (1U << 1)
/** @brief 主接收模式（PRIM_RX） */
#define MODULE_NRF24L01_CONFIG_PRIMARY_RECEIVE (1U << 0)

/* ======================== 状态寄存器位定义 ======================== */

/** @brief 接收数据就绪（RX_DR） */
#define MODULE_NRF24L01_STATUS_RECEIVE_DATA_READY (1U << 6)
/** @brief 发送数据已发送（TX_DS） */
#define MODULE_NRF24L01_STATUS_TRANSMIT_DATA_SENT (1U << 5)
/** @brief 达到最大重发次数（MAX_RT） */
#define MODULE_NRF24L01_STATUS_MAXIMUM_RETRANSMIT (1U << 4)

/* ======================== 内部函数 ======================== */

/**
 * @brief SPI 全双工交换（带 CSN 片选控制）
 * @param me 设备对象
 * @param transmit_data 发送数据指针
 * @param receive_data 接收数据指针
 * @param data_size 数据大小（字节）
 * @return 执行状态
 * @note CSN 在整个事务期间保持低电平
 */
static module_nrf24l01_status_t module_nrf24l01_exchange(module_nrf24l01_t *me,
                                                         const uint8_t *transmit_data,
                                                         uint8_t *receive_data, size_t data_size)
{
    bsp_status_t status;

    // 选中芯片（CSN 低电平）
    if (bsp_gpio_write(me->chip_select_gpio, false) != BSP_STATUS_OK)
    {
        return MODULE_NRF24L01_STATUS_TRANSPORT_ERROR;
    }
    // 执行 SPI 全双工交换
    status = bsp_spi_exchange(me->spi, transmit_data, receive_data, data_size,
                              BSP_TRANSFER_MODE_BLOCKING, me->spi_timeout_ms);
    // 取消选中（CSN 高电平）
    if (bsp_gpio_write(me->chip_select_gpio, true) != BSP_STATUS_OK)
    {
        return MODULE_NRF24L01_STATUS_TRANSPORT_ERROR;
    }
    return (status == BSP_STATUS_OK) ? MODULE_NRF24L01_STATUS_OK
                                     : MODULE_NRF24L01_STATUS_TRANSPORT_ERROR;
}

/**
 * @brief 执行单字节命令（不附带数据）
 * @param me 设备对象
 * @param command 命令字节
 * @param status_register 输出状态寄存器值（可为 NULL）
 * @return 执行状态
 */
static module_nrf24l01_status_t module_nrf24l01_command(module_nrf24l01_t *me, uint8_t command,
                                                        uint8_t *status_register)
{
    uint8_t receive_value = 0U;
    const module_nrf24l01_status_t status =
        module_nrf24l01_exchange(me, &command, &receive_value, 1U);
    if ((status == MODULE_NRF24L01_STATUS_OK) && (status_register != NULL))
    {
        *status_register = receive_value;
    }
    return status;
}

/**
 * @brief 读取寄存器
 * @param me 设备对象
 * @param register_address 寄存器地址
 * @param register_data 输出数据缓冲区
 * @param data_size 数据大小（最多 5 字节）
 * @return 执行状态
 * @note 第一个字节为命令，后续字节为寄存器数据
 */
static module_nrf24l01_status_t module_nrf24l01_read_register(module_nrf24l01_t *me,
                                                              uint8_t register_address,
                                                              uint8_t *register_data,
                                                              size_t data_size)
{
    uint8_t transmit_buffer[MODULE_NRF24L01_MAXIMUM_ADDRESS_SIZE + 1U];
    uint8_t receive_buffer[MODULE_NRF24L01_MAXIMUM_ADDRESS_SIZE + 1U];
    module_nrf24l01_status_t status;

    // 参数校验
    if ((register_data == NULL) || (data_size == 0U) ||
        (data_size > MODULE_NRF24L01_MAXIMUM_ADDRESS_SIZE))
    {
        return MODULE_NRF24L01_STATUS_INVALID_ARGUMENT;
    }
    // 填充发送缓冲区为 NOP（读操作时 MOSI 发送 NOP）
    memset(transmit_buffer, MODULE_NRF24L01_COMMAND_NOP, data_size + 1U);
    // 命令 = 读寄存器命令 | 寄存器地址（低 5 位）
    transmit_buffer[0] = MODULE_NRF24L01_COMMAND_READ_REGISTER | (register_address & 0x1FU);

    status = module_nrf24l01_exchange(me, transmit_buffer, receive_buffer, data_size + 1U);
    if (status == MODULE_NRF24L01_STATUS_OK)
    {
        // 跳过第一个字节（命令响应），拷贝寄存器数据
        memcpy(register_data, &receive_buffer[1], data_size);
    }
    return status;
}

/**
 * @brief 写入寄存器
 * @param me 设备对象
 * @param register_address 寄存器地址
 * @param register_data 数据缓冲区
 * @param data_size 数据大小（最多 5 字节）
 * @return 执行状态
 */
static module_nrf24l01_status_t module_nrf24l01_write_register(module_nrf24l01_t *me,
                                                               uint8_t register_address,
                                                               const uint8_t *register_data,
                                                               size_t data_size)
{
    uint8_t transmit_buffer[MODULE_NRF24L01_MAXIMUM_ADDRESS_SIZE + 1U];
    uint8_t receive_buffer[MODULE_NRF24L01_MAXIMUM_ADDRESS_SIZE + 1U];

    if ((register_data == NULL) || (data_size == 0U) ||
        (data_size > MODULE_NRF24L01_MAXIMUM_ADDRESS_SIZE))
    {
        return MODULE_NRF24L01_STATUS_INVALID_ARGUMENT;
    }
    // 命令 = 写寄存器命令 | 寄存器地址（低 5 位）
    transmit_buffer[0] = MODULE_NRF24L01_COMMAND_WRITE_REGISTER | (register_address & 0x1FU);
    memcpy(&transmit_buffer[1], register_data, data_size);
    return module_nrf24l01_exchange(me, transmit_buffer, receive_buffer, data_size + 1U);
}

/**
 * @brief 写入单个字节寄存器
 * @param me 设备对象
 * @param register_address 寄存器地址
 * @param register_value 寄存器值
 * @return 执行状态
 */
static module_nrf24l01_status_t module_nrf24l01_write_single_register(module_nrf24l01_t *me,
                                                                      uint8_t register_address,
                                                                      uint8_t register_value)
{
    return module_nrf24l01_write_register(me, register_address, &register_value, 1U);
}

/**
 * @brief 设置工作模式
 * @param me 设备对象
 * @param mode 目标模式
 * @return 执行状态
 * @note 关闭 CE，修改 CONFIG 寄存器，延时等待上电稳定
 */
static module_nrf24l01_status_t module_nrf24l01_set_mode(module_nrf24l01_t *me,
                                                         module_nrf24l01_mode_t mode)
{
    uint8_t configuration = me->configuration_register | MODULE_NRF24L01_CONFIG_POWER_UP;

    // 先关闭 CE（确保模式切换安全）
    if (bsp_gpio_write(me->chip_enable_gpio, false) != BSP_STATUS_OK)
    {
        return MODULE_NRF24L01_STATUS_TRANSPORT_ERROR;
    }
    // 设置 PRIM_RX 位：接收模式置 1，发送模式置 0
    if (mode == MODULE_NRF24L01_MODE_RECEIVE)
    {
        configuration |= MODULE_NRF24L01_CONFIG_PRIMARY_RECEIVE;
    }
    else
    {
        configuration &= (uint8_t)(~MODULE_NRF24L01_CONFIG_PRIMARY_RECEIVE);
    }

    // 写配置寄存器
    if (module_nrf24l01_write_single_register(me, MODULE_NRF24L01_REGISTER_CONFIG, configuration) !=
        MODULE_NRF24L01_STATUS_OK)
    {
        return MODULE_NRF24L01_STATUS_TRANSPORT_ERROR;
    }
    me->mode = mode;
    // 等待上电稳定（1.5ms）
    me->delay_us(1500U, me->delay_user_context);
    return MODULE_NRF24L01_STATUS_OK;
}

/* ======================== module_device 虚函数实现 ======================== */

/**
 * @brief 设备启动回调（转发至 module_nrf24l01_start）
 */
static module_device_status_t module_nrf24l01_device_start(module_device_t *const device_base)
{
    module_nrf24l01_t *const me = MODULE_CONTAINER_OF(device_base, module_nrf24l01_t, super);
    return (module_nrf24l01_start(me) == MODULE_NRF24L01_STATUS_OK)
               ? MODULE_DEVICE_STATUS_OK
               : MODULE_DEVICE_STATUS_OPERATION_FAILED;
}

/**
 * @brief 设备停止回调（转发至 module_nrf24l01_stop）
 */
static module_device_status_t module_nrf24l01_device_stop(module_device_t *const device_base)
{
    module_nrf24l01_t *const me = MODULE_CONTAINER_OF(device_base, module_nrf24l01_t, super);
    return (module_nrf24l01_stop(me) == MODULE_NRF24L01_STATUS_OK)
               ? MODULE_DEVICE_STATUS_OK
               : MODULE_DEVICE_STATUS_OPERATION_FAILED;
}

/**
 * @brief 设备更新回调（nRF24L01 不需要周期更新，空操作）
 */
static module_device_status_t module_nrf24l01_device_update(module_device_t *const device_base,
                                                            uint32_t elapsed_time_ms)
{
    (void)device_base;
    (void)elapsed_time_ms;
    return MODULE_DEVICE_STATUS_OK;
}

/** nRF24L01 的设备操作表 */
static const module_device_ops_t s_module_nrf24l01_ops = {
    .start = module_nrf24l01_device_start,
    .stop = module_nrf24l01_device_stop,
    .update = module_nrf24l01_device_update,
};

/* ======================== 公共 API ======================== */

/**
 * @brief 初始化 nRF24L01 设备
 * @param me 设备对象
 * @param config 配置参数
 * @return 执行状态
 */
module_nrf24l01_status_t module_nrf24l01_init(module_nrf24l01_t *me,
                                              const module_nrf24l01_config_t *config)
{
    uint8_t radio_frequency_setup;
    uint8_t retransmit_delay_field;

    /* -------- 参数校验 -------- */
    if ((me == NULL) || (config == NULL) || (config->spi == NULL) ||
        !bsp_device_is_initialized(&config->spi->super) || (config->chip_enable_gpio == NULL) ||
        (config->chip_select_gpio == NULL) ||
        !bsp_device_is_initialized(&config->chip_enable_gpio->super) ||
        !bsp_device_is_initialized(&config->chip_select_gpio->super) || (config->channel > 125U) ||
        (config->address_size < 3U) || (config->address_size > 5U) ||
        (config->payload_size == 0U) ||
        (config->payload_size > MODULE_NRF24L01_MAXIMUM_PAYLOAD_SIZE) ||
        (config->automatic_retransmit_count > 15U) ||
        (config->automatic_retransmit_delay_us < 250U) ||
        (config->automatic_retransmit_delay_us > 4000U) ||
        ((config->automatic_retransmit_delay_us % 250U) != 0U) ||
        (config->data_rate > MODULE_NRF24L01_DATA_RATE_250_KBPS) ||
        (config->output_power > MODULE_NRF24L01_OUTPUT_POWER_0_DBM) || (config->delay_us == NULL))
    {
        return MODULE_NRF24L01_STATUS_INVALID_ARGUMENT;
    }

    /* -------- 初始化对象 -------- */
    *me = (module_nrf24l01_t){0};
    me->spi = config->spi;
    me->chip_enable_gpio = config->chip_enable_gpio;
    me->chip_select_gpio = config->chip_select_gpio;
    me->channel = config->channel;
    me->address_size = config->address_size;
    me->payload_size = config->payload_size;

    // 配置寄存器：使能 CRC，2 字节 CRC
    me->configuration_register =
        MODULE_NRF24L01_CONFIG_ENABLE_CRC | MODULE_NRF24L01_CONFIG_CRC_TWO_BYTES;

    // RF 设置寄存器：输出功率 + 数据率
    radio_frequency_setup = (uint8_t)((uint8_t)config->output_power << 1U);
    if (config->data_rate == MODULE_NRF24L01_DATA_RATE_2_MBPS)
    {
        radio_frequency_setup |= (1U << 3); // RF_DR_HIGH = 1, RF_DR_LOW = 0 → 2Mbps
    }
    else if (config->data_rate == MODULE_NRF24L01_DATA_RATE_250_KBPS)
    {
        radio_frequency_setup |= (1U << 5); // RF_DR_LOW = 1 → 250kbps
    }
    me->radio_frequency_setup_register = radio_frequency_setup;

    // 自动重发设置：延时（250us 步进 - 1）+ 重发次数
    retransmit_delay_field = (uint8_t)(config->automatic_retransmit_delay_us / 250U - 1U);
    me->automatic_retransmit_setup_register =
        (uint8_t)((retransmit_delay_field << 4U) | config->automatic_retransmit_count);

    me->spi_timeout_ms = config->spi_timeout_ms;
    me->delay_us = config->delay_us;
    me->delay_user_context = config->delay_user_context;

    /* -------- 初始化基类 -------- */
    if (module_device_init_base(&me->super, &s_module_nrf24l01_ops, config->logical_name,
                                config->registration_key) != MODULE_DEVICE_STATUS_OK)
    {
        return MODULE_NRF24L01_STATUS_INVALID_ARGUMENT;
    }

    /* -------- 初始化 GPIO -------- */
    // CE = 0（待机），CSN = 1（取消片选）
    if (bsp_gpio_write(me->chip_enable_gpio, false) != BSP_STATUS_OK ||
        bsp_gpio_write(me->chip_select_gpio, true) != BSP_STATUS_OK)
    {
        module_device_abort_init(&me->super);
        return MODULE_NRF24L01_STATUS_TRANSPORT_ERROR;
    }

    /* -------- 配置寄存器 -------- */
    if ((module_nrf24l01_write_single_register(me, MODULE_NRF24L01_REGISTER_CONFIG,
                                               me->configuration_register) !=
         MODULE_NRF24L01_STATUS_OK) ||
        // 自动应答
        (module_nrf24l01_write_single_register(
             me, MODULE_NRF24L01_REGISTER_ENABLE_AUTO_ACK,
             config->automatic_acknowledge_enabled ? 0x01U : 0x00U) != MODULE_NRF24L01_STATUS_OK) ||
        // 启用接收地址（默认只启用管道0）
        (module_nrf24l01_write_single_register(me, MODULE_NRF24L01_REGISTER_ENABLE_RECEIVE_ADDRESS,
                                               0x01U) != MODULE_NRF24L01_STATUS_OK) ||
        // 地址宽度（寄存器值 = 地址宽度 - 2）
        (module_nrf24l01_write_single_register(me, MODULE_NRF24L01_REGISTER_SETUP_ADDRESS_WIDTH,
                                               (uint8_t)(config->address_size - 2U)) !=
         MODULE_NRF24L01_STATUS_OK) ||
        // 自动重发设置
        (module_nrf24l01_write_single_register(me, MODULE_NRF24L01_REGISTER_SETUP_RETRANSMIT,
                                               me->automatic_retransmit_setup_register) !=
         MODULE_NRF24L01_STATUS_OK) ||
        // 频道
        (module_nrf24l01_write_single_register(me, MODULE_NRF24L01_REGISTER_RADIO_FREQUENCY_CHANNEL,
                                               me->channel) != MODULE_NRF24L01_STATUS_OK) ||
        // RF 设置
        (module_nrf24l01_write_single_register(me, MODULE_NRF24L01_REGISTER_RADIO_FREQUENCY_SETUP,
                                               me->radio_frequency_setup_register) !=
         MODULE_NRF24L01_STATUS_OK) ||
        // 管道0 载荷宽度
        (module_nrf24l01_write_single_register(
             me, MODULE_NRF24L01_REGISTER_RECEIVE_PAYLOAD_WIDTH_PIPE_0, me->payload_size) !=
         MODULE_NRF24L01_STATUS_OK))
    {
        module_device_abort_init(&me->super);
        return MODULE_NRF24L01_STATUS_TRANSPORT_ERROR;
    }

    /* -------- 完成初始化 -------- */
    if (module_device_complete_init(&me->super) != MODULE_DEVICE_STATUS_OK)
    {
        module_device_abort_init(&me->super);
        return MODULE_NRF24L01_STATUS_INVALID_ARGUMENT;
    }
    return MODULE_NRF24L01_STATUS_OK;
}

/**
 * @brief 启动设备
 * @param me 设备对象
 * @return 执行状态
 * @note 读取 CONFIG 验证设备存在，清空 FIFO 和中断标志
 */
module_nrf24l01_status_t module_nrf24l01_start(module_nrf24l01_t *me)
{
    uint8_t configuration_readback;

    if ((me == NULL) || !module_device_is_initialized(&me->super))
    {
        return MODULE_NRF24L01_STATUS_NOT_INITIALIZED;
    }

    // 读取 CONFIG 寄存器，验证设备存在
    if (module_nrf24l01_read_register(me, MODULE_NRF24L01_REGISTER_CONFIG, &configuration_readback,
                                      1U) != MODULE_NRF24L01_STATUS_OK)
    {
        return MODULE_NRF24L01_STATUS_TRANSPORT_ERROR;
    }
    if (configuration_readback != me->configuration_register)
    {
        return MODULE_NRF24L01_STATUS_DEVICE_NOT_FOUND;
    }

    // 清空 FIFO 和中断标志
    if ((module_nrf24l01_flush_transmit(me) != MODULE_NRF24L01_STATUS_OK) ||
        (module_nrf24l01_flush_receive(me) != MODULE_NRF24L01_STATUS_OK) ||
        // 写 STATUS 寄存器清除所有中断标志（写 1 清除）
        (module_nrf24l01_write_single_register(
             me, MODULE_NRF24L01_REGISTER_STATUS,
             MODULE_NRF24L01_STATUS_RECEIVE_DATA_READY | MODULE_NRF24L01_STATUS_TRANSMIT_DATA_SENT |
                 MODULE_NRF24L01_STATUS_MAXIMUM_RETRANSMIT) != MODULE_NRF24L01_STATUS_OK))
    {
        return MODULE_NRF24L01_STATUS_TRANSPORT_ERROR;
    }

    me->is_started = true;
    // 进入待机模式
    return module_nrf24l01_set_mode(me, MODULE_NRF24L01_MODE_STANDBY);
}

/**
 * @brief 停止设备
 * @param me 设备对象
 * @return 执行状态
 */
module_nrf24l01_status_t module_nrf24l01_stop(module_nrf24l01_t *me)
{
    if ((me == NULL) || !module_device_is_initialized(&me->super))
    {
        return MODULE_NRF24L01_STATUS_NOT_INITIALIZED;
    }

    // CE = 0（待机）
    if (bsp_gpio_write(me->chip_enable_gpio, false) != BSP_STATUS_OK ||
        // 关闭上电（清除 PWR_UP 位）
        module_nrf24l01_write_single_register(me, MODULE_NRF24L01_REGISTER_CONFIG,
                                              me->configuration_register) !=
            MODULE_NRF24L01_STATUS_OK)
    {
        return MODULE_NRF24L01_STATUS_TRANSPORT_ERROR;
    }

    me->is_started = false;
    me->transmit_pending = false;
    me->mode = MODULE_NRF24L01_MODE_STANDBY;
    return MODULE_NRF24L01_STATUS_OK;
}

/**
 * @brief 设置接收地址
 * @param me 设备对象
 * @param pipe_index 管道索引（0~5）
 * @param address 地址数据
 * @param address_size 地址大小
 * @return 执行状态
 */
module_nrf24l01_status_t module_nrf24l01_set_receive_address(module_nrf24l01_t *me,
                                                             uint8_t pipe_index,
                                                             const uint8_t *address,
                                                             size_t address_size)
{
    if ((me == NULL) || (address == NULL) || (pipe_index > 5U))
    {
        return MODULE_NRF24L01_STATUS_INVALID_ARGUMENT;
    }
    if (!module_device_is_initialized(&me->super))
    {
        return MODULE_NRF24L01_STATUS_NOT_INITIALIZED;
    }
    if (address_size != me->address_size)
    {
        return MODULE_NRF24L01_STATUS_INVALID_ARGUMENT;
    }
    // 管道0~1 使用完整地址，管道2~5 只使用低字节（与管道1共享高字节）
    return module_nrf24l01_write_register(
        me, (uint8_t)(MODULE_NRF24L01_REGISTER_RECEIVE_ADDRESS_PIPE_0 + pipe_index), address,
        (pipe_index < 2U) ? address_size : 1U);
}

/**
 * @brief 启用/禁用接收管道
 * @param me 设备对象
 * @param pipe_index 管道索引（0~5）
 * @param is_enabled true=启用，false=禁用
 * @return 执行状态
 */
module_nrf24l01_status_t
module_nrf24l01_set_receive_pipe_enabled(module_nrf24l01_t *me, uint8_t pipe_index, bool is_enabled)
{
    uint8_t enabled_pipe_mask;

    if ((me == NULL) || (pipe_index > 5U))
    {
        return MODULE_NRF24L01_STATUS_INVALID_ARGUMENT;
    }
    if (!module_device_is_initialized(&me->super))
    {
        return MODULE_NRF24L01_STATUS_NOT_INITIALIZED;
    }

    // 读取当前启用的管道掩码
    if (module_nrf24l01_read_register(me, MODULE_NRF24L01_REGISTER_ENABLE_RECEIVE_ADDRESS,
                                      &enabled_pipe_mask, 1U) != MODULE_NRF24L01_STATUS_OK)
    {
        return MODULE_NRF24L01_STATUS_TRANSPORT_ERROR;
    }

    // 修改指定位
    if (is_enabled)
    {
        enabled_pipe_mask |= (uint8_t)(1U << pipe_index);
    }
    else
    {
        enabled_pipe_mask &= (uint8_t)(~(uint8_t)(1U << pipe_index));
    }

    // 写入管道掩码和载荷宽度（禁用时载荷宽度设为 0）
    if ((module_nrf24l01_write_single_register(me, MODULE_NRF24L01_REGISTER_ENABLE_RECEIVE_ADDRESS,
                                               enabled_pipe_mask) != MODULE_NRF24L01_STATUS_OK) ||
        (module_nrf24l01_write_single_register(
             me, (uint8_t)(MODULE_NRF24L01_REGISTER_RECEIVE_PAYLOAD_WIDTH_PIPE_0 + pipe_index),
             is_enabled ? me->payload_size : 0U) != MODULE_NRF24L01_STATUS_OK))
    {
        return MODULE_NRF24L01_STATUS_TRANSPORT_ERROR;
    }
    return MODULE_NRF24L01_STATUS_OK;
}

/**
 * @brief 设置发送地址
 * @param me 设备对象
 * @param address 地址数据
 * @param address_size 地址大小
 * @return 执行状态
 * @note 同时设置 TX_ADDR 和 RX_ADDR_P0（自动应答需要）
 */
module_nrf24l01_status_t module_nrf24l01_set_transmit_address(module_nrf24l01_t *me,
                                                              const uint8_t *address,
                                                              size_t address_size)
{
    if ((me == NULL) || (address == NULL))
    {
        return MODULE_NRF24L01_STATUS_INVALID_ARGUMENT;
    }
    if (!module_device_is_initialized(&me->super))
    {
        return MODULE_NRF24L01_STATUS_NOT_INITIALIZED;
    }
    if (address_size != me->address_size)
    {
        return MODULE_NRF24L01_STATUS_INVALID_ARGUMENT;
    }

    // 设置发送地址
    if (module_nrf24l01_write_register(me, MODULE_NRF24L01_REGISTER_TRANSMIT_ADDRESS, address,
                                       address_size) != MODULE_NRF24L01_STATUS_OK)
    {
        return MODULE_NRF24L01_STATUS_TRANSPORT_ERROR;
    }
    // 同时设置管道0 接收地址（用于自动应答）
    return module_nrf24l01_write_register(me, MODULE_NRF24L01_REGISTER_RECEIVE_ADDRESS_PIPE_0,
                                          address, address_size);
}

/**
 * @brief 启动接收模式
 * @param me 设备对象
 * @return 执行状态
 */
module_nrf24l01_status_t module_nrf24l01_start_receive(module_nrf24l01_t *me)
{
    module_nrf24l01_status_t status;

    if ((me == NULL) || !module_device_is_initialized(&me->super))
    {
        return MODULE_NRF24L01_STATUS_NOT_INITIALIZED;
    }
    if (!me->is_started)
    {
        return MODULE_NRF24L01_STATUS_NOT_STARTED;
    }
    if (me->transmit_pending)
    {
        return MODULE_NRF24L01_STATUS_BUSY;
    }

    // 设置接收模式
    status = module_nrf24l01_set_mode(me, MODULE_NRF24L01_MODE_RECEIVE);
    if (status != MODULE_NRF24L01_STATUS_OK)
    {
        return status;
    }
    // CE = 1 开始接收
    return (bsp_gpio_write(me->chip_enable_gpio, true) == BSP_STATUS_OK)
               ? MODULE_NRF24L01_STATUS_OK
               : MODULE_NRF24L01_STATUS_TRANSPORT_ERROR;
}

/**
 * @brief 发送数据
 * @param me 设备对象
 * @param payload 载荷数据
 * @param payload_size 载荷大小（需与配置一致）
 * @return 执行状态
 * @note 启动发送后需轮询 poll_transmit 确认完成
 */
module_nrf24l01_status_t module_nrf24l01_transmit(module_nrf24l01_t *me, const uint8_t *payload,
                                                  size_t payload_size)
{
    uint8_t transmit_buffer[MODULE_NRF24L01_MAXIMUM_PAYLOAD_SIZE + 1U];
    uint8_t receive_buffer[MODULE_NRF24L01_MAXIMUM_PAYLOAD_SIZE + 1U];
    module_nrf24l01_status_t status;

    /* -------- 参数校验 -------- */
    if ((me == NULL) || (payload == NULL))
    {
        return MODULE_NRF24L01_STATUS_INVALID_ARGUMENT;
    }
    if (!module_device_is_initialized(&me->super))
    {
        return MODULE_NRF24L01_STATUS_NOT_INITIALIZED;
    }
    if (payload_size != me->payload_size)
    {
        return MODULE_NRF24L01_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_started)
    {
        return MODULE_NRF24L01_STATUS_NOT_STARTED;
    }
    if (me->transmit_pending)
    {
        return MODULE_NRF24L01_STATUS_BUSY;
    }

    /* -------- 切换到发送模式 -------- */
    status = module_nrf24l01_set_mode(me, MODULE_NRF24L01_MODE_TRANSMIT);
    if (status != MODULE_NRF24L01_STATUS_OK)
    {
        return status;
    }

    // 清除中断标志（TX_DS 和 MAX_RT）
    (void)module_nrf24l01_write_single_register(me, MODULE_NRF24L01_REGISTER_STATUS,
                                                MODULE_NRF24L01_STATUS_TRANSMIT_DATA_SENT |
                                                    MODULE_NRF24L01_STATUS_MAXIMUM_RETRANSMIT);

    /* -------- 写载荷到发送 FIFO -------- */
    transmit_buffer[0] = MODULE_NRF24L01_COMMAND_WRITE_PAYLOAD;
    memcpy(&transmit_buffer[1], payload, payload_size);
    status = module_nrf24l01_exchange(me, transmit_buffer, receive_buffer, payload_size + 1U);
    if (status != MODULE_NRF24L01_STATUS_OK)
    {
        return status;
    }

    /* -------- CE 脉冲触发发送 -------- */
    // CE = 1（启动发送）
    if (bsp_gpio_write(me->chip_enable_gpio, true) != BSP_STATUS_OK)
    {
        return MODULE_NRF24L01_STATUS_TRANSPORT_ERROR;
    }
    // 最小 CE 脉冲宽度：10us（推荐 15us）
    me->delay_us(15U, me->delay_user_context);
    // CE = 0（发送完成后自动进入待机）
    if (bsp_gpio_write(me->chip_enable_gpio, false) != BSP_STATUS_OK)
    {
        return MODULE_NRF24L01_STATUS_TRANSPORT_ERROR;
    }

    me->transmit_pending = true;
    return MODULE_NRF24L01_STATUS_OK;
}

/**
 * @brief 轮询发送状态
 * @param me 设备对象
 * @return OK=发送成功，BUSY=仍在发送，MAXIMUM_RETRANSMIT=重发失败
 */
module_nrf24l01_status_t module_nrf24l01_poll_transmit(module_nrf24l01_t *me)
{
    uint8_t status_register;

    if ((me == NULL) || !module_device_is_initialized(&me->super))
    {
        return MODULE_NRF24L01_STATUS_NOT_INITIALIZED;
    }
    if (!me->transmit_pending)
    {
        return MODULE_NRF24L01_STATUS_NO_DATA;
    }

    // NOP 命令读取状态寄存器
    if (module_nrf24l01_command(me, MODULE_NRF24L01_COMMAND_NOP, &status_register) !=
        MODULE_NRF24L01_STATUS_OK)
    {
        return MODULE_NRF24L01_STATUS_TRANSPORT_ERROR;
    }

    // 检查是否达到最大重发次数
    if ((status_register & MODULE_NRF24L01_STATUS_MAXIMUM_RETRANSMIT) != 0U)
    {
        me->transmit_pending = false;
        (void)module_nrf24l01_write_single_register(me, MODULE_NRF24L01_REGISTER_STATUS,
                                                    MODULE_NRF24L01_STATUS_MAXIMUM_RETRANSMIT);
        (void)module_nrf24l01_flush_transmit(me); // 清空发送 FIFO
        return MODULE_NRF24L01_STATUS_MAXIMUM_RETRANSMIT;
    }

    // 检查是否发送成功
    if ((status_register & MODULE_NRF24L01_STATUS_TRANSMIT_DATA_SENT) != 0U)
    {
        me->transmit_pending = false;
        (void)module_nrf24l01_write_single_register(me, MODULE_NRF24L01_REGISTER_STATUS,
                                                    MODULE_NRF24L01_STATUS_TRANSMIT_DATA_SENT);
        return MODULE_NRF24L01_STATUS_OK;
    }

    // 仍在发送中
    return MODULE_NRF24L01_STATUS_BUSY;
}

/**
 * @brief 接收数据
 * @param me 设备对象
 * @param payload 输出载荷
 * @param payload_capacity 载荷缓冲区容量
 * @param pipe_index 输出管道索引（可为 NULL）
 * @return OK=收到数据，NO_DATA=FIFO 空
 */
module_nrf24l01_status_t module_nrf24l01_receive(module_nrf24l01_t *me, uint8_t *payload,
                                                 size_t payload_capacity, uint8_t *pipe_index)
{
    uint8_t status_register;
    uint8_t transmit_buffer[MODULE_NRF24L01_MAXIMUM_PAYLOAD_SIZE + 1U];
    uint8_t receive_buffer[MODULE_NRF24L01_MAXIMUM_PAYLOAD_SIZE + 1U];

    /* -------- 参数校验 -------- */
    if ((me == NULL) || (payload == NULL) || (payload_capacity < me->payload_size))
    {
        return MODULE_NRF24L01_STATUS_INVALID_ARGUMENT;
    }
    if (!module_device_is_initialized(&me->super))
    {
        return MODULE_NRF24L01_STATUS_NOT_INITIALIZED;
    }
    if (!me->is_started)
    {
        return MODULE_NRF24L01_STATUS_NOT_STARTED;
    }

    /* -------- 检查 RX FIFO 是否有数据 -------- */
    if (module_nrf24l01_command(me, MODULE_NRF24L01_COMMAND_NOP, &status_register) !=
        MODULE_NRF24L01_STATUS_OK)
    {
        return MODULE_NRF24L01_STATUS_TRANSPORT_ERROR;
    }
    if ((status_register & MODULE_NRF24L01_STATUS_RECEIVE_DATA_READY) == 0U)
    {
        return MODULE_NRF24L01_STATUS_NO_DATA;
    }

    /* -------- 读取载荷 -------- */
    memset(transmit_buffer, MODULE_NRF24L01_COMMAND_NOP, me->payload_size + 1U);
    transmit_buffer[0] = MODULE_NRF24L01_COMMAND_READ_PAYLOAD;
    if (module_nrf24l01_exchange(me, transmit_buffer, receive_buffer, me->payload_size + 1U) !=
        MODULE_NRF24L01_STATUS_OK)
    {
        return MODULE_NRF24L01_STATUS_TRANSPORT_ERROR;
    }
    memcpy(payload, &receive_buffer[1], me->payload_size);

    // 输出管道号（从状态寄存器位 1~3 提取）
    if (pipe_index != NULL)
    {
        *pipe_index = (uint8_t)((status_register >> 1U) & 0x07U);
    }

    // 清除 RX_DR 中断标志
    return module_nrf24l01_write_single_register(me, MODULE_NRF24L01_REGISTER_STATUS,
                                                 MODULE_NRF24L01_STATUS_RECEIVE_DATA_READY);
}

/**
 * @brief 获取发送观察统计
 * @param me 设备对象
 * @param lost_packet_count 输出丢包计数（4 位）
 * @param retransmit_count 输出重发计数（4 位）
 * @return 执行状态
 */
module_nrf24l01_status_t module_nrf24l01_get_observe_transmit(module_nrf24l01_t *me,
                                                              uint8_t *lost_packet_count,
                                                              uint8_t *retransmit_count)
{
    uint8_t observe_transmit;

    if ((me == NULL) || (lost_packet_count == NULL) || (retransmit_count == NULL))
    {
        return MODULE_NRF24L01_STATUS_INVALID_ARGUMENT;
    }
    if (!module_device_is_initialized(&me->super))
    {
        return MODULE_NRF24L01_STATUS_NOT_INITIALIZED;
    }

    // 读取 OBSERVE_TX 寄存器
    if (module_nrf24l01_read_register(me, MODULE_NRF24L01_REGISTER_OBSERVE_TRANSMIT,
                                      &observe_transmit, 1U) != MODULE_NRF24L01_STATUS_OK)
    {
        return MODULE_NRF24L01_STATUS_TRANSPORT_ERROR;
    }
    // 高 4 位：丢包计数（PLOS_CNT），低 4 位：重发计数（ARC_CNT）
    *lost_packet_count = (uint8_t)(observe_transmit >> 4U);
    *retransmit_count = (uint8_t)(observe_transmit & 0x0FU);
    return MODULE_NRF24L01_STATUS_OK;
}

/**
 * @brief 清空发送 FIFO
 */
module_nrf24l01_status_t module_nrf24l01_flush_transmit(module_nrf24l01_t *me)
{
    if (me == NULL)
    {
        return MODULE_NRF24L01_STATUS_INVALID_ARGUMENT;
    }
    if (!module_device_is_initialized(&me->super))
    {
        return MODULE_NRF24L01_STATUS_NOT_INITIALIZED;
    }
    return module_nrf24l01_command(me, MODULE_NRF24L01_COMMAND_FLUSH_TRANSMIT, NULL);
}

/**
 * @brief 清空接收 FIFO
 */
module_nrf24l01_status_t module_nrf24l01_flush_receive(module_nrf24l01_t *me)
{
    if (me == NULL)
    {
        return MODULE_NRF24L01_STATUS_INVALID_ARGUMENT;
    }
    if (!module_device_is_initialized(&me->super))
    {
        return MODULE_NRF24L01_STATUS_NOT_INITIALIZED;
    }
    return module_nrf24l01_command(me, MODULE_NRF24L01_COMMAND_FLUSH_RECEIVE, NULL);
}