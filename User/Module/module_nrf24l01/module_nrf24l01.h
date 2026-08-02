/**
 * @file module_nrf24l01.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief nRF24L01(+) 2.4GHz 收发器驱动头文件
 * @version 1.0
 * @date 2026-07-28
 * @copyright Copyright (c) 2026
 *
 * @note 基于 bsp_spi_t、CE GPIO、CSN GPIO 和注入的微秒延时回调。
 *       支持地址、管道、速率、功率、自动应答、自动重发、发送轮询和接收。
 */

#ifndef MODULE_NRF24L01_H
#define MODULE_NRF24L01_H

#include "bsp_gpio.h"      // GPIO BSP 抽象层
#include "bsp_spi.h"       // SPI BSP 抽象层
#include "module_device.h" // 模块设备基类

#ifdef __cplusplus
extern "C"
{
#endif

/* ======================== 最大容量常量 ======================== */

/** @brief 最大有效载荷大小（32 字节，nRF24L01+ 支持） */
#define MODULE_NRF24L01_MAXIMUM_PAYLOAD_SIZE (32U)
/** @brief 最大地址大小（5 字节） */
#define MODULE_NRF24L01_MAXIMUM_ADDRESS_SIZE (5U)

    /* ======================== 状态码枚举 ======================== */

    /**
     * @brief nRF24L01 模块状态码
     */
    typedef enum
    {
        MODULE_NRF24L01_STATUS_OK = 0,             // 操作成功
        MODULE_NRF24L01_STATUS_NO_DATA,            // 无数据（接收 FIFO 空）
        MODULE_NRF24L01_STATUS_BUSY,               // 正在发送或忙
        MODULE_NRF24L01_STATUS_MAXIMUM_RETRANSMIT, // 达到最大重发次数
        MODULE_NRF24L01_STATUS_INVALID_ARGUMENT,   // 参数非法
        MODULE_NRF24L01_STATUS_NOT_INITIALIZED,    // 对象未初始化
        MODULE_NRF24L01_STATUS_NOT_STARTED,        // 未启动
        MODULE_NRF24L01_STATUS_TRANSPORT_ERROR,    // SPI 传输错误
        MODULE_NRF24L01_STATUS_DEVICE_NOT_FOUND    // 设备不存在（读取配置不匹配）
    } module_nrf24l01_status_t;

    /* ======================== 数据率枚举 ======================== */

    /**
     * @brief 空中数据率
     */
    typedef enum
    {
        MODULE_NRF24L01_DATA_RATE_1_MBPS = 0, // 1 Mbps
        MODULE_NRF24L01_DATA_RATE_2_MBPS,     // 2 Mbps
        MODULE_NRF24L01_DATA_RATE_250_KBPS    // 250 kbps（增强型）
    } module_nrf24l01_data_rate_t;

    /* ======================== 输出功率枚举 ======================== */

    /**
     * @brief 输出功率
     */
    typedef enum
    {
        MODULE_NRF24L01_OUTPUT_POWER_NEGATIVE_18_DBM = 0, // -18 dBm
        MODULE_NRF24L01_OUTPUT_POWER_NEGATIVE_12_DBM,     // -12 dBm
        MODULE_NRF24L01_OUTPUT_POWER_NEGATIVE_6_DBM,      // -6 dBm
        MODULE_NRF24L01_OUTPUT_POWER_0_DBM                // 0 dBm
    } module_nrf24l01_output_power_t;

    /* ======================== 工作模式枚举 ======================== */

    /**
     * @brief nRF24L01 工作模式
     */
    typedef enum
    {
        MODULE_NRF24L01_MODE_STANDBY = 0, // 待机（CE=0）
        MODULE_NRF24L01_MODE_RECEIVE,     // 接收模式（PRIM_RX=1, CE=1）
        MODULE_NRF24L01_MODE_TRANSMIT     // 发送模式（PRIM_RX=0, CE=1）
    } module_nrf24l01_mode_t;

    /* ======================== 延时回调类型 ======================== */

    /**
     * @brief 微秒级延时回调
     * @param delay_us 延时微秒数
     * @param user_context 用户上下文
     * @note 用于 CE 脉冲和上电等待，非阻塞环境可接受
     */
    typedef void (*module_nrf24l01_delay_us_t)(uint32_t delay_us, void *user_context);

    /* ======================== 配置结构体 ======================== */

    /**
     * @brief nRF24L01 初始化配置
     */
    typedef struct
    {
        bsp_spi_t *spi;                              // SPI BSP 基类
        bsp_gpio_t *chip_enable_gpio;                // CE GPIO（控制收发切换）
        bsp_gpio_t *chip_select_gpio;                // CSN GPIO（片选，低有效）
        uint8_t channel;                             // 频道（0~125）
        uint8_t address_size;                        // 地址宽度（3~5 字节）
        const uint8_t *link_address;                 // 双方共用链路地址，长度为 address_size
        uint8_t payload_size;                        // 固定载荷大小（1~32 字节）
        uint8_t automatic_retransmit_count;          // 自动重发次数（0~15）
        uint16_t automatic_retransmit_delay_us;      // 重发延时（250~4000us，步进250us）
        module_nrf24l01_data_rate_t data_rate;       // 数据率
        module_nrf24l01_output_power_t output_power; // 输出功率
        bool automatic_acknowledge_enabled;          // 是否启用自动应答
        uint32_t spi_timeout_ms;                     // SPI 超时（毫秒）
        module_nrf24l01_delay_us_t delay_us;         // 微秒延时回调
        void *delay_user_context;                    // 延时回调用户上下文
        const char *logical_name;                    // 逻辑名称
        uint32_t registration_key;                   // 注册键值
    } module_nrf24l01_config_t;

    /* ======================== 对象结构体 ======================== */

    /**
     * @brief nRF24L01 设备对象
     */
    typedef struct
    {
        module_device_t super;        // 设备基类
        bsp_spi_t *spi;               // SPI BSP 基类
        bsp_gpio_t *chip_enable_gpio; // CE GPIO
        bsp_gpio_t *chip_select_gpio; // CSN GPIO
        uint8_t channel;              // 频道
        uint8_t address_size;         // 地址宽度
        uint8_t link_address[MODULE_NRF24L01_MAXIMUM_ADDRESS_SIZE];
        uint8_t payload_size;                        // 固定载荷大小
        uint8_t configuration_register;              // 配置寄存器缓存
        uint8_t radio_frequency_setup_register;      // RF 设置寄存器缓存
        uint8_t automatic_retransmit_setup_register; // 自动重发设置寄存器缓存
        uint32_t spi_timeout_ms;                     // SPI 超时
        module_nrf24l01_delay_us_t delay_us;         // 延时回调
        void *delay_user_context;                    // 延时上下文
        module_nrf24l01_mode_t mode;                 // 当前工作模式
        bool transmit_pending;                       // 是否有发送事务待轮询
        bool is_started;                             // 是否已启动
    } module_nrf24l01_t;

    /* ======================== 公共 API ======================== */

    /**
     * @brief 初始化 nRF24L01 设备
     * @param me 设备对象
     * @param config 配置参数
     * @return 执行状态
     */
    module_nrf24l01_status_t module_nrf24l01_init(module_nrf24l01_t *me,
                                                  const module_nrf24l01_config_t *config);

    /**
     * @brief 启动设备（配置寄存器、验证 Chip ID、清 FIFO）
     * @param me 设备对象
     * @return 执行状态
     */
    module_nrf24l01_status_t module_nrf24l01_start(module_nrf24l01_t *me);

    /**
     * @brief 停止设备（关闭 CE、进入待机）
     * @param me 设备对象
     * @return 执行状态
     */
    module_nrf24l01_status_t module_nrf24l01_stop(module_nrf24l01_t *me);

    /**
     * @brief 设置接收地址（指定管道）
     * @param me 设备对象
     * @param pipe_index 管道索引（0~5）
     * @param address 地址数据
     * @param address_size 地址大小（需与配置一致）
     * @return 执行状态
     */
    module_nrf24l01_status_t module_nrf24l01_set_receive_address(module_nrf24l01_t *me,
                                                                 uint8_t pipe_index,
                                                                 const uint8_t *address,
                                                                 size_t address_size);

    /**
     * @brief 启用/禁用接收管道
     * @param me 设备对象
     * @param pipe_index 管道索引（0~5）
     * @param is_enabled true=启用，false=禁用
     * @return 执行状态
     */
    module_nrf24l01_status_t module_nrf24l01_set_receive_pipe_enabled(module_nrf24l01_t *me,
                                                                      uint8_t pipe_index,
                                                                      bool is_enabled);

    /**
     * @brief 设置发送地址
     * @param me 设备对象
     * @param address 地址数据
     * @param address_size 地址大小（需与配置一致）
     * @return 执行状态
     * @note 同时设置 TX_ADDR 和 RX_ADDR_P0（自动应答需要）
     */
    module_nrf24l01_status_t module_nrf24l01_set_transmit_address(module_nrf24l01_t *me,
                                                                  const uint8_t *address,
                                                                  size_t address_size);

    /**
     * @brief 启动接收模式
     * @param me 设备对象
     * @return 执行状态
     */
    module_nrf24l01_status_t module_nrf24l01_start_receive(module_nrf24l01_t *me);

    /**
     * @brief 发送数据
     * @param me 设备对象
     * @param payload 载荷数据
     * @param payload_size 载荷大小（必须与配置一致）
     * @return 执行状态
     * @note 启动发送后需轮询 poll_transmit 确认完成
     */
    module_nrf24l01_status_t module_nrf24l01_transmit(module_nrf24l01_t *me, const uint8_t *payload,
                                                      size_t payload_size);

    /**
     * @brief 轮询发送状态
     * @param me 设备对象
     * @return OK=发送成功，BUSY=仍在发送，MAXIMUM_RETRANSMIT=重发失败
     * @note 应在发送后周期性调用直到返回非 BUSY
     */
    module_nrf24l01_status_t module_nrf24l01_poll_transmit(module_nrf24l01_t *me);

    /**
     * @brief 接收数据
     * @param me 设备对象
     * @param payload 输出载荷
     * @param payload_capacity 载荷缓冲区容量（>= 配置载荷长度）
     * @param pipe_index 输出管道索引（可为 NULL）
     * @return OK=收到数据，NO_DATA=FIFO 空
     */
    module_nrf24l01_status_t module_nrf24l01_receive(module_nrf24l01_t *me, uint8_t *payload,
                                                     size_t payload_capacity, uint8_t *pipe_index);

    /**
     * @brief 获取发送观察统计
     * @param me 设备对象
     * @param lost_packet_count 输出丢包计数（4 位）
     * @param retransmit_count 输出重发计数（4 位）
     * @return 执行状态
     */
    module_nrf24l01_status_t module_nrf24l01_get_observe_transmit(module_nrf24l01_t *me,
                                                                  uint8_t *lost_packet_count,
                                                                  uint8_t *retransmit_count);

    /**
     * @brief 清空发送 FIFO
     * @param me 设备对象
     * @return 执行状态
     */
    module_nrf24l01_status_t module_nrf24l01_flush_transmit(module_nrf24l01_t *me);

    /**
     * @brief 清空接收 FIFO
     * @param me 设备对象
     * @return 执行状态
     */
    module_nrf24l01_status_t module_nrf24l01_flush_receive(module_nrf24l01_t *me);

#ifdef __cplusplus
}
#endif

#endif /* MODULE_NRF24L01_H */
