/**
 * @file module_bluetooth.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 通用蓝牙串口模块头文件
 * @version 1.0
 * @date 2026-07-28
 * @copyright Copyright (c) 2026
 *
 * @note 基于 bsp_usart_t 的通用蓝牙串口模块，不绑定具体蓝牙芯片协议。
 *       提供异步接收、双缓冲转交、在线超时、原始数据发送和 AT 命令发送。
 */

#ifndef MODULE_BLUETOOTH_H
#define MODULE_BLUETOOTH_H

#include "bsp_usart.h"     // USART BSP 抽象层
#include "module_device.h" // 模块设备基类

#ifdef __cplusplus
extern "C"
{
#endif

    /* ======================== 状态码枚举 ======================== */

    /**
     * @brief 蓝牙模块状态码
     */
    typedef enum
    {
        MODULE_BLUETOOTH_STATUS_OK = 0,           // 操作成功
        MODULE_BLUETOOTH_STATUS_INVALID_ARGUMENT, // 参数非法
        MODULE_BLUETOOTH_STATUS_NOT_INITIALIZED,  // 对象未初始化
        MODULE_BLUETOOTH_STATUS_NOT_STARTED,      // 未启动
        MODULE_BLUETOOTH_STATUS_TRANSPORT_ERROR,  // USART 传输错误
        MODULE_BLUETOOTH_STATUS_OFFLINE           // 设备离线（超时未收到数据）
    } module_bluetooth_status_t;

    /* ======================== 接收回调类型 ======================== */

    /**
     * @brief 接收数据回调函数（任务上下文执行）
     * @param receive_data 接收数据指针（指向 processing_buffer）
     * @param data_size 数据大小（字节）
     * @param user_context 用户上下文
     * @note 回调在 module_bluetooth_update 中执行，属于任务上下文。
     *       数据在回调期间有效，如需长期保存需自行复制。
     */
    typedef void (*module_bluetooth_receive_callback_t)(const uint8_t *receive_data,
                                                        size_t data_size, void *user_context);

    /* ======================== 配置结构体 ======================== */

    /**
     * @brief 蓝牙模块初始化配置
     */
    typedef struct
    {
        bsp_usart_t *usart;               // USART BSP 基类（必须已初始化）
        uint8_t *receive_buffer;          // DMA 接收缓冲区（调用者分配）
        size_t receive_capacity;          // 接收缓冲区大小
        uint8_t *processing_buffer;       // 任务处理缓冲区（调用者分配）
        size_t processing_capacity;       // 处理缓冲区大小（>= receive_capacity）
        uint32_t transmit_timeout_ms;     // 发送超时（毫秒）
        uint32_t receive_timeout_ms;      // 接收超时（用于 USART 接收）
        uint32_t offline_timeout_ms;      // 离线超时（无数据则置离线）
        bsp_transfer_mode_t receive_mode; // 接收模式（仅 INTERRUPT 或 DMA，不支持 BLOCKING）
        const char *logical_name;         // 设备逻辑名称
        uint32_t registration_key;        // 注册键值
        module_bluetooth_receive_callback_t receive_callback; // 接收回调（可为 NULL）
        void *user_context;                                   // 回调用户上下文
    } module_bluetooth_config_t;

    /* ======================== 对象结构体 ======================== */

    /**
     * @brief 蓝牙设备对象
     */
    typedef struct
    {
        module_device_t super;                                // 设备基类
        bsp_usart_t *usart;                                   // USART BSP 基类
        uint8_t *receive_buffer;                              // DMA 接收缓冲区（外部引用）
        size_t receive_capacity;                              // 接收缓冲区大小
        uint8_t *processing_buffer;                           // 任务处理缓冲区（外部引用）
        size_t processing_capacity;                           // 处理缓冲区大小
        uint32_t transmit_timeout_ms;                         // 发送超时 (ms)
        uint32_t receive_timeout_ms;                          // 接收超时 (ms)
        uint32_t offline_timeout_ms;                          // 离线超时 (ms)
        uint32_t receive_elapsed_time_ms;                     // 距上次接收的时间 (ms)
        uint32_t receive_overrun_count;                       // 接收覆盖计数
        uint32_t receive_restart_error_count;                 // 重启 DMA 接收失败计数
        volatile size_t pending_receive_size;                 // 待处理数据大小（ISR 写入）
        bsp_transfer_mode_t receive_mode;                     // 接收模式
        module_bluetooth_receive_callback_t receive_callback; // 接收回调
        void *user_context;                                   // 回调用户上下文
        bool is_online;                                       // 是否在线
        volatile bool is_receive_pending;                     // 是否有待处理数据（ISR 置位）
        bool is_started;                                      // 是否已启动
    } module_bluetooth_t;

    /* ======================== 公共 API ======================== */

    /**
     * @brief 初始化蓝牙模块
     * @param me 设备对象
     * @param config 初始化配置
     * @return 执行状态
     */
    module_bluetooth_status_t module_bluetooth_init(module_bluetooth_t *me,
                                                    const module_bluetooth_config_t *config);

    /**
     * @brief 启动蓝牙模块（注册回调并启动空闲接收）
     * @param me 设备对象
     * @return 执行状态
     */
    module_bluetooth_status_t module_bluetooth_start(module_bluetooth_t *me);

    /**
     * @brief 停止蓝牙模块（中止接收、注销回调）
     * @param me 设备对象
     * @return 执行状态
     */
    module_bluetooth_status_t module_bluetooth_stop(module_bluetooth_t *me);

    /**
     * @brief 发送原始二进制数据
     * @param me 设备对象
     * @param transmit_data 发送数据指针
     * @param data_size 数据大小（字节）
     * @param transfer_mode 传输模式（BLOCKING/INTERRUPT/DMA）
     * @return 执行状态
     */
    module_bluetooth_status_t module_bluetooth_transmit(module_bluetooth_t *me,
                                                        const uint8_t *transmit_data,
                                                        size_t data_size,
                                                        bsp_transfer_mode_t transfer_mode);

    /**
     * @brief 发送以 NULL 结尾的 AT 命令字符串（阻塞发送）
     * @param me 设备对象
     * @param command 命令字符串（以 '\0' 结尾）
     * @return 执行状态
     */
    module_bluetooth_status_t module_bluetooth_send_command(module_bluetooth_t *me,
                                                            const char *command);

    /**
     * @brief 周期更新（处理待接收数据 + 更新在线超时）
     * @param me 设备对象
     * @param elapsed_time_ms 距上次更新的时间 (ms)
     * @return 执行状态
     */
    module_bluetooth_status_t module_bluetooth_update(module_bluetooth_t *me,
                                                      uint32_t elapsed_time_ms);

    /**
     * @brief 检查蓝牙模块是否在线
     * @param me 设备对象
     * @return true=在线（已启动且最近收到数据）
     */
    bool module_bluetooth_is_online(const module_bluetooth_t *me);

#ifdef __cplusplus
}
#endif

#endif /* MODULE_BLUETOOTH_H */