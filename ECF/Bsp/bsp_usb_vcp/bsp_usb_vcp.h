/**
 * @file bsp_usb_vcp.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief USB CDC 虚拟串口通用抽象层头文件
 * @version 1.0
 * @date 2026-07-27
 * @copyright Copyright (c) 2026
 *
 * @note USB CDC 虚拟串口抽象，向 Module 提供异步字节流，
 *       而不暴露 USB Device 中间件类型（如 USBD_CDC_HandleTypeDef）。
 *       端点号、描述符、USB Device 类对象和中间件回调均留在平台 Port。
 */

#ifndef BSP_USB_VCP_H
#define BSP_USB_VCP_H

#include "bsp_common.h" // 包含基础类型、状态码、设备基类等

#ifdef __cplusplus
extern "C"
{
#endif

    /* 前向声明，避免循环依赖 */
    typedef struct bsp_usb_vcp bsp_usb_vcp_t;
    typedef struct bsp_usb_vcp_device bsp_usb_vcp_device_t;

    /* ---------- 高层虚表（面向应用层） ---------- */
    /**
     * @brief USB VCP 操作虚表，继承自 bsp_device_ops_t
     * @note 派生类必须保持 super 为第一成员
     *       transmit 和 receive 必须实现
     *       abort、get_connected、get_busy 为可选
     */
    typedef struct
    {
        bsp_device_ops_t super; // 父类虚表（含 deinit）

        /**
         * @brief 发送数据
         * @param me 基类指针
         * @param transmit_data 发送数据指针
         * @param data_size 数据大小（字节）
         * @param timeout_ms 超时时间（毫秒）
         * @return 执行状态
         * @note 忙时返回 BSP_STATUS_BUSY
         *       调用者必须让发送缓冲区保持有效直到发送完成通知
         */
        bsp_status_t (*transmit)(bsp_usb_vcp_t *const me, const uint8_t *transmit_data,
                                 size_t data_size, uint32_t timeout_ms);

        /**
         * @brief 接收数据（登记接收缓冲区）
         * @param me 基类指针
         * @param receive_data 接收缓冲区指针
         * @param data_capacity 缓冲区容量
         * @return 执行状态
         * @note 异步接收：平台收到数据后通过回调通知实际长度
         *       如果已有待接收数据，返回 BSP_STATUS_BUSY
         */
        bsp_status_t (*receive)(bsp_usb_vcp_t *const me, uint8_t *receive_data,
                                size_t data_capacity);

        /**
         * @brief 中止当前事务
         * @param me 基类指针
         * @return 执行状态
         */
        bsp_status_t (*abort)(bsp_usb_vcp_t *const me);

        /**
         * @brief 查询 USB 主机连接状态
         * @param me 基类指针（const）
         * @param is_connected 输出是否已连接（已枚举）
         * @return 执行状态
         */
        bsp_status_t (*get_connected)(const bsp_usb_vcp_t *const me, bool *is_connected);

        /**
         * @brief 查询 USB 发送是否忙
         * @param me 基类指针（const）
         * @param is_busy 输出是否忙
         * @return 执行状态
         */
        bsp_status_t (*get_busy)(const bsp_usb_vcp_t *const me, bool *is_busy);

    } bsp_usb_vcp_ops_t;

    /* ---------- 基类 ---------- */
    /**
     * @brief USB VCP 基类结构体
     */
    struct bsp_usb_vcp
    {
        bsp_device_t super;            // 设备基类
        bsp_event_callback_t callback; // 事件回调函数
        void *user_context;            // 回调用户上下文
    };

    /* ---------- 底层驱动操作表（平台实现） ---------- */
    /**
     * @brief 平台相关 USB VCP 驱动操作表
     * @note transmit/receive 必须实现，其余为可选
     *       端点号、描述符、USB Device 类对象均由平台端管理
     */
    typedef struct
    {
        bsp_status_t (*init)(void *device_handle);   // 初始化（可选）
        bsp_status_t (*deinit)(void *device_handle); // 反初始化（可选）
        bsp_status_t (*transmit)(void *device_handle, const uint8_t *transmit_data,
                                 size_t data_size, uint32_t timeout_ms); // 发送（必须）
        bsp_status_t (*receive)(void *device_handle, uint8_t *receive_data,
                                size_t data_capacity); // 接收（必须）
        bsp_status_t (*abort)(void *device_handle);    // 中止（可选）
        bsp_status_t (*get_connected)(const void *device_handle,
                                      bool *is_connected);                  // 连接状态（可选）
        bsp_status_t (*get_busy)(const void *device_handle, bool *is_busy); // 忙查询（可选）
    } bsp_usb_vcp_driver_ops_t;

    /* ---------- 派生设备对象 ---------- */
    /**
     * @brief USB VCP 设备对象（派生类）
     */
    struct bsp_usb_vcp_device
    {
        bsp_usb_vcp_t super;                        // 基类实例
        const bsp_usb_vcp_driver_ops_t *driver_ops; // 底层驱动操作表
    };

    /* ---------- 配置结构 ---------- */
    /**
     * @brief USB VCP 初始化配置
     */
    typedef struct
    {
        void *device_handle;                        // 平台设备句柄
        const bsp_usb_vcp_driver_ops_t *driver_ops; // 底层驱动表
        bsp_event_callback_t callback;              // 事件回调（可为 NULL）
        void *user_context;                         // 回调用户上下文
    } bsp_usb_vcp_config_t;

    /* ---------- 公共 API 声明 ---------- */

    /**
     * @brief 初始化 USB VCP 设备
     * @param me 设备对象指针
     * @param config 配置参数
     * @return 执行状态
     */
    bsp_status_t bsp_usb_vcp_init(bsp_usb_vcp_device_t *const me,
                                  const bsp_usb_vcp_config_t *const config);

    /**
     * @brief 将派生对象转为基类指针（向上转型）
     */
    bsp_usb_vcp_t *bsp_usb_vcp_as_base(bsp_usb_vcp_device_t *const me);

    /**
     * @brief 设置事件回调
     * @param me 基类指针
     * @param callback 回调函数指针
     * @param user_context 用户上下文
     * @return 执行状态
     */
    bsp_status_t bsp_usb_vcp_set_callback(bsp_usb_vcp_t *const me, bsp_event_callback_t callback,
                                          void *user_context);

    /**
     * @brief 发送数据
     * @param me 基类指针
     * @param transmit_data 发送数据指针
     * @param data_size 数据大小（字节），必须大于 0
     * @param timeout_ms 超时时间（毫秒）
     * @return 执行状态
     */
    bsp_status_t bsp_usb_vcp_transmit(bsp_usb_vcp_t *const me, const uint8_t *transmit_data,
                                      size_t data_size, uint32_t timeout_ms);

    /**
     * @brief 接收数据（登记接收缓冲区）
     * @param me 基类指针
     * @param receive_data 接收缓冲区指针
     * @param data_capacity 缓冲区容量，必须大于 0
     * @return 执行状态
     */
    bsp_status_t bsp_usb_vcp_receive(bsp_usb_vcp_t *const me, uint8_t *receive_data,
                                     size_t data_capacity);

    /**
     * @brief 中止当前事务
     * @param me 基类指针
     * @return 执行状态
     */
    bsp_status_t bsp_usb_vcp_abort(bsp_usb_vcp_t *const me);

    /**
     * @brief 查询 USB 主机连接状态
     * @param me 基类指针（const）
     * @param is_connected 输出是否已连接（已枚举）
     * @return 执行状态
     */
    bsp_status_t bsp_usb_vcp_get_connected(const bsp_usb_vcp_t *const me, bool *is_connected);

    /**
     * @brief 查询 USB 发送是否忙
     * @param me 基类指针（const）
     * @param is_busy 输出是否忙
     * @return 执行状态
     */
    bsp_status_t bsp_usb_vcp_get_busy(const bsp_usb_vcp_t *const me, bool *is_busy);

    /**
     * @brief 事件通知函数（供底层驱动调用，向上层传递事件）
     * @param me 基类指针
     * @param event 事件类型
     * @param status 状态码
     * @param transferred_size 已传输的数据量（字节数）
     */
    void bsp_usb_vcp_notify(bsp_usb_vcp_t *const me, bsp_event_t event, bsp_status_t status,
                            size_t transferred_size);

#ifdef __cplusplus
}
#endif

#endif