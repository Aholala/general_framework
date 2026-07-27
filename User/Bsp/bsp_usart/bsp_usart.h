/**
 * @file bsp_usart.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief USART/UART 通用抽象层头文件
 * @version 1.0
 * @date 2026-07-27
 * @copyright Copyright (c) 2026
 *
 * @note 通用 USART/UART 字节流接口，支持固定长度接收、空闲线接收、
 *       阻塞/中断/DMA 传输、中止和忙状态查询。
 *       波特率、字长、校验、停止位、反相和 DMA 通道由平台端配置。
 */

#ifndef BSP_USART_H
#define BSP_USART_H

#include "bsp_common.h" // 包含基础类型、状态码、设备基类、传输模式等

#ifdef __cplusplus
extern "C"
{
#endif

    /* 前向声明，避免循环依赖 */
    typedef struct bsp_usart bsp_usart_t;
    typedef struct bsp_usart_device bsp_usart_device_t;

    /* ---------- 高层虚表（面向应用层） ---------- */
    /**
     * @brief USART 操作虚表，继承自 bsp_device_ops_t
     * @note 派生类必须保持 super 为第一成员
     *       transmit 和 receive 必须实现
     *       receive_to_idle、abort、get_busy 为可选
     */
    typedef struct
    {
        bsp_device_ops_t super; // 父类虚表（含 deinit）

        /**
         * @brief 发送数据
         * @param me 基类指针
         * @param data 发送数据指针
         * @param size 数据大小（字节）
         * @param mode 传输模式（阻塞/中断/DMA）
         * @param timeout_ms 超时时间（毫秒）
         * @return 执行状态
         */
        bsp_status_t (*transmit)(bsp_usart_t *const me, const uint8_t *data, size_t size,
                                 bsp_transfer_mode_t mode, uint32_t timeout_ms);

        /**
         * @brief 接收数据（固定长度）
         * @param me 基类指针
         * @param data 接收缓冲区指针
         * @param size 数据大小（字节）
         * @param mode 传输模式（阻塞/中断/DMA）
         * @param timeout_ms 超时时间（毫秒）
         * @return 执行状态
         */
        bsp_status_t (*receive)(bsp_usart_t *const me, uint8_t *data, size_t size,
                                bsp_transfer_mode_t mode, uint32_t timeout_ms);

        /**
         * @brief 空闲线接收（不定长协议）
         * @param me 基类指针
         * @param data 接收缓冲区指针
         * @param capacity 缓冲区容量
         * @param mode 传输模式（阻塞/中断/DMA）
         * @param timeout_ms 超时时间（毫秒）
         * @return 执行状态
         * @note 收到空闲线或缓冲区满时通过回调通知
         */
        bsp_status_t (*receive_to_idle)(bsp_usart_t *const me, uint8_t *data, size_t capacity,
                                        bsp_transfer_mode_t mode, uint32_t timeout_ms);

        /**
         * @brief 中止当前事务
         * @param me 基类指针
         * @return 执行状态
         */
        bsp_status_t (*abort)(bsp_usart_t *const me);

        /**
         * @brief 查询 USART 是否忙
         * @param me 基类指针（const）
         * @param is_busy 输出是否忙
         * @return 执行状态
         */
        bsp_status_t (*get_busy)(const bsp_usart_t *const me, bool *is_busy);

    } bsp_usart_ops_t;

    /* ---------- 基类 ---------- */
    /**
     * @brief USART 基类结构体
     */
    struct bsp_usart
    {
        bsp_device_t super;            // 设备基类
        bsp_event_callback_t callback; // 事件回调函数
        void *user_context;            // 回调用户上下文
    };

    /* ---------- 底层驱动操作表（平台实现） ---------- */
    /**
     * @brief 平台相关 USART 驱动操作表
     * @note transmit/receive 必须实现，其余为可选
     *       波特率、字长、校验等由平台端配置
     */
    typedef struct
    {
        bsp_status_t (*init)(void *device_handle);   // 初始化（可选）
        bsp_status_t (*deinit)(void *device_handle); // 反初始化（可选）
        bsp_status_t (*transmit)(void *device_handle, const uint8_t *data, size_t size,
                                 bsp_transfer_mode_t mode, uint32_t timeout_ms); // 发送（必须）
        bsp_status_t (*receive)(void *device_handle, uint8_t *data, size_t size,
                                bsp_transfer_mode_t mode, uint32_t timeout_ms); // 接收（必须）
        bsp_status_t (*receive_to_idle)(void *device_handle, uint8_t *data, size_t capacity,
                                        bsp_transfer_mode_t mode,
                                        uint32_t timeout_ms);               // 空闲线接收（可选）
        bsp_status_t (*abort)(void *device_handle);                         // 中止（可选）
        bsp_status_t (*get_busy)(const void *device_handle, bool *is_busy); // 忙查询（可选）
    } bsp_usart_driver_ops_t;

    /* ---------- 派生设备对象 ---------- */
    /**
     * @brief USART 设备对象（派生类）
     */
    struct bsp_usart_device
    {
        bsp_usart_t super;                        // 基类实例
        const bsp_usart_driver_ops_t *driver_ops; // 底层驱动操作表
    };

    /* ---------- 配置结构 ---------- */
    /**
     * @brief USART 初始化配置
     */
    typedef struct
    {
        void *device_handle;                      // 平台设备句柄
        const bsp_usart_driver_ops_t *driver_ops; // 底层驱动表
        bsp_event_callback_t callback;            // 事件回调（可为 NULL）
        void *user_context;                       // 回调用户上下文
    } bsp_usart_config_t;

    /* ---------- 公共 API 声明 ---------- */

    /**
     * @brief 初始化 USART 设备
     * @param me 设备对象指针
     * @param config 配置参数
     * @return 执行状态
     */
    bsp_status_t bsp_usart_init(bsp_usart_device_t *const me,
                                const bsp_usart_config_t *const config);

    /**
     * @brief 将派生对象转为基类指针（向上转型）
     */
    bsp_usart_t *bsp_usart_as_base(bsp_usart_device_t *const me);

    /**
     * @brief 设置事件回调
     * @param me 基类指针
     * @param callback 回调函数指针
     * @param user_context 用户上下文
     * @return 执行状态
     */
    bsp_status_t bsp_usart_set_callback(bsp_usart_t *const me, bsp_event_callback_t callback,
                                        void *user_context);

    /**
     * @brief 发送数据
     * @param me 基类指针
     * @param data 发送数据指针
     * @param size 数据大小（字节），必须大于 0
     * @param mode 传输模式（阻塞/中断/DMA）
     * @param timeout_ms 超时时间（毫秒）
     * @return 执行状态
     */
    bsp_status_t bsp_usart_transmit(bsp_usart_t *const me, const uint8_t *data, size_t size,
                                    bsp_transfer_mode_t mode, uint32_t timeout_ms);

    /**
     * @brief 接收数据（固定长度）
     * @param me 基类指针
     * @param data 接收缓冲区指针
     * @param size 数据大小（字节），必须大于 0
     * @param mode 传输模式（阻塞/中断/DMA）
     * @param timeout_ms 超时时间（毫秒）
     * @return 执行状态
     */
    bsp_status_t bsp_usart_receive(bsp_usart_t *const me, uint8_t *data, size_t size,
                                   bsp_transfer_mode_t mode, uint32_t timeout_ms);

    /**
     * @brief 空闲线接收（不定长协议）
     * @param me 基类指针
     * @param data 接收缓冲区指针
     * @param capacity 缓冲区容量，必须大于 0
     * @param mode 传输模式（阻塞/中断/DMA）
     * @param timeout_ms 超时时间（毫秒）
     * @return 执行状态，若驱动未实现则返回 BSP_STATUS_UNSUPPORTED
     */
    bsp_status_t bsp_usart_receive_to_idle(bsp_usart_t *const me, uint8_t *data, size_t capacity,
                                           bsp_transfer_mode_t mode, uint32_t timeout_ms);

    /**
     * @brief 中止当前事务
     * @param me 基类指针
     * @return 执行状态，若驱动未实现则返回 BSP_STATUS_UNSUPPORTED
     */
    bsp_status_t bsp_usart_abort(bsp_usart_t *const me);

    /**
     * @brief 查询 USART 是否忙
     * @param me 基类指针（const）
     * @param is_busy 输出是否忙
     * @return 执行状态，若驱动未实现则返回 BSP_STATUS_UNSUPPORTED
     */
    bsp_status_t bsp_usart_get_busy(const bsp_usart_t *const me, bool *is_busy);

    /**
     * @brief 事件通知函数（供底层驱动调用，向上层传递事件）
     * @param me 基类指针
     * @param event 事件类型
     * @param status 状态码
     * @param transferred_size 已传输的数据量（字节数）
     */
    void bsp_usart_notify(bsp_usart_t *const me, bsp_event_t event, bsp_status_t status,
                          size_t transferred_size);

#ifdef __cplusplus
}
#endif

#endif /* BSP_USART_H */