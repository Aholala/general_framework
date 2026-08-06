/**
 * @file bsp_spi.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief SPI 通用抽象层头文件
 * @version 1.0
 * @date 2026-07-27
 * @copyright Copyright (c) 2026
 *
 * @note 定义 SPI 的多态接口，支持发送、接收、全双工交换、中止和忙状态查询。
 *       三种传输模式（阻塞/中断/DMA）统一使用同一对象。
 */

#ifndef BSP_SPI_H
#define BSP_SPI_H

#include "bsp_common.h" // 包含基础类型、状态码、设备基类、传输模式等

#ifdef __cplusplus
extern "C"
{
#endif

    /* 前向声明，避免循环依赖 */
    typedef struct bsp_spi bsp_spi_t;
    typedef struct bsp_spi_device bsp_spi_device_t;

    /* ---------- 高层虚表（面向应用层） ---------- */
    /**
     * @brief SPI 操作虚表，继承自 bsp_device_ops_t
     * @note 派生类必须保持 super 为第一成员
     *       transmit 和 receive 必须实现，exchange/abort/get_busy 可选
     */
    typedef struct
    {
        bsp_device_ops_t super; // 父类虚表（含 deinit）

        /**
         * @brief 发送数据（只发不收）
         */
        bsp_status_t (*transmit)(bsp_spi_t *const, const uint8_t *, size_t, bsp_transfer_mode_t,
                                 uint32_t);

        /**
         * @brief 接收数据（只收不发，平台驱动负责产生时钟）
         */
        bsp_status_t (*receive)(bsp_spi_t *const, uint8_t *, size_t, bsp_transfer_mode_t, uint32_t);

        /**
         * @brief 全双工交换（等长收发同时进行）
         */
        bsp_status_t (*exchange)(bsp_spi_t *const, const uint8_t *, uint8_t *, size_t,
                                 bsp_transfer_mode_t, uint32_t);

        /**
         * @brief 中止当前异步事务
         */
        bsp_status_t (*abort)(bsp_spi_t *const);

        /**
         * @brief 查询总线是否忙
         */
        bsp_status_t (*get_busy)(const bsp_spi_t *const, bool *);

    } bsp_spi_ops_t;

    /* ---------- 基类 ---------- */
    /**
     * @brief SPI 基类结构体
     */
    struct bsp_spi
    {
        bsp_device_t super;            // 设备基类
        bsp_event_callback_t callback; // 事件回调函数
        void *user_context;            // 回调用户上下文
    };

    /* ---------- 底层驱动操作表（平台实现） ---------- */
    /**
     * @brief 平台相关 SPI 驱动操作表
     * @note transmit/receive 必须实现，其余为可选（公共层会检查 NULL）
     */
    typedef struct
    {
        bsp_status_t (*init)(void *);   // 初始化硬件（可选）
        bsp_status_t (*deinit)(void *); // 反初始化（可选）
        bsp_status_t (*transmit)(void *, const uint8_t *, size_t, bsp_transfer_mode_t,
                                 uint32_t); // 发送（必须）
        bsp_status_t (*receive)(void *, uint8_t *, size_t, bsp_transfer_mode_t,
                                uint32_t); // 接收（必须）
        bsp_status_t (*exchange)(void *, const uint8_t *, uint8_t *, size_t, bsp_transfer_mode_t,
                                 uint32_t);             // 全双工交换（可选）
        bsp_status_t (*abort)(void *);                  // 中止（可选）
        bsp_status_t (*get_busy)(const void *, bool *); // 忙查询（可选）
    } bsp_spi_driver_ops_t;

    /* ---------- 派生设备对象 ---------- */
    /**
     * @brief SPI 设备对象（派生类）
     */
    struct bsp_spi_device
    {
        bsp_spi_t super;                        // 基类实例
        const bsp_spi_driver_ops_t *driver_ops; // 底层驱动操作表
    };

    /* ---------- 配置结构 ---------- */
    /**
     * @brief SPI 初始化配置
     */
    typedef struct
    {
        void *device_handle;                    // 平台设备句柄
        const bsp_spi_driver_ops_t *driver_ops; // 底层驱动表
        bsp_event_callback_t callback;          // 事件回调（可为 NULL）
        void *user_context;                     // 回调用户上下文
    } bsp_spi_config_t;

    /* ---------- 公共 API 声明 ---------- */

    /**
     * @brief 初始化 SPI 设备
     * @param me 设备对象指针
     * @param config 配置参数
     * @return 执行状态
     */
    bsp_status_t bsp_spi_init(bsp_spi_device_t *const me, const bsp_spi_config_t *const config);

    /**
     * @brief 将派生对象转为基类指针（向上转型）
     */
    bsp_spi_t *bsp_spi_as_base(bsp_spi_device_t *const me);

    /**
     * @brief 设置事件回调
     */
    bsp_status_t bsp_spi_set_callback(bsp_spi_t *const me, bsp_event_callback_t callback,
                                      void *user_context);

    /**
     * @brief 发送数据
     * @param me 基类指针
     * @param data 发送数据指针
     * @param size 数据大小（字节）
     * @param mode 传输模式（阻塞/中断/DMA）
     * @param timeout_ms 超时时间（毫秒）
     * @return 执行状态
     */
    bsp_status_t bsp_spi_transmit(bsp_spi_t *const me, const uint8_t *data, size_t size,
                                  bsp_transfer_mode_t mode, uint32_t timeout_ms);

    /**
     * @brief 接收数据
     * @param me 基类指针
     * @param data 接收缓冲区指针
     * @param size 数据大小（字节）
     * @param mode 传输模式
     * @param timeout_ms 超时时间
     * @return 执行状态
     */
    bsp_status_t bsp_spi_receive(bsp_spi_t *const me, uint8_t *data, size_t size,
                                 bsp_transfer_mode_t mode, uint32_t timeout_ms);

    /**
     * @brief 全双工交换
     * @param me 基类指针
     * @param transmit_data 发送数据指针
     * @param receive_data 接收缓冲区指针
     * @param size 数据大小（字节）
     * @param mode 传输模式
     * @param timeout_ms 超时时间
     * @return 执行状态
     */
    bsp_status_t bsp_spi_exchange(bsp_spi_t *const me, const uint8_t *transmit_data,
                                  uint8_t *receive_data, size_t size, bsp_transfer_mode_t mode,
                                  uint32_t timeout_ms);

    /**
     * @brief 中止当前事务
     * @param me 基类指针
     * @return 执行状态
     */
    bsp_status_t bsp_spi_abort(bsp_spi_t *const me);

    /**
     * @brief 查询总线是否忙
     * @param me 基类指针（const）
     * @param is_busy 输出是否忙
     * @return 执行状态
     */
    bsp_status_t bsp_spi_get_busy(const bsp_spi_t *const me, bool *is_busy);

    /**
     * @brief 事件通知函数（供底层驱动调用，向上层传递事件）
     * @param me 基类指针
     * @param event 事件类型
     * @param status 状态码
     * @param transferred_size 已传输的数据量（字节数）
     */
    void bsp_spi_notify(bsp_spi_t *const me, bsp_event_t event, bsp_status_t status,
                        size_t transferred_size);

#ifdef __cplusplus
}
#endif

#endif /* BSP_SPI_H */