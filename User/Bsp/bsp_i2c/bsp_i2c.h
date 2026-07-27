/**
 * @file bsp_i2c.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief I2C 通用抽象层头文件
 * @note 定义 7 位地址 I2C 主机接口，支持直接收发、寄存器访问、设备探测、
 *       中止和忙状态查询，三种传输模式统一使用同一对象。
 * @version 1.0
 * @date 2026-07-25
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef BSP_I2C_H
#define BSP_I2C_H

#include "bsp_common.h" // 包含基础类型、状态码、设备基类、传输模式等

#ifdef __cplusplus
extern "C"
{
#endif

    /* 前向声明，避免循环依赖 */
    typedef struct bsp_i2c bsp_i2c_t;
    typedef struct bsp_i2c_device bsp_i2c_device_t;

    /* ---------- 枚举类型 ---------- */
    /**
     * @brief I2C 寄存器地址大小
     * @note 用于内存读写操作，指定寄存器地址是 8 位还是 16 位
     */
    typedef enum
    {
        BSP_I2C_MEMORY_ADDRESS_8_BIT = 1, // 8 位寄存器地址
        BSP_I2C_MEMORY_ADDRESS_16_BIT = 2 // 16 位寄存器地址
    } bsp_i2c_memory_address_size_t;

    /* ---------- 高层虚表（面向应用层） ---------- */
    /**
     * @brief I2C 操作虚表，继承自 bsp_device_ops_t
     * @note 派生类必须保持 super 为第一成员
     *       所有地址参数均为 7 位地址（0~0x7F），平台驱动负责左移
     */
    typedef struct
    {
        bsp_device_ops_t super; // 父类虚表（含 deinit）

        /**
         * @brief 发送数据到从设备
         */
        bsp_status_t (*transmit)(bsp_i2c_t *const, uint16_t, const uint8_t *, size_t,
                                 bsp_transfer_mode_t, uint32_t);

        /**
         * @brief 从从设备接收数据
         */
        bsp_status_t (*receive)(bsp_i2c_t *const, uint16_t, uint8_t *, size_t, bsp_transfer_mode_t,
                                uint32_t);

        /**
         * @brief 向寄存器地址写入数据（8/16 位寄存器地址）
         */
        bsp_status_t (*memory_write)(bsp_i2c_t *const, uint16_t, uint16_t,
                                     bsp_i2c_memory_address_size_t, const uint8_t *, size_t,
                                     bsp_transfer_mode_t, uint32_t);

        /**
         * @brief 从寄存器地址读取数据（8/16 位寄存器地址）
         */
        bsp_status_t (*memory_read)(bsp_i2c_t *const, uint16_t, uint16_t,
                                    bsp_i2c_memory_address_size_t, uint8_t *, size_t,
                                    bsp_transfer_mode_t, uint32_t);

        /**
         * @brief 检查设备是否就绪（发送 START + 地址，检测 ACK）
         */
        bsp_status_t (*is_device_ready)(bsp_i2c_t *const, uint16_t, uint32_t, uint32_t);

        /**
         * @brief 中止当前事务（生成 STOP 或复位）
         */
        bsp_status_t (*abort)(bsp_i2c_t *const, uint16_t);

        /**
         * @brief 查询总线是否忙（是否有传输进行中）
         */
        bsp_status_t (*get_busy)(const bsp_i2c_t *const, bool *);

    } bsp_i2c_ops_t;

    /* ---------- 基类 ---------- */
    /**
     * @brief I2C 基类结构体
     */
    struct bsp_i2c
    {
        bsp_device_t super;            // 设备基类
        bsp_event_callback_t callback; // 事件回调函数
        void *user_context;            // 回调用户上下文
    };

    /* ---------- 底层驱动操作表（平台实现） ---------- */
    /**
     * @brief 平台相关 I2C 驱动操作表
     * @note transmit/receive 必须实现，其余为可选
     *       所有地址参数均为 7 位地址（0~0x7F）
     */
    typedef struct
    {
        bsp_status_t (*init)(void *);   // 初始化硬件（可选）
        bsp_status_t (*deinit)(void *); // 反初始化（可选）
        bsp_status_t (*transmit)(void *, uint16_t, const uint8_t *, size_t, bsp_transfer_mode_t,
                                 uint32_t); // 发送（必须）
        bsp_status_t (*receive)(void *, uint16_t, uint8_t *, size_t, bsp_transfer_mode_t,
                                uint32_t); // 接收（必须）
        bsp_status_t (*memory_write)(void *, uint16_t, uint16_t, bsp_i2c_memory_address_size_t,
                                     const uint8_t *, size_t, bsp_transfer_mode_t, uint32_t);
        bsp_status_t (*memory_read)(void *, uint16_t, uint16_t, bsp_i2c_memory_address_size_t,
                                    uint8_t *, size_t, bsp_transfer_mode_t, uint32_t);
        bsp_status_t (*is_device_ready)(void *, uint16_t, uint32_t, uint32_t);
        bsp_status_t (*abort)(void *, uint16_t);
        bsp_status_t (*get_busy)(const void *, bool *);
    } bsp_i2c_driver_ops_t;

    /* ---------- 派生设备对象 ---------- */
    /**
     * @brief I2C 设备对象（派生类）
     */
    struct bsp_i2c_device
    {
        bsp_i2c_t super;                        // 基类实例
        const bsp_i2c_driver_ops_t *driver_ops; // 底层驱动操作表
    };

    /* ---------- 配置结构 ---------- */
    /**
     * @brief I2C 初始化配置
     */
    typedef struct
    {
        void *device_handle;                    // 平台设备句柄
        const bsp_i2c_driver_ops_t *driver_ops; // 底层驱动表
        bsp_event_callback_t callback;          // 事件回调（可为 NULL）
        void *user_context;                     // 回调用户上下文
    } bsp_i2c_config_t;

    /* ---------- 公共 API 声明 ---------- */

    /**
     * @brief 初始化 I2C 设备
     * @param me 设备对象指针
     * @param config 配置参数
     * @return 执行状态
     */
    bsp_status_t bsp_i2c_init(bsp_i2c_device_t *const me, const bsp_i2c_config_t *const config);

    /**
     * @brief 将派生对象转为基类指针（向上转型）
     */
    bsp_i2c_t *bsp_i2c_as_base(bsp_i2c_device_t *const me);

    /**
     * @brief 设置事件回调
     */
    bsp_status_t bsp_i2c_set_callback(bsp_i2c_t *const me, bsp_event_callback_t callback,
                                      void *user_context);

    /**
     * @brief 发送数据到从设备
     * @param me 基类指针
     * @param address_7bit 7 位从设备地址（0~0x7F）
     * @param data 发送数据指针
     * @param size 数据大小（字节）
     * @param mode 传输模式（阻塞/中断/DMA）
     * @param timeout_ms 超时时间（毫秒）
     * @return 执行状态
     */
    bsp_status_t bsp_i2c_transmit(bsp_i2c_t *const me, uint16_t address_7bit, const uint8_t *data,
                                  size_t size, bsp_transfer_mode_t mode, uint32_t timeout_ms);

    /**
     * @brief 从从设备接收数据
     * @param me 基类指针
     * @param address_7bit 7 位从设备地址（0~0x7F）
     * @param data 接收缓冲区指针
     * @param size 数据大小（字节）
     * @param mode 传输模式（阻塞/中断/DMA）
     * @param timeout_ms 超时时间（毫秒）
     * @return 执行状态
     */
    bsp_status_t bsp_i2c_receive(bsp_i2c_t *const me, uint16_t address_7bit, uint8_t *data,
                                 size_t size, bsp_transfer_mode_t mode, uint32_t timeout_ms);

    /**
     * @brief 向寄存器地址写入数据
     * @param me 基类指针
     * @param address_7bit 7 位从设备地址（0~0x7F）
     * @param memory_address 寄存器地址（8 或 16 位）
     * @param address_size 寄存器地址大小（8/16 位）
     * @param data 数据指针
     * @param size 数据大小（字节）
     * @param mode 传输模式
     * @param timeout_ms 超时时间（毫秒）
     * @return 执行状态
     */
    bsp_status_t bsp_i2c_memory_write(bsp_i2c_t *const me, uint16_t address_7bit,
                                      uint16_t memory_address,
                                      bsp_i2c_memory_address_size_t address_size,
                                      const uint8_t *data, size_t size, bsp_transfer_mode_t mode,
                                      uint32_t timeout_ms);

    /**
     * @brief 从寄存器地址读取数据
     * @param me 基类指针
     * @param address_7bit 7 位从设备地址（0~0x7F）
     * @param memory_address 寄存器地址（8 或 16 位）
     * @param address_size 寄存器地址大小（8/16 位）
     * @param data 接收缓冲区指针
     * @param size 数据大小（字节）
     * @param mode 传输模式
     * @param timeout_ms 超时时间（毫秒）
     * @return 执行状态
     */
    bsp_status_t bsp_i2c_memory_read(bsp_i2c_t *const me, uint16_t address_7bit,
                                     uint16_t memory_address,
                                     bsp_i2c_memory_address_size_t address_size, uint8_t *data,
                                     size_t size, bsp_transfer_mode_t mode, uint32_t timeout_ms);

    /**
     * @brief 检查设备是否就绪
     * @param me 基类指针
     * @param address_7bit 7 位从设备地址（0~0x7F）
     * @param trial_count 重试次数
     * @param timeout_ms 每次尝试的超时时间（毫秒）
     * @return 执行状态
     */
    bsp_status_t bsp_i2c_is_device_ready(bsp_i2c_t *const me, uint16_t address_7bit,
                                         uint32_t trial_count, uint32_t timeout_ms);

    /**
     * @brief 中止当前事务
     * @param me 基类指针
     * @param address_7bit 7 位从设备地址（0~0x7F）
     * @return 执行状态
     */
    bsp_status_t bsp_i2c_abort(bsp_i2c_t *const me, uint16_t address_7bit);

    /**
     * @brief 查询总线是否忙
     * @param me 基类指针（const）
     * @param is_busy 输出是否忙
     * @return 执行状态
     */
    bsp_status_t bsp_i2c_get_busy(const bsp_i2c_t *const me, bool *is_busy);

    /**
     * @brief 事件通知函数（供底层驱动调用，向上层传递事件）
     * @param me 基类指针
     * @param event 事件类型
     * @param status 状态码
     * @param transferred_size 已传输的数据量（字节数）
     */
    void bsp_i2c_notify(bsp_i2c_t *const me, bsp_event_t event, bsp_status_t status,
                        size_t transferred_size);

#ifdef __cplusplus
}
#endif

#endif