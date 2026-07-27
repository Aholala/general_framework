/**
 * @file bsp_dac.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief DAC 通用抽象层头文件
 * @note 定义 DAC 输出的多态接口，支持静态输出和 DMA 波形播放。
 * @version 1.0
 * @date 2026-07-27
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef BSP_DAC_H
#define BSP_DAC_H

#include "bsp_common.h" // 包含基础类型、状态码、设备基类等

#ifdef __cplusplus
    extern "C"
{
#endif

    /* 前向声明，避免循环依赖 */
    typedef struct bsp_dac bsp_dac_t;
    typedef struct bsp_dac_device bsp_dac_device_t;

    /* ---------- 高层虚表（面向应用层） ---------- */
    /**
     * @brief DAC 操作虚表，继承自 bsp_device_ops_t
     * @note 派生类必须保持 super 为第一成员
     */
    typedef struct
    {
        bsp_device_ops_t super;                                           // 父类虚表（含 deinit）
        bsp_status_t (*start)(bsp_dac_t *const me);                       // 启动 DAC 输出
        bsp_status_t (*stop)(bsp_dac_t *const me);                        // 停止 DAC 输出
        bsp_status_t (*set_raw)(bsp_dac_t *const me, uint32_t raw_value); // 设置原始值
        bsp_status_t (*get_raw)(const bsp_dac_t *const me, uint32_t *raw_value); // 获取原始值
        bsp_status_t (*start_dma)(bsp_dac_t *const me, const uint32_t *sample_buffer,
                                  size_t sample_count); // 启动 DMA 输出
        bsp_status_t (*stop_dma)(bsp_dac_t *const me);  // 停止 DMA 输出
    } bsp_dac_ops_t;

    /* ---------- 基类 ---------- */
    /**
     * @brief DAC 基类结构体
     */
    struct bsp_dac
    {
        bsp_device_t super;            // 设备基类
        bsp_event_callback_t callback; // 事件回调函数
        void *user_context;            // 回调用户上下文
        float reference_voltage_v;     // 参考电压（伏特）
        uint32_t maximum_raw_value;    // 最大原始值（由分辨率决定）
    };

    /* ---------- 底层驱动操作表（平台实现） ---------- */
    /**
     * @brief 平台相关 DAC 驱动操作表
     * @note 所有函数接收 device_handle 和通道号
     */
    typedef struct
    {
        bsp_status_t (*init)(void *device_handle, uint32_t channel);   // 初始化硬件（可选）
        bsp_status_t (*deinit)(void *device_handle, uint32_t channel); // 反初始化（可选）
        bsp_status_t (*start)(void *device_handle, uint32_t channel);  // 启动输出
        bsp_status_t (*stop)(void *device_handle, uint32_t channel);   // 停止输出
        bsp_status_t (*set_raw)(void *device_handle, uint32_t channel,
                                uint32_t raw_value); // 设置原始值
        bsp_status_t (*get_raw)(const void *device_handle, uint32_t channel,
                                uint32_t *raw_value); // 获取原始值
        bsp_status_t (*start_dma)(void *device_handle, uint32_t channel,
                                  const uint32_t *sample_buffer, size_t sample_count); // DMA 启动
        bsp_status_t (*stop_dma)(void *device_handle, uint32_t channel);               // DMA 停止
    } bsp_dac_driver_ops_t;

    /* ---------- 派生设备对象 ---------- */
    /**
     * @brief DAC 设备对象（派生类）
     */
    struct bsp_dac_device
    {
        bsp_dac_t super;                        // 基类实例
        const bsp_dac_driver_ops_t *driver_ops; // 底层驱动操作表
        uint32_t channel;                       // 逻辑通道号
    };

    /* ---------- 配置结构 ---------- */
    /**
     * @brief DAC 初始化配置
     */
    typedef struct
    {
        void *device_handle;                    // 平台设备句柄
        const bsp_dac_driver_ops_t *driver_ops; // 底层驱动表
        uint32_t channel;                       // 逻辑通道号
        uint8_t resolution_bits;                // 分辨率位数（1~31）
        float reference_voltage_v;              // 参考电压（>0 且有限）
        bsp_event_callback_t callback;          // 事件回调（可为 NULL）
        void *user_context;                     // 回调用户上下文
    } bsp_dac_config_t;

    /* ---------- 公共 API 声明 ---------- */

    /**
     * @brief 初始化 DAC 设备
     * @param me 设备对象指针
     * @param config 配置参数
     * @return 执行状态
     */
    bsp_status_t bsp_dac_init(bsp_dac_device_t *const me, const bsp_dac_config_t *const config);

    /**
     * @brief 将派生对象转为基类指针（向上转型）
     * @param me 派生对象指针
     * @return 基类指针
     */
    bsp_dac_t *bsp_dac_as_base(bsp_dac_device_t *const me);

    /**
     * @brief 设置事件回调
     * @param me 基类指针
     * @param callback 回调函数
     * @param user_context 用户上下文
     * @return 执行状态
     */
    bsp_status_t bsp_dac_set_callback(bsp_dac_t *const me, bsp_event_callback_t callback,
                                      void *user_context);

    /**
     * @brief 启动 DAC 输出
     * @param me 基类指针
     * @return 执行状态
     */
    bsp_status_t bsp_dac_start(bsp_dac_t *const me);

    /**
     * @brief 停止 DAC 输出
     * @param me 基类指针
     * @return 执行状态
     */
    bsp_status_t bsp_dac_stop(bsp_dac_t *const me);

    /**
     * @brief 设置原始值
     * @param me 基类指针
     * @param raw_value 原始码（0 ~ maximum_raw_value）
     * @return 执行状态
     */
    bsp_status_t bsp_dac_set_raw(bsp_dac_t *const me, uint32_t raw_value);

    /**
     * @brief 获取原始值
     * @param me 基类指针（const）
     * @param raw_value 输出原始码
     * @return 执行状态
     */
    bsp_status_t bsp_dac_get_raw(const bsp_dac_t *const me, uint32_t *raw_value);

    /**
     * @brief 设置归一化值（0.0 ~ 1.0）
     * @param me 基类指针
     * @param normalized_value 归一化值
     * @return 执行状态
     */
    bsp_status_t bsp_dac_set_normalized(bsp_dac_t *const me, float normalized_value);

    /**
     * @brief 设置电压值（0 ~ 参考电压）
     * @param me 基类指针
     * @param voltage_v 电压值（伏特）
     * @return 执行状态
     */
    bsp_status_t bsp_dac_set_voltage(bsp_dac_t *const me, float voltage_v);

    /**
     * @brief 启动 DMA 输出
     * @param me 基类指针
     * @param sample_buffer 样本缓冲区（调用者持有，传输期间必须保持有效）
     * @param sample_count 样本数量
     * @return 执行状态
     */
    bsp_status_t bsp_dac_start_dma(bsp_dac_t *const me, const uint32_t *sample_buffer,
                                   size_t sample_count);

    /**
     * @brief 停止 DMA 输出
     * @param me 基类指针
     * @return 执行状态
     */
    bsp_status_t bsp_dac_stop_dma(bsp_dac_t *const me);

    /**
     * @brief 事件通知函数（供底层驱动调用，向上层传递事件）
     * @param me 基类指针
     * @param event 事件类型
     * @param status 状态码
     * @param transferred_size 传输数量（如 DMA 计数值）
     */
    void bsp_dac_notify(bsp_dac_t *const me, bsp_event_t event, bsp_status_t status,
                        size_t transferred_size);

#ifdef __cplusplus
}
#endif

#endif