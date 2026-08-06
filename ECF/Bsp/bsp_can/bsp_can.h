/**
 * @file bsp_can.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief CAN 通用抽象层头文件
 * @version 1.0
 * @date 2026-07-22
 * @copyright Copyright (c) 2026
 */

#ifndef BSP_CAN_H
#define BSP_CAN_H

#include "bsp_common.h" // 包含基础类型、状态码、设备基类等

#ifdef __cplusplus
extern "C"
{
#endif

    /* 前向声明，避免循环依赖 */
    typedef struct bsp_can bsp_can_t;
    typedef struct bsp_can_device bsp_can_device_t;

    /* ---------- 枚举类型 ---------- */
    /**
     * @brief CAN 标识符类型
     */
    typedef enum
    {
        BSP_CAN_ID_STANDARD = 0, // 11 位标准 ID
        BSP_CAN_ID_EXTENDED      // 29 位扩展 ID
    } bsp_can_id_type_t;

    /**
     * @brief CAN 帧类型
     */
    typedef enum
    {
        BSP_CAN_FRAME_DATA = 0, // 数据帧
        BSP_CAN_FRAME_REMOTE    // 远程帧（无数据）
    } bsp_can_frame_type_t;

    /**
     * @brief CAN 接收 FIFO 选择
     */
    typedef enum
    {
        BSP_CAN_RX_FIFO_0 = 0, // FIFO 0
        BSP_CAN_RX_FIFO_1      // FIFO 1
    } bsp_can_receive_fifo_t;

    /* ---------- 数据模型 ---------- */
    /**
     * @brief CAN 帧结构
     * @note data_length 范围为 0~8，数据域长度为 8 字节
     */
    typedef struct
    {
        uint32_t identifier;             // 11 或 29 位 ID
        bsp_can_id_type_t id_type;       // 标准/扩展
        bsp_can_frame_type_t frame_type; // 数据/远程
        uint8_t data_length;             // 有效数据长度 (0-8)
        uint8_t data[8];                 // 数据负载
    } bsp_can_frame_t;

    /**
     * @brief 硬件过滤器配置
     * @note filter_index 由平台端解释，例如 STM32 的过滤器号
     */
    typedef struct
    {
        uint32_t identifier;                 // 匹配的 ID
        uint32_t mask;                       // 掩码（1 表示关心该位）
        bsp_can_id_type_t id_type;           // 标准/扩展
        bsp_can_receive_fifo_t receive_fifo; // 匹配帧送入的 FIFO
        uint32_t filter_index;               // 平台相关过滤器槽位
    } bsp_can_filter_t;

    /* ---------- 高层虚表（面向应用层） ---------- */
    /**
     * @brief CAN 操作虚表，继承自 bsp_device_ops_t
     */
    typedef struct
    {
        bsp_device_ops_t super;                  // 父类虚表（含 deinit）
        bsp_status_t (*start)(bsp_can_t *const); // 启动 CAN
        bsp_status_t (*stop)(bsp_can_t *const);  // 停止 CAN
        bsp_status_t (*configure_filter)(bsp_can_t *const, const bsp_can_filter_t *); // 配置过滤器
        bsp_status_t (*transmit)(bsp_can_t *const, const bsp_can_frame_t *,
                                 uint32_t); // 发送帧（阻塞）
        bsp_status_t (*receive)(bsp_can_t *const, bsp_can_receive_fifo_t,
                                bsp_can_frame_t *);                            // 接收帧
        bsp_status_t (*get_tx_free_level)(const bsp_can_t *const, uint32_t *); // 查询发送邮箱空闲数
    } bsp_can_ops_t;

    /* ---------- 基类 ---------- */
    /**
     * @brief CAN 基类结构体，包含设备基类和回调信息
     */
    struct bsp_can
    {
        bsp_device_t super;            // 设备基类
        bsp_event_callback_t callback; // 事件回调函数
        void *user_context;            // 回调用户上下文
    };

    /* ---------- 底层驱动操作表（平台实现） ---------- */
    /**
     * @brief 平台相关驱动操作表
     * @note 所有函数接收 device_handle 和必要参数，不依赖具体硬件
     */
    typedef struct
    {
        bsp_status_t (*init)(void *);                                        // 初始化硬件（可选）
        bsp_status_t (*deinit)(void *);                                      // 反初始化（可选）
        bsp_status_t (*start)(void *);                                       // 启动通信
        bsp_status_t (*stop)(void *);                                        // 停止通信
        bsp_status_t (*configure_filter)(void *, const bsp_can_filter_t *);  // 配置过滤器
        bsp_status_t (*transmit)(void *, const bsp_can_frame_t *, uint32_t); // 发送
        bsp_status_t (*receive)(void *, bsp_can_receive_fifo_t, bsp_can_frame_t *); // 接收
        bsp_status_t (*get_tx_free_level)(const void *, uint32_t *); // 获取空闲邮箱（可选）
    } bsp_can_driver_ops_t;

    /* ---------- 派生设备对象 ---------- */
    /**
     * @brief CAN 设备对象（派生类）
     */
    struct bsp_can_device
    {
        bsp_can_t super;                        // 基类实例
        const bsp_can_driver_ops_t *driver_ops; // 底层驱动操作表
    };

    /* ---------- 配置结构 ---------- */
    /**
     * @brief CAN 初始化配置
     */
    typedef struct
    {
        void *device_handle;                    // 平台设备句柄（如 FDCAN_HandleTypeDef*）
        const bsp_can_driver_ops_t *driver_ops; // 底层驱动表
        bsp_event_callback_t callback;          // 事件回调（可为 NULL）
        void *user_context;                     // 回调用户上下文
    } bsp_can_config_t;

    /* ---------- 公共 API 声明 ---------- */

    /**
     * @brief 初始化 CAN 设备
     * @param me 设备对象指针
     * @param config 配置参数
     * @return 执行状态
     */
    bsp_status_t bsp_can_init(bsp_can_device_t *const me, const bsp_can_config_t *const config);

    /**
     * @brief 转为基类指针
     */
    bsp_can_t *bsp_can_as_base(bsp_can_device_t *const me);

    /**
     * @brief 设置事件回调
     */
    bsp_status_t bsp_can_set_callback(bsp_can_t *const me, bsp_event_callback_t callback,
                                      void *user_context);

    /**
     * @brief 启动 CAN
     */
    bsp_status_t bsp_can_start(bsp_can_t *const me);

    /**
     * @brief 停止 CAN
     */
    bsp_status_t bsp_can_stop(bsp_can_t *const me);

    /**
     * @brief 配置硬件过滤器
     */
    bsp_status_t bsp_can_configure_filter(bsp_can_t *const me, const bsp_can_filter_t *filter);

    /**
     * @brief 发送帧（阻塞）
     */
    bsp_status_t bsp_can_transmit(bsp_can_t *const me, const bsp_can_frame_t *frame,
                                  uint32_t timeout_ms);

    /**
     * @brief 接收帧（从指定 FIFO 读取一帧）
     */
    bsp_status_t bsp_can_receive(bsp_can_t *const me, bsp_can_receive_fifo_t receive_fifo,
                                 bsp_can_frame_t *frame);

    /**
     * @brief 获取发送邮箱空闲数（可选）
     */
    bsp_status_t bsp_can_get_transmit_free_level(const bsp_can_t *const me, uint32_t *free_level);

    /**
     * @brief 事件通知（由底层驱动在中断中调用）
     */
    void bsp_can_notify(bsp_can_t *const me, bsp_event_t event, bsp_status_t status,
                        size_t transferred_size);

#ifdef __cplusplus
}
#endif

#endif