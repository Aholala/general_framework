/**
 * @file bsp_encoder.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 增量编码器通用抽象层头文件
 * @note 定义编码器计数、方向读取及增量计算接口。
 * @version 1.0
 * @date 2026-07-27
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef BSP_ENCODER_H
#define BSP_ENCODER_H

#include "bsp_common.h" // 包含基础类型、状态码、设备基类等

#ifdef __cplusplus
extern "C"
{
#endif

    /* 前向声明，避免循环依赖 */
    typedef struct bsp_encoder bsp_encoder_t;
    typedef struct bsp_encoder_device bsp_encoder_device_t;

    /* ---------- 方向枚举 ---------- */
    /**
     * @brief 编码器旋转方向
     * @note STOPPED 表示计数器未发生有效变化，由硬件或驱动判定
     */
    typedef enum
    {
        BSP_ENCODER_DIRECTION_STOPPED = 0, // 停止（无脉冲变化）
        BSP_ENCODER_DIRECTION_FORWARD,     // 正向（计数值递增）
        BSP_ENCODER_DIRECTION_REVERSE      // 反向（计数值递减）
    } bsp_encoder_direction_t;

    /* ---------- 高层虚表（面向应用层） ---------- */
    /**
     * @brief 编码器操作虚表，继承自 bsp_device_ops_t
     * @note 派生类必须保持 super 为第一成员
     */
    typedef struct
    {
        bsp_device_ops_t super;                                            // 父类虚表（含 deinit）
        bsp_status_t (*start)(bsp_encoder_t *const me);                    // 启动计数
        bsp_status_t (*stop)(bsp_encoder_t *const me);                     // 停止计数
        bsp_status_t (*set_count)(bsp_encoder_t *const me, int32_t count); // 设置计数值
        bsp_status_t (*get_count)(const bsp_encoder_t *const me, int32_t *count); // 获取计数值
        bsp_status_t (*get_direction)(const bsp_encoder_t *const me,
                                      bsp_encoder_direction_t *direction); // 获取方向
    } bsp_encoder_ops_t;

    /* ---------- 基类 ---------- */
    /**
     * @brief 编码器基类结构体
     * @note 包含用于 delta 计算的历史值和模数配置
     */
    struct bsp_encoder
    {
        bsp_device_t super;       // 设备基类
        int32_t previous_count;   // 上一次读取的计数值（用于增量计算）
        uint32_t counter_modulus; // 计数模数（0 表示无回绕，即无限范围）
    };

    /* ---------- 底层驱动操作表（平台实现） ---------- */
    /**
     * @brief 平台相关编码器驱动操作表
     * @note 所有函数接收 device_handle 作为第一个参数
     */
    typedef struct
    {
        bsp_status_t (*init)(void *device_handle);                            // 初始化（可选）
        bsp_status_t (*deinit)(void *device_handle);                          // 反初始化（可选）
        bsp_status_t (*start)(void *device_handle);                           // 启动计数
        bsp_status_t (*stop)(void *device_handle);                            // 停止计数
        bsp_status_t (*set_count)(void *device_handle, int32_t count);        // 设置计数值
        bsp_status_t (*get_count)(const void *device_handle, int32_t *count); // 获取计数值
        bsp_status_t (*get_direction)(const void *device_handle,
                                      bsp_encoder_direction_t *direction); // 获取方向
    } bsp_encoder_driver_ops_t;

    /* ---------- 派生设备对象 ---------- */
    /**
     * @brief 编码器设备对象（派生类）
     */
    struct bsp_encoder_device
    {
        bsp_encoder_t super;                        // 基类实例
        const bsp_encoder_driver_ops_t *driver_ops; // 底层驱动操作表
    };

    /* ---------- 配置结构 ---------- */
    /**
     * @brief 编码器初始化配置
     */
    typedef struct
    {
        void *device_handle;                        // 平台设备句柄
        const bsp_encoder_driver_ops_t *driver_ops; // 底层驱动表
        uint32_t counter_modulus;                   // 计数模数（0 表示无回绕，有效值 >=2）
    } bsp_encoder_config_t;

    /* ---------- 公共 API 声明 ---------- */

    /**
     * @brief 初始化编码器设备
     * @param me 设备对象指针
     * @param config 配置参数
     * @return 执行状态
     */
    bsp_status_t bsp_encoder_init(bsp_encoder_device_t *const me,
                                  const bsp_encoder_config_t *const config);

    /**
     * @brief 将派生对象转为基类指针（向上转型）
     */
    bsp_encoder_t *bsp_encoder_as_base(bsp_encoder_device_t *const me);

    /**
     * @brief 启动编码器计数
     */
    bsp_status_t bsp_encoder_start(bsp_encoder_t *const me);

    /**
     * @brief 停止编码器计数
     */
    bsp_status_t bsp_encoder_stop(bsp_encoder_t *const me);

    /**
     * @brief 复位计数值为 0（同时重置内部历史值）
     */
    bsp_status_t bsp_encoder_reset(bsp_encoder_t *const me);

    /**
     * @brief 设置当前计数值
     */
    bsp_status_t bsp_encoder_set_count(bsp_encoder_t *const me, int32_t count);

    /**
     * @brief 获取当前计数值
     */
    bsp_status_t bsp_encoder_get_count(const bsp_encoder_t *const me, int32_t *count);

    /**
     * @brief 获取自上次调用以来的计数值增量（带模数回绕处理）
     * @param me 基类指针（非常量，会修改内部 previous_count）
     * @param count_delta 输出增量（有符号）
     * @return 执行状态
     */
    bsp_status_t bsp_encoder_get_delta(bsp_encoder_t *const me, int32_t *count_delta);

    /**
     * @brief 获取旋转方向
     */
    bsp_status_t bsp_encoder_get_direction(const bsp_encoder_t *const me,
                                           bsp_encoder_direction_t *direction);

#ifdef __cplusplus
}
#endif

#endif