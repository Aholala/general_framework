/**
 * @file bsp_rng.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 硬件随机数发生器（RNG）通用抽象层头文件
 * @note 定义硬件 RNG 的多态接口，支持获取单个 32 位随机数和填充缓冲区。
 * @version 1.0
 * @date 2026-07-27
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef BSP_RNG_H
#define BSP_RNG_H

#include "bsp_common.h" // 包含基础类型、状态码、设备基类等

#ifdef __cplusplus
    extern "C"
{
#endif

    /* 前向声明，避免循环依赖 */
    typedef struct bsp_rng bsp_rng_t;
    typedef struct bsp_rng_device bsp_rng_device_t;

    /* ---------- 高层虚表（面向应用层） ---------- */
    /**
     * @brief RNG 操作虚表，继承自 bsp_device_ops_t
     * @note 派生类必须保持 super 为第一成员
     */
    typedef struct
    {
        bsp_device_ops_t super;                                       // 父类虚表（含 deinit）
        bsp_status_t (*get_uint32)(bsp_rng_t *me, uint32_t *value);   // 获取 32 位随机数
        bsp_status_t (*fill)(bsp_rng_t *me, void *data, size_t size); // 填充缓冲区
    } bsp_rng_ops_t;

    /* ---------- 基类 ---------- */
    /**
     * @brief RNG 基类结构体
     * @note 目前基类仅为 bsp_device_t 的简单包装
     */
    struct bsp_rng
    {
        bsp_device_t super; // 设备基类
    };

    /* ---------- 底层驱动操作表（平台实现） ---------- */
    /**
     * @brief 平台相关 RNG 驱动操作表
     * @note 所有函数接收 device_handle 和必要参数
     *       驱动必须检测时钟错误、种子错误和超时，不得在失败时返回未经标记的伪随机值
     */
    typedef struct
    {
        bsp_status_t (*init)(void *handle);                          // 初始化硬件 RNG 模块
        bsp_status_t (*deinit)(void *handle);                        // 反初始化
        bsp_status_t (*get_uint32)(void *handle, uint32_t *value);   // 获取 32 位随机数
        bsp_status_t (*fill)(void *handle, void *data, size_t size); // 填充缓冲区
    } bsp_rng_driver_ops_t;

    /* ---------- 派生设备对象 ---------- */
    /**
     * @brief RNG 设备对象（派生类）
     */
    struct bsp_rng_device
    {
        bsp_rng_t super;                        // 基类实例
        const bsp_rng_driver_ops_t *driver_ops; // 底层驱动操作表
    };

    /* ---------- 配置结构 ---------- */
    /**
     * @brief RNG 初始化配置
     */
    typedef struct
    {
        void *device_handle;                    // 平台设备句柄（如 RNG_HandleTypeDef*）
        const bsp_rng_driver_ops_t *driver_ops; // 底层驱动表
    } bsp_rng_config_t;

    /* ---------- 公共 API 声明 ---------- */

    /**
     * @brief 初始化 RNG 设备
     * @param me 设备对象指针
     * @param config 配置参数
     * @return 执行状态
     */
    bsp_status_t bsp_rng_init(bsp_rng_device_t *me, const bsp_rng_config_t *config);

    /**
     * @brief 转为基类指针
     */
    bsp_rng_t *bsp_rng_as_base(bsp_rng_device_t *me);

    /**
     * @brief 获取单个 32 位随机数
     * @param me 基类指针
     * @param value 输出随机数值
     * @return 执行状态
     */
    bsp_status_t bsp_rng_get_uint32(bsp_rng_t *me, uint32_t *value);

    /**
     * @brief 填充缓冲区
     * @param me 基类指针
     * @param data 输出缓冲区指针
     * @param size 缓冲区大小（字节），必须大于 0
     * @return 执行状态
     */
    bsp_status_t bsp_rng_fill(bsp_rng_t *me, void *data, size_t size);

#ifdef __cplusplus
}
#endif

#endif