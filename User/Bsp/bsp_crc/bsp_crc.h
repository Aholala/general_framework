/**
 * @file bsp_crc.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief CRC 硬件抽象层头文件
 * @note 定义 CRC 计算的多态接口，支持硬件加速的 CRC 计算
 * @version 1.0
 * @date 2026-07-27
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef BSP_CRC_H
#define BSP_CRC_H

#include "bsp_common.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /* 前向声明，避免循环依赖 */
    typedef struct bsp_crc bsp_crc_t;
    typedef struct bsp_crc_device bsp_crc_device_t;

    /* ---------- 高层虚表（面向应用层） ---------- */
    /**
     * @brief CRC 操作虚表，继承自 bsp_device_ops_t
     * @note 派生类必须保持 super 为第一成员
     */
    typedef struct
    {
        bsp_device_ops_t super; // 父类虚表（含 deinit）
        bsp_status_t (*calculate)(bsp_crc_t *me, const void *data, size_t size,
                                  uint32_t initial_value, uint32_t *result); // CRC 计算虚函数
    } bsp_crc_ops_t;

    /* ---------- 基类 ---------- */
    /**
     * @brief CRC 基类结构体
     * @note 目前基类仅为 bsp_device_t 的简单包装，未来可扩展配置字段
     */
    struct bsp_crc
    {
        bsp_device_t super; // 设备基类
    };

    /* ---------- 底层驱动操作表（平台实现） ---------- */
    /**
     * @brief 平台相关 CRC 驱动操作表
     * @note 所有函数接收 device_handle 和必要参数
     */
    typedef struct
    {
        bsp_status_t (*init)(void *handle);   // 初始化硬件 CRC 模块
        bsp_status_t (*deinit)(void *handle); // 反初始化
        bsp_status_t (*calculate)(void *handle, const void *data, size_t size,
                                  uint32_t initial_value, uint32_t *result); // 计算 CRC
    } bsp_crc_driver_ops_t;

    /* ---------- 派生设备对象 ---------- */
    /**
     * @brief CRC 设备对象（派生类）
     */
    struct bsp_crc_device
    {
        bsp_crc_t super;                        // 基类实例
        const bsp_crc_driver_ops_t *driver_ops; // 底层驱动操作表
    };

    /* ---------- 配置结构 ---------- */
    /**
     * @brief CRC 初始化配置
     */
    typedef struct
    {
        void *device_handle;                    // 平台设备句柄（如 CRC_HandleTypeDef*）
        const bsp_crc_driver_ops_t *driver_ops; // 底层驱动表
    } bsp_crc_config_t;

    /* ---------- 公共 API 声明 ---------- */

    /**
     * @brief 初始化 CRC 设备
     * @param me 设备对象指针
     * @param config 配置参数
     * @return 执行状态
     */
    bsp_status_t bsp_crc_init(bsp_crc_device_t *me, const bsp_crc_config_t *config);

    /**
     * @brief 转为基类指针
     */
    bsp_crc_t *bsp_crc_as_base(bsp_crc_device_t *me);

    /**
     * @brief 计算 CRC
     * @param me 基类指针
     * @param data 数据指针
     * @param size 数据大小（字节）
     * @param initial_value 初始值（可用于链式校验）
     * @param result 输出计算结果
     * @return 执行状态
     */
    bsp_status_t bsp_crc_calculate(bsp_crc_t *me, const void *data, size_t size,
                                   uint32_t initial_value, uint32_t *result);

#ifdef __cplusplus
}
#endif

#endif /* BSP_CRC_H */