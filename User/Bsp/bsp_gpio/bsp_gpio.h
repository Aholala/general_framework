/**
 * @file bsp_gpio.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 通用数字 GPIO 抽象层头文件
 * @note 定义 GPIO 的读、写、翻转接口，不保存引脚号，由平台句柄封装。
 * @version 1.0
 * @date 2026-07-27
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef BSP_GPIO_H
#define BSP_GPIO_H

#include "bsp_common.h" // 包含基础类型、状态码、设备基类等

#ifdef __cplusplus
extern "C"
{
#endif

    /* 前向声明，避免循环依赖 */
    typedef struct bsp_gpio bsp_gpio_t;
    typedef struct bsp_gpio_device bsp_gpio_device_t;

    /* ---------- 高层虚表（面向应用层） ---------- */
    /**
     * @brief GPIO 操作虚表，继承自 bsp_device_ops_t
     * @note 派生类必须保持 super 为第一成员
     */
    typedef struct
    {
        bsp_device_ops_t super;                                          // 父类虚表（含 deinit）
        bsp_status_t (*read)(const bsp_gpio_t *const me, bool *is_high); // 读取电平
        bsp_status_t (*write)(bsp_gpio_t *const me, bool is_high);       // 写入电平
        bsp_status_t (*toggle)(bsp_gpio_t *const me);                    // 翻转电平
    } bsp_gpio_ops_t;

    /* ---------- 基类 ---------- */
    /**
     * @brief GPIO 基类结构体
     * @note 当前基类仅为 bsp_device_t 的包装，无额外字段
     */
    struct bsp_gpio
    {
        bsp_device_t super; // 设备基类
    };

    /* ---------- 底层驱动操作表（平台实现） ---------- */
    /**
     * @brief 平台相关 GPIO 驱动操作表
     * @note 所有函数接收 device_handle 作为第一个参数
     *       read 必须实现，write/toggle 可选（若未实现返回 UNSUPPORTED）
     */
    typedef struct
    {
        bsp_status_t (*init)(void *device_handle);                      // 初始化（可选）
        bsp_status_t (*deinit)(void *device_handle);                    // 反初始化（可选）
        bsp_status_t (*read)(const void *device_handle, bool *is_high); // 读取电平（必须）
        bsp_status_t (*write)(void *device_handle, bool is_high);       // 写入电平（可选）
        bsp_status_t (*toggle)(void *device_handle);                    // 翻转电平（可选）
    } bsp_gpio_driver_ops_t;

    /* ---------- 派生设备对象 ---------- */
    /**
     * @brief GPIO 设备对象（派生类）
     */
    struct bsp_gpio_device
    {
        bsp_gpio_t super;                        // 基类实例
        const bsp_gpio_driver_ops_t *driver_ops; // 底层驱动操作表
    };

    /* ---------- 配置结构 ---------- */
    /**
     * @brief GPIO 初始化配置
     */
    typedef struct
    {
        void *device_handle;                     // 平台设备句柄（封装端口/引脚）
        const bsp_gpio_driver_ops_t *driver_ops; // 底层驱动表
    } bsp_gpio_config_t;

    /* ---------- 公共 API 声明 ---------- */

    /**
     * @brief 初始化 GPIO 设备
     * @param me 设备对象指针
     * @param config 配置参数
     * @return 执行状态
     */
    bsp_status_t bsp_gpio_init(bsp_gpio_device_t *const me, const bsp_gpio_config_t *const config);

    /**
     * @brief 将派生对象转为基类指针（向上转型）
     */
    bsp_gpio_t *bsp_gpio_as_base(bsp_gpio_device_t *const me);

    /**
     * @brief 读取 GPIO 逻辑电平
     * @param me 基类指针（const）
     * @param is_high 输出电平（true=高，false=低）
     * @return 执行状态
     */
    bsp_status_t bsp_gpio_read(const bsp_gpio_t *const me, bool *is_high);

    /**
     * @brief 写入 GPIO 逻辑电平
     * @param me 基类指针
     * @param is_high 电平值（true=高，false=低）
     * @return 执行状态
     */
    bsp_status_t bsp_gpio_write(bsp_gpio_t *const me, bool is_high);

    /**
     * @brief 翻转 GPIO 逻辑电平
     * @param me 基类指针
     * @return 执行状态
     */
    bsp_status_t bsp_gpio_toggle(bsp_gpio_t *const me);

#ifdef __cplusplus
}
#endif

#endif