/**
 * @file bsp_watchdog.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 硬件看门狗通用抽象层头文件
 * @version 1.0
 * @date 2026-07-27
 * @copyright Copyright (c) 2026
 *
 * @note 硬件看门狗抽象，提供刷新、实际超时时间查询和看门狗复位来源检测。
 *       通用层不决定独立看门狗或窗口看门狗，也不配置具体寄存器。
 */

#ifndef BSP_WATCHDOG_H
#define BSP_WATCHDOG_H

#include "bsp_common.h" // 包含基础类型、状态码、设备基类等

#ifdef __cplusplus
extern "C"
{
#endif

    /* 前向声明，避免循环依赖 */
    typedef struct bsp_watchdog bsp_watchdog_t;
    typedef struct bsp_watchdog_device bsp_watchdog_device_t;

    /* ---------- 高层虚表（面向应用层） ---------- */
    /**
     * @brief 看门狗操作虚表，继承自 bsp_device_ops_t
     * @note 派生类必须保持 super 为第一成员
     *       refresh 必须实现
     *       get_timeout_ms、get_reset_detected 为可选
     */
    typedef struct
    {
        bsp_device_ops_t super; // 父类虚表（含 deinit）

        /**
         * @brief 刷新看门狗（喂狗）
         * @param me 基类指针
         * @return 执行状态
         * @note 必须在允许的时间窗口内调用
         */
        bsp_status_t (*refresh)(bsp_watchdog_t *const me);

        /**
         * @brief 获取看门狗实际超时时间
         * @param me 基类指针（const）
         * @param timeout_ms 输出超时时间（毫秒）
         * @return 执行状态
         * @note 超时时间由低速时钟和分频决定，存在器差和温漂
         */
        bsp_status_t (*get_timeout_ms)(const bsp_watchdog_t *const me, uint32_t *timeout_ms);

        /**
         * @brief 检测上次复位是否由看门狗导致
         * @param me 基类指针（const）
         * @param reset_detected 输出是否由看门狗复位
         * @return 执行状态
         * @note 应在系统启动早期调用，然后清除硬件复位标志
         */
        bsp_status_t (*get_reset_detected)(const bsp_watchdog_t *const me, bool *reset_detected);

    } bsp_watchdog_ops_t;

    /* ---------- 基类 ---------- */
    /**
     * @brief 看门狗基类结构体
     * @note 目前基类仅为 bsp_device_t 的简单包装
     */
    struct bsp_watchdog
    {
        bsp_device_t super; // 设备基类
    };

    /* ---------- 底层驱动操作表（平台实现） ---------- */
    /**
     * @brief 平台相关看门狗驱动操作表
     * @note refresh 必须实现，其余为可选
     *       某些 MCU 看门狗启动后无法停止，deinit 可以返回 UNSUPPORTED
     */
    typedef struct
    {
        bsp_status_t (*init)(void *device_handle);    // 初始化（可选）
        bsp_status_t (*deinit)(void *device_handle);  // 反初始化（可选）
        bsp_status_t (*refresh)(void *device_handle); // 刷新（必须）
        bsp_status_t (*get_timeout_ms)(const void *device_handle,
                                       uint32_t *timeout_ms); // 获取超时（可选）
        bsp_status_t (*get_reset_detected)(const void *device_handle,
                                           bool *reset_detected); // 复位检测（可选）
    } bsp_watchdog_driver_ops_t;

    /* ---------- 派生设备对象 ---------- */
    /**
     * @brief 看门狗设备对象（派生类）
     */
    struct bsp_watchdog_device
    {
        bsp_watchdog_t super;                        // 基类实例
        const bsp_watchdog_driver_ops_t *driver_ops; // 底层驱动操作表
    };

    /* ---------- 配置结构 ---------- */
    /**
     * @brief 看门狗初始化配置
     */
    typedef struct
    {
        void *device_handle;                         // 平台设备句柄
        const bsp_watchdog_driver_ops_t *driver_ops; // 底层驱动表
    } bsp_watchdog_config_t;

    /* ---------- 公共 API ---------- */

    /**
     * @brief 初始化看门狗设备
     * @param me 设备对象指针
     * @param config 配置参数
     * @return 执行状态
     */
    bsp_status_t bsp_watchdog_init(bsp_watchdog_device_t *const me,
                                   const bsp_watchdog_config_t *const config);

    /**
     * @brief 将派生对象转为基类指针（向上转型）
     */
    bsp_watchdog_t *bsp_watchdog_as_base(bsp_watchdog_device_t *const me);

    /**
     * @brief 刷新看门狗（喂狗）
     * @param me 基类指针
     * @return 执行状态
     */
    bsp_status_t bsp_watchdog_refresh(bsp_watchdog_t *const me);

    /**
     * @brief 获取看门狗实际超时时间
     * @param me 基类指针（const）
     * @param timeout_ms 输出超时时间（毫秒）
     * @return 执行状态
     */
    bsp_status_t bsp_watchdog_get_timeout_ms(const bsp_watchdog_t *const me, uint32_t *timeout_ms);

    /**
     * @brief 检测上次复位是否由看门狗导致
     * @param me 基类指针（const）
     * @param reset_detected 输出是否由看门狗复位
     * @return 执行状态
     */
    bsp_status_t bsp_watchdog_get_reset_detected(const bsp_watchdog_t *const me,
                                                 bool *reset_detected);

#ifdef __cplusplus
}
#endif

#endif