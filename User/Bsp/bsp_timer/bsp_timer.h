/**
 * @file bsp_timer.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 通用基本定时器抽象层头文件
 * @version 1.0
 * @date 2026-07-27
 * @copyright Copyright (c) 2026
 *
 * @note 定义基本定时器的启动、停止、计数器、周期、频率和到期通知接口。
 *       预分频、时钟树、自动重装模式和 IRQ 优先级由平台端配置。
 */

#ifndef BSP_TIMER_H
#define BSP_TIMER_H

#include "bsp_common.h" // 包含基础类型、状态码、设备基类等

#ifdef __cplusplus
extern "C"
{
#endif

    /* 前向声明，避免循环依赖 */
    typedef struct bsp_timer bsp_timer_t;
    typedef struct bsp_timer_device bsp_timer_device_t;

    /**
     * @brief 定时器到期回调函数类型
     * @param me 触发到期的定时器对象指针
     * @param user_context 注册时传入的用户上下文
     * @note 回调在 ISR 上下文中执行，必须快速返回，不可阻塞
     */
    typedef void (*bsp_timer_callback_t)(bsp_timer_t *const me, void *user_context);

    /* ---------- 高层虚表（面向应用层） ---------- */
    /**
     * @brief 定时器操作虚表，继承自 bsp_device_ops_t
     * @note 派生类必须保持 super 为第一成员
     *       所有函数均为必须实现（由 bsp_timer_init 校验）
     */
    typedef struct
    {
        bsp_device_ops_t super;                       // 父类虚表（含 deinit）
        bsp_status_t (*start)(bsp_timer_t *const me); // 启动定时器
        bsp_status_t (*stop)(bsp_timer_t *const me);  // 停止定时器
        bsp_status_t (*set_counter)(bsp_timer_t *const me, uint32_t counter_ticks); // 设置计数器
        bsp_status_t (*get_counter)(const bsp_timer_t *const me,
                                    uint32_t *counter_ticks);                     // 获取计数器
        bsp_status_t (*set_period)(bsp_timer_t *const me, uint32_t period_ticks); // 设置周期
        bsp_status_t (*get_period)(const bsp_timer_t *const me, uint32_t *period_ticks); // 获取周期
        bsp_status_t (*get_frequency)(const bsp_timer_t *const me,
                                      uint32_t *frequency_hz); // 获取频率
    } bsp_timer_ops_t;

    /* ---------- 基类 ---------- */
    /**
     * @brief 定时器基类结构体
     */
    struct bsp_timer
    {
        bsp_device_t super;            // 设备基类
        bsp_timer_callback_t callback; // 到期回调函数
        void *user_context;            // 回调用户上下文
    };

    /* ---------- 底层驱动操作表（平台实现） ---------- */
    /**
     * @brief 平台相关定时器驱动操作表
     * @note 所有函数均为必须实现（由 bsp_timer_init 校验）
     *       预分频、时钟源等由平台端在 init 或硬件配置中处理
     */
    typedef struct
    {
        bsp_status_t (*init)(void *device_handle);   // 初始化（可选）
        bsp_status_t (*deinit)(void *device_handle); // 反初始化（可选）
        bsp_status_t (*start)(void *device_handle);  // 启动计数
        bsp_status_t (*stop)(void *device_handle);   // 停止计数
        bsp_status_t (*set_counter)(void *device_handle, uint32_t counter_ticks); // 设置计数器
        bsp_status_t (*get_counter)(const void *device_handle,
                                    uint32_t *counter_ticks);                          // 获取计数器
        bsp_status_t (*set_period)(void *device_handle, uint32_t period_ticks);        // 设置周期
        bsp_status_t (*get_period)(const void *device_handle, uint32_t *period_ticks); // 获取周期
        bsp_status_t (*get_frequency)(const void *device_handle,
                                      uint32_t *frequency_hz); // 获取频率
    } bsp_timer_driver_ops_t;

    /* ---------- 派生设备对象 ---------- */
    /**
     * @brief 定时器设备对象（派生类）
     */
    struct bsp_timer_device
    {
        bsp_timer_t super;                        // 基类实例
        const bsp_timer_driver_ops_t *driver_ops; // 底层驱动操作表
    };

    /* ---------- 配置结构 ---------- */
    /**
     * @brief 定时器初始化配置
     */
    typedef struct
    {
        void *device_handle;                      // 平台设备句柄
        const bsp_timer_driver_ops_t *driver_ops; // 底层驱动表
        bsp_timer_callback_t callback;            // 到期回调（可为 NULL）
        void *user_context;                       // 回调用户上下文
    } bsp_timer_config_t;

    /* ---------- 公共 API 声明 ---------- */

    /**
     * @brief 初始化定时器设备
     * @param me 设备对象指针
     * @param config 配置参数
     * @return 执行状态
     */
    bsp_status_t bsp_timer_init(bsp_timer_device_t *const me,
                                const bsp_timer_config_t *const config);

    /**
     * @brief 将派生对象转为基类指针（向上转型）
     */
    bsp_timer_t *bsp_timer_as_base(bsp_timer_device_t *const me);

    /**
     * @brief 设置定时器到期回调
     * @param me 基类指针
     * @param callback 回调函数指针（可为 NULL）
     * @param user_context 用户上下文
     * @return 执行状态
     */
    bsp_status_t bsp_timer_set_callback(bsp_timer_t *const me, bsp_timer_callback_t callback,
                                        void *user_context);

    /**
     * @brief 启动定时器
     */
    bsp_status_t bsp_timer_start(bsp_timer_t *const me);

    /**
     * @brief 停止定时器
     */
    bsp_status_t bsp_timer_stop(bsp_timer_t *const me);

    /**
     * @brief 复位计数器为 0
     */
    bsp_status_t bsp_timer_reset(bsp_timer_t *const me);

    /**
     * @brief 设置计数器值
     * @param me 基类指针
     * @param counter_ticks 计数器值（tick）
     * @return 执行状态
     */
    bsp_status_t bsp_timer_set_counter(bsp_timer_t *const me, uint32_t counter_ticks);

    /**
     * @brief 获取当前计数器值
     * @param me 基类指针（const）
     * @param counter_ticks 输出计数器值
     * @return 执行状态
     */
    bsp_status_t bsp_timer_get_counter(const bsp_timer_t *const me, uint32_t *counter_ticks);

    /**
     * @brief 设置定时器周期
     * @param me 基类指针
     * @param period_ticks 周期值（tick），必须大于 0
     * @return 执行状态
     */
    bsp_status_t bsp_timer_set_period(bsp_timer_t *const me, uint32_t period_ticks);

    /**
     * @brief 获取当前周期值
     * @param me 基类指针（const）
     * @param period_ticks 输出周期值
     * @return 执行状态
     */
    bsp_status_t bsp_timer_get_period(const bsp_timer_t *const me, uint32_t *period_ticks);

    /**
     * @brief 获取定时器时钟频率
     * @param me 基类指针（const）
     * @param frequency_hz 输出频率（Hz）
     * @return 执行状态
     */
    bsp_status_t bsp_timer_get_frequency(const bsp_timer_t *const me, uint32_t *frequency_hz);

    /**
     * @brief 定时器到期通知函数（由底层驱动在 ISR 中调用）
     * @param me 基类指针
     * @note 该函数在 ISR 上下文中被调用，会触发用户回调
     */
    void bsp_timer_notify_elapsed(bsp_timer_t *const me);

#ifdef __cplusplus
}
#endif

#endif