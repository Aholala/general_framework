/**
 * @file bsp_timebase.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 时间基准通用抽象层头文件
 * @version 1.0
 * @date 2026-07-27
 * @copyright Copyright (c) 2026
 *
 * @note 基于自由运行周期计数器的单调时间基准，可由 Cortex DWT、
 *       通用 32 位定时器或其他硬件实现。提供时间点、回绕安全的
 *       耗时计算、周期/微秒转换和短延时。
 */

#ifndef BSP_TIMEBASE_H
#define BSP_TIMEBASE_H

#include "bsp_common.h" // 包含基础类型、状态码、设备基类等

#ifdef __cplusplus
extern "C"
{
#endif

    /* 前向声明，避免循环依赖 */
    typedef struct bsp_timebase bsp_timebase_t;
    typedef struct bsp_timebase_device bsp_timebase_device_t;

    /* ---------- 时间点结构体 ---------- */
    /**
     * @brief 时间点结构体
     * @note 本质上是当前周期计数值的快照
     *       用于计算时间差（经过的周期数）
     */
    typedef struct
    {
        uint32_t cycle_count; // 周期计数值（快照）
    } bsp_timebase_time_point_t;

    /* ---------- 高层虚表（面向应用层） ---------- */
    /**
     * @brief 时间基准操作虚表，继承自 bsp_device_ops_t
     * @note 派生类必须保持 super 为第一成员
     *       get_cycle_count 和 get_frequency 必须实现
     *       reset 为可选
     */
    typedef struct
    {
        bsp_device_ops_t super;                          // 父类虚表（含 deinit）
        bsp_status_t (*reset)(bsp_timebase_t *const me); // 复位计数器（可选）
        bsp_status_t (*get_cycle_count)(const bsp_timebase_t *const me,
                                        uint32_t *cycle_count); // 获取计数
        bsp_status_t (*get_frequency)(const bsp_timebase_t *const me,
                                      uint32_t *frequency_hz); // 获取频率
    } bsp_timebase_ops_t;

    /* ---------- 基类 ---------- */
    /**
     * @brief 时间基准基类结构体
     * @note 目前基类仅为 bsp_device_t 的简单包装
     */
    struct bsp_timebase
    {
        bsp_device_t super; // 设备基类
    };

    /* ---------- 底层驱动操作表（平台实现） ---------- */
    /**
     * @brief 平台相关时间基准驱动操作表
     * @note get_cycle_count 和 get_frequency 必须实现
     *       init/deinit/reset 为可选
     *       计数器必须单调递增并按 uint32_t 自然回绕
     *       频率在对象使用期间应保持稳定
     */
    typedef struct
    {
        bsp_status_t (*init)(void *device_handle);   // 初始化（可选）
        bsp_status_t (*deinit)(void *device_handle); // 反初始化（可选）
        bsp_status_t (*reset)(void *device_handle);  // 复位计数器（可选）
        bsp_status_t (*get_cycle_count)(const void *device_handle,
                                        uint32_t *cycle_count); // 获取计数（必须）
        bsp_status_t (*get_frequency)(const void *device_handle,
                                      uint32_t *frequency_hz); // 获取频率（必须）
    } bsp_timebase_driver_ops_t;

    /* ---------- 派生设备对象 ---------- */
    /**
     * @brief 时间基准设备对象（派生类）
     */
    struct bsp_timebase_device
    {
        bsp_timebase_t super;                        // 基类实例
        const bsp_timebase_driver_ops_t *driver_ops; // 底层驱动操作表
    };

    /* ---------- 配置结构 ---------- */
    /**
     * @brief 时间基准初始化配置
     */
    typedef struct
    {
        void *device_handle;                         // 平台设备句柄
        const bsp_timebase_driver_ops_t *driver_ops; // 底层驱动表
    } bsp_timebase_config_t;

    /* ---------- 公共 API ---------- */

    /**
     * @brief 初始化时间基准设备
     * @param me 设备对象指针
     * @param config 配置参数
     * @return 执行状态
     */
    bsp_status_t bsp_timebase_init(bsp_timebase_device_t *const me,
                                   const bsp_timebase_config_t *const config);

    /**
     * @brief 将派生对象转为基类指针（向上转型）
     */
    bsp_timebase_t *bsp_timebase_as_base(bsp_timebase_device_t *const me);

    /**
     * @brief 复位周期计数器
     * @param me 基类指针
     * @return 执行状态（若驱动未实现返回 UNSUPPORTED）
     */
    bsp_status_t bsp_timebase_reset(bsp_timebase_t *const me);

    /**
     * @brief 获取当前周期计数值
     * @param me 基类指针（const）
     * @param cycle_count 输出周期计数值
     * @return 执行状态
     */
    bsp_status_t bsp_timebase_get_cycle_count(const bsp_timebase_t *const me,
                                              uint32_t *cycle_count);

    /**
     * @brief 获取时间基准频率（Hz）
     * @param me 基类指针（const）
     * @param frequency_hz 输出频率
     * @return 执行状态
     */
    bsp_status_t bsp_timebase_get_frequency(const bsp_timebase_t *const me, uint32_t *frequency_hz);

    /**
     * @brief 获取当前时间点
     * @param me 基类指针（const）
     * @param time_point 输出时间点结构体
     * @return 执行状态
     */
    bsp_status_t bsp_timebase_now(const bsp_timebase_t *const me,
                                  bsp_timebase_time_point_t *time_point);

    /**
     * @brief 计算从起始时间点到现在的经过周期数
     * @param me 基类指针（const）
     * @param start_time 起始时间点
     * @param elapsed_cycles 输出经过的周期数
     * @return 执行状态
     * @note 使用无符号减法，正确处理一次 32 位回绕
     */
    bsp_status_t bsp_timebase_elapsed_cycles(const bsp_timebase_t *const me,
                                             bsp_timebase_time_point_t start_time,
                                             uint32_t *elapsed_cycles);

    /**
     * @brief 将周期数转换为微秒数
     * @param me 基类指针（const）
     * @param cycle_count 周期数
     * @param time_us 输出微秒数
     * @return 执行状态
     */
    bsp_status_t bsp_timebase_cycles_to_us(const bsp_timebase_t *const me, uint32_t cycle_count,
                                           uint32_t *time_us);

    /**
     * @brief 将微秒数转换为周期数（向上取整）
     * @param me 基类指针（const）
     * @param time_us 微秒数
     * @param cycle_count 输出周期数
     * @return 执行状态
     */
    bsp_status_t bsp_timebase_us_to_cycles(const bsp_timebase_t *const me, uint32_t time_us,
                                           uint32_t *cycle_count);

    /**
     * @brief 微秒级忙等待延时
     * @param me 基类指针（const）
     * @param delay_us 延时微秒数
     * @return 执行状态
     * @note 同步忙等待，占用 CPU。仅适用于短延时，不可代替 RTOS 延时
     */
    bsp_status_t bsp_timebase_delay_us(const bsp_timebase_t *const me, uint32_t delay_us);

    /**
     * @brief 检查从起始时间点是否已经过指定微秒数
     * @param me 基类指针（const）
     * @param start_time 起始时间点
     * @param duration_us 持续时间（微秒）
     * @param has_elapsed 输出是否已超时
     * @return 执行状态
     * @note 非阻塞，用于超时判断
     */
    bsp_status_t bsp_timebase_has_elapsed_us(const bsp_timebase_t *const me,
                                             bsp_timebase_time_point_t start_time,
                                             uint32_t duration_us, bool *has_elapsed);

#ifdef __cplusplus
}
#endif

#endif