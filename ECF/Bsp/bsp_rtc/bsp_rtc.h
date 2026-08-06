/**
 * @file bsp_rtc.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 实时时钟（RTC）通用抽象层头文件
 * @version 1.0
 * @date 2026-07-27
 * @copyright Copyright (c) 2026
 *
 * @note 定义 RTC 的多态接口，支持结构化时间和 Unix 时间戳的获取与设置。
 *       所有地址参数均为 7 位地址（0~0x7F），平台驱动负责左移。
 */

#ifndef BSP_RTC_H
#define BSP_RTC_H

#include "bsp_common.h" // 包含基础类型、状态码、设备基类等

#ifdef __cplusplus
extern "C"
{
#endif

    /* 前向声明，避免循环依赖 */
    typedef struct bsp_rtc bsp_rtc_t;
    typedef struct bsp_rtc_device bsp_rtc_device_t;

    /* ---------- 时间结构体 ---------- */
    /**
     * @brief RTC 时间结构体
     * @note 包含年月日时分秒和毫秒
     *       - year 为完整年份（如 2026），不是两位数
     *       - millisecond 为毫秒部分（0-999），精度取决于硬件
     *       - 月、日、时、分、秒均为常见范围（1-12, 1-31, 0-23, 0-59, 0-59）
     */
    typedef struct
    {
        uint16_t year;        // 完整年份（如 2026）
        uint8_t month;        // 月份（1-12）
        uint8_t day;          // 日期（1-31）
        uint8_t hour;         // 小时（0-23）
        uint8_t minute;       // 分钟（0-59）
        uint8_t second;       // 秒（0-59）
        uint16_t millisecond; // 毫秒（0-999）
    } bsp_rtc_time_t;

    /* ---------- 高层虚表（面向应用层） ---------- */
    /**
     * @brief RTC 操作虚表，继承自 bsp_device_ops_t
     * @note 派生类必须保持 super 为第一成员
     *       包含三个 RTC 特有的虚函数
     */
    typedef struct
    {
        bsp_device_ops_t super;                                        // 父类虚表（含 deinit）
        bsp_status_t (*get_time)(bsp_rtc_t *me, bsp_rtc_time_t *time); // 获取结构化时间
        bsp_status_t (*set_time)(bsp_rtc_t *me, const bsp_rtc_time_t *time); // 设置结构化时间
        bsp_status_t (*get_unix_time)(bsp_rtc_t *me, uint64_t *unix_time_s); // 获取 Unix 时间戳
    } bsp_rtc_ops_t;

    /* ---------- 基类 ---------- */
    /**
     * @brief RTC 基类结构体
     * @note 目前基类仅为 bsp_device_t 的简单包装
     *       未来可在此增加配置字段（如时区偏移）
     */
    struct bsp_rtc
    {
        bsp_device_t super; // 设备基类
    };

    /* ---------- 底层驱动操作表（平台实现） ---------- */
    /**
     * @brief 平台相关 RTC 驱动操作表
     * @note 所有函数接收 device_handle 和必要参数
     *       所有函数均为必须实现（由 bsp_rtc_init 校验）
     */
    typedef struct
    {
        bsp_status_t (*init)(void *handle);                                 // 初始化 RTC 硬件
        bsp_status_t (*deinit)(void *handle);                               // 反初始化
        bsp_status_t (*get_time)(void *handle, bsp_rtc_time_t *time);       // 获取结构化时间
        bsp_status_t (*set_time)(void *handle, const bsp_rtc_time_t *time); // 设置结构化时间
        bsp_status_t (*get_unix_time)(void *handle, uint64_t *unix_time_s); // 获取 Unix 时间戳
    } bsp_rtc_driver_ops_t;

    /* ---------- 派生设备对象 ---------- */
    /**
     * @brief RTC 设备对象（派生类）
     * @note 包含基类 bsp_rtc_t 和底层驱动操作表指针
     *       驱动操作表由构造时注入，派生类不拥有
     */
    struct bsp_rtc_device
    {
        bsp_rtc_t super;                        // 基类实例
        const bsp_rtc_driver_ops_t *driver_ops; // 底层驱动操作表
    };

    /* ---------- 配置结构 ---------- */
    /**
     * @brief RTC 初始化配置
     * @note device_handle 的具体类型由平台定义，通用层仅作为不透明指针传递
     */
    typedef struct
    {
        void *device_handle;                    // 平台设备句柄
        const bsp_rtc_driver_ops_t *driver_ops; // 底层驱动表
    } bsp_rtc_config_t;

    /* ---------- 公共 API 声明 ---------- */

    /**
     * @brief 初始化 RTC 设备
     * @param me 设备对象指针（bsp_rtc_device_t）
     * @param config 配置参数指针
     * @return 执行状态
     * @note 所有驱动函数指针必须非空
     */
    bsp_status_t bsp_rtc_init(bsp_rtc_device_t *me, const bsp_rtc_config_t *config);

    /**
     * @brief 将派生对象转为基类指针（向上转型）
     * @param me 派生对象指针（bsp_rtc_device_t）
     * @return 基类指针（bsp_rtc_t）
     */
    bsp_rtc_t *bsp_rtc_as_base(bsp_rtc_device_t *me);

    /**
     * @brief 获取结构化时间
     * @param me 基类指针（bsp_rtc_t）
     * @param time 输出时间结构体指针
     * @return 执行状态
     */
    bsp_status_t bsp_rtc_get_time(bsp_rtc_t *me, bsp_rtc_time_t *time);

    /**
     * @brief 设置结构化时间
     * @param me 基类指针（bsp_rtc_t）
     * @param time 时间结构体指针（只读）
     * @return 执行状态
     * @note 公共层会进行基本范围校验，平台驱动应进行深度校验
     */
    bsp_status_t bsp_rtc_set_time(bsp_rtc_t *me, const bsp_rtc_time_t *time);

    /**
     * @brief 获取 Unix 时间戳
     * @param me 基类指针（bsp_rtc_t）
     * @param unix_time_s 输出 Unix 时间戳（秒，64 位）
     * @return 执行状态
     * @note 使用 uint64_t 避免 2038 年问题
     */
    bsp_status_t bsp_rtc_get_unix_time(bsp_rtc_t *me, uint64_t *unix_time_s);

#ifdef __cplusplus
}
#endif

#endif