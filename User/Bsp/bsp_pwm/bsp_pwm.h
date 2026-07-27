/**
 * @file bsp_pwm.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief PWM 通用抽象层头文件
 * @note 定义 PWM 通道的启动/停止、频率设置、脉冲宽度和归一化占空比接口。
 *       定时器实例、通道号和引脚复用通过平台句柄与配置注入。
 * @version 1.0
 * @date 2026-07-26
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef BSP_PWM_H
#define BSP_PWM_H

#include "bsp_common.h" // 包含基础类型、状态码、设备基类等

#ifdef __cplusplus
extern "C"
{
#endif

    /* 前向声明，避免循环依赖 */
    typedef struct bsp_pwm bsp_pwm_t;
    typedef struct bsp_pwm_device bsp_pwm_device_t;

    /* ---------- 高层虚表（面向应用层） ---------- */
    /**
     * @brief PWM 操作虚表，继承自 bsp_device_ops_t
     * @note 派生类必须保持 super 为第一成员
     */
    typedef struct
    {
        bsp_device_ops_t super;                     // 父类虚表（含 deinit）
        bsp_status_t (*start)(bsp_pwm_t *const me); // 启动 PWM 输出
        bsp_status_t (*stop)(bsp_pwm_t *const me);  // 停止 PWM 输出
        bsp_status_t (*set_frequency)(bsp_pwm_t *const me, uint32_t frequency_hz); // 设置频率
        bsp_status_t (*get_frequency)(const bsp_pwm_t *const me,
                                      uint32_t *frequency_hz); // 获取频率
        bsp_status_t (*set_pulse)(bsp_pwm_t *const me,
                                  uint32_t pulse_ticks); // 设置脉冲宽度（比较值）
        bsp_status_t (*get_pulse)(const bsp_pwm_t *const me, uint32_t *pulse_ticks); // 获取脉冲宽度
        bsp_status_t (*get_period)(const bsp_pwm_t *const me, uint32_t *period_ticks); // 获取周期值
    } bsp_pwm_ops_t;

    /* ---------- 基类 ---------- */
    /**
     * @brief PWM 基类结构体
     * @note 当前基类仅为 bsp_device_t 的包装，无额外字段
     */
    struct bsp_pwm
    {
        bsp_device_t super; // 设备基类
    };

    /* ---------- 底层驱动操作表（平台实现） ---------- */
    /**
     * @brief 平台相关 PWM 驱动操作表
     * @note 所有函数接收 device_handle 和通道号
     *       所有函数均为必须实现（由 bsp_pwm_init 校验）
     */
    typedef struct
    {
        bsp_status_t (*init)(void *device_handle, uint32_t channel);   // 初始化（可选）
        bsp_status_t (*deinit)(void *device_handle, uint32_t channel); // 反初始化（可选）
        bsp_status_t (*start)(void *device_handle, uint32_t channel);  // 启动输出
        bsp_status_t (*stop)(void *device_handle, uint32_t channel);   // 停止输出
        bsp_status_t (*set_frequency)(void *device_handle, uint32_t channel,
                                      uint32_t frequency_hz); // 设置频率
        bsp_status_t (*get_frequency)(const void *device_handle, uint32_t channel,
                                      uint32_t *frequency_hz); // 获取频率
        bsp_status_t (*set_pulse)(void *device_handle, uint32_t channel,
                                  uint32_t pulse_ticks); // 设置脉冲宽度
        bsp_status_t (*get_pulse)(const void *device_handle, uint32_t channel,
                                  uint32_t *pulse_ticks); // 获取脉冲宽度
        bsp_status_t (*get_period)(const void *device_handle, uint32_t channel,
                                   uint32_t *period_ticks); // 获取周期值
    } bsp_pwm_driver_ops_t;

    /* ---------- 派生设备对象 ---------- */
    /**
     * @brief PWM 设备对象（派生类）
     */
    struct bsp_pwm_device
    {
        bsp_pwm_t super;                        // 基类实例
        const bsp_pwm_driver_ops_t *driver_ops; // 底层驱动操作表
        uint32_t channel;                       // 逻辑通道号
    };

    /* ---------- 配置结构 ---------- */
    /**
     * @brief PWM 初始化配置
     */
    typedef struct
    {
        void *device_handle;                    // 平台设备句柄
        const bsp_pwm_driver_ops_t *driver_ops; // 底层驱动表
        uint32_t channel;                       // 逻辑通道号
    } bsp_pwm_config_t;

    /* ---------- 公共 API 声明 ---------- */

    /**
     * @brief 初始化 PWM 设备
     * @param me 设备对象指针
     * @param config 配置参数
     * @return 执行状态
     */
    bsp_status_t bsp_pwm_init(bsp_pwm_device_t *const me, const bsp_pwm_config_t *const config);

    /**
     * @brief 将派生对象转为基类指针（向上转型）
     */
    bsp_pwm_t *bsp_pwm_as_base(bsp_pwm_device_t *const me);

    /**
     * @brief 启动 PWM 输出
     * @param me 基类指针
     * @return 执行状态
     */
    bsp_status_t bsp_pwm_start(bsp_pwm_t *const me);

    /**
     * @brief 停止 PWM 输出
     * @param me 基类指针
     * @return 执行状态
     */
    bsp_status_t bsp_pwm_stop(bsp_pwm_t *const me);

    /**
     * @brief 设置 PWM 频率
     * @param me 基类指针
     * @param frequency_hz 频率（Hz），必须大于 0
     * @return 执行状态
     */
    bsp_status_t bsp_pwm_set_frequency(bsp_pwm_t *const me, uint32_t frequency_hz);

    /**
     * @brief 获取 PWM 频率
     * @param me 基类指针（const）
     * @param frequency_hz 输出频率（Hz）
     * @return 执行状态
     */
    bsp_status_t bsp_pwm_get_frequency(const bsp_pwm_t *const me, uint32_t *frequency_hz);

    /**
     * @brief 设置脉冲宽度（比较值）
     * @param me 基类指针
     * @param pulse_ticks 高电平比较值（以定时器 tick 为单位），不能超过周期值
     * @return 执行状态
     */
    bsp_status_t bsp_pwm_set_pulse(bsp_pwm_t *const me, uint32_t pulse_ticks);

    /**
     * @brief 获取脉冲宽度（比较值）
     * @param me 基类指针（const）
     * @param pulse_ticks 输出高电平比较值
     * @return 执行状态
     */
    bsp_status_t bsp_pwm_get_pulse(const bsp_pwm_t *const me, uint32_t *pulse_ticks);

    /**
     * @brief 设置归一化占空比
     * @param me 基类指针
     * @param duty_cycle 占空比（0.0~1.0）
     * @return 执行状态
     */
    bsp_status_t bsp_pwm_set_duty_cycle(bsp_pwm_t *const me, float duty_cycle);

    /**
     * @brief 获取归一化占空比
     * @param me 基类指针（const）
     * @param duty_cycle 输出占空比（0.0~1.0）
     * @return 执行状态
     */
    bsp_status_t bsp_pwm_get_duty_cycle(const bsp_pwm_t *const me, float *duty_cycle);

#ifdef __cplusplus
}
#endif

#endif