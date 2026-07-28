/**
 * @file module_motor_health.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 多电机健康聚合器头文件
 * @version 1.0
 * @date 2026-07-28
 * @copyright Copyright (c) 2026
 *
 * @note 将注册、在线、故障、使能和温度等状态转换为稳定的可用性数组，
 *       供底盘降级运动学和 App 安全状态机使用。
 *       支持过流、编码器突跳、跟踪误差、堵转、饱和和总线错误等扩展诊断。
 */

#ifndef MODULE_MOTOR_HEALTH_H
#define MODULE_MOTOR_HEALTH_H

#include "module_motor.h" // 电机基类

#include <stdbool.h> // bool
#include <stddef.h>  // size_t
#include <stdint.h>  // uint32_t

#ifdef __cplusplus
extern "C"
{
#endif

/* ======================== 健康原因位掩码 ======================== */

/** @brief 无故障 */
#define MODULE_MOTOR_HEALTH_REASON_NONE (0U)
/** @brief 电机未注册到注册表 */
#define MODULE_MOTOR_HEALTH_REASON_NOT_REGISTERED (1U << 0)
/** @brief 反馈离线（超时未收到数据） */
#define MODULE_MOTOR_HEALTH_REASON_OFFLINE (1U << 1)
/** @brief 电机基类处于故障状态（FAULT） */
#define MODULE_MOTOR_HEALTH_REASON_MOTOR_FAULT (1U << 2)
/** @brief 电机未使能（DISABLED 状态） */
#define MODULE_MOTOR_HEALTH_REASON_NOT_ENABLED (1U << 3)
/** @brief 电机过温（超过配置阈值） */
#define MODULE_MOTOR_HEALTH_REASON_OVER_TEMPERATURE (1U << 4)
/** @brief 过流（超过配置阈值） */
#define MODULE_MOTOR_HEALTH_REASON_OVER_CURRENT (1U << 5)
/** @brief 编码器突跳（单周期变化超过阈值） */
#define MODULE_MOTOR_HEALTH_REASON_ENCODER_JUMP (1U << 6)
/** @brief 跟踪误差超限（命令值与实际值偏差过大） */
#define MODULE_MOTOR_HEALTH_REASON_TRACKING_ERROR (1U << 7)
/** @brief 堵转（电流大但速度接近零） */
#define MODULE_MOTOR_HEALTH_REASON_STALL (1U << 8)
/** @brief 输出饱和（命令值达到限幅） */
#define MODULE_MOTOR_HEALTH_REASON_OUTPUT_SATURATED (1U << 9)
/** @brief 总线错误（CAN 错误计数变化） */
#define MODULE_MOTOR_HEALTH_REASON_BUS_ERROR (1U << 10)

    /* ======================== 观察数据结构 ======================== */

    /**
     * @brief 健康观察数据（由 observer 回调提供）
     * @note 包含控制器输出、跟踪误差、输出限幅和总线错误计数
     */
    typedef struct
    {
        float commanded_effort;   // 命令值（控制器输出）
        float tracking_error;     // 跟踪误差（命令值 - 实际值）
        float output_limit;       // 输出限幅值（正数）
        uint32_t bus_error_count; // 总线错误计数（如 CAN 错误）
        bool is_valid;            // 观察数据是否有效
    } module_motor_health_observation_t;

    /**
     * @brief 健康观察回调函数
     * @param motor 电机对象
     * @param observation 输出观察数据
     * @param user_context 用户上下文
     * @return true=观察数据有效，false=无效
     * @note 由调用者实现，提供命令值、跟踪误差、输出限幅和总线错误计数
     */
    typedef bool (*module_motor_health_observer_t)(const module_motor_t *motor,
                                                   module_motor_health_observation_t *observation,
                                                   void *user_context);

    /* ======================== 状态码枚举 ======================== */

    /**
     * @brief 健康模块状态码
     */
    typedef enum
    {
        MODULE_MOTOR_HEALTH_STATUS_OK = 0,           // 所有电机可用
        MODULE_MOTOR_HEALTH_STATUS_DEGRADED,         // 部分电机不可用
        MODULE_MOTOR_HEALTH_STATUS_INVALID_ARGUMENT, // 参数非法
        MODULE_MOTOR_HEALTH_STATUS_NOT_INITIALIZED,  // 对象未初始化
        MODULE_MOTOR_HEALTH_STATUS_MOTOR_ERROR       // 电机操作错误
    } module_motor_health_status_t;

    /* ======================== 健康状态结构体 ======================== */

    /**
     * @brief 单电机的健康状态（由健康模块维护）
     */
    typedef struct
    {
        uint32_t reason_mask;                // 原因位掩码（指示哪些条件不满足）
        uint32_t fault_elapsed_time_ms;      // 故障已持续累积时间（毫秒）
        uint32_t recovery_elapsed_time_ms;   // 恢复已持续累积时间（毫秒）
        uint32_t stall_elapsed_time_ms;      // 堵转已持续累积时间（毫秒）
        uint32_t saturation_elapsed_time_ms; // 饱和已持续累积时间（毫秒）
        uint32_t previous_raw_position;      // 上次编码器原始值
        uint32_t previous_bus_error_count;   // 上次总线错误计数
        bool has_previous_sample;            // 是否有上次采样数据
        bool is_available;                   // 是否可用（健康）
    } module_motor_health_state_t;

    /* ======================== 配置结构体 ======================== */

    /**
     * @brief 健康模块配置
     * @note 所有数组容量至少为 motor_count，并覆盖对象生命周期
     *       不需要的检查传 NULL 即可关闭
     */
    typedef struct
    {
        module_motor_t *const *motors;              // 电机指针数组（只读）
        size_t motor_count;                         // 电机数量
        module_motor_health_state_t *state_storage; // 状态存储数组（调用者分配）
        const float *maximum_temperature_c;         // 逐电机最大温度（℃），NULL 表示禁用
        const float *maximum_current_a;             // 逐电机最大电流（A），NULL 表示禁用
        const uint32_t *maximum_encoder_step;       // 逐电机最大编码器步长，NULL 表示禁用
        const uint32_t *encoder_modulus;            // 逐电机编码器模数（如 8192），NULL 表示禁用
        const float *maximum_tracking_error;        // 逐电机最大跟踪误差，NULL 表示禁用
        const float *stall_current_a;               // 逐电机堵转电流阈值（A），NULL 表示禁用
        const float *stall_velocity_rad_per_s;      // 逐电机堵转速度阈值（rad/s），NULL 表示禁用
        float output_saturation_ratio;              // 输出饱和比例（0~1）
        uint32_t stall_confirmation_time_ms;        // 堵转确认时间（毫秒）
        uint32_t saturation_confirmation_time_ms;   // 饱和确认时间（毫秒）
        module_motor_health_observer_t observer;    // 观察回调（可为 NULL）
        void *observer_user_context;                // 观察回调用户上下文
        uint32_t fault_confirmation_time_ms;        // 故障确认时间（毫秒）
        uint32_t recovery_confirmation_time_ms;     // 恢复确认时间（毫秒）
        bool require_enabled_state;                 // 是否要求电机处于 ENABLED 状态
        bool manage_feedback_time;                  // 是否由本模块管理反馈超时
    } module_motor_health_config_t;

    /* ======================== 对象结构体 ======================== */

    /**
     * @brief 健康模块对象
     */
    typedef struct
    {
        module_motor_t *const *motors;            // 电机指针数组（引用外部）
        size_t motor_count;                       // 电机数量
        module_motor_health_state_t *states;      // 状态数组（引用外部）
        const float *maximum_temperature_c;       // 最大温度数组
        const float *maximum_current_a;           // 最大电流数组
        const uint32_t *maximum_encoder_step;     // 最大编码器步长数组
        const uint32_t *encoder_modulus;          // 编码器模数数组
        const float *maximum_tracking_error;      // 最大跟踪误差数组
        const float *stall_current_a;             // 堵转电流阈值数组
        const float *stall_velocity_rad_per_s;    // 堵转速度阈值数组
        float output_saturation_ratio;            // 输出饱和比例
        uint32_t stall_confirmation_time_ms;      // 堵转确认时间
        uint32_t saturation_confirmation_time_ms; // 饱和确认时间
        module_motor_health_observer_t observer;  // 观察回调
        void *observer_user_context;              // 观察回调用户上下文
        uint32_t fault_confirmation_time_ms;      // 故障确认时间
        uint32_t recovery_confirmation_time_ms;   // 恢复确认时间
        bool require_enabled_state;               // 是否要求使能状态
        bool manage_feedback_time;                // 是否管理反馈超时
        bool is_initialized;                      // 是否已初始化
    } module_motor_health_t;

    /* ======================== 公共 API ======================== */

    /**
     * @brief 初始化健康模块
     * @param me 健康模块对象
     * @param config 配置参数
     * @return 执行状态
     */
    module_motor_health_status_t
    module_motor_health_init(module_motor_health_t *me, const module_motor_health_config_t *config);

    /**
     * @brief 周期更新健康状态
     * @param me 健康模块对象
     * @param elapsed_time_ms 距上次更新的时间（毫秒）
     * @return OK=所有电机可用，DEGRADED=部分不可用
     * @note 应由一个周期任务统一调用
     */
    module_motor_health_status_t module_motor_health_update(module_motor_health_t *me,
                                                            uint32_t elapsed_time_ms);

    /**
     * @brief 获取所有电机的可用性状态
     * @param me 健康模块对象
     * @param motor_is_available 输出可用性数组（调用者分配）
     * @param output_capacity 输出数组容量
     * @return OK=所有可用，DEGRADED=部分不可用
     */
    module_motor_health_status_t
    module_motor_health_get_availability(const module_motor_health_t *me, bool *motor_is_available,
                                         size_t output_capacity);

    /**
     * @brief 获取指定电机的健康状态
     * @param me 健康模块对象
     * @param motor_index 电机索引
     * @return 健康状态指针，若索引无效则返回 NULL
     */
    const module_motor_health_state_t *
    module_motor_health_get_state(const module_motor_health_t *me, size_t motor_index);

#ifdef __cplusplus
}
#endif

#endif /* MODULE_MOTOR_HEALTH_H */