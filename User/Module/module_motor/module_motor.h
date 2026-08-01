/**
 * @file module_motor.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 通用电机基类头文件
 * @version 1.0
 * @date 2026-07-28
 * @copyright Copyright (c) 2026
 *
 * @note 为电机对象提供统一的注册、使能、失能、目标设置、周期更新和反馈接口。
 *       派生类第一成员必须命名为 super，虚表使用只读 module_motor_ops_t。
 */

#ifndef MODULE_MOTOR_H
#define MODULE_MOTOR_H

#include "alg_pid.h"

#include <stdbool.h> // bool
#include <stddef.h>  // size_t, offsetof
#include <stdint.h>  // uint32_t

#ifdef __cplusplus
extern "C"
{
#endif

/* ======================== 容器宏 ======================== */

/**
 * @brief 通过成员指针获取包含该成员的父结构体指针
 * @param member_pointer 成员指针
 * @param parent_type 父结构体类型
 * @param member_name 成员名称
 * @return 父结构体指针
 */
#define MODULE_MOTOR_CONTAINER_OF(member_pointer, parent_type, member_name)                        \
    ((parent_type *)((uint8_t *)(member_pointer) - offsetof(parent_type, member_name)))

/**
 * @brief 编译期检查电机派生对象是否将 super 放在首成员
 * @param derived_type 完整的派生对象类型
 */
#define MODULE_MOTOR_STATIC_ASSERT_SUPER_FIRST(derived_type)                                      \
    _Static_assert(offsetof(derived_type, super) == 0U, #derived_type " must place super first")

    /* ======================== 前向声明 ======================== */

    typedef struct module_motor module_motor_t;
    typedef struct module_motor_registry module_motor_registry_t;

    /* ======================== 状态码枚举 ======================== */

    /**
     * @brief 电机模块状态码
     */
    typedef enum
    {
        MODULE_MOTOR_STATUS_OK = 0,              // 操作成功
        MODULE_MOTOR_STATUS_INVALID_ARGUMENT,    // 参数非法
        MODULE_MOTOR_STATUS_NOT_INITIALIZED,     // 对象未初始化
        MODULE_MOTOR_STATUS_NOT_REGISTERED,      // 未注册到注册表
        MODULE_MOTOR_STATUS_ALREADY_REGISTERED,  // 已注册（重复注册）
        MODULE_MOTOR_STATUS_DUPLICATE_KEY,       // 注册键值重复
        MODULE_MOTOR_STATUS_NO_RESOURCE,         // 注册表已满
        MODULE_MOTOR_STATUS_OUT_OF_RANGE,        // 参数超出范围
        MODULE_MOTOR_STATUS_UNSUPPORTED,         // 操作不支持
        MODULE_MOTOR_STATUS_TRANSPORT_ERROR,     // 通信错误
        MODULE_MOTOR_STATUS_FEEDBACK_UNAVAILABLE // 反馈不可用（离线或故障）
    } module_motor_status_t;

    /* ======================== 电机状态枚举 ======================== */

    /**
     * @brief 电机运行状态
     */
    typedef enum
    {
        MODULE_MOTOR_STATE_DISABLED = 0, // 禁用（无输出）
        MODULE_MOTOR_STATE_ENABLED,      // 使能（允许输出）
        MODULE_MOTOR_STATE_FAULT         // 故障（输出被禁止）
    } module_motor_state_t;

    /**
     * @brief PID 算法形式
     * @note POSITIONAL 指位置式算法，不代表电机位置环。
     */
    typedef enum
    {
        MODULE_MOTOR_PID_POSITIONAL = 0,
        MODULE_MOTOR_PID_INCREMENTAL
    } module_motor_pid_form_t;

    /** @brief 单个电机控制环的 PID 配置 */
    typedef struct
    {
        module_motor_pid_form_t form;
        alg_pid_config_t positional_config;
        alg_pid_incremental_config_t incremental_config;
    } module_motor_pid_config_t;

    /** @brief 可选择位置式或增量式算法的 PID 控制环 */
    typedef struct
    {
        module_motor_pid_form_t form;
        union
        {
            alg_pid_t positional;
            alg_pid_incremental_t incremental;
        } controller;
        bool is_initialized;
    } module_motor_pid_t;

    /* ======================== 反馈数据结构 ======================== */

    /**
     * @brief 电机反馈数据（由派生类填充）
     * @note 所有物理量使用 SI 单位：位置 rad，速度 rad/s，扭矩 Nm，电流 A，温度 °C
     */
    typedef struct
    {
        float position_rad;                    // 位置（弧度）
        float velocity_rad_per_s;              // 速度（rad/s）
        float torque_nm;                       // 扭矩（Nm）
        float current_a;                       // 电流（A）
        float motor_temperature_c;             // 电机温度（°C）
        int16_t current_raw;                   // 原始电流值（协议原始值）
        uint32_t raw_position;                 // 原始位置值（协议原始值）
        uint32_t update_count;                 // 反馈更新计数
        uint32_t elapsed_time_since_update_ms; // 距上次更新的时间（毫秒）
        bool is_current_a_valid;               // current_a 是否有效（已换算）
        bool is_online;                        // 是否在线（最近收到反馈）
    } module_motor_feedback_t;

    /* ======================== 虚表结构 ======================== */

    /**
     * @brief 电机操作虚表（由派生类在 .c 中静态定义）
     * @note 所有函数必须实现（不能为 NULL）
     */
    typedef struct
    {
        module_motor_status_t (*enable)(module_motor_t *const me);  // 使能电机
        module_motor_status_t (*disable)(module_motor_t *const me); // 禁用电机
        module_motor_status_t (*set_target)(module_motor_t *const me,
                                            float target_value);                       // 设置目标
        module_motor_status_t (*update)(module_motor_t *const me, float delta_time_s); // 周期更新
    } module_motor_ops_t;

    /* ======================== 电机基类 ======================== */

    /**
     * @brief 电机基类
     * @note 派生类必须将 super 作为第一个成员
     */
    struct module_motor
    {
        const module_motor_ops_t *vptr;   // 虚表指针（只读）
        const char *motor_name;           // 调试可见的电机名称（调用者长期持有字符串）
        uint32_t registration_key;        // 注册键值（唯一标识）
        uint32_t motor_identifier;        // 电机协议 ID 或主机 ID
        size_t registry_index;            // 在注册表中的索引
        module_motor_state_t state;       // 当前运行状态
        module_motor_feedback_t feedback; // 反馈数据
        float delta_time_s;               // 最近一次成功控制更新的时间步长
        uint64_t total_runtime_us;         // 累计成功更新时间（微秒，包含失能状态）
        uint64_t enabled_runtime_us;       // 累计使能运行时间（微秒）
        uint32_t control_update_count;     // 成功控制更新次数
        module_motor_status_t last_update_status; // 最近一次 update 状态
        uint32_t feedback_timeout_ms;     // 反馈超时时间（0 表示禁用）
        bool is_initialized;              // 是否已初始化
        bool is_registered;               // 是否已注册到注册表
    };

    /* ======================== 注册表结构 ======================== */

    /**
     * @brief 电机注册表
     * @note 存储由调用者分配，不分配动态内存
     */
    struct module_motor_registry
    {
        module_motor_t **motor_storage; // 电机数组（由调用者分配）
        size_t motor_capacity;          // 数组容量
        size_t motor_count;             // 当前电机数量
        bool is_initialized;            // 是否已初始化
    };

    /* ======================== 公共 API ======================== */

    module_motor_status_t module_motor_pid_init(module_motor_pid_t *const me,
                                                const module_motor_pid_config_t *const config);
    module_motor_status_t module_motor_pid_reset(module_motor_pid_t *const me,
                                                 float measurement,
                                                 float initial_output);
    module_motor_status_t module_motor_pid_update(module_motor_pid_t *const me,
                                                  float setpoint,
                                                  float measurement,
                                                  float delta_time_s,
                                                  float *const output);
    const alg_pid_terms_t *module_motor_pid_get_terms(const module_motor_pid_t *const me);

    /**
     * @brief 初始化电机基类
     * @param me 电机对象
     * @param vptr 虚表指针
     * @param motor_name 调试可见的电机名称
     * @param registration_key 注册键值
     * @param motor_identifier 电机协议 ID 或主机 ID
     * @return 执行状态
     */
    module_motor_status_t module_motor_init_base(module_motor_t *const me,
                                                 const module_motor_ops_t *const vptr,
                                                 const char *const motor_name,
                                                 uint32_t registration_key,
                                                 uint32_t motor_identifier);

    /**
     * @brief 初始化电机注册表
     * @param me 注册表对象
     * @param motor_storage 电机存储数组（调用者分配）
     * @param motor_capacity 数组容量
     * @return 执行状态
     */
    module_motor_status_t module_motor_registry_init(module_motor_registry_t *const me,
                                                     module_motor_t **const motor_storage,
                                                     size_t motor_capacity);

    /**
     * @brief 注册电机到注册表
     * @param me 注册表对象
     * @param motor 电机对象
     * @return 执行状态
     * @note 检查重复键值，若注册表已满则返回 NO_RESOURCE
     */
    module_motor_status_t module_motor_registry_register(module_motor_registry_t *const me,
                                                         module_motor_t *const motor);

    /**
     * @brief 从注册表注销电机
     * @param me 注册表对象
     * @param motor 电机对象
     * @return 执行状态
     */
    module_motor_status_t module_motor_registry_unregister(module_motor_registry_t *const me,
                                                           module_motor_t *const motor);

    /**
     * @brief 根据注册键值查找电机
     * @param me 注册表对象
     * @param registration_key 注册键值
     * @return 电机指针，若未找到则返回 NULL
     */
    module_motor_t *module_motor_registry_find(const module_motor_registry_t *const me,
                                               uint32_t registration_key);

    /**
     * @brief 获取注册表中电机数量
     * @param me 注册表对象
     * @return 电机数量
     */
    size_t module_motor_registry_get_count(const module_motor_registry_t *const me);

    /**
     * @brief 使能电机（调用虚表 enable）
     * @param me 电机对象
     * @return 执行状态
     * @note 未注册、故障或离线状态下无法使能
     */
    module_motor_status_t module_motor_enable(module_motor_t *const me);

    /**
     * @brief 禁用电机（调用虚表 disable）
     * @param me 电机对象
     * @return 执行状态
     */
    module_motor_status_t module_motor_disable(module_motor_t *const me);

    /**
     * @brief 清除故障状态（仅当反馈在线时）
     * @param me 电机对象
     * @return 执行状态
     * @note 清除后状态变为 DISABLED，需要显式 enable 才能恢复输出
     */
    module_motor_status_t module_motor_clear_fault(module_motor_t *const me);

    /**
     * @brief 设置目标值（调用虚表 set_target）
     * @param me 电机对象
     * @param target_value 目标值（含义取决于派生类）
     * @return 执行状态
     */
    module_motor_status_t module_motor_set_target(module_motor_t *const me, float target_value);

    /**
     * @brief 周期更新电机（调用虚表 update）
     * @param me 电机对象
     * @param delta_time_s 时间步长（秒）
     * @return 执行状态
     * @note 若反馈离线且电机使能，自动进入故障状态
     */
    module_motor_status_t module_motor_update(module_motor_t *const me, float delta_time_s);

    /**
     * @brief 设置反馈超时时间
     * @param me 电机对象
     * @param feedback_timeout_ms 超时时间（毫秒），0 表示禁用
     * @return 执行状态
     */
    module_motor_status_t module_motor_set_feedback_timeout(module_motor_t *const me,
                                                            uint32_t feedback_timeout_ms);

    /**
     * @brief 更新反馈超时计时（需周期性调用）
     * @param me 电机对象
     * @param elapsed_time_ms 距上次调用的时间（毫秒）
     * @return 执行状态
     */
    module_motor_status_t module_motor_update_feedback_time(module_motor_t *const me,
                                                            uint32_t elapsed_time_ms);

    /**
     * @brief 通知反馈已更新（由派生类在解析反馈后调用）
     * @param me 电机对象
     * @return 执行状态
     */
    module_motor_status_t module_motor_notify_feedback(module_motor_t *const me);

    /**
     * @brief 获取反馈数据指针
     * @param me 电机对象
     * @return 反馈指针，未注册或未初始化则返回 NULL
     */
    const module_motor_feedback_t *module_motor_get_feedback(const module_motor_t *const me);

#ifdef __cplusplus
}
#endif

#endif /* MODULE_MOTOR_H */
