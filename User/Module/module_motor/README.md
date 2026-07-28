好的，我将为您提供电机基类模块 `module_motor.c` 和 `module_motor.h` 的逐行中文注释，以及完整的 README 使用指南。

---

## 文件 1：`module_motor.h`（逐行注释）

```c
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

#include <stdbool.h>     // bool
#include <stddef.h>      // size_t, offsetof
#include <stdint.h>      // uint32_t

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

    /* ======================== 前向声明 ======================== */

    typedef struct module_motor module_motor_t;
    typedef struct module_motor_registry module_motor_registry_t;

    /* ======================== 状态码枚举 ======================== */

    /**
     * @brief 电机模块状态码
     */
    typedef enum
    {
        MODULE_MOTOR_STATUS_OK = 0,               // 操作成功
        MODULE_MOTOR_STATUS_INVALID_ARGUMENT,      // 参数非法
        MODULE_MOTOR_STATUS_NOT_INITIALIZED,       // 对象未初始化
        MODULE_MOTOR_STATUS_NOT_REGISTERED,        // 未注册到注册表
        MODULE_MOTOR_STATUS_ALREADY_REGISTERED,    // 已注册（重复注册）
        MODULE_MOTOR_STATUS_DUPLICATE_KEY,         // 注册键值重复
        MODULE_MOTOR_STATUS_NO_RESOURCE,           // 注册表已满
        MODULE_MOTOR_STATUS_OUT_OF_RANGE,          // 参数超出范围
        MODULE_MOTOR_STATUS_UNSUPPORTED,           // 操作不支持
        MODULE_MOTOR_STATUS_TRANSPORT_ERROR,       // 通信错误
        MODULE_MOTOR_STATUS_FEEDBACK_UNAVAILABLE   // 反馈不可用（离线或故障）
    } module_motor_status_t;

    /* ======================== 电机状态枚举 ======================== */

    /**
     * @brief 电机运行状态
     */
    typedef enum
    {
        MODULE_MOTOR_STATE_DISABLED = 0,   // 禁用（无输出）
        MODULE_MOTOR_STATE_ENABLED,        // 使能（允许输出）
        MODULE_MOTOR_STATE_FAULT           // 故障（输出被禁止）
    } module_motor_state_t;

    /* ======================== 反馈数据结构 ======================== */

    /**
     * @brief 电机反馈数据（由派生类填充）
     * @note 所有物理量使用 SI 单位：位置 rad，速度 rad/s，扭矩 Nm，电流 A，温度 °C
     */
    typedef struct
    {
        float position_rad;                     // 位置（弧度）
        float velocity_rad_per_s;               // 速度（rad/s）
        float torque_nm;                        // 扭矩（Nm）
        float current_a;                        // 电流（A）
        float motor_temperature_c;              // 电机温度（°C）
        int16_t current_raw;                    // 原始电流值（协议原始值）
        uint32_t raw_position;                  // 原始位置值（协议原始值）
        uint32_t update_count;                  // 反馈更新计数
        uint32_t elapsed_time_since_update_ms;  // 距上次更新的时间（毫秒）
        bool is_current_a_valid;                // current_a 是否有效（已换算）
        bool is_online;                         // 是否在线（最近收到反馈）
    } module_motor_feedback_t;

    /* ======================== 虚表结构 ======================== */

    /**
     * @brief 电机操作虚表（由派生类在 .c 中静态定义）
     * @note 所有函数必须实现（不能为 NULL）
     */
    typedef struct
    {
        module_motor_status_t (*enable)(module_motor_t *const me);      // 使能电机
        module_motor_status_t (*disable)(module_motor_t *const me);     // 禁用电机
        module_motor_status_t (*set_target)(module_motor_t *const me, float target_value); // 设置目标
        module_motor_status_t (*update)(module_motor_t *const me, float delta_time_s);      // 周期更新
    } module_motor_ops_t;

    /* ======================== 电机基类 ======================== */

    /**
     * @brief 电机基类
     * @note 派生类必须将 super 作为第一个成员
     */
    struct module_motor
    {
        const module_motor_ops_t *vptr;       // 虚表指针（只读）
        const char *logical_name;             // 逻辑名称
        uint32_t registration_key;            // 注册键值（唯一标识）
        size_t registry_index;                // 在注册表中的索引
        module_motor_state_t state;           // 当前运行状态
        module_motor_feedback_t feedback;     // 反馈数据
        uint32_t feedback_timeout_ms;         // 反馈超时时间（0 表示禁用）
        bool is_initialized;                  // 是否已初始化
        bool is_registered;                   // 是否已注册到注册表
    };

    /* ======================== 注册表结构 ======================== */

    /**
     * @brief 电机注册表
     * @note 存储由调用者分配，不分配动态内存
     */
    struct module_motor_registry
    {
        module_motor_t **motor_storage;       // 电机数组（由调用者分配）
        size_t motor_capacity;                // 数组容量
        size_t motor_count;                   // 当前电机数量
        bool is_initialized;                  // 是否已初始化
    };

    /* ======================== 公共 API ======================== */

    /**
     * @brief 初始化电机基类
     * @param me 电机对象
     * @param vptr 虚表指针
     * @param logical_name 逻辑名称
     * @param registration_key 注册键值
     * @return 执行状态
     */
    module_motor_status_t module_motor_init_base(module_motor_t *const me,
                                                 const module_motor_ops_t *const vptr,
                                                 const char *const logical_name,
                                                 uint32_t registration_key);

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
```

---

## 文件 2：`module_motor.c`（逐行注释）

```c
/**
 * @file module_motor.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 通用电机基类实现
 * @version 1.0
 * @date 2026-07-28
 * @copyright Copyright (c) 2026
 *
 * @note 提供统一的注册、使能、失能、目标设置、周期更新和反馈管理。
 *       反馈超时检测、故障状态管理和注册表管理。
 */

#include "module_motor.h"

#include <math.h>       // isfinite
#include <stddef.h>     // NULL, SIZE_MAX

/**
 * @brief 校验电机是否已注册且有效
 * @param me 电机对象
 * @return 执行状态
 */
static module_motor_status_t module_motor_validate_registered(const module_motor_t *const me)
{
    if (me == NULL)
    {
        return MODULE_MOTOR_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized || (me->vptr == NULL))
    {
        return MODULE_MOTOR_STATUS_NOT_INITIALIZED;
    }
    return me->is_registered ? MODULE_MOTOR_STATUS_OK : MODULE_MOTOR_STATUS_NOT_REGISTERED;
}

/**
 * @brief 因反馈离线进入故障状态
 * @param me 电机对象
 * @return 执行状态
 * @note 调用派生类的 disable 清零输出，然后置状态为 FAULT
 */
static module_motor_status_t module_motor_enter_feedback_fault(module_motor_t *const me)
{
    module_motor_status_t status;

    // 只有使能状态才需要进入故障
    if (me->state != MODULE_MOTOR_STATE_ENABLED)
    {
        return MODULE_MOTOR_STATUS_FEEDBACK_UNAVAILABLE;
    }
    // 调用派生类 disable（应该清零命令）
    status = me->vptr->disable(me);
    me->state = MODULE_MOTOR_STATE_FAULT;
    return (status == MODULE_MOTOR_STATUS_OK) ? MODULE_MOTOR_STATUS_FEEDBACK_UNAVAILABLE : status;
}

/* ======================== 基类初始化 ======================== */

/**
 * @brief 初始化电机基类
 * @param me 电机对象
 * @param vptr 虚表指针
 * @param logical_name 逻辑名称
 * @param registration_key 注册键值
 * @return 执行状态
 * @note 检查所有虚函数是否非空
 */
module_motor_status_t module_motor_init_base(module_motor_t *const me,
                                             const module_motor_ops_t *const vptr,
                                             const char *const logical_name,
                                             uint32_t registration_key)
{
    // ---- 参数校验 ----
    if ((me == NULL) || (vptr == NULL) || (logical_name == NULL) || (vptr->enable == NULL) ||
        (vptr->disable == NULL) || (vptr->set_target == NULL) || (vptr->update == NULL))
    {
        return MODULE_MOTOR_STATUS_INVALID_ARGUMENT;
    }

    // ---- 初始化基类字段 ----
    me->vptr = vptr;
    me->logical_name = logical_name;
    me->registration_key = registration_key;
    me->registry_index = SIZE_MAX;              // 未注册状态
    me->state = MODULE_MOTOR_STATE_DISABLED;
    me->feedback = (module_motor_feedback_t){0};
    me->feedback_timeout_ms = 0U;
    me->is_registered = false;
    me->is_initialized = true;
    return MODULE_MOTOR_STATUS_OK;
}

/* ======================== 注册表管理 ======================== */

/**
 * @brief 初始化电机注册表
 */
module_motor_status_t module_motor_registry_init(module_motor_registry_t *const me,
                                                 module_motor_t **const motor_storage,
                                                 size_t motor_capacity)
{
    size_t motor_index;

    if ((me == NULL) || (motor_storage == NULL) || (motor_capacity == 0U))
    {
        return MODULE_MOTOR_STATUS_INVALID_ARGUMENT;
    }

    // 清空存储数组
    for (motor_index = 0U; motor_index < motor_capacity; ++motor_index)
    {
        motor_storage[motor_index] = NULL;
    }

    me->motor_storage = motor_storage;
    me->motor_capacity = motor_capacity;
    me->motor_count = 0U;
    me->is_initialized = true;
    return MODULE_MOTOR_STATUS_OK;
}

/**
 * @brief 注册电机到注册表
 */
module_motor_status_t module_motor_registry_register(module_motor_registry_t *const me,
                                                     module_motor_t *const motor)
{
    size_t motor_index;

    // ---- 参数校验 ----
    if ((me == NULL) || (motor == NULL))
    {
        return MODULE_MOTOR_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized || !motor->is_initialized)
    {
        return MODULE_MOTOR_STATUS_NOT_INITIALIZED;
    }
    if (motor->is_registered)
    {
        return MODULE_MOTOR_STATUS_ALREADY_REGISTERED;
    }

    // ---- 检查容量 ----
    if (me->motor_count >= me->motor_capacity)
    {
        return MODULE_MOTOR_STATUS_NO_RESOURCE;
    }

    // ---- 检查重复键值 ----
    for (motor_index = 0U; motor_index < me->motor_count; ++motor_index)
    {
        if (me->motor_storage[motor_index]->registration_key == motor->registration_key)
        {
            return MODULE_MOTOR_STATUS_DUPLICATE_KEY;
        }
    }

    // ---- 注册 ----
    motor->registry_index = me->motor_count;
    me->motor_storage[me->motor_count] = motor;
    ++me->motor_count;
    motor->is_registered = true;
    return MODULE_MOTOR_STATUS_OK;
}

/**
 * @brief 从注册表注销电机
 */
module_motor_status_t module_motor_registry_unregister(module_motor_registry_t *const me,
                                                       module_motor_t *const motor)
{
    size_t motor_index;

    if ((me == NULL) || (motor == NULL))
    {
        return MODULE_MOTOR_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized || !motor->is_registered ||
        (motor->registry_index >= me->motor_count) ||
        (me->motor_storage[motor->registry_index] != motor))
    {
        return MODULE_MOTOR_STATUS_NOT_REGISTERED;
    }

    // 后续电机前移，更新索引
    for (motor_index = motor->registry_index; (motor_index + 1U) < me->motor_count; ++motor_index)
    {
        me->motor_storage[motor_index] = me->motor_storage[motor_index + 1U];
        me->motor_storage[motor_index]->registry_index = motor_index;
    }

    // 清空最后一个位置
    --me->motor_count;
    me->motor_storage[me->motor_count] = NULL;

    // 清除电机注册状态
    motor->registry_index = SIZE_MAX;
    motor->is_registered = false;
    motor->state = MODULE_MOTOR_STATE_DISABLED;   // 禁用状态
    return MODULE_MOTOR_STATUS_OK;
}

/**
 * @brief 根据注册键值查找电机
 */
module_motor_t *module_motor_registry_find(const module_motor_registry_t *const me,
                                           uint32_t registration_key)
{
    size_t motor_index;

    if ((me == NULL) || !me->is_initialized)
    {
        return NULL;
    }
    for (motor_index = 0U; motor_index < me->motor_count; ++motor_index)
    {
        if (me->motor_storage[motor_index]->registration_key == registration_key)
        {
            return me->motor_storage[motor_index];
        }
    }
    return NULL;
}

/**
 * @brief 获取注册表中电机数量
 */
size_t module_motor_registry_get_count(const module_motor_registry_t *const me)
{
    return ((me != NULL) && me->is_initialized) ? me->motor_count : 0U;
}

/* ======================== 电机操作 ======================== */

/**
 * @brief 使能电机
 * @param me 电机对象
 * @return 执行状态
 * @note 检查注册、故障状态和反馈在线状态
 */
module_motor_status_t module_motor_enable(module_motor_t *const me)
{
    module_motor_status_t status = module_motor_validate_registered(me);

    // 故障状态不能使能
    if ((status == MODULE_MOTOR_STATUS_OK) && (me->state == MODULE_MOTOR_STATE_FAULT))
    {
        return MODULE_MOTOR_STATUS_FEEDBACK_UNAVAILABLE;
    }
    // 反馈离线不能使能
    if ((status == MODULE_MOTOR_STATUS_OK) && !me->feedback.is_online)
    {
        return MODULE_MOTOR_STATUS_FEEDBACK_UNAVAILABLE;
    }
    return (status == MODULE_MOTOR_STATUS_OK) ? me->vptr->enable(me) : status;
}

/**
 * @brief 禁用电机
 */
module_motor_status_t module_motor_disable(module_motor_t *const me)
{
    module_motor_status_t status = module_motor_validate_registered(me);
    return (status == MODULE_MOTOR_STATUS_OK) ? me->vptr->disable(me) : status;
}

/**
 * @brief 清除故障状态
 * @param me 电机对象
 * @return 执行状态
 * @note 只有反馈在线时才能清除故障
 */
module_motor_status_t module_motor_clear_fault(module_motor_t *const me)
{
    module_motor_status_t status = module_motor_validate_registered(me);

    if (status != MODULE_MOTOR_STATUS_OK)
    {
        return status;
    }
    // 反馈离线不能清除故障
    if (!me->feedback.is_online)
    {
        return MODULE_MOTOR_STATUS_FEEDBACK_UNAVAILABLE;
    }
    if (me->state == MODULE_MOTOR_STATE_FAULT)
    {
        me->state = MODULE_MOTOR_STATE_DISABLED;   // 恢复到禁用状态
    }
    return MODULE_MOTOR_STATUS_OK;
}

/**
 * @brief 设置目标值
 */
module_motor_status_t module_motor_set_target(module_motor_t *const me, float target_value)
{
    module_motor_status_t status = module_motor_validate_registered(me);
    return (status == MODULE_MOTOR_STATUS_OK) ? me->vptr->set_target(me, target_value) : status;
}

/**
 * @brief 周期更新电机
 * @param me 电机对象
 * @param delta_time_s 时间步长（秒）
 * @return 执行状态
 * @note 若反馈离线且使能，自动进入故障状态
 */
module_motor_status_t module_motor_update(module_motor_t *const me, float delta_time_s)
{
    module_motor_status_t status = module_motor_validate_registered(me);

    // 使能状态下反馈离线 → 进入故障
    if ((status == MODULE_MOTOR_STATUS_OK) && (me->state == MODULE_MOTOR_STATE_ENABLED) &&
        !me->feedback.is_online)
    {
        return module_motor_enter_feedback_fault(me);
    }
    // 检查时间步长有效性
    if ((status == MODULE_MOTOR_STATUS_OK) && (!isfinite(delta_time_s) || (delta_time_s <= 0.0F)))
    {
        return MODULE_MOTOR_STATUS_OUT_OF_RANGE;
    }
    return (status == MODULE_MOTOR_STATUS_OK) ? me->vptr->update(me, delta_time_s) : status;
}

/* ======================== 反馈管理 ======================== */

/**
 * @brief 设置反馈超时时间
 * @param me 电机对象
 * @param feedback_timeout_ms 超时时间（毫秒），0 表示禁用
 */
module_motor_status_t module_motor_set_feedback_timeout(module_motor_t *const me,
                                                        uint32_t feedback_timeout_ms)
{
    if (me == NULL)
    {
        return MODULE_MOTOR_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return MODULE_MOTOR_STATUS_NOT_INITIALIZED;
    }
    me->feedback_timeout_ms = feedback_timeout_ms;
    return MODULE_MOTOR_STATUS_OK;
}

/**
 * @brief 更新反馈超时计时
 * @param me 电机对象
 * @param elapsed_time_ms 距上次调用的时间（毫秒）
 * @return 执行状态
 * @note 需周期性调用（由任务或健康管理器驱动）
 */
module_motor_status_t module_motor_update_feedback_time(module_motor_t *const me,
                                                        uint32_t elapsed_time_ms)
{
    if (me == NULL)
    {
        return MODULE_MOTOR_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return MODULE_MOTOR_STATUS_NOT_INITIALIZED;
    }
    if (!me->feedback.is_online)
    {
        return MODULE_MOTOR_STATUS_OK;
    }

    // 累加时间（防溢出）
    if (elapsed_time_ms > (UINT32_MAX - me->feedback.elapsed_time_since_update_ms))
    {
        me->feedback.elapsed_time_since_update_ms = UINT32_MAX;
    }
    else
    {
        me->feedback.elapsed_time_since_update_ms += elapsed_time_ms;
    }

    // 检查超时
    if ((me->feedback_timeout_ms > 0U) &&
        (me->feedback.elapsed_time_since_update_ms >= me->feedback_timeout_ms))
    {
        me->feedback.is_online = false;
        // 若使能状态，进入故障
        if (me->state == MODULE_MOTOR_STATE_ENABLED)
        {
            return module_motor_enter_feedback_fault(me);
        }
    }
    return MODULE_MOTOR_STATUS_OK;
}

/**
 * @brief 通知反馈已更新（由派生类在解析反馈后调用）
 * @param me 电机对象
 * @return 执行状态
 * @note 重置超时计时，标记在线，递增更新计数
 */
module_motor_status_t module_motor_notify_feedback(module_motor_t *const me)
{
    if (me == NULL)
    {
        return MODULE_MOTOR_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return MODULE_MOTOR_STATUS_NOT_INITIALIZED;
    }
    // 重置超时计时
    me->feedback.elapsed_time_since_update_ms = 0U;
    me->feedback.is_online = true;

    // 递增更新计数（防溢出）
    if (me->feedback.update_count != UINT32_MAX)
    {
        ++me->feedback.update_count;
    }
    return MODULE_MOTOR_STATUS_OK;
}

/**
 * @brief 获取反馈数据指针
 * @param me 电机对象
 * @return 反馈指针，未注册或未初始化则返回 NULL
 */
const module_motor_feedback_t *module_motor_get_feedback(const module_motor_t *const me)
{
    return (module_motor_validate_registered(me) == MODULE_MOTOR_STATUS_OK) ? &me->feedback : NULL;
}
```

---

# 通用电机基类 (module_motor) —— 完整使用指南

## 1. 模块概述

`module_motor_t` 为所有电机对象提供统一的注册、使能、失能、目标设置、周期更新和反馈接口。派生类只需实现虚表中的四个函数（`enable`、`disable`、`set_target`、`update`），即可获得完整的生命周期管理和反馈超时检测能力。

**核心功能**：

- 统一的电机生命周期管理（初始化 → 注册 → 使能 → 更新 → 禁用 → 注销）
- 注册表管理（按 `registration_key` 查找和去重）
- 反馈超时检测（自动进入故障状态）
- 故障状态管理（防止离线时误使能）
- 反馈数据统一存储（位置、速度、扭矩、电流、温度）

**设计哲学**：

- **关注点分离**：基类管理生命周期和状态，派生类管理具体通信协议。
- **安全优先**：反馈离线时自动禁用输出，防止失控。
- **零动态内存**：所有存储由调用者分配。

## 2. 设计边界

| **模块负责**                               | **模块不负责**                       |
| :----------------------------------------- | :----------------------------------- |
| 注册、使能、禁用、目标设置、更新的统一接口 | 具体通信协议（CAN/SPI/UART）实现     |
| 反馈超时检测和故障管理                     | 反馈数据的采集和解析（由派生类完成） |
| 注册表管理和查找                           | PID 控制算法（由派生类或上层实现）   |
| 状态机管理（DISABLED → ENABLED → FAULT）   | 电机参数的具体语义                   |

## 3. 对象关系

```text
module_device_t                    (设备基类：vptr、logical_name)
└── module_motor_t                 (电机基类：状态、反馈、注册信息)
    ├── module_dji_motor_t         (大疆电机)
    ├── module_dm_motor_t          (达妙电机)
    └── module_xxx_motor_t         (其他电机)
```

## 4. 核心概念

### 4.1 注册表

注册表是一个简单的电机容器，用于：

- 按 `registration_key` 查找电机
- 防止重复注册
- 获取电机数量

### 4.2 反馈超时

- 派生类每次收到有效反馈时调用 `module_motor_notify_feedback()`
- 基类通过 `module_motor_update_feedback_time()` 累积时间
- 超时后 `feedback.is_online = false`
- 若电机处于 `ENABLED` 状态，自动进入 `FAULT` 状态

### 4.3 状态机

```text
                    +-------------------+
                    |   DISABLED        |
                    |   (无输出)        |
                    +--------+----------+
                             | enable()
                             v
                    +--------+----------+
                    |   ENABLED         |
                    |   (允许输出)      |
                    +--------+----------+
                             |
                             | 反馈超时/错误
                             v
                    +--------+----------+
                    |   FAULT           |
                    |   (输出被禁止)    |
                    +-------------------+
                             |
                             | clear_fault()
                             v
                    +--------+----------+
                    |   DISABLED        |
                    +-------------------+
```

## 5. API 参考

| 函数                                | 说明           | 返回值                                         |
| :---------------------------------- | :------------- | :--------------------------------------------- |
| `module_motor_init_base`            | 初始化电机基类 | `OK` / `INVALID_ARGUMENT`                      |
| `module_motor_registry_init`        | 初始化注册表   | `OK` / `INVALID_ARGUMENT`                      |
| `module_motor_registry_register`    | 注册电机       | `OK` / `DUPLICATE_KEY` / `NO_RESOURCE`         |
| `module_motor_registry_unregister`  | 注销电机       | `OK` / `NOT_REGISTERED`                        |
| `module_motor_registry_find`        | 查找电机       | 电机指针 / `NULL`                              |
| `module_motor_registry_get_count`   | 获取电机数量   | 数量                                           |
| `module_motor_enable`               | 使能电机       | `OK` / `FEEDBACK_UNAVAILABLE`                  |
| `module_motor_disable`              | 禁用电机       | `OK` / `NOT_REGISTERED`                        |
| `module_motor_clear_fault`          | 清除故障       | `OK` / `FEEDBACK_UNAVAILABLE`                  |
| `module_motor_set_target`           | 设置目标值     | `OK` / `NOT_REGISTERED`                        |
| `module_motor_update`               | 周期更新       | `OK` / `OUT_OF_RANGE` / `FEEDBACK_UNAVAILABLE` |
| `module_motor_set_feedback_timeout` | 设置超时       | `OK` / `NOT_INITIALIZED`                       |
| `module_motor_update_feedback_time` | 更新超时计时   | `OK` / `FEEDBACK_UNAVAILABLE`                  |
| `module_motor_notify_feedback`      | 通知反馈更新   | `OK` / `NOT_INITIALIZED`                       |
| `module_motor_get_feedback`         | 获取反馈指针   | 反馈指针 / `NULL`                              |

## 6. 使用示例

### 6.1 定义派生类

```c
// 派生类头文件
typedef struct {
    module_motor_t super;
    bsp_can_t *can;
    uint32_t can_id;
    float target_value;
    int16_t command;
} module_xxx_motor_t;

// 派生类 .c 文件
static module_motor_status_t xxx_enable(module_motor_t *base) {
    // 发送使能命令
    return MODULE_MOTOR_STATUS_OK;
}

static module_motor_status_t xxx_disable(module_motor_t *base) {
    module_xxx_motor_t *me = MODULE_MOTOR_CONTAINER_OF(base, module_xxx_motor_t, super);
    me->command = 0;
    return MODULE_MOTOR_STATUS_OK;
}

static module_motor_status_t xxx_set_target(module_motor_t *base, float target) {
    module_xxx_motor_t *me = MODULE_MOTOR_CONTAINER_OF(base, module_xxx_motor_t, super);
    me->target_value = target;
    return MODULE_MOTOR_STATUS_OK;
}

static module_motor_status_t xxx_update(module_motor_t *base, float dt) {
    module_xxx_motor_t *me = MODULE_MOTOR_CONTAINER_OF(base, module_xxx_motor_t, super);
    // 计算命令值并发送
    return MODULE_MOTOR_STATUS_OK;
}

static const module_motor_ops_t s_ops = {
    .enable = xxx_enable,
    .disable = xxx_disable,
    .set_target = xxx_set_target,
    .update = xxx_update,
};
```

### 6.2 初始化与注册

```c
static module_xxx_motor_t motor;
static module_motor_registry_t registry;
static module_motor_t *motor_storage[8];

module_motor_registry_init(&registry, motor_storage, 8);

module_xxx_config_t cfg = {
    .logical_name = "motor_1",
    .registration_key = 1,
    .can = can_ptr,
    .can_id = 0x100,
};

module_xxx_init(&motor, &cfg);   // 内部调用 module_motor_init_base
module_motor_registry_register(&registry, &motor.super);

// 设置反馈超时（100ms）
module_motor_set_feedback_timeout(&motor.super, 100);
```

### 6.3 使能与控制

```c
module_motor_t *motor_base = &motor.super;

// 使能（需要反馈在线）
if (module_motor_enable(motor_base) == MODULE_MOTOR_STATUS_OK) {
    module_motor_set_target(motor_base, 1.57F);  // 目标位置 90°
}

// 周期更新（10ms 控制循环）
void control_loop(float dt) {
    // 更新反馈超时
    module_motor_update_feedback_time(motor_base, (uint32_t)(dt * 1000));
    // 更新电机（计算输出）
    module_motor_update(motor_base, dt);
}
```

### 6.4 反馈处理（在 CAN 回调中）

```c
void can_rx_callback(const bsp_can_frame_t *frame) {
    // 解析反馈数据
    motor.super.feedback.position_rad = ...;
    motor.super.feedback.velocity_rad_per_s = ...;
    // 通知基类反馈已更新
    module_motor_notify_feedback(&motor.super);
}
```

### 6.5 故障恢复

```c
// 检查反馈状态
const module_motor_feedback_t *fb = module_motor_get_feedback(&motor.super);
if (!fb->is_online) {
    // 反馈离线，等待恢复
    return;
}

// 清除故障并重新使能
if (module_motor_clear_fault(&motor.super) == MODULE_MOTOR_STATUS_OK) {
    module_motor_enable(&motor.super);
}
```

## 7. 反馈数据结构详解

| 字段                           | 类型     | 说明                                       |
| :----------------------------- | :------- | :----------------------------------------- |
| `position_rad`                 | float    | 位置（弧度）                               |
| `velocity_rad_per_s`           | float    | 速度（rad/s）                              |
| `torque_nm`                    | float    | 扭矩（Nm）                                 |
| `current_a`                    | float    | 电流（A），需 `is_current_a_valid` 为 true |
| `motor_temperature_c`          | float    | 电机温度（°C）                             |
| `current_raw`                  | int16_t  | 原始电流值（协议特定）                     |
| `raw_position`                 | uint32_t | 原始位置值（协议特定）                     |
| `update_count`                 | uint32_t | 反馈更新次数（可用于诊断）                 |
| `elapsed_time_since_update_ms` | uint32_t | 距上次反馈的时间（基类管理）               |
| `is_current_a_valid`           | bool     | `current_a` 是否有效                       |
| `is_online`                    | bool     | 是否在线（基类管理）                       |

## 8. 错误码速查

| 错误码                 | 触发场景                       |
| :--------------------- | :----------------------------- |
| `INVALID_ARGUMENT`     | 参数为空                       |
| `NOT_INITIALIZED`      | 对象未初始化                   |
| `NOT_REGISTERED`       | 电机未注册到注册表             |
| `ALREADY_REGISTERED`   | 重复注册                       |
| `DUPLICATE_KEY`        | 注册键值重复                   |
| `NO_RESOURCE`          | 注册表已满                     |
| `OUT_OF_RANGE`         | `delta_time_s <= 0` 或非有限数 |
| `UNSUPPORTED`          | 操作不支持（未使用）           |
| `TRANSPORT_ERROR`      | 通信错误（由派生类返回）       |
| `FEEDBACK_UNAVAILABLE` | 反馈离线或故障                 |

## 9. 建议验证测试项

- [ ] 注册表初始化和注册/注销功能
- [ ] 重复键值检测
- [ ] 注册表满时返回 `NO_RESOURCE`
- [ ] 使能前检查反馈在线状态
- [ ] 反馈超时后自动进入故障
- [ ] 故障状态下无法使能
- [ ] 清除故障后恢复 `DISABLED` 状态
- [ ] `update_feedback_time` 溢出保护
- [ ] `notify_feedback` 重置超时计时
- [ ] 两个电机独立管理

---

**总结**：`module_motor` 为所有电机类型提供了统一的基类和生命周期管理，通过注册表实现多电机管理，通过反馈超时检测确保安全。派生类只需实现四个虚函数即可获得完整的电机管理能力，适合各种电机驱动方案的统一调度。
