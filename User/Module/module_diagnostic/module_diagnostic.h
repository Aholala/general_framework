/**
 * @file module_diagnostic.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 统一健康诊断注册表头文件
 * @version 1.0
 * @date 2026-07-28
 * @copyright Copyright (c) 2026
 *
 * @note 各 BSP、模块或控制器只提供“当前是否健康”的只读探针；
 *       诊断对象负责故障确认、防抖恢复、严重度聚合、发生次数、锁存与事件通知。
 *       探针不得阻塞或修改设备状态。
 */

#ifndef MODULE_DIAGNOSTIC_H
#define MODULE_DIAGNOSTIC_H

#include "module_device.h" // 设备基类

#ifdef __cplusplus
extern "C"
{
#endif

    /* ======================== 严重等级枚举 ======================== */

    /**
     * @brief 诊断严重等级（由低到高）
     */
    typedef enum
    {
        MODULE_DIAGNOSTIC_SEVERITY_INFO = 0, // 信息（正常状态）
        MODULE_DIAGNOSTIC_SEVERITY_WARNING,  // 警告（需关注）
        MODULE_DIAGNOSTIC_SEVERITY_ERROR,    // 错误（功能受限）
        MODULE_DIAGNOSTIC_SEVERITY_FATAL     // 致命（需立即处理）
    } module_diagnostic_severity_t;

    /* ======================== 探针函数类型 ======================== */

    /**
     * @brief 健康探针函数类型
     * @param user_context 用户上下文
     * @param[out] detail_code 详细错误码（可输出额外信息）
     * @return true=健康（无故障），false=故障
     * @note 探针应快速执行，不得阻塞或修改设备状态。
     *       返回值仅表示当前瞬间的健康状态，诊断模块会进行防抖确认。
     */
    typedef bool (*module_diagnostic_probe_t)(void *user_context, uint32_t *detail_code);

    /* ======================== 诊断条目结构体 ======================== */

    /**
     * @brief 诊断条目（由调用者静态定义）
     */
    typedef struct
    {
        uint16_t diagnostic_id;                // 诊断 ID（唯一标识）
        module_diagnostic_severity_t severity; // 严重等级
        module_diagnostic_probe_t probe;       // 健康探针函数
        void *user_context;                    // 探针用户上下文
        uint32_t confirmation_time_ms;         // 故障确认时间（毫秒），防抖
        uint32_t recovery_time_ms;             // 恢复确认时间（毫秒），防抖
        bool is_latched;                       // 是否锁存（故障发生后保持，需手动清除）
    } module_diagnostic_entry_t;

    /* ======================== 诊断状态结构体 ======================== */

    /**
     * @brief 诊断运行状态（由诊断模块维护）
     */
    typedef struct
    {
        uint32_t detail_code;              // 最近一次探针返回的详细错误码
        uint32_t fault_elapsed_time_ms;    // 故障已持续累积时间（毫秒）
        uint32_t recovery_elapsed_time_ms; // 恢复已持续累积时间（毫秒）
        uint32_t occurrence_count;         // 故障发生次数（每次从健康→故障计数一次）
        bool is_active;                    // 当前是否处于活动故障状态
        bool is_latched;                   // 是否已被锁存（可由调用者清除）
    } module_diagnostic_state_t;

    /* ======================== 事件回调类型 ======================== */

    /**
     * @brief 诊断事件回调（状态变化时调用）
     * @param entry 对应的诊断条目
     * @param state 对应的诊断状态
     * @param became_active true=变为故障，false=变为健康
     * @param user_context 事件回调用户上下文
     * @note 回调在 module_diagnostic_update 中执行（任务上下文）
     */
    typedef void (*module_diagnostic_event_callback_t)(const module_diagnostic_entry_t *entry,
                                                       const module_diagnostic_state_t *state,
                                                       bool became_active, void *user_context);

    /* ======================== 配置结构体 ======================== */

    /**
     * @brief 诊断模块配置
     */
    typedef struct
    {
        const module_diagnostic_entry_t *entries;          // 诊断条目数组（静态常量）
        module_diagnostic_state_t *state_storage;          // 状态存储数组（调用者分配）
        size_t entry_count;                                // 条目数量
        module_diagnostic_event_callback_t event_callback; // 事件回调（可为 NULL）
        void *event_user_context;                          // 事件回调用户上下文
        const char *logical_name;                          // 逻辑名称
        uint32_t registration_key;                         // 注册键值
    } module_diagnostic_config_t;

    /* ======================== 对象结构体 ======================== */

    /**
     * @brief 诊断设备对象
     */
    typedef struct
    {
        module_device_t super;                                // 设备基类
        const module_diagnostic_entry_t *entries;             // 诊断条目表（引用外部）
        module_diagnostic_state_t *states;                    // 状态数组（引用外部）
        size_t entry_count;                                   // 条目数量
        module_diagnostic_event_callback_t event_callback;    // 事件回调
        void *event_user_context;                             // 事件回调用户上下文
        module_diagnostic_severity_t highest_active_severity; // 当前最高活动故障严重等级
        uint32_t active_count;                                // 当前活动故障数量
        bool is_started;                                      // 是否已启动
    } module_diagnostic_t;

    /* ======================== 公共 API ======================== */

    /**
     * @brief 初始化诊断模块
     * @param me 诊断对象
     * @param config 配置参数
     * @return 执行状态
     * @note 检查条目有效性（probe 非空、严重等级合法、ID 唯一）
     */
    module_device_status_t module_diagnostic_init(module_diagnostic_t *me,
                                                  const module_diagnostic_config_t *config);

    /**
     * @brief 清除锁存标志（仅当故障已恢复时）
     * @param me 诊断对象
     * @param diagnostic_id 诊断 ID
     * @return 执行状态（若故障仍活动则返回 OPERATION_FAILED）
     */
    module_device_status_t module_diagnostic_clear_latched(module_diagnostic_t *me,
                                                           uint16_t diagnostic_id);

    /**
     * @brief 获取指定诊断的当前状态
     * @param me 诊断对象
     * @param diagnostic_id 诊断 ID
     * @return 状态指针，若未找到则返回 NULL
     */
    const module_diagnostic_state_t *module_diagnostic_get_state(const module_diagnostic_t *me,
                                                                 uint16_t diagnostic_id);

    /**
     * @brief 检查是否存在达到或超过指定严重等级的故障
     * @param me 诊断对象
     * @param minimum_severity 最低严重等级
     * @return true=存在，false=不存在
     */
    bool module_diagnostic_has_severity(const module_diagnostic_t *me,
                                        module_diagnostic_severity_t minimum_severity);

#ifdef __cplusplus
}
#endif

#endif /* MODULE_DIAGNOSTIC_H */