/**
 * @file module_diagnostic.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 统一健康诊断注册表实现
 * @version 1.0
 * @date 2026-07-28
 * @copyright Copyright (c) 2026
 *
 * @note 诊断模块负责故障确认防抖、恢复防抖、严重度聚合、发生次数、锁存与事件通知。
 *       探针在 update 周期中被调用，不得阻塞或修改设备状态。
 */

#include "module_diagnostic.h"

MODULE_STATIC_ASSERT_SUPER_FIRST(module_diagnostic_t);

/* ======================== 前向声明 ======================== */

static module_device_status_t module_diagnostic_start_device(module_device_t *device);
static module_device_status_t module_diagnostic_stop_device(module_device_t *device);
static module_device_status_t module_diagnostic_update_device(module_device_t *device,
                                                              uint32_t elapsed_time_ms);

/* ======================== 虚表 ======================== */

/**
 * @brief 诊断模块的设备虚表
 * @note start/stop/update 均实现
 */
static const module_device_ops_t s_module_diagnostic_device_ops = {
    .start = module_diagnostic_start_device,
    .stop = module_diagnostic_stop_device,
    .update = module_diagnostic_update_device,
};

/* ======================== 内部工具函数 ======================== */

/**
 * @brief 安全累加时间（防溢出）
 * @param accumulated 已累积时间
 * @param elapsed 要累加的时间
 * @return 累加后的时间（饱和到 UINT32_MAX）
 */
static uint32_t module_diagnostic_add_time(uint32_t accumulated, uint32_t elapsed)
{
    return (elapsed > UINT32_MAX - accumulated) ? UINT32_MAX : accumulated + elapsed;
}

/* ======================== 公共 API ======================== */

/**
 * @brief 初始化诊断模块
 */
module_device_status_t module_diagnostic_init(module_diagnostic_t *me,
                                              const module_diagnostic_config_t *config)
{
    size_t entry_index;

    // ---- 参数校验 ----
    if ((me == NULL) || (config == NULL) || (config->entries == NULL) ||
        (config->state_storage == NULL) || (config->entry_count == 0U))
    {
        return MODULE_DEVICE_STATUS_INVALID_ARGUMENT;
    }

    // ---- 第一阶段构造：初始化基类 ----
    if (module_device_init_base(&me->super, &s_module_diagnostic_device_ops, config->logical_name,
                                config->registration_key) != MODULE_DEVICE_STATUS_OK)
    {
        return MODULE_DEVICE_STATUS_INVALID_ARGUMENT;
    }

    // ---- 校验诊断条目 ----
    for (entry_index = 0U; entry_index < config->entry_count; ++entry_index)
    {
        size_t previous_index;

        // probe 必须非空，严重等级必须在合法范围内
        if ((config->entries[entry_index].probe == NULL) ||
            (config->entries[entry_index].severity > MODULE_DIAGNOSTIC_SEVERITY_FATAL))
        {
            module_device_abort_init(&me->super);
            return MODULE_DEVICE_STATUS_INVALID_ARGUMENT;
        }

        // 检查 diagnostic_id 是否唯一
        for (previous_index = 0U; previous_index < entry_index; ++previous_index)
        {
            if (config->entries[previous_index].diagnostic_id ==
                config->entries[entry_index].diagnostic_id)
            {
                module_device_abort_init(&me->super);
                return MODULE_DEVICE_STATUS_INVALID_ARGUMENT;
            }
        }

        // 初始化状态存储（清零）
        config->state_storage[entry_index] = (module_diagnostic_state_t){0};
    }

    // ---- 保存配置到对象 ----
    me->entries = config->entries;
    me->states = config->state_storage;
    me->entry_count = config->entry_count;
    me->event_callback = config->event_callback;
    me->event_user_context = config->event_user_context;
    me->highest_active_severity = MODULE_DIAGNOSTIC_SEVERITY_INFO;
    me->active_count = 0U;
    me->is_started = false;

    // ---- 第二阶段构造 ----
    return module_device_complete_init(&me->super);
}

/**
 * @brief 清除锁存标志
 */
module_device_status_t module_diagnostic_clear_latched(module_diagnostic_t *me,
                                                       uint16_t diagnostic_id)
{
    size_t entry_index;

    // 状态检查
    if ((me == NULL) || !module_device_is_initialized(&me->super))
    {
        return MODULE_DEVICE_STATUS_NOT_INITIALIZED;
    }

    // 查找对应的诊断条目
    for (entry_index = 0U; entry_index < me->entry_count; ++entry_index)
    {
        if (me->entries[entry_index].diagnostic_id == diagnostic_id)
        {
            // 如果故障仍处于活动状态，不允许清除锁存
            if (me->states[entry_index].is_active)
            {
                return MODULE_DEVICE_STATUS_OPERATION_FAILED;
            }
            me->states[entry_index].is_latched = false;
            return MODULE_DEVICE_STATUS_OK;
        }
    }
    return MODULE_DEVICE_STATUS_INVALID_ARGUMENT; // ID 未找到
}

/**
 * @brief 获取指定诊断的状态
 */
const module_diagnostic_state_t *module_diagnostic_get_state(const module_diagnostic_t *me,
                                                             uint16_t diagnostic_id)
{
    size_t entry_index;

    if ((me == NULL) || !module_device_is_initialized(&me->super))
    {
        return NULL;
    }

    for (entry_index = 0U; entry_index < me->entry_count; ++entry_index)
    {
        if (me->entries[entry_index].diagnostic_id == diagnostic_id)
        {
            return &me->states[entry_index];
        }
    }
    return NULL;
}

/**
 * @brief 检查是否存在达到或超过指定严重等级的故障
 */
bool module_diagnostic_has_severity(const module_diagnostic_t *me,
                                    module_diagnostic_severity_t minimum_severity)
{
    return (me != NULL) && module_device_is_initialized(&me->super) && (me->active_count > 0U) &&
           (me->highest_active_severity >= minimum_severity);
}

/* ======================== 虚函数实现 ======================== */

/**
 * @brief 启动诊断模块（仅设置标志）
 */
static module_device_status_t module_diagnostic_start_device(module_device_t *device)
{
    module_diagnostic_t *const me = MODULE_CONTAINER_OF(device, module_diagnostic_t, super);
    me->is_started = true;
    return MODULE_DEVICE_STATUS_OK;
}

/**
 * @brief 停止诊断模块（仅清除标志）
 */
static module_device_status_t module_diagnostic_stop_device(module_device_t *device)
{
    module_diagnostic_t *const me = MODULE_CONTAINER_OF(device, module_diagnostic_t, super);
    me->is_started = false;
    return MODULE_DEVICE_STATUS_OK;
}

/**
 * @brief 更新所有诊断条目（核心逻辑）
 * @param device 基类指针
 * @param elapsed_time_ms 距上次更新的时间（毫秒）
 * @return 执行状态
 * @note 此函数应在周期任务中调用（建议 10~100ms）
 *       依次调用所有探针，进行防抖确认，更新状态，触发事件回调
 */
static module_device_status_t module_diagnostic_update_device(module_device_t *device,
                                                              uint32_t elapsed_time_ms)
{
    module_diagnostic_t *const me = MODULE_CONTAINER_OF(device, module_diagnostic_t, super);
    size_t entry_index;

    // 检查是否已启动
    if (!me->is_started)
    {
        return MODULE_DEVICE_STATUS_OPERATION_FAILED;
    }

    // 重置聚合统计（重新计算）
    me->active_count = 0U;
    me->highest_active_severity = MODULE_DIAGNOSTIC_SEVERITY_INFO;

    // 遍历所有诊断条目
    for (entry_index = 0U; entry_index < me->entry_count; ++entry_index)
    {
        const module_diagnostic_entry_t *const entry = &me->entries[entry_index];
        module_diagnostic_state_t *const state = &me->states[entry_index];
        uint32_t detail_code = 0U;
        const bool previous_active = state->is_active;

        // 调用探针检查健康状态
        const bool healthy = entry->probe(entry->user_context, &detail_code);
        state->detail_code = detail_code; // 保存详细错误码

        if (!healthy)
        {
            // ---- 故障状态 ----
            state->recovery_elapsed_time_ms = 0U; // 重置恢复计时
            state->fault_elapsed_time_ms =
                module_diagnostic_add_time(state->fault_elapsed_time_ms, elapsed_time_ms);

            // 达到确认时间后判定为活动故障
            if (state->fault_elapsed_time_ms >= entry->confirmation_time_ms)
            {
                state->is_active = true;
                // 若条目配置为锁存，或已经锁存，则保持锁存
                state->is_latched = entry->is_latched || state->is_latched;
                // 如果是从健康变为故障，则增加发生次数
                if (!previous_active)
                {
                    ++state->occurrence_count;
                }
            }
        }
        else
        {
            // ---- 健康状态 ----
            state->fault_elapsed_time_ms = 0U; // 重置故障计时
            state->recovery_elapsed_time_ms =
                module_diagnostic_add_time(state->recovery_elapsed_time_ms, elapsed_time_ms);

            // 达到恢复时间后清除故障标志（除非锁存）
            if (state->recovery_elapsed_time_ms >= entry->recovery_time_ms)
            {
                state->is_active = false;
            }
        }

        // 更新聚合统计
        if (state->is_active)
        {
            ++me->active_count;
            if (entry->severity > me->highest_active_severity)
            {
                me->highest_active_severity = entry->severity;
            }
        }

        // 状态变化时触发事件回调（在任务上下文中执行）
        if ((previous_active != state->is_active) && (me->event_callback != NULL))
        {
            me->event_callback(entry, state, state->is_active, me->event_user_context);
        }
    }

    return MODULE_DEVICE_STATUS_OK;
}
