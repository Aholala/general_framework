/**
 * @file module_device.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 模块设备基类实现
 * @version 1.0
 * @date 2026-07-28
 * @copyright Copyright (c) 2026
 *
 * @note 提供设备对象的两阶段构造、虚表分派、状态查询等基础设施。
 */

#include "module_device.h"

#include <stddef.h> // NULL

/**
 * @brief 初始化基类（第一阶段）
 */
module_device_status_t module_device_init_base(module_device_t *const me,
                                               const module_device_ops_t *const vptr,
                                               const char *const logical_name,
                                               uint32_t registration_key)
{
    // 参数校验：对象、名称和统一生命周期契约必须完整
    if ((me == NULL) || (vptr == NULL) || (logical_name == NULL) || (vptr->start == NULL) ||
        (vptr->stop == NULL) || (vptr->update == NULL))
    {
        return MODULE_DEVICE_STATUS_INVALID_ARGUMENT;
    }
    // 填充基类字段
    me->vptr = vptr;
    me->logical_name = logical_name;
    me->registration_key = registration_key;
    me->object_magic = MODULE_DEVICE_OBJECT_MAGIC; // 写入魔数
    me->is_initialized = false;                    // 尚未完成初始化
    return MODULE_DEVICE_STATUS_OK;
}

/**
 * @brief 完成初始化（第二阶段）
 */
module_device_status_t module_device_complete_init(module_device_t *const me)
{
    // 参数校验：对象非空，魔数正确，虚表和名称存在
    if ((me == NULL) || (me->object_magic != MODULE_DEVICE_OBJECT_MAGIC) || (me->vptr == NULL) ||
        (me->logical_name == NULL))
    {
        return MODULE_DEVICE_STATUS_INVALID_ARGUMENT;
    }
    // 防止重复初始化
    if (me->is_initialized)
    {
        return MODULE_DEVICE_STATUS_ALREADY_INITIALIZED;
    }
    me->is_initialized = true;
    return MODULE_DEVICE_STATUS_OK;
}

/**
 * @brief 中止初始化（清理状态）
 */
void module_device_abort_init(module_device_t *const me)
{
    if (me == NULL)
    {
        return;
    }
    // 清除所有字段，留下确定的未初始化对象
    me->vptr = NULL;
    me->logical_name = NULL;
    me->registration_key = 0U;
    me->object_magic = 0U;
    me->is_initialized = false;
}

/**
 * @brief 启动设备（调用虚表 start）
 */
module_device_status_t module_device_start(module_device_t *const me)
{
    if (me == NULL)
    {
        return MODULE_DEVICE_STATUS_INVALID_ARGUMENT;
    }
    if (!module_device_is_initialized(me))
    {
        return MODULE_DEVICE_STATUS_NOT_INITIALIZED;
    }
    // 检查虚表 start 是否实现
    if (me->vptr->start == NULL)
    {
        return MODULE_DEVICE_STATUS_UNSUPPORTED;
    }
    return me->vptr->start(me);
}

/**
 * @brief 停止设备（调用虚表 stop）
 */
module_device_status_t module_device_stop(module_device_t *const me)
{
    if (me == NULL)
    {
        return MODULE_DEVICE_STATUS_INVALID_ARGUMENT;
    }
    if (!module_device_is_initialized(me))
    {
        return MODULE_DEVICE_STATUS_NOT_INITIALIZED;
    }
    if (me->vptr->stop == NULL)
    {
        return MODULE_DEVICE_STATUS_UNSUPPORTED;
    }
    return me->vptr->stop(me);
}

/**
 * @brief 更新设备（调用虚表 update）
 */
module_device_status_t module_device_update(module_device_t *const me, uint32_t elapsed_time_ms)
{
    if (me == NULL)
    {
        return MODULE_DEVICE_STATUS_INVALID_ARGUMENT;
    }
    if (!module_device_is_initialized(me))
    {
        return MODULE_DEVICE_STATUS_NOT_INITIALIZED;
    }
    if (me->vptr->update == NULL)
    {
        return MODULE_DEVICE_STATUS_UNSUPPORTED;
    }
    return me->vptr->update(me, elapsed_time_ms);
}

/**
 * @brief 检查设备是否已初始化
 */
bool module_device_is_initialized(const module_device_t *const me)
{
    return (me != NULL) && (me->object_magic == MODULE_DEVICE_OBJECT_MAGIC) && (me->vptr != NULL) &&
           (me->logical_name != NULL) && me->is_initialized;
}

/**
 * @brief 获取逻辑名称
 */
const char *module_device_get_logical_name(const module_device_t *const me)
{
    return module_device_is_initialized(me) ? me->logical_name : NULL;
}

/**
 * @brief 获取注册键值
 */
uint32_t module_device_get_registration_key(const module_device_t *const me)
{
    return module_device_is_initialized(me) ? me->registration_key : 0U;
}
