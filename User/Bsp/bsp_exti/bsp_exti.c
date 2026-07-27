/**
 * @file bsp_exti.c
 * @brief 外部中断通用抽象层实现
 * @note 提供 EXTI 中断源的启用/禁用、回调注册和事件通知功能。
 */

#include "bsp_exti.h" // 包含 EXTI 抽象层头文件
#include <stddef.h>   // 提供 NULL

/**
 * @brief 从基类指针获取派生设备对象（非常量版本）
 * @param me bsp_exti_t 基类指针
 * @return 对应的 bsp_exti_device_t 对象指针
 */
static bsp_exti_device_t *bsp_exti_get_device(bsp_exti_t *const me)
{
    // 利用 container_of 宏从基类成员地址反推出包含它的结构体地址
    return BSP_CONTAINER_OF(me, bsp_exti_device_t, super);
}

/**
 * @brief 从基类虚表指针获取高层操作表
 * @param me bsp_exti_t 基类指针（const）
 * @return 对应的 bsp_exti_ops_t 操作表指针（只读）
 */
static const bsp_exti_ops_t *bsp_exti_get_ops(const bsp_exti_t *const me)
{
    // 基类 bsp_device_t 的 vptr 指向 bsp_exti_ops_t 中的 super 成员
    return BSP_CONTAINER_OF_CONST(me->super.vptr, bsp_exti_ops_t, super);
}

/**
 * @brief EXTI 设备反初始化（作为 device 层的 deinit 回调）
 * @param device_base bsp_device_t 基类指针
 * @return 执行状态
 */
static bsp_status_t bsp_exti_device_deinit(bsp_device_t *const device_base)
{
    // 从 device 基类反推出 bsp_exti_t 基类地址
    bsp_exti_t *const exti_base = BSP_CONTAINER_OF(device_base, bsp_exti_t, super);
    // 获取派生设备对象
    bsp_exti_device_t *const me = bsp_exti_get_device(exti_base);
    // 如果驱动没有提供 deinit，视为无需清理，直接成功
    return (me->driver_ops->deinit != NULL) ? me->driver_ops->deinit(device_base->device_handle)
                                            : BSP_STATUS_OK;
}

/**
 * @brief 启用外部中断（转发至底层驱动）
 * @param exti_base 基类指针
 * @return 执行状态
 */
static bsp_status_t bsp_exti_device_enable(bsp_exti_t *const exti_base)
{
    bsp_exti_device_t *const me = bsp_exti_get_device(exti_base);
    // 调用驱动层的 enable，传入设备句柄
    return me->driver_ops->enable(exti_base->super.device_handle);
}

/**
 * @brief 禁用外部中断（转发至底层驱动）
 * @param exti_base 基类指针
 * @return 执行状态
 */
static bsp_status_t bsp_exti_device_disable(bsp_exti_t *const exti_base)
{
    bsp_exti_device_t *const me = bsp_exti_get_device(exti_base);
    // 调用驱动层的 disable，传入设备句柄
    return me->driver_ops->disable(exti_base->super.device_handle);
}

/* 定义 EXTI 设备层的操作表（虚表），将所有转发函数填入 */
static const bsp_exti_ops_t s_bsp_exti_device_ops = {
    .super = {.deinit = bsp_exti_device_deinit}, // 继承自 device 的 deinit
    .enable = bsp_exti_device_enable,            // 启用转发
    .disable = bsp_exti_device_disable           // 禁用转发
};

/**
 * @brief 初始化 EXTI 设备实例
 * @param me 设备对象指针
 * @param config 配置参数指针
 * @return 执行状态
 */
bsp_status_t bsp_exti_init(bsp_exti_device_t *const me, const bsp_exti_config_t *const config)
{
    bsp_status_t status;

    // 参数合法性检查：对象、配置、设备句柄、驱动表、必须实现 enable/disable
    if ((me == NULL) || (config == NULL) || (config->device_handle == NULL) ||
        (config->driver_ops == NULL) || (config->driver_ops->enable == NULL) ||
        (config->driver_ops->disable == NULL))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    // 预先标记为未初始化，避免中途失败时留下错误状态
    me->super.super.is_initialized = false;
    // 保存底层驱动操作表
    me->driver_ops = config->driver_ops;
    // 如果驱动提供了 init 回调，则调用以初始化硬件
    if (config->driver_ops->init != NULL)
    {
        status = config->driver_ops->init(config->device_handle);
        if (status != BSP_STATUS_OK)
        {
            return status; // 底层初始化失败则直接返回
        }
    }
    // 设置回调函数和用户上下文
    me->super.callback = config->callback;
    me->super.user_context = config->user_context;
    // 调用 device 基类初始化，注册虚表并保存设备句柄
    return bsp_device_init(&me->super.super, &s_bsp_exti_device_ops.super, config->device_handle);
}

/**
 * @brief 将派生对象转为基类指针（向上转型）
 * @param me 派生对象指针
 * @return 基类指针，若输入为空则返回 NULL
 */
bsp_exti_t *bsp_exti_as_base(bsp_exti_device_t *const me)
{
    return (me != NULL) ? &me->super : NULL;
}

/**
 * @brief 设置 EXTI 事件回调函数（运行时更换）
 * @param me 基类指针
 * @param callback 回调函数指针（可为 NULL）
 * @param user_context 用户上下文
 * @return 执行状态
 * @note 若中断可能并发发生，建议先禁用中断再更换回调
 */
bsp_status_t bsp_exti_set_callback(bsp_exti_t *const me, bsp_exti_callback_t callback,
                                   void *user_context)
{
    // 参数校验：对象非空
    if (me == NULL)
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    // 检查对象是否已初始化
    if (!bsp_device_is_initialized(&me->super))
    {
        return BSP_STATUS_NOT_INITIALIZED;
    }
    // 更新回调函数和用户上下文
    me->callback = callback;
    me->user_context = user_context;
    return BSP_STATUS_OK;
}

/**
 * @brief 启用外部中断（公共接口）
 * @param me 基类指针
 * @return 执行状态
 */
bsp_status_t bsp_exti_enable(bsp_exti_t *const me)
{
    // 参数校验：对象非空
    if (me == NULL)
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    // 检查对象是否已初始化
    if (!bsp_device_is_initialized(&me->super))
    {
        return BSP_STATUS_NOT_INITIALIZED;
    }
    // 通过虚表调用 enable
    return bsp_exti_get_ops(me)->enable(me);
}

/**
 * @brief 禁用外部中断（公共接口）
 * @param me 基类指针
 * @return 执行状态
 */
bsp_status_t bsp_exti_disable(bsp_exti_t *const me)
{
    // 参数校验：对象非空
    if (me == NULL)
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    // 检查对象是否已初始化
    if (!bsp_device_is_initialized(&me->super))
    {
        return BSP_STATUS_NOT_INITIALIZED;
    }
    // 通过虚表调用 disable
    return bsp_exti_get_ops(me)->disable(me);
}

/**
 * @brief 中断通知函数（由底层驱动在 ISR 中调用）
 * @param me 基类指针
 * @note 仅在对象有效且回调非空时调用回调
 *       回调运行在 ISR 上下文中，必须遵循中断编程规范
 */
void bsp_exti_notify(bsp_exti_t *const me)
{
    // 只有在对象有效、已初始化且回调非空时才调用
    if ((me != NULL) && bsp_device_is_initialized(&me->super) && (me->callback != NULL))
    {
        // 调用用户注册的回调，传入对象自身和用户上下文
        me->callback(me, me->user_context);
    }
}