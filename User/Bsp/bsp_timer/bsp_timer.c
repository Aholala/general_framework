/**
 * @file bsp_timer.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 通用基本定时器抽象层实现
 * @version 1.0
 * @date 2026-07-25
 * @copyright Copyright (c) 2026
 *
 * @note 封装启动、停止、计数器、周期、时钟频率和周期到期通知。
 *       预分频、时钟树、自动重装模式和 IRQ 优先级由平台端配置。
 */

#include "bsp_timer.h" // 包含定时器抽象层头文件
#include <stddef.h>    // 提供 NULL

/**
 * @brief 从基类指针获取派生设备对象（非常量版本）
 * @param timer_base bsp_timer_t 基类指针
 * @return 对应的 bsp_timer_device_t 对象指针
 */
static bsp_timer_device_t *bsp_timer_get_device(bsp_timer_t *const timer_base)
{
    // 利用 container_of 宏从基类成员地址反推出包含它的结构体地址
    return BSP_CONTAINER_OF(timer_base, bsp_timer_device_t, super);
}

/**
 * @brief 从基类指针获取派生设备对象（常量版本）
 * @param timer_base const bsp_timer_t 指针
 * @return 对应的 const bsp_timer_device_t 指针
 */
static const bsp_timer_device_t *bsp_timer_get_device_const(const bsp_timer_t *const timer_base)
{
    return BSP_CONTAINER_OF_CONST(timer_base, bsp_timer_device_t, super);
}

/**
 * @brief 从基类虚表指针获取高层操作表
 * @param timer_base const bsp_timer_t 指针
 * @return 对应的 bsp_timer_ops_t 操作表指针（只读）
 */
static const bsp_timer_ops_t *bsp_timer_get_ops(const bsp_timer_t *const timer_base)
{
    // 基类 bsp_device_t 的 vptr 指向 bsp_timer_ops_t 中的 super 成员
    return BSP_CONTAINER_OF_CONST(timer_base->super.vptr, bsp_timer_ops_t, super);
}

/**
 * @brief 定时器设备反初始化（作为 device 层的 deinit 回调）
 * @param device_base bsp_device_t 基类指针
 * @return 执行状态
 */
static bsp_status_t bsp_timer_device_deinit(bsp_device_t *const device_base)
{
    // 从 device 基类反推出 bsp_timer_t 基类地址
    bsp_timer_t *const timer_base = BSP_CONTAINER_OF(device_base, bsp_timer_t, super);
    bsp_timer_device_t *const me = bsp_timer_get_device(timer_base);
    // 如果驱动没有提供 deinit，视为无需清理，直接成功
    if (me->driver_ops->deinit == NULL)
    {
        return BSP_STATUS_OK;
    }
    // 调用底层驱动的 deinit，传入设备句柄
    return me->driver_ops->deinit(device_base->device_handle);
}

/**
 * @brief 宏：生成无额外参数的转发函数（start/stop）
 */
#define BSP_TIMER_FORWARD(name, member)                                                            \
    static bsp_status_t name(bsp_timer_t *const timer_base)                                        \
    {                                                                                              \
        bsp_timer_device_t *const me = bsp_timer_get_device(timer_base);                           \
        return me->driver_ops->member(timer_base->super.device_handle);                            \
    }

// 生成启动转发函数
BSP_TIMER_FORWARD(bsp_timer_device_start, start)
// 生成停止转发函数
BSP_TIMER_FORWARD(bsp_timer_device_stop, stop)

/**
 * @brief 设置计数器值（转发至底层驱动）
 * @param timer_base 基类指针
 * @param counter_ticks 计数器值（tick）
 * @return 执行状态
 */
static bsp_status_t bsp_timer_device_set_counter(bsp_timer_t *const timer_base,
                                                 uint32_t counter_ticks)
{
    bsp_timer_device_t *const me = bsp_timer_get_device(timer_base);
    return me->driver_ops->set_counter(timer_base->super.device_handle, counter_ticks);
}

/**
 * @brief 获取当前计数器值（转发至底层驱动）
 * @param timer_base 基类指针（const）
 * @param counter_ticks 输出计数器值
 * @return 执行状态
 */
static bsp_status_t bsp_timer_device_get_counter(const bsp_timer_t *const timer_base,
                                                 uint32_t *counter_ticks)
{
    const bsp_timer_device_t *const me = bsp_timer_get_device_const(timer_base);
    return me->driver_ops->get_counter(timer_base->super.device_handle, counter_ticks);
}

/**
 * @brief 设置定时器周期（转发至底层驱动）
 * @param timer_base 基类指针
 * @param period_ticks 周期值（tick）
 * @return 执行状态
 */
static bsp_status_t bsp_timer_device_set_period(bsp_timer_t *const timer_base,
                                                uint32_t period_ticks)
{
    bsp_timer_device_t *const me = bsp_timer_get_device(timer_base);
    return me->driver_ops->set_period(timer_base->super.device_handle, period_ticks);
}

/**
 * @brief 获取定时器周期（转发至底层驱动）
 * @param timer_base 基类指针（const）
 * @param period_ticks 输出周期值
 * @return 执行状态
 */
static bsp_status_t bsp_timer_device_get_period(const bsp_timer_t *const timer_base,
                                                uint32_t *period_ticks)
{
    const bsp_timer_device_t *const me = bsp_timer_get_device_const(timer_base);
    return me->driver_ops->get_period(timer_base->super.device_handle, period_ticks);
}

/**
 * @brief 获取定时器时钟频率（转发至底层驱动）
 * @param timer_base 基类指针（const）
 * @param frequency_hz 输出频率（Hz）
 * @return 执行状态
 */
static bsp_status_t bsp_timer_device_get_frequency(const bsp_timer_t *const timer_base,
                                                   uint32_t *frequency_hz)
{
    const bsp_timer_device_t *const me = bsp_timer_get_device_const(timer_base);
    return me->driver_ops->get_frequency(timer_base->super.device_handle, frequency_hz);
}

/* 定义定时器设备层的操作表（虚表），将所有转发函数填入 */
static const bsp_timer_ops_t s_bsp_timer_device_ops = {
    .super = {.deinit = bsp_timer_device_deinit},   // 继承自 device 的 deinit
    .start = bsp_timer_device_start,                // 启动转发
    .stop = bsp_timer_device_stop,                  // 停止转发
    .set_counter = bsp_timer_device_set_counter,    // 设置计数器转发
    .get_counter = bsp_timer_device_get_counter,    // 获取计数器转发
    .set_period = bsp_timer_device_set_period,      // 设置周期转发
    .get_period = bsp_timer_device_get_period,      // 获取周期转发
    .get_frequency = bsp_timer_device_get_frequency // 获取频率转发
};

/**
 * @brief 校验定时器对象是否有效且已初始化
 * @param me bsp_timer_t 指针
 * @return 状态，成功则 BSP_STATUS_OK
 */
static bsp_status_t bsp_timer_validate(const bsp_timer_t *const me)
{
    if (me == NULL)
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    // 调用底层 device 的初始化状态检查
    return bsp_device_is_initialized(&me->super) ? BSP_STATUS_OK : BSP_STATUS_NOT_INITIALIZED;
}

/**
 * @brief 初始化定时器设备实例
 * @param me 设备对象指针
 * @param config 配置参数指针
 * @return 执行状态
 */
bsp_status_t bsp_timer_init(bsp_timer_device_t *const me, const bsp_timer_config_t *const config)
{
    bsp_status_t status;
    // 参数合法性检查：对象、配置、设备句柄、驱动表、所有关键操作必须实现
    if ((me == NULL) || (config == NULL) || (config->device_handle == NULL) ||
        (config->driver_ops == NULL) || (config->driver_ops->start == NULL) ||
        (config->driver_ops->stop == NULL) || (config->driver_ops->set_counter == NULL) ||
        (config->driver_ops->get_counter == NULL) || (config->driver_ops->set_period == NULL) ||
        (config->driver_ops->get_period == NULL) || (config->driver_ops->get_frequency == NULL))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    // 预先标记为未初始化，避免中途失败时留下错误状态
    me->super.super.is_initialized = false;
    // 保存底层驱动操作表
    me->driver_ops = config->driver_ops;
    // 如果驱动提供了 init 回调，则调用以初始化硬件
    if (me->driver_ops->init != NULL)
    {
        status = me->driver_ops->init(config->device_handle);
        if (status != BSP_STATUS_OK)
        {
            return status; // 底层初始化失败则直接返回
        }
    }
    // 设置回调函数和用户上下文
    me->super.callback = config->callback;
    me->super.user_context = config->user_context;
    // 调用 device 基类初始化，注册虚表并保存设备句柄
    return bsp_device_init(&me->super.super, &s_bsp_timer_device_ops.super, config->device_handle);
}

/**
 * @brief 将派生对象转为基类指针（向上转型）
 * @param me 派生对象指针
 * @return 基类指针，若输入为空则返回 NULL
 */
bsp_timer_t *bsp_timer_as_base(bsp_timer_device_t *const me)
{
    return (me != NULL) ? &me->super : NULL;
}

/**
 * @brief 设置定时器事件回调（公共接口）
 * @param me 基类指针
 * @param callback 回调函数指针
 * @param user_context 用户上下文
 * @return 执行状态
 * @note 回调函数在 ISR 上下文中执行，必须快速返回，不可阻塞
 */
bsp_status_t bsp_timer_set_callback(bsp_timer_t *const me, bsp_timer_callback_t callback,
                                    void *user_context)
{
    bsp_status_t status = bsp_timer_validate(me);
    if (status == BSP_STATUS_OK)
    {
        me->callback = callback;
        me->user_context = user_context;
    }
    return status;
}

/**
 * @brief 启动定时器（公共接口）
 * @param me 基类指针
 * @return 执行状态
 */
bsp_status_t bsp_timer_start(bsp_timer_t *const me)
{
    bsp_status_t status = bsp_timer_validate(me);
    return (status == BSP_STATUS_OK) ? bsp_timer_get_ops(me)->start(me) : status;
}

/**
 * @brief 停止定时器（公共接口）
 * @param me 基类指针
 * @return 执行状态
 */
bsp_status_t bsp_timer_stop(bsp_timer_t *const me)
{
    bsp_status_t status = bsp_timer_validate(me);
    return (status == BSP_STATUS_OK) ? bsp_timer_get_ops(me)->stop(me) : status;
}

/**
 * @brief 复位计数器为 0（公共接口）
 * @param me 基类指针
 * @return 执行状态
 */
bsp_status_t bsp_timer_reset(bsp_timer_t *const me)
{
    // 复用 set_counter(0)
    return bsp_timer_set_counter(me, 0U);
}

/**
 * @brief 设置计数器值（公共接口）
 * @param me 基类指针
 * @param counter_ticks 计数器值（tick）
 * @return 执行状态
 */
bsp_status_t bsp_timer_set_counter(bsp_timer_t *const me, uint32_t counter_ticks)
{
    bsp_status_t status = bsp_timer_validate(me);
    return (status == BSP_STATUS_OK) ? bsp_timer_get_ops(me)->set_counter(me, counter_ticks)
                                     : status;
}

/**
 * @brief 获取当前计数器值（公共接口）
 * @param me 基类指针（const）
 * @param counter_ticks 输出计数器值
 * @return 执行状态
 */
bsp_status_t bsp_timer_get_counter(const bsp_timer_t *const me, uint32_t *counter_ticks)
{
    bsp_status_t status = bsp_timer_validate(me);
    // 输出指针非空检查
    if ((status == BSP_STATUS_OK) && (counter_ticks == NULL))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    return (status == BSP_STATUS_OK) ? bsp_timer_get_ops(me)->get_counter(me, counter_ticks)
                                     : status;
}

/**
 * @brief 设置定时器周期（公共接口）
 * @param me 基类指针
 * @param period_ticks 周期值（tick），必须大于 0
 * @return 执行状态，若 period_ticks 为 0 则返回 OUT_OF_RANGE
 */
bsp_status_t bsp_timer_set_period(bsp_timer_t *const me, uint32_t period_ticks)
{
    bsp_status_t status = bsp_timer_validate(me);
    // 周期必须大于 0
    if ((status == BSP_STATUS_OK) && (period_ticks == 0U))
    {
        return BSP_STATUS_OUT_OF_RANGE;
    }
    return (status == BSP_STATUS_OK) ? bsp_timer_get_ops(me)->set_period(me, period_ticks) : status;
}

/**
 * @brief 获取当前周期值（公共接口）
 * @param me 基类指针（const）
 * @param period_ticks 输出周期值
 * @return 执行状态
 */
bsp_status_t bsp_timer_get_period(const bsp_timer_t *const me, uint32_t *period_ticks)
{
    bsp_status_t status = bsp_timer_validate(me);
    // 输出指针非空检查
    if ((status == BSP_STATUS_OK) && (period_ticks == NULL))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    return (status == BSP_STATUS_OK) ? bsp_timer_get_ops(me)->get_period(me, period_ticks) : status;
}

/**
 * @brief 获取定时器时钟频率（公共接口）
 * @param me 基类指针（const）
 * @param frequency_hz 输出频率（Hz）
 * @return 执行状态
 */
bsp_status_t bsp_timer_get_frequency(const bsp_timer_t *const me, uint32_t *frequency_hz)
{
    bsp_status_t status = bsp_timer_validate(me);
    // 输出指针非空检查
    if ((status == BSP_STATUS_OK) && (frequency_hz == NULL))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    return (status == BSP_STATUS_OK) ? bsp_timer_get_ops(me)->get_frequency(me, frequency_hz)
                                     : status;
}

/**
 * @brief 定时器到期通知函数（由底层驱动在 ISR 中调用）
 * @param me 基类指针
 * @note 仅在对象有效且回调非空时调用
 *       回调运行在 ISR 上下文中，必须遵循中断编程规范
 */
void bsp_timer_notify_elapsed(bsp_timer_t *const me)
{
    if ((me != NULL) && bsp_device_is_initialized(&me->super) && (me->callback != NULL))
    {
        // 调用用户注册的回调，传入对象自身和用户上下文
        me->callback(me, me->user_context);
    }
}