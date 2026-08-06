/**
 * @file bsp_watchdog.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 硬件看门狗通用抽象层实现
 * @version 1.0
 * @date 2026-07-27
 * @copyright Copyright (c) 2026
 *
 * @note 提供看门狗刷新、实际超时时间查询和看门狗复位来源检测。
 *       通用层不决定独立看门狗或窗口看门狗，也不配置具体寄存器。
 */

#include "bsp_watchdog.h" // 包含看门狗抽象层头文件

BSP_STATIC_ASSERT_SUPER_FIRST(bsp_watchdog_device_t);
#include <stddef.h>       // 提供 NULL

/**
 * @brief 从基类指针获取派生设备对象（非常量版本）
 * @param me bsp_watchdog_t 基类指针
 * @return 对应的 bsp_watchdog_device_t 对象指针
 */
static bsp_watchdog_device_t *bsp_watchdog_get_device(bsp_watchdog_t *const me)
{
    // 利用 container_of 宏从基类成员地址反推出包含它的结构体地址
    return BSP_CONTAINER_OF(me, bsp_watchdog_device_t, super);
}

/**
 * @brief 从基类指针获取派生设备对象（常量版本）
 * @param me const bsp_watchdog_t 指针
 * @return 对应的 const bsp_watchdog_device_t 指针
 */
static const bsp_watchdog_device_t *bsp_watchdog_get_device_const(const bsp_watchdog_t *const me)
{
    return BSP_CONTAINER_OF_CONST(me, bsp_watchdog_device_t, super);
}

/**
 * @brief 从基类虚表指针获取高层操作表
 * @param me const bsp_watchdog_t 指针
 * @return 对应的 bsp_watchdog_ops_t 操作表指针（只读）
 */
static const bsp_watchdog_ops_t *bsp_watchdog_get_ops(const bsp_watchdog_t *const me)
{
    // 基类 bsp_device_t 的 vptr 指向 bsp_watchdog_ops_t 中的 super 成员
    return BSP_CONTAINER_OF_CONST(me->super.vptr, bsp_watchdog_ops_t, super);
}

/**
 * @brief 看门狗设备反初始化（作为 device 层的 deinit 回调）
 * @param device_base bsp_device_t 基类指针
 * @return 执行状态
 * @note 某些 MCU 看门狗启动后无法停止，平台 deinit 可以返回 UNSUPPORTED
 */
static bsp_status_t bsp_watchdog_device_deinit(bsp_device_t *const device_base)
{
    // 从 device 基类反推出 bsp_watchdog_t 基类地址
    bsp_watchdog_t *const watchdog_base = BSP_CONTAINER_OF(device_base, bsp_watchdog_t, super);
    bsp_watchdog_device_t *const me = bsp_watchdog_get_device(watchdog_base);
    // 如果驱动没有提供 deinit，视为无需清理，直接成功
    return (me->driver_ops->deinit != NULL) ? me->driver_ops->deinit(device_base->device_handle)
                                            : BSP_STATUS_OK;
}

/**
 * @brief 刷新看门狗（喂狗，转发至底层驱动）
 * @param watchdog_base 基类指针
 * @return 执行状态
 * @note 必须在允许的时间窗口内调用，否则可能导致复位
 */
static bsp_status_t bsp_watchdog_device_refresh(bsp_watchdog_t *const watchdog_base)
{
    bsp_watchdog_device_t *const me = bsp_watchdog_get_device(watchdog_base);
    // 调用驱动层的 refresh，传入设备句柄
    return me->driver_ops->refresh(watchdog_base->super.device_handle);
}

/**
 * @brief 获取看门狗实际超时时间（转发至底层驱动，可选）
 * @param watchdog_base 基类指针（const）
 * @param timeout_ms 输出超时时间（毫秒）
 * @return 若驱动未实现则返回 BSP_STATUS_UNSUPPORTED
 * @note 超时时间由低速时钟和分频决定，存在器差和温漂
 */
static bsp_status_t bsp_watchdog_device_get_timeout_ms(const bsp_watchdog_t *const watchdog_base,
                                                       uint32_t *timeout_ms)
{
    const bsp_watchdog_device_t *const me = bsp_watchdog_get_device_const(watchdog_base);
    return (me->driver_ops->get_timeout_ms != NULL)
               ? me->driver_ops->get_timeout_ms(watchdog_base->super.device_handle, timeout_ms)
               : BSP_STATUS_UNSUPPORTED;
}

/**
 * @brief 检测上次复位是否由看门狗导致（转发至底层驱动，可选）
 * @param watchdog_base 基类指针（const）
 * @param reset_detected 输出是否由看门狗复位
 * @return 若驱动未实现则返回 BSP_STATUS_UNSUPPORTED
 * @note 应在系统启动早期读取并记录复位原因，再清除硬件复位标志
 */
static bsp_status_t
bsp_watchdog_device_get_reset_detected(const bsp_watchdog_t *const watchdog_base,
                                       bool *reset_detected)
{
    const bsp_watchdog_device_t *const me = bsp_watchdog_get_device_const(watchdog_base);
    return (me->driver_ops->get_reset_detected != NULL)
               ? me->driver_ops->get_reset_detected(watchdog_base->super.device_handle,
                                                    reset_detected)
               : BSP_STATUS_UNSUPPORTED;
}

/* 定义看门狗设备层的操作表（虚表），将所有转发函数填入 */
static const bsp_watchdog_ops_t s_bsp_watchdog_device_ops = {
    .super = {.deinit = bsp_watchdog_device_deinit},              // 继承自 device 的 deinit
    .refresh = bsp_watchdog_device_refresh,                       // 刷新转发
    .get_timeout_ms = bsp_watchdog_device_get_timeout_ms,         // 获取超时转发（可选）
    .get_reset_detected = bsp_watchdog_device_get_reset_detected, // 复位检测转发（可选）
};

/**
 * @brief 校验看门狗对象是否有效且已初始化
 * @param me bsp_watchdog_t 指针
 * @return 状态，成功则 BSP_STATUS_OK
 */
static bsp_status_t bsp_watchdog_validate(const bsp_watchdog_t *const me)
{
    if (me == NULL)
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    // 调用底层 device 的初始化状态检查
    return bsp_device_is_initialized(&me->super) ? BSP_STATUS_OK : BSP_STATUS_NOT_INITIALIZED;
}

/**
 * @brief 初始化看门狗设备实例
 * @param me 设备对象指针
 * @param config 配置参数指针
 * @return 执行状态
 * @note refresh 必须实现，get_timeout_ms/get_reset_detected 为可选
 */
bsp_status_t bsp_watchdog_init(bsp_watchdog_device_t *const me,
                               const bsp_watchdog_config_t *const config)
{
    bsp_status_t status;

    // 参数合法性检查：对象、配置、设备句柄、驱动表、必须实现 refresh
    if ((me == NULL) || (config == NULL) || (config->device_handle == NULL) ||
        (config->driver_ops == NULL) || (config->driver_ops->refresh == NULL))
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
    // 调用 device 基类初始化，注册虚表并保存设备句柄
    return bsp_device_init(&me->super.super, &s_bsp_watchdog_device_ops.super,
                           config->device_handle);
}

/**
 * @brief 将派生对象转为基类指针（向上转型）
 * @param me 派生对象指针
 * @return 基类指针，若输入为空则返回 NULL
 */
bsp_watchdog_t *bsp_watchdog_as_base(bsp_watchdog_device_t *const me)
{
    return (me != NULL) ? &me->super : NULL;
}

/**
 * @brief 刷新看门狗（公共接口）
 * @param me 基类指针
 * @return 执行状态
 * @note 必须在允许的时间窗口内调用
 *       不要在定时器 ISR 中无条件喂狗，应由健康监督任务根据关键任务心跳决定
 */
bsp_status_t bsp_watchdog_refresh(bsp_watchdog_t *const me)
{
    const bsp_status_t status = bsp_watchdog_validate(me);
    // 校验通过后通过虚表调用 refresh
    return (status == BSP_STATUS_OK) ? bsp_watchdog_get_ops(me)->refresh(me) : status;
}

/**
 * @brief 获取看门狗实际超时时间（公共接口）
 * @param me 基类指针（const）
 * @param timeout_ms 输出超时时间（毫秒）
 * @return 执行状态，若驱动未实现则返回 BSP_STATUS_UNSUPPORTED
 */
bsp_status_t bsp_watchdog_get_timeout_ms(const bsp_watchdog_t *const me, uint32_t *timeout_ms)
{
    const bsp_status_t status = bsp_watchdog_validate(me);

    if (status != BSP_STATUS_OK)
    {
        return status;
    }
    // 输出指针非空检查
    if (timeout_ms == NULL)
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    // 通过虚表调用 get_timeout_ms（可能返回 UNSUPPORTED）
    return bsp_watchdog_get_ops(me)->get_timeout_ms(me, timeout_ms);
}

/**
 * @brief 检测上次复位是否由看门狗导致（公共接口）
 * @param me 基类指针（const）
 * @param reset_detected 输出是否由看门狗复位
 * @return 执行状态，若驱动未实现则返回 BSP_STATUS_UNSUPPORTED
 * @note 应在系统启动早期调用，然后清除硬件复位标志
 */
bsp_status_t bsp_watchdog_get_reset_detected(const bsp_watchdog_t *const me, bool *reset_detected)
{
    const bsp_status_t status = bsp_watchdog_validate(me);

    if (status != BSP_STATUS_OK)
    {
        return status;
    }
    // 输出指针非空检查
    if (reset_detected == NULL)
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    // 通过虚表调用 get_reset_detected（可能返回 UNSUPPORTED）
    return bsp_watchdog_get_ops(me)->get_reset_detected(me, reset_detected);
}
