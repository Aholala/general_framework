/**
 * @file bsp_gpio.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 通用数字 GPIO 抽象层实现
 * @note 提供 GPIO 的读、写、翻转操作，不保存引脚号/端口号，由平台句柄封装。
 * @version 1.0
 * @date 2026-07-27
 *
 * @copyright Copyright (c) 2026
 *
 */

#include <stddef.h> 
#include "bsp_gpio.h"

BSP_STATIC_ASSERT_SUPER_FIRST(bsp_gpio_device_t);

/**
 * @brief 从基类指针获取派生设备对象（非常量版本）
 * @param me bsp_gpio_t 基类指针
 * @return 对应的 bsp_gpio_device_t 对象指针
 */
    static bsp_gpio_device_t *bsp_gpio_get_device(bsp_gpio_t *const me)
{
    // 利用 container_of 宏从基类成员地址反推出包含它的结构体地址
    return BSP_CONTAINER_OF(me, bsp_gpio_device_t, super);
}

/**
 * @brief 从基类指针获取派生设备对象（常量版本）
 * @param me const bsp_gpio_t 指针
 * @return 对应的 const bsp_gpio_device_t 指针
 */
static const bsp_gpio_device_t *bsp_gpio_get_device_const(const bsp_gpio_t *const me)
{
    return BSP_CONTAINER_OF_CONST(me, bsp_gpio_device_t, super);
}

/**
 * @brief 从基类虚表指针获取高层操作表
 * @param me const bsp_gpio_t 指针
 * @return 对应的 bsp_gpio_ops_t 操作表指针（只读）
 */
static const bsp_gpio_ops_t *bsp_gpio_get_ops(const bsp_gpio_t *const me)
{
    // 基类 bsp_device_t 的 vptr 指向 bsp_gpio_ops_t 中的 super 成员
    return BSP_CONTAINER_OF_CONST(me->super.vptr, bsp_gpio_ops_t, super);
}

/**
 * @brief GPIO 设备反初始化（作为 device 层的 deinit 回调）
 * @param device_base bsp_device_t 基类指针
 * @return 执行状态
 */
static bsp_status_t bsp_gpio_device_deinit(bsp_device_t *const device_base)
{
    // 从 device 基类反推出 bsp_gpio_t 基类地址
    bsp_gpio_t *const gpio_base = BSP_CONTAINER_OF(device_base, bsp_gpio_t, super);
    // 获取派生设备对象
    bsp_gpio_device_t *const me = bsp_gpio_get_device(gpio_base);
    // 如果驱动没有提供 deinit，视为无需清理，直接成功
    return (me->driver_ops->deinit != NULL) ? me->driver_ops->deinit(device_base->device_handle)
                                            : BSP_STATUS_OK;
}

/**
 * @brief 读取 GPIO 电平（转发至底层驱动）
 * @param gpio_base 基类指针（const）
 * @param is_high 输出逻辑电平（true=高，false=低）
 * @return 执行状态
 */
static bsp_status_t bsp_gpio_device_read(const bsp_gpio_t *const gpio_base, bool *is_high)
{
    const bsp_gpio_device_t *const me = bsp_gpio_get_device_const(gpio_base);
    // 调用驱动层的 read，传入设备句柄和输出指针
    return me->driver_ops->read(gpio_base->super.device_handle, is_high);
}

/**
 * @brief 写入 GPIO 电平（转发至底层驱动）
 * @param gpio_base 基类指针
 * @param is_high 逻辑电平（true=高，false=低）
 * @return 若驱动未实现 write 则返回 BSP_STATUS_UNSUPPORTED
 */
static bsp_status_t bsp_gpio_device_write(bsp_gpio_t *const gpio_base, bool is_high)
{
    bsp_gpio_device_t *const me = bsp_gpio_get_device(gpio_base);
    // 检查底层是否实现了 write，否则返回不支持
    return (me->driver_ops->write != NULL)
               ? me->driver_ops->write(gpio_base->super.device_handle, is_high)
               : BSP_STATUS_UNSUPPORTED;
}

/**
 * @brief 翻转 GPIO 电平（转发至底层驱动）
 * @param gpio_base 基类指针
 * @return 若驱动未实现 toggle 则返回 BSP_STATUS_UNSUPPORTED
 */
static bsp_status_t bsp_gpio_device_toggle(bsp_gpio_t *const gpio_base)
{
    bsp_gpio_device_t *const me = bsp_gpio_get_device(gpio_base);
    // 检查底层是否实现了 toggle，否则返回不支持
    return (me->driver_ops->toggle != NULL) ? me->driver_ops->toggle(gpio_base->super.device_handle)
                                            : BSP_STATUS_UNSUPPORTED;
}

/* 定义 GPIO 设备层的操作表（虚表），将所有转发函数填入 */
static const bsp_gpio_ops_t s_bsp_gpio_device_ops = {
    .super = {.deinit = bsp_gpio_device_deinit}, // 继承自 device 的 deinit
    .read = bsp_gpio_device_read,                // 读取转发
    .write = bsp_gpio_device_write,              // 写入转发
    .toggle = bsp_gpio_device_toggle,            // 翻转转发
};

/**
 * @brief 校验 GPIO 对象是否有效且已初始化
 * @param me bsp_gpio_t 指针
 * @return 状态，成功则 BSP_STATUS_OK
 */
static bsp_status_t bsp_gpio_validate(const bsp_gpio_t *const me)
{
    if (me == NULL)
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    // 调用底层 device 的初始化状态检查
    return bsp_device_is_initialized(&me->super) ? BSP_STATUS_OK : BSP_STATUS_NOT_INITIALIZED;
}

/**
 * @brief 初始化 GPIO 设备实例
 * @param me 设备对象指针
 * @param config 配置参数指针
 * @return 执行状态
 */
bsp_status_t bsp_gpio_init(bsp_gpio_device_t *const me, const bsp_gpio_config_t *const config)
{
    bsp_status_t status;

    // 参数合法性检查：对象、配置、设备句柄、驱动表、必须实现 read
    if ((me == NULL) || (config == NULL) || (config->device_handle == NULL) ||
        (config->driver_ops == NULL) || (config->driver_ops->read == NULL))
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
    return bsp_device_init(&me->super.super, &s_bsp_gpio_device_ops.super, config->device_handle);
}

/**
 * @brief 将派生对象转为基类指针（向上转型）
 * @param me 派生对象指针
 * @return 基类指针，若输入为空则返回 NULL
 */
bsp_gpio_t *bsp_gpio_as_base(bsp_gpio_device_t *const me)
{
    return (me != NULL) ? &me->super : NULL;
}

/**
 * @brief 读取 GPIO 逻辑电平（公共接口）
 * @param me 基类指针（const）
 * @param is_high 输出逻辑电平（true=高，false=低）
 * @return 执行状态
 */
bsp_status_t bsp_gpio_read(const bsp_gpio_t *const me, bool *is_high)
{
    const bsp_status_t status = bsp_gpio_validate(me);

    if (status != BSP_STATUS_OK)
    {
        return status;
    }
    // 输出指针非空检查
    if (is_high == NULL)
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    // 通过虚表调用 read
    return bsp_gpio_get_ops(me)->read(me, is_high);
}

/**
 * @brief 写入 GPIO 逻辑电平（公共接口）
 * @param me 基类指针
 * @param is_high 逻辑电平（true=高，false=低）
 * @return 执行状态
 */
bsp_status_t bsp_gpio_write(bsp_gpio_t *const me, bool is_high)
{
    const bsp_status_t status = bsp_gpio_validate(me);
    // 校验通过后通过虚表调用 write
    return (status == BSP_STATUS_OK) ? bsp_gpio_get_ops(me)->write(me, is_high) : status;
}

/**
 * @brief 翻转 GPIO 逻辑电平（公共接口）
 * @param me 基类指针
 * @return 执行状态
 */
bsp_status_t bsp_gpio_toggle(bsp_gpio_t *const me)
{
    const bsp_status_t status = bsp_gpio_validate(me);
    // 校验通过后通过虚表调用 toggle
    return (status == BSP_STATUS_OK) ? bsp_gpio_get_ops(me)->toggle(me) : status;
}
