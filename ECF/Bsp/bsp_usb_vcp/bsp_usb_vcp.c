/**
 * @file bsp_usb_vcp.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief USB CDC 虚拟串口通用抽象层实现
 * @version 1.0
 * @date 2026-07-27
 * @copyright Copyright (c) 2026
 *
 * @note USB CDC 虚拟串口抽象，向 Module 提供异步字节流，
 *       而不暴露 USB Device 中间件类型（如 USBD_CDC_HandleTypeDef）。
 *       端点号、描述符、USB Device 类对象和中间件回调均留在平台 Port。
 */

#include "bsp_usb_vcp.h" // 包含 USB VCP 抽象层头文件

BSP_STATIC_ASSERT_SUPER_FIRST(bsp_usb_vcp_device_t);
#include <stddef.h>      // 提供 NULL

/**
 * @brief 从基类指针获取派生设备对象（非常量版本）
 * @param usb_vcp_base bsp_usb_vcp_t 基类指针
 * @return 对应的 bsp_usb_vcp_device_t 对象指针
 */
static bsp_usb_vcp_device_t *bsp_usb_vcp_get_device(bsp_usb_vcp_t *const usb_vcp_base)
{
    // 利用 container_of 宏从基类成员地址反推出包含它的结构体地址
    return BSP_CONTAINER_OF(usb_vcp_base, bsp_usb_vcp_device_t, super);
}

/**
 * @brief 从基类指针获取派生设备对象（常量版本）
 * @param usb_vcp_base const bsp_usb_vcp_t 指针
 * @return 对应的 const bsp_usb_vcp_device_t 指针
 */
static const bsp_usb_vcp_device_t *
bsp_usb_vcp_get_device_const(const bsp_usb_vcp_t *const usb_vcp_base)
{
    return BSP_CONTAINER_OF_CONST(usb_vcp_base, bsp_usb_vcp_device_t, super);
}

/**
 * @brief 从基类虚表指针获取高层操作表
 * @param usb_vcp_base const bsp_usb_vcp_t 指针
 * @return 对应的 bsp_usb_vcp_ops_t 操作表指针（只读）
 */
static const bsp_usb_vcp_ops_t *bsp_usb_vcp_get_ops(const bsp_usb_vcp_t *const usb_vcp_base)
{
    // 基类 bsp_device_t 的 vptr 指向 bsp_usb_vcp_ops_t 中的 super 成员
    return BSP_CONTAINER_OF_CONST(usb_vcp_base->super.vptr, bsp_usb_vcp_ops_t, super);
}

/**
 * @brief 校验 USB VCP 对象是否有效且已初始化
 * @param me bsp_usb_vcp_t 指针
 * @return 状态，成功则 BSP_STATUS_OK
 */
static bsp_status_t bsp_usb_vcp_validate(const bsp_usb_vcp_t *const me)
{
    if (me == NULL)
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    // 调用底层 device 的初始化状态检查
    return bsp_device_is_initialized(&me->super) ? BSP_STATUS_OK : BSP_STATUS_NOT_INITIALIZED;
}

/**
 * @brief USB VCP 设备反初始化（作为 device 层的 deinit 回调）
 * @param device_base bsp_device_t 基类指针
 * @return 执行状态
 */
static bsp_status_t bsp_usb_vcp_device_deinit(bsp_device_t *const device_base)
{
    // 从 device 基类反推出 bsp_usb_vcp_t 基类地址
    bsp_usb_vcp_t *const usb_vcp_base = BSP_CONTAINER_OF(device_base, bsp_usb_vcp_t, super);
    bsp_usb_vcp_device_t *const me = bsp_usb_vcp_get_device(usb_vcp_base);
    // 如果驱动没有提供 deinit，视为无需清理，直接成功
    if (me->driver_ops->deinit == NULL)
    {
        return BSP_STATUS_OK;
    }
    // 调用底层驱动的 deinit，传入设备句柄
    return me->driver_ops->deinit(device_base->device_handle);
}

/**
 * @brief 发送数据（转发至底层驱动）
 * @param usb_vcp_base 基类指针
 * @param transmit_data 发送数据指针
 * @param data_size 数据大小（字节）
 * @param timeout_ms 超时时间（毫秒）
 * @return 执行状态
 * @note USB CDC 通常不会立即复制发送数组，调用者必须让发送缓冲区保持有效直到发送完成通知
 */
static bsp_status_t bsp_usb_vcp_device_transmit(bsp_usb_vcp_t *const usb_vcp_base,
                                                const uint8_t *transmit_data, size_t data_size,
                                                uint32_t timeout_ms)
{
    bsp_usb_vcp_device_t *const me = bsp_usb_vcp_get_device(usb_vcp_base);
    // 调用驱动层的 transmit，传入设备句柄、数据和超时
    return me->driver_ops->transmit(usb_vcp_base->super.device_handle, transmit_data, data_size,
                                    timeout_ms);
}

/**
 * @brief 接收数据（转发至底层驱动）
 * @param usb_vcp_base 基类指针
 * @param receive_data 接收缓冲区指针
 * @param data_capacity 缓冲区容量
 * @return 执行状态
 * @note 接收是异步的：平台收到 USB 数据后完成缓存处理，再通过回调通知实际长度
 */
static bsp_status_t bsp_usb_vcp_device_receive(bsp_usb_vcp_t *const usb_vcp_base,
                                               uint8_t *receive_data, size_t data_capacity)
{
    bsp_usb_vcp_device_t *const me = bsp_usb_vcp_get_device(usb_vcp_base);
    // 调用驱动层的 receive，传入设备句柄和缓冲区
    return me->driver_ops->receive(usb_vcp_base->super.device_handle, receive_data, data_capacity);
}

/**
 * @brief 中止当前事务（转发至底层驱动，可选）
 * @param usb_vcp_base 基类指针
 * @return 若驱动未实现则返回 BSP_STATUS_UNSUPPORTED
 */
static bsp_status_t bsp_usb_vcp_device_abort(bsp_usb_vcp_t *const usb_vcp_base)
{
    bsp_usb_vcp_device_t *const me = bsp_usb_vcp_get_device(usb_vcp_base);
    return (me->driver_ops->abort != NULL)
               ? me->driver_ops->abort(usb_vcp_base->super.device_handle)
               : BSP_STATUS_UNSUPPORTED;
}

/**
 * @brief 查询 USB 主机连接状态（转发至底层驱动，可选）
 * @param usb_vcp_base 基类指针（const）
 * @param is_connected 输出是否已连接（已枚举）
 * @return 若驱动未实现则返回 BSP_STATUS_UNSUPPORTED
 */
static bsp_status_t bsp_usb_vcp_device_get_connected(const bsp_usb_vcp_t *const usb_vcp_base,
                                                     bool *is_connected)
{
    const bsp_usb_vcp_device_t *const me = bsp_usb_vcp_get_device_const(usb_vcp_base);
    return (me->driver_ops->get_connected != NULL)
               ? me->driver_ops->get_connected(usb_vcp_base->super.device_handle, is_connected)
               : BSP_STATUS_UNSUPPORTED;
}

/**
 * @brief 查询 USB 发送是否忙（转发至底层驱动，可选）
 * @param usb_vcp_base 基类指针（const）
 * @param is_busy 输出是否忙
 * @return 若驱动未实现则返回 BSP_STATUS_UNSUPPORTED
 */
static bsp_status_t bsp_usb_vcp_device_get_busy(const bsp_usb_vcp_t *const usb_vcp_base,
                                                bool *is_busy)
{
    const bsp_usb_vcp_device_t *const me = bsp_usb_vcp_get_device_const(usb_vcp_base);
    return (me->driver_ops->get_busy != NULL)
               ? me->driver_ops->get_busy(usb_vcp_base->super.device_handle, is_busy)
               : BSP_STATUS_UNSUPPORTED;
}

/* 定义 USB VCP 设备层的操作表（虚表），将所有转发函数填入 */
static const bsp_usb_vcp_ops_t s_bsp_usb_vcp_device_ops = {
    .super = {.deinit = bsp_usb_vcp_device_deinit},    // 继承自 device 的 deinit
    .transmit = bsp_usb_vcp_device_transmit,           // 发送转发
    .receive = bsp_usb_vcp_device_receive,             // 接收转发
    .abort = bsp_usb_vcp_device_abort,                 // 中止转发（可选）
    .get_connected = bsp_usb_vcp_device_get_connected, // 连接状态查询转发（可选）
    .get_busy = bsp_usb_vcp_device_get_busy,           // 忙状态查询转发（可选）
};

/**
 * @brief 初始化 USB VCP 设备实例
 * @param me 设备对象指针
 * @param config 配置参数指针
 * @return 执行状态
 */
bsp_status_t bsp_usb_vcp_init(bsp_usb_vcp_device_t *const me,
                              const bsp_usb_vcp_config_t *const config)
{
    bsp_status_t status;
    // 参数合法性检查：对象、配置、设备句柄、驱动表、必须实现 transmit/receive
    if ((me == NULL) || (config == NULL) || (config->device_handle == NULL) ||
        (config->driver_ops == NULL) || (config->driver_ops->transmit == NULL) ||
        (config->driver_ops->receive == NULL))
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
    return bsp_device_init(&me->super.super, &s_bsp_usb_vcp_device_ops.super,
                           config->device_handle);
}

/**
 * @brief 将派生对象转为基类指针（向上转型）
 * @param me 派生对象指针
 * @return 基类指针，若输入为空则返回 NULL
 */
bsp_usb_vcp_t *bsp_usb_vcp_as_base(bsp_usb_vcp_device_t *const me)
{
    return (me != NULL) ? &me->super : NULL;
}

/**
 * @brief 设置 USB VCP 事件回调函数
 * @param me 基类指针
 * @param callback 回调函数指针
 * @param user_context 用户上下文
 * @return 执行状态
 */
bsp_status_t bsp_usb_vcp_set_callback(bsp_usb_vcp_t *const me, bsp_event_callback_t callback,
                                      void *user_context)
{
    bsp_status_t status = bsp_usb_vcp_validate(me);
    if (status == BSP_STATUS_OK)
    {
        me->callback = callback;
        me->user_context = user_context;
    }
    return status;
}

/**
 * @brief 发送数据（公共接口）
 * @param me 基类指针
 * @param transmit_data 发送数据指针
 * @param data_size 数据大小（字节），必须大于 0
 * @param timeout_ms 超时时间（毫秒）
 * @return 执行状态
 * @note USB 发送是异步的，忙时返回 BSP_STATUS_BUSY
 *       调用者必须让发送缓冲区保持有效直到发送完成通知
 */
bsp_status_t bsp_usb_vcp_transmit(bsp_usb_vcp_t *const me, const uint8_t *transmit_data,
                                  size_t data_size, uint32_t timeout_ms)
{
    bsp_status_t status = bsp_usb_vcp_validate(me);
    if (status != BSP_STATUS_OK)
    {
        return status;
    }
    // 数据指针非空、大小非零检查
    if ((transmit_data == NULL) || (data_size == 0U))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    // 通过虚表调用 transmit
    return bsp_usb_vcp_get_ops(me)->transmit(me, transmit_data, data_size, timeout_ms);
}

/**
 * @brief 接收数据（公共接口）
 * @param me 基类指针
 * @param receive_data 接收缓冲区指针
 * @param data_capacity 缓冲区容量，必须大于 0
 * @return 执行状态
 * @note 接收是异步的：平台收到数据后通过回调通知
 *       如果已有待接收数据，返回 BSP_STATUS_BUSY
 */
bsp_status_t bsp_usb_vcp_receive(bsp_usb_vcp_t *const me, uint8_t *receive_data,
                                 size_t data_capacity)
{
    bsp_status_t status = bsp_usb_vcp_validate(me);
    if (status != BSP_STATUS_OK)
    {
        return status;
    }
    // 数据指针非空、容量非零检查
    if ((receive_data == NULL) || (data_capacity == 0U))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    // 通过虚表调用 receive
    return bsp_usb_vcp_get_ops(me)->receive(me, receive_data, data_capacity);
}

/**
 * @brief 中止当前事务（公共接口）
 * @param me 基类指针
 * @return 执行状态，若驱动未实现则返回 BSP_STATUS_UNSUPPORTED
 */
bsp_status_t bsp_usb_vcp_abort(bsp_usb_vcp_t *const me)
{
    bsp_status_t status = bsp_usb_vcp_validate(me);
    // 校验通过后通过虚表调用 abort（可能返回 UNSUPPORTED）
    return (status == BSP_STATUS_OK) ? bsp_usb_vcp_get_ops(me)->abort(me) : status;
}

/**
 * @brief 查询 USB 主机连接状态（公共接口）
 * @param me 基类指针（const）
 * @param is_connected 输出是否已连接（已枚举）
 * @return 执行状态，若驱动未实现则返回 BSP_STATUS_UNSUPPORTED
 * @note USB 枚举可能在 MCU 初始化后才完成，发送前应检查连接状态
 */
bsp_status_t bsp_usb_vcp_get_connected(const bsp_usb_vcp_t *const me, bool *is_connected)
{
    bsp_status_t status = bsp_usb_vcp_validate(me);
    // 输出指针非空检查
    if ((status == BSP_STATUS_OK) && (is_connected == NULL))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    // 通过虚表调用 get_connected（可能返回 UNSUPPORTED）
    return (status == BSP_STATUS_OK) ? bsp_usb_vcp_get_ops(me)->get_connected(me, is_connected)
                                     : status;
}

/**
 * @brief 查询 USB 发送是否忙（公共接口）
 * @param me 基类指针（const）
 * @param is_busy 输出是否忙
 * @return 执行状态，若驱动未实现则返回 BSP_STATUS_UNSUPPORTED
 */
bsp_status_t bsp_usb_vcp_get_busy(const bsp_usb_vcp_t *const me, bool *is_busy)
{
    bsp_status_t status = bsp_usb_vcp_validate(me);
    // 输出指针非空检查
    if ((status == BSP_STATUS_OK) && (is_busy == NULL))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    // 通过虚表调用 get_busy（可能返回 UNSUPPORTED）
    return (status == BSP_STATUS_OK) ? bsp_usb_vcp_get_ops(me)->get_busy(me, is_busy) : status;
}

/**
 * @brief 事件通知函数（由底层驱动在中断或回调中调用）
 * @param me 基类指针
 * @param event 事件类型
 * @param status 状态码
 * @param transferred_size 已传输的数据量（字节数）
 */
void bsp_usb_vcp_notify(bsp_usb_vcp_t *const me, bsp_event_t event, bsp_status_t status,
                        size_t transferred_size)
{
    if ((me != NULL) && bsp_device_is_initialized(&me->super) && (me->callback != NULL))
    {
        me->callback(event, status, transferred_size, me->user_context);
    }
}
