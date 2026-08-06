/**
 * @file bsp_spi.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief SPI 通用抽象层实现
 * @version 1.0
 * @date 2026-07-27
 * @copyright Copyright (c) 2026
 *
 * @note 支持发送、接收、全双工交换、中止和忙状态查询。
 *       三种传输模式（阻塞/中断/DMA）统一使用同一接口。
 */

#include "bsp_spi.h" // 包含 SPI 抽象层头文件

BSP_STATIC_ASSERT_SUPER_FIRST(bsp_spi_device_t);
#include <stddef.h>  // 提供 NULL 和 size_t

/**
 * @brief 从基类指针获取派生设备对象（非常量版本）
 * @param spi_base bsp_spi_t 基类指针
 * @return 对应的 bsp_spi_device_t 对象指针
 */
static bsp_spi_device_t *bsp_spi_get_device(bsp_spi_t *const spi_base)
{
    // 利用 container_of 宏从基类成员地址反推出包含它的结构体地址
    return BSP_CONTAINER_OF(spi_base, bsp_spi_device_t, super);
}

/**
 * @brief 从基类指针获取派生设备对象（常量版本）
 * @param spi_base const bsp_spi_t 指针
 * @return 对应的 const bsp_spi_device_t 指针
 */
static const bsp_spi_device_t *bsp_spi_get_device_const(const bsp_spi_t *const spi_base)
{
    return BSP_CONTAINER_OF_CONST(spi_base, bsp_spi_device_t, super);
}

/**
 * @brief 从基类虚表指针获取高层操作表
 * @param spi_base const bsp_spi_t 指针
 * @return 对应的 bsp_spi_ops_t 操作表指针（只读）
 */
static const bsp_spi_ops_t *bsp_spi_get_ops(const bsp_spi_t *const spi_base)
{
    // 基类 bsp_device_t 的 vptr 指向 bsp_spi_ops_t 中的 super 成员
    return BSP_CONTAINER_OF_CONST(spi_base->super.vptr, bsp_spi_ops_t, super);
}

/**
 * @brief SPI 设备反初始化（作为 device 层的 deinit 回调）
 * @param device_base bsp_device_t 基类指针
 * @return 执行状态
 */
static bsp_status_t bsp_spi_device_deinit(bsp_device_t *const device_base)
{
    // 从 device 基类反推出 bsp_spi_t 基类地址
    bsp_spi_t *const spi_base = BSP_CONTAINER_OF(device_base, bsp_spi_t, super);
    bsp_spi_device_t *const me = bsp_spi_get_device(spi_base);
    // 如果驱动没有提供 deinit，视为无需清理，直接成功
    return (me->driver_ops->deinit != NULL) ? me->driver_ops->deinit(device_base->device_handle)
                                            : BSP_STATUS_OK;
}

/**
 * @brief 发送数据（转发至底层驱动）
 * @param spi_base 基类指针
 * @param transmit_data 发送数据指针
 * @param data_size 数据大小（字节）
 * @param mode 传输模式（阻塞/中断/DMA）
 * @param timeout_ms 超时时间（毫秒）
 * @return 执行状态
 */
static bsp_status_t bsp_spi_device_transmit(bsp_spi_t *const spi_base, const uint8_t *transmit_data,
                                            size_t data_size, bsp_transfer_mode_t mode,
                                            uint32_t timeout_ms)
{
    bsp_spi_device_t *const me = bsp_spi_get_device(spi_base);
    // 调用驱动层的 transmit，传入设备句柄和所有参数
    return me->driver_ops->transmit(spi_base->super.device_handle, transmit_data, data_size, mode,
                                    timeout_ms);
}

/**
 * @brief 接收数据（转发至底层驱动）
 * @param spi_base 基类指针
 * @param receive_data 接收缓冲区指针
 * @param data_size 数据大小（字节）
 * @param mode 传输模式
 * @param timeout_ms 超时时间
 * @return 执行状态
 */
static bsp_status_t bsp_spi_device_receive(bsp_spi_t *const spi_base, uint8_t *receive_data,
                                           size_t data_size, bsp_transfer_mode_t mode,
                                           uint32_t timeout_ms)
{
    bsp_spi_device_t *const me = bsp_spi_get_device(spi_base);
    return me->driver_ops->receive(spi_base->super.device_handle, receive_data, data_size, mode,
                                   timeout_ms);
}

/**
 * @brief 全双工交换（转发至底层驱动，可选）
 * @param spi_base 基类指针
 * @param transmit_data 发送数据指针
 * @param receive_data 接收缓冲区指针
 * @param data_size 数据大小（字节）
 * @param mode 传输模式
 * @param timeout_ms 超时时间
 * @return 若驱动未实现则返回 BSP_STATUS_UNSUPPORTED
 */
static bsp_status_t bsp_spi_device_exchange(bsp_spi_t *const spi_base, const uint8_t *transmit_data,
                                            uint8_t *receive_data, size_t data_size,
                                            bsp_transfer_mode_t mode, uint32_t timeout_ms)
{
    bsp_spi_device_t *const me = bsp_spi_get_device(spi_base);
    // 检查底层是否实现了 exchange，否则返回不支持
    return (me->driver_ops->exchange != NULL)
               ? me->driver_ops->exchange(spi_base->super.device_handle, transmit_data,
                                          receive_data, data_size, mode, timeout_ms)
               : BSP_STATUS_UNSUPPORTED;
}

/**
 * @brief 中止当前事务（转发至底层驱动，可选）
 * @param spi_base 基类指针
 * @return 若驱动未实现则返回 BSP_STATUS_UNSUPPORTED
 */
static bsp_status_t bsp_spi_device_abort(bsp_spi_t *const spi_base)
{
    bsp_spi_device_t *const me = bsp_spi_get_device(spi_base);
    return (me->driver_ops->abort != NULL) ? me->driver_ops->abort(spi_base->super.device_handle)
                                           : BSP_STATUS_UNSUPPORTED;
}

/**
 * @brief 查询总线是否忙（转发至底层驱动，可选）
 * @param spi_base 基类指针（const）
 * @param is_busy 输出是否忙
 * @return 若驱动未实现则返回 BSP_STATUS_UNSUPPORTED
 */
static bsp_status_t bsp_spi_device_get_busy(const bsp_spi_t *const spi_base, bool *is_busy)
{
    const bsp_spi_device_t *const me = bsp_spi_get_device_const(spi_base);
    return (me->driver_ops->get_busy != NULL)
               ? me->driver_ops->get_busy(spi_base->super.device_handle, is_busy)
               : BSP_STATUS_UNSUPPORTED;
}

/* 定义 SPI 设备层的操作表（虚表），将所有转发函数填入 */
static const bsp_spi_ops_t s_bsp_spi_device_ops = {
    .super = {.deinit = bsp_spi_device_deinit}, // 继承自 device 的 deinit
    .transmit = bsp_spi_device_transmit,        // 发送转发
    .receive = bsp_spi_device_receive,          // 接收转发
    .exchange = bsp_spi_device_exchange,        // 全双工交换转发（可选）
    .abort = bsp_spi_device_abort,              // 中止转发（可选）
    .get_busy = bsp_spi_device_get_busy,        // 忙状态查询转发（可选）
};

/**
 * @brief 初始化 SPI 设备实例
 * @param me 设备对象指针
 * @param config 配置参数指针
 * @return 执行状态
 */
bsp_status_t bsp_spi_init(bsp_spi_device_t *const me, const bsp_spi_config_t *const config)
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
    return bsp_device_init(&me->super.super, &s_bsp_spi_device_ops.super, config->device_handle);
}

/**
 * @brief 将派生对象转为基类指针（向上转型）
 * @param me 派生对象指针
 * @return 基类指针，若输入为空则返回 NULL
 */
bsp_spi_t *bsp_spi_as_base(bsp_spi_device_t *const me)
{
    return (me != NULL) ? &me->super : NULL;
}

/**
 * @brief 校验 SPI 对象是否有效且已初始化
 * @param me bsp_spi_t 指针
 * @return 状态，成功则 BSP_STATUS_OK
 */
static bsp_status_t bsp_spi_validate(const bsp_spi_t *const me)
{
    if (me == NULL)
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    return bsp_device_is_initialized(&me->super) ? BSP_STATUS_OK : BSP_STATUS_NOT_INITIALIZED;
}

/**
 * @brief 设置 SPI 事件回调函数
 * @param me 基类指针
 * @param callback 回调函数指针
 * @param user_context 用户上下文
 * @return 执行状态
 */
bsp_status_t bsp_spi_set_callback(bsp_spi_t *const me, bsp_event_callback_t callback,
                                  void *user_context)
{
    const bsp_status_t status = bsp_spi_validate(me);
    if (status != BSP_STATUS_OK)
    {
        return status;
    }
    me->callback = callback;
    me->user_context = user_context;
    return BSP_STATUS_OK;
}

/**
 * @brief 发送数据（公共接口）
 * @param me 基类指针
 * @param data 发送数据指针
 * @param size 数据大小（字节）
 * @param mode 传输模式（阻塞/中断/DMA）
 * @param timeout_ms 超时时间（毫秒）
 * @return 执行状态
 */
bsp_status_t bsp_spi_transmit(bsp_spi_t *const me, const uint8_t *data, size_t size,
                              bsp_transfer_mode_t mode, uint32_t timeout_ms)
{
    const bsp_status_t status = bsp_spi_validate(me);
    if (status != BSP_STATUS_OK)
    {
        return status;
    }
    // 数据指针非空、大小非零、传输模式合法
    if ((data == NULL) || (size == 0U) || !bsp_transfer_mode_is_valid(mode))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    // 通过虚表调用 transmit
    return bsp_spi_get_ops(me)->transmit(me, data, size, mode, timeout_ms);
}

/**
 * @brief 接收数据（公共接口）
 * @param me 基类指针
 * @param data 接收缓冲区指针
 * @param size 数据大小（字节）
 * @param mode 传输模式
 * @param timeout_ms 超时时间
 * @return 执行状态
 */
bsp_status_t bsp_spi_receive(bsp_spi_t *const me, uint8_t *data, size_t size,
                             bsp_transfer_mode_t mode, uint32_t timeout_ms)
{
    const bsp_status_t status = bsp_spi_validate(me);
    if (status != BSP_STATUS_OK)
    {
        return status;
    }
    // 数据指针非空、大小非零、传输模式合法
    if ((data == NULL) || (size == 0U) || !bsp_transfer_mode_is_valid(mode))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    // 通过虚表调用 receive
    return bsp_spi_get_ops(me)->receive(me, data, size, mode, timeout_ms);
}

/**
 * @brief 全双工交换（公共接口）
 * @param me 基类指针
 * @param transmit_data 发送数据指针
 * @param receive_data 接收缓冲区指针
 * @param size 数据大小（字节）
 * @param mode 传输模式
 * @param timeout_ms 超时时间
 * @return 执行状态
 */
bsp_status_t bsp_spi_exchange(bsp_spi_t *const me, const uint8_t *transmit_data,
                              uint8_t *receive_data, size_t size, bsp_transfer_mode_t mode,
                              uint32_t timeout_ms)
{
    const bsp_status_t status = bsp_spi_validate(me);
    if (status != BSP_STATUS_OK)
    {
        return status;
    }
    // 发送/接收指针非空、大小非零、传输模式合法
    if ((transmit_data == NULL) || (receive_data == NULL) || (size == 0U) ||
        !bsp_transfer_mode_is_valid(mode))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    // 通过虚表调用 exchange（可能返回 UNSUPPORTED）
    return bsp_spi_get_ops(me)->exchange(me, transmit_data, receive_data, size, mode, timeout_ms);
}

/**
 * @brief 中止当前事务（公共接口）
 * @param me 基类指针
 * @return 执行状态
 */
bsp_status_t bsp_spi_abort(bsp_spi_t *const me)
{
    const bsp_status_t status = bsp_spi_validate(me);
    // 校验通过后通过虚表调用 abort（可能返回 UNSUPPORTED）
    return (status == BSP_STATUS_OK) ? bsp_spi_get_ops(me)->abort(me) : status;
}

/**
 * @brief 查询总线是否忙（公共接口）
 * @param me 基类指针（const）
 * @param is_busy 输出是否忙
 * @return 执行状态
 */
bsp_status_t bsp_spi_get_busy(const bsp_spi_t *const me, bool *is_busy)
{
    const bsp_status_t status = bsp_spi_validate(me);
    if (status != BSP_STATUS_OK)
    {
        return status;
    }
    // 输出指针非空检查
    if (is_busy == NULL)
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    // 通过虚表调用 get_busy（可能返回 UNSUPPORTED）
    return bsp_spi_get_ops(me)->get_busy(me, is_busy);
}

/**
 * @brief 事件通知函数（由底层驱动在中断中调用）
 * @param me 基类指针
 * @param event 事件类型
 * @param status 状态码
 * @param transferred_size 已传输的数据量（字节数）
 */
void bsp_spi_notify(bsp_spi_t *const me, bsp_event_t event, bsp_status_t status,
                    size_t transferred_size)
{
    if ((me != NULL) && bsp_device_is_initialized(&me->super) && (me->callback != NULL))
    {
        me->callback(event, status, transferred_size, me->user_context);
    }
}
