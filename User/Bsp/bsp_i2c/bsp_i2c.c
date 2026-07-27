/**
 * @file bsp_i2c.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief I2C 通用抽象层实现
 * @note 支持 7 位地址的发送/接收，8/16 位寄存器地址访问，设备就绪探测，
 *       中止和忙状态查询，三种传输模式（阻塞/中断/DMA）。
 * @version 1.0
 * @date 2026-07-25
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "bsp_i2c.h" // 包含 I2C 抽象层头文件
#include <stddef.h>  // 提供 NULL 和 size_t

/**
 * @brief 从基类指针获取派生设备对象（非常量版本）
 * @param i2c_base bsp_i2c_t 基类指针
 * @return 对应的 bsp_i2c_device_t 对象指针
 */
static bsp_i2c_device_t *bsp_i2c_get_device(bsp_i2c_t *const i2c_base)
{
    // 利用 container_of 宏从基类成员地址反推出包含它的结构体地址
    return BSP_CONTAINER_OF(i2c_base, bsp_i2c_device_t, super);
}

/**
 * @brief 从基类指针获取派生设备对象（常量版本）
 * @param i2c_base const bsp_i2c_t 指针
 * @return 对应的 const bsp_i2c_device_t 指针
 */
static const bsp_i2c_device_t *bsp_i2c_get_device_const(const bsp_i2c_t *const i2c_base)
{
    return BSP_CONTAINER_OF_CONST(i2c_base, bsp_i2c_device_t, super);
}

/**
 * @brief 从基类虚表指针获取高层操作表
 * @param i2c_base const bsp_i2c_t 指针
 * @return 对应的 bsp_i2c_ops_t 操作表指针（只读）
 */
static const bsp_i2c_ops_t *bsp_i2c_get_ops(const bsp_i2c_t *const i2c_base)
{
    // 基类 bsp_device_t 的 vptr 指向 bsp_i2c_ops_t 中的 super 成员
    return BSP_CONTAINER_OF_CONST(i2c_base->super.vptr, bsp_i2c_ops_t, super);
}

/**
 * @brief I2C 设备反初始化（作为 device 层的 deinit 回调）
 * @param device_base bsp_device_t 基类指针
 * @return 执行状态
 */
static bsp_status_t bsp_i2c_device_deinit(bsp_device_t *const device_base)
{
    // 从 device 基类反推出 bsp_i2c_t 基类地址
    bsp_i2c_t *const i2c_base = BSP_CONTAINER_OF(device_base, bsp_i2c_t, super);
    bsp_i2c_device_t *const me = bsp_i2c_get_device(i2c_base);
    // 如果驱动没有提供 deinit，视为无需清理，直接成功
    return (me->driver_ops->deinit != NULL) ? me->driver_ops->deinit(device_base->device_handle)
                                            : BSP_STATUS_OK;
}

/**
 * @brief 发送数据（转发至底层驱动）
 * @param i2c_base 基类指针
 * @param address_7bit 7 位从设备地址
 * @param transmit_data 发送数据指针
 * @param data_size 数据大小（字节）
 * @param mode 传输模式（阻塞/中断/DMA）
 * @param timeout_ms 超时时间（毫秒）
 * @return 执行状态
 */
static bsp_status_t bsp_i2c_device_transmit(bsp_i2c_t *const i2c_base, uint16_t address_7bit,
                                            const uint8_t *transmit_data, size_t data_size,
                                            bsp_transfer_mode_t mode, uint32_t timeout_ms)
{
    bsp_i2c_device_t *const me = bsp_i2c_get_device(i2c_base);
    // 调用驱动层的 transmit，传入设备句柄和所有参数
    return me->driver_ops->transmit(i2c_base->super.device_handle, address_7bit, transmit_data,
                                    data_size, mode, timeout_ms);
}

/**
 * @brief 接收数据（转发至底层驱动）
 * @param i2c_base 基类指针
 * @param address_7bit 7 位从设备地址
 * @param receive_data 接收缓冲区指针
 * @param data_size 数据大小（字节）
 * @param mode 传输模式（阻塞/中断/DMA）
 * @param timeout_ms 超时时间（毫秒）
 * @return 执行状态
 */
static bsp_status_t bsp_i2c_device_receive(bsp_i2c_t *const i2c_base, uint16_t address_7bit,
                                           uint8_t *receive_data, size_t data_size,
                                           bsp_transfer_mode_t mode, uint32_t timeout_ms)
{
    bsp_i2c_device_t *const me = bsp_i2c_get_device(i2c_base);
    return me->driver_ops->receive(i2c_base->super.device_handle, address_7bit, receive_data,
                                   data_size, mode, timeout_ms);
}

/**
 * @brief 向寄存器地址写入数据（转发至底层驱动，可选）
 * @param i2c_base 基类指针
 * @param address 7 位从设备地址
 * @param memory 寄存器地址（8 或 16 位）
 * @param address_size 寄存器地址大小（8/16 位）
 * @param data 数据指针
 * @param size 数据大小（字节）
 * @param mode 传输模式
 * @param timeout_ms 超时时间
 * @return 若驱动未实现则返回 BSP_STATUS_UNSUPPORTED
 */
static bsp_status_t bsp_i2c_device_memory_write(bsp_i2c_t *const i2c_base, uint16_t address,
                                                uint16_t memory,
                                                bsp_i2c_memory_address_size_t address_size,
                                                const uint8_t *data, size_t size,
                                                bsp_transfer_mode_t mode, uint32_t timeout_ms)
{
    bsp_i2c_device_t *const me = bsp_i2c_get_device(i2c_base);
    // 检查底层是否实现了 memory_write，否则返回不支持
    return (me->driver_ops->memory_write != NULL)
               ? me->driver_ops->memory_write(i2c_base->super.device_handle, address, memory,
                                              address_size, data, size, mode, timeout_ms)
               : BSP_STATUS_UNSUPPORTED;
}

/**
 * @brief 从寄存器地址读取数据（转发至底层驱动，可选）
 * @param i2c_base 基类指针
 * @param address 7 位从设备地址
 * @param memory 寄存器地址（8 或 16 位）
 * @param address_size 寄存器地址大小（8/16 位）
 * @param data 接收缓冲区指针
 * @param size 数据大小（字节）
 * @param mode 传输模式
 * @param timeout_ms 超时时间
 * @return 若驱动未实现则返回 BSP_STATUS_UNSUPPORTED
 */
static bsp_status_t bsp_i2c_device_memory_read(bsp_i2c_t *const i2c_base, uint16_t address,
                                               uint16_t memory,
                                               bsp_i2c_memory_address_size_t address_size,
                                               uint8_t *data, size_t size, bsp_transfer_mode_t mode,
                                               uint32_t timeout_ms)
{
    bsp_i2c_device_t *const me = bsp_i2c_get_device(i2c_base);
    return (me->driver_ops->memory_read != NULL)
               ? me->driver_ops->memory_read(i2c_base->super.device_handle, address, memory,
                                             address_size, data, size, mode, timeout_ms)
               : BSP_STATUS_UNSUPPORTED;
}

/**
 * @brief 检查设备是否就绪（转发至底层驱动，可选）
 * @param i2c_base 基类指针
 * @param address 7 位从设备地址
 * @param trials 重试次数
 * @param timeout_ms 每次尝试的超时时间
 * @return 若驱动未实现则返回 BSP_STATUS_UNSUPPORTED
 */
static bsp_status_t bsp_i2c_device_is_ready(bsp_i2c_t *const i2c_base, uint16_t address,
                                            uint32_t trials, uint32_t timeout_ms)
{
    bsp_i2c_device_t *const me = bsp_i2c_get_device(i2c_base);
    return (me->driver_ops->is_device_ready != NULL)
               ? me->driver_ops->is_device_ready(i2c_base->super.device_handle, address, trials,
                                                 timeout_ms)
               : BSP_STATUS_UNSUPPORTED;
}

/**
 * @brief 中止当前事务（转发至底层驱动，可选）
 * @param i2c_base 基类指针
 * @param address_7bit 7 位从设备地址
 * @return 若驱动未实现则返回 BSP_STATUS_UNSUPPORTED
 */
static bsp_status_t bsp_i2c_device_abort(bsp_i2c_t *const i2c_base, uint16_t address_7bit)
{
    bsp_i2c_device_t *const me = bsp_i2c_get_device(i2c_base);
    return (me->driver_ops->abort != NULL)
               ? me->driver_ops->abort(i2c_base->super.device_handle, address_7bit)
               : BSP_STATUS_UNSUPPORTED;
}

/**
 * @brief 查询总线是否忙（转发至底层驱动，可选）
 * @param i2c_base 基类指针（const）
 * @param is_busy 输出是否忙
 * @return 若驱动未实现则返回 BSP_STATUS_UNSUPPORTED
 */
static bsp_status_t bsp_i2c_device_get_busy(const bsp_i2c_t *const i2c_base, bool *is_busy)
{
    const bsp_i2c_device_t *const me = bsp_i2c_get_device_const(i2c_base);
    return (me->driver_ops->get_busy != NULL)
               ? me->driver_ops->get_busy(i2c_base->super.device_handle, is_busy)
               : BSP_STATUS_UNSUPPORTED;
}

/* 定义 I2C 设备层的操作表（虚表），将所有转发函数填入 */
static const bsp_i2c_ops_t s_bsp_i2c_device_ops = {
    .super = {.deinit = bsp_i2c_device_deinit},  // 继承自 device 的 deinit
    .transmit = bsp_i2c_device_transmit,         // 发送转发
    .receive = bsp_i2c_device_receive,           // 接收转发
    .memory_write = bsp_i2c_device_memory_write, // 寄存器写入转发（可选）
    .memory_read = bsp_i2c_device_memory_read,   // 寄存器读取转发（可选）
    .is_device_ready = bsp_i2c_device_is_ready,  // 设备就绪探测转发（可选）
    .abort = bsp_i2c_device_abort,               // 中止转发（可选）
    .get_busy = bsp_i2c_device_get_busy,         // 忙状态查询转发（可选）
};

/**
 * @brief 初始化 I2C 设备实例
 * @param me 设备对象指针
 * @param config 配置参数指针
 * @return 执行状态
 */
bsp_status_t bsp_i2c_init(bsp_i2c_device_t *const me, const bsp_i2c_config_t *const config)
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
    return bsp_device_init(&me->super.super, &s_bsp_i2c_device_ops.super, config->device_handle);
}

/**
 * @brief 将派生对象转为基类指针（向上转型）
 * @param me 派生对象指针
 * @return 基类指针，若输入为空则返回 NULL
 */
bsp_i2c_t *bsp_i2c_as_base(bsp_i2c_device_t *const me)
{
    return (me != NULL) ? &me->super : NULL;
}

/**
 * @brief 校验 I2C 对象是否有效且已初始化
 * @param me bsp_i2c_t 指针
 * @return 状态，成功则 BSP_STATUS_OK
 */
static bsp_status_t bsp_i2c_validate(const bsp_i2c_t *const me)
{
    if (me == NULL)
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    return bsp_device_is_initialized(&me->super) ? BSP_STATUS_OK : BSP_STATUS_NOT_INITIALIZED;
}

/**
 * @brief 校验传输参数（对象有效性、地址、数据和大小）
 * @param me bsp_i2c_t 指针
 * @param address 7 位从设备地址
 * @param data 数据指针（发送或接收）
 * @param size 数据大小
 * @return 校验状态
 */
static bsp_status_t bsp_i2c_validate_transfer(const bsp_i2c_t *const me, uint16_t address,
                                              const void *data, size_t size)
{
    const bsp_status_t status = bsp_i2c_validate(me);
    if (status != BSP_STATUS_OK)
    {
        return status;
    }
    // 数据指针和大小非空检查
    if ((data == NULL) || (size == 0U))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    // 地址必须在 0~0x7F 范围内（7 位地址）
    return (address <= 0x7FU) ? BSP_STATUS_OK : BSP_STATUS_OUT_OF_RANGE;
}

/**
 * @brief 设置 I2C 事件回调函数
 * @param me 基类指针
 * @param callback 回调函数指针
 * @param user_context 用户上下文
 * @return 执行状态
 */
bsp_status_t bsp_i2c_set_callback(bsp_i2c_t *const me, bsp_event_callback_t callback,
                                  void *user_context)
{
    const bsp_status_t status = bsp_i2c_validate(me);
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
 * @param address 7 位从设备地址
 * @param data 发送数据指针
 * @param size 数据大小（字节）
 * @param mode 传输模式（阻塞/中断/DMA）
 * @param timeout_ms 超时时间（毫秒）
 * @return 执行状态
 */
bsp_status_t bsp_i2c_transmit(bsp_i2c_t *const me, uint16_t address, const uint8_t *data,
                              size_t size, bsp_transfer_mode_t mode, uint32_t timeout_ms)
{
    // 校验传输参数
    const bsp_status_t status = bsp_i2c_validate_transfer(me, address, data, size);
    // 校验传输模式是否合法
    if ((status == BSP_STATUS_OK) && !bsp_transfer_mode_is_valid(mode))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    // 校验通过后通过虚表调用 transmit
    return (status == BSP_STATUS_OK)
               ? bsp_i2c_get_ops(me)->transmit(me, address, data, size, mode, timeout_ms)
               : status;
}

/**
 * @brief 接收数据（公共接口）
 * @param me 基类指针
 * @param address 7 位从设备地址
 * @param data 接收缓冲区指针
 * @param size 数据大小（字节）
 * @param mode 传输模式（阻塞/中断/DMA）
 * @param timeout_ms 超时时间（毫秒）
 * @return 执行状态
 */
bsp_status_t bsp_i2c_receive(bsp_i2c_t *const me, uint16_t address, uint8_t *data, size_t size,
                             bsp_transfer_mode_t mode, uint32_t timeout_ms)
{
    const bsp_status_t status = bsp_i2c_validate_transfer(me, address, data, size);
    // 校验传输模式是否合法
    if ((status == BSP_STATUS_OK) && !bsp_transfer_mode_is_valid(mode))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    // 校验通过后通过虚表调用 receive
    return (status == BSP_STATUS_OK)
               ? bsp_i2c_get_ops(me)->receive(me, address, data, size, mode, timeout_ms)
               : status;
}

/**
 * @brief 检查寄存器地址大小是否合法
 * @param size 地址大小枚举
 * @return true 合法（8 位或 16 位）
 */
static bool bsp_i2c_is_address_size_valid(bsp_i2c_memory_address_size_t size)
{
    return (size == BSP_I2C_MEMORY_ADDRESS_8_BIT) || (size == BSP_I2C_MEMORY_ADDRESS_16_BIT);
}

/**
 * @brief 向寄存器地址写入数据（公共接口）
 * @param me 基类指针
 * @param address 7 位从设备地址
 * @param memory 寄存器地址（8 或 16 位）
 * @param address_size 寄存器地址大小（8/16 位）
 * @param data 数据指针
 * @param size 数据大小（字节）
 * @param mode 传输模式
 * @param timeout_ms 超时时间
 * @return 执行状态
 */
bsp_status_t bsp_i2c_memory_write(bsp_i2c_t *const me, uint16_t address, uint16_t memory,
                                  bsp_i2c_memory_address_size_t address_size, const uint8_t *data,
                                  size_t size, bsp_transfer_mode_t mode, uint32_t timeout_ms)
{
    const bsp_status_t status = bsp_i2c_validate_transfer(me, address, data, size);
    if (status != BSP_STATUS_OK)
    {
        return status;
    }
    // 校验传输模式是否合法
    if (!bsp_transfer_mode_is_valid(mode))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    // 校验寄存器地址大小是否合法
    if (!bsp_i2c_is_address_size_valid(address_size))
    {
        return BSP_STATUS_OUT_OF_RANGE;
    }
    // 通过虚表调用 memory_write
    return bsp_i2c_get_ops(me)->memory_write(me, address, memory, address_size, data, size, mode,
                                             timeout_ms);
}

/**
 * @brief 从寄存器地址读取数据（公共接口）
 * @param me 基类指针
 * @param address 7 位从设备地址
 * @param memory 寄存器地址（8 或 16 位）
 * @param address_size 寄存器地址大小（8/16 位）
 * @param data 接收缓冲区指针
 * @param size 数据大小（字节）
 * @param mode 传输模式
 * @param timeout_ms 超时时间
 * @return 执行状态
 */
bsp_status_t bsp_i2c_memory_read(bsp_i2c_t *const me, uint16_t address, uint16_t memory,
                                 bsp_i2c_memory_address_size_t address_size, uint8_t *data,
                                 size_t size, bsp_transfer_mode_t mode, uint32_t timeout_ms)
{
    const bsp_status_t status = bsp_i2c_validate_transfer(me, address, data, size);
    if (status != BSP_STATUS_OK)
    {
        return status;
    }
    // 校验传输模式是否合法
    if (!bsp_transfer_mode_is_valid(mode))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    // 校验寄存器地址大小是否合法
    if (!bsp_i2c_is_address_size_valid(address_size))
    {
        return BSP_STATUS_OUT_OF_RANGE;
    }
    // 通过虚表调用 memory_read
    return bsp_i2c_get_ops(me)->memory_read(me, address, memory, address_size, data, size, mode,
                                            timeout_ms);
}

/**
 * @brief 检查设备是否就绪（公共接口）
 * @param me 基类指针
 * @param address 7 位从设备地址
 * @param trials 重试次数
 * @param timeout_ms 每次尝试的超时时间
 * @return 执行状态
 */
bsp_status_t bsp_i2c_is_device_ready(bsp_i2c_t *const me, uint16_t address, uint32_t trials,
                                     uint32_t timeout_ms)
{
    const bsp_status_t status = bsp_i2c_validate(me);
    if (status != BSP_STATUS_OK)
    {
        return status;
    }
    // 地址必须在 0~0x7F 范围内，尝试次数必须大于 0
    if ((address > 0x7FU) || (trials == 0U))
    {
        return BSP_STATUS_OUT_OF_RANGE;
    }
    // 通过虚表调用 is_device_ready
    return bsp_i2c_get_ops(me)->is_device_ready(me, address, trials, timeout_ms);
}

/**
 * @brief 中止当前事务（公共接口）
 * @param me 基类指针
 * @param address 7 位从设备地址
 * @return 执行状态
 */
bsp_status_t bsp_i2c_abort(bsp_i2c_t *const me, uint16_t address)
{
    const bsp_status_t status = bsp_i2c_validate(me);
    if (status != BSP_STATUS_OK)
    {
        return status;
    }
    // 地址必须在 0~0x7F 范围内
    if (address > 0x7FU)
    {
        return BSP_STATUS_OUT_OF_RANGE;
    }
    // 通过虚表调用 abort
    return bsp_i2c_get_ops(me)->abort(me, address);
}

/**
 * @brief 查询总线是否忙（公共接口）
 * @param me 基类指针（const）
 * @param is_busy 输出是否忙
 * @return 执行状态
 */
bsp_status_t bsp_i2c_get_busy(const bsp_i2c_t *const me, bool *is_busy)
{
    const bsp_status_t status = bsp_i2c_validate(me);
    if (status != BSP_STATUS_OK)
    {
        return status;
    }
    // 输出指针非空检查
    if (is_busy == NULL)
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    // 通过虚表调用 get_busy
    return bsp_i2c_get_ops(me)->get_busy(me, is_busy);
}

/**
 * @brief 事件通知函数（由底层驱动在中断中调用）
 * @param me 基类指针
 * @param event 事件类型
 * @param status 状态码
 * @param transferred_size 已传输的数据量（字节数）
 * @note 仅在对象有效且回调非空时调用
 */
void bsp_i2c_notify(bsp_i2c_t *const me, bsp_event_t event, bsp_status_t status,
                    size_t transferred_size)
{
    if ((me != NULL) && bsp_device_is_initialized(&me->super) && (me->callback != NULL))
    {
        me->callback(event, status, transferred_size, me->user_context);
    }
}