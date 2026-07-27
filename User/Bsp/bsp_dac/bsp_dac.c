/**
 * @file bsp_dac.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief DAC 通用抽象层实现
 * @note 提供 DAC 输出的多态接口，支持静态输出和 DMA 波形播放。
 * @version 1.0
 * @date 2026-07-27
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "bsp_dac.h" // 包含 DAC 抽象层头文件
#include <math.h>    // 提供 isfinite() 浮点检测
#include <stddef.h>  // 提供 NULL 和 size_t

/**
 * @brief 从基类指针获取派生设备对象（非常量版本）
 * @param dac_base bsp_dac_t 基类指针
 * @return 对应的 bsp_dac_device_t 对象指针
 */
static bsp_dac_device_t *bsp_dac_get_device(bsp_dac_t *const dac_base)
{
    // 利用 container_of 宏从基类成员地址反推出包含它的结构体地址
    return BSP_CONTAINER_OF(dac_base, bsp_dac_device_t, super);
}

/**
 * @brief 从基类指针获取派生设备对象（常量版本）
 * @param dac_base const bsp_dac_t 指针
 * @return 对应的 const bsp_dac_device_t 指针
 */
static const bsp_dac_device_t *bsp_dac_get_device_const(const bsp_dac_t *const dac_base)
{
    return BSP_CONTAINER_OF_CONST(dac_base, bsp_dac_device_t, super);
}

/**
 * @brief 从基类虚表指针获取高层操作表
 * @param dac_base const bsp_dac_t 指针
 * @return 对应的 bsp_dac_ops_t 操作表指针（只读）
 */
static const bsp_dac_ops_t *bsp_dac_get_ops(const bsp_dac_t *const dac_base)
{
    // 基类 bsp_device_t 的 vptr 指向 bsp_dac_ops_t 中的 super 成员
    return BSP_CONTAINER_OF_CONST(dac_base->super.vptr, bsp_dac_ops_t, super);
}

/**
 * @brief DAC 设备反初始化（作为 device 层的 deinit 回调）
 * @param device_base bsp_device_t 基类指针
 * @return 执行状态
 */
static bsp_status_t bsp_dac_device_deinit(bsp_device_t *const device_base)
{
    // 从 device 基类反推出 bsp_dac_t 基类地址
    bsp_dac_t *const dac_base = BSP_CONTAINER_OF(device_base, bsp_dac_t, super);
    bsp_dac_device_t *const me = bsp_dac_get_device(dac_base);
    // 如果驱动没有提供 deinit，视为无需清理，直接成功
    if (me->driver_ops->deinit == NULL)
    {
        return BSP_STATUS_OK;
    }
    // 调用底层驱动的 deinit，传入设备句柄和通道号
    return me->driver_ops->deinit(device_base->device_handle, me->channel);
}

/**
 * @brief 宏：生成无额外参数的转发函数（start/stop）
 */
#define BSP_DAC_FORWARD(name, member)                                                              \
    static bsp_status_t name(bsp_dac_t *const dac_base)                                            \
    {                                                                                              \
        bsp_dac_device_t *const me = bsp_dac_get_device(dac_base);                                 \
        return me->driver_ops->member(dac_base->super.device_handle, me->channel);                 \
    }

// 生成启动转发函数
BSP_DAC_FORWARD(bsp_dac_device_start, start)
// 生成停止转发函数
BSP_DAC_FORWARD(bsp_dac_device_stop, stop)

/**
 * @brief 停止 DAC 的 DMA 传输（转发至底层驱动）
 * @param dac_base 基类指针
 * @return 状态，若驱动不支持则返回 BSP_STATUS_UNSUPPORTED
 */
static bsp_status_t bsp_dac_device_stop_dma(bsp_dac_t *const dac_base)
{
    bsp_dac_device_t *const me = bsp_dac_get_device(dac_base);
    // 检查底层是否实现了 stop_dma，否则返回不支持
    return (me->driver_ops->stop_dma != NULL)
               ? me->driver_ops->stop_dma(dac_base->super.device_handle, me->channel)
               : BSP_STATUS_UNSUPPORTED;
}

/**
 * @brief 设置原始值（转发至底层驱动）
 * @param dac_base 基类指针
 * @param raw_value 原始码
 * @return 执行状态
 */
static bsp_status_t bsp_dac_device_set_raw(bsp_dac_t *const dac_base, uint32_t raw_value)
{
    bsp_dac_device_t *const me = bsp_dac_get_device(dac_base);
    return me->driver_ops->set_raw(dac_base->super.device_handle, me->channel, raw_value);
}

/**
 * @brief 获取原始值（转发至底层驱动）
 * @param dac_base 基类指针（const）
 * @param raw_value 输出原始码
 * @return 执行状态
 */
static bsp_status_t bsp_dac_device_get_raw(const bsp_dac_t *const dac_base, uint32_t *raw_value)
{
    const bsp_dac_device_t *const me = bsp_dac_get_device_const(dac_base);
    return me->driver_ops->get_raw(dac_base->super.device_handle, me->channel, raw_value);
}

/**
 * @brief 启动 DAC 的 DMA 输出（转发至底层驱动）
 * @param dac_base 基类指针
 * @param sample_buffer 样本缓冲区（只读）
 * @param sample_count 样本数量
 * @return 若驱动不支持则返回 BSP_STATUS_UNSUPPORTED
 */
static bsp_status_t bsp_dac_device_start_dma(bsp_dac_t *const dac_base,
                                             const uint32_t *sample_buffer, size_t sample_count)
{
    bsp_dac_device_t *const me = bsp_dac_get_device(dac_base);
    return (me->driver_ops->start_dma != NULL)
               ? me->driver_ops->start_dma(dac_base->super.device_handle, me->channel,
                                           sample_buffer, sample_count)
               : BSP_STATUS_UNSUPPORTED;
}

/* 定义 DAC 设备层的操作表（虚表），将所有转发函数填入 */
static const bsp_dac_ops_t s_bsp_dac_device_ops = {
    .super = {.deinit = bsp_dac_device_deinit}, // 继承自 device 的 deinit
    .start = bsp_dac_device_start,              // 启动转发
    .stop = bsp_dac_device_stop,                // 停止转发
    .set_raw = bsp_dac_device_set_raw,          // 设置原始值转发
    .get_raw = bsp_dac_device_get_raw,          // 获取原始值转发
    .start_dma = bsp_dac_device_start_dma,      // DMA 启动转发
    .stop_dma = bsp_dac_device_stop_dma         // DMA 停止转发
};

/**
 * @brief 校验 DAC 对象是否有效且已初始化
 * @param me bsp_dac_t 指针
 * @return 状态，成功则 BSP_STATUS_OK
 */
static bsp_status_t bsp_dac_validate(const bsp_dac_t *const me)
{
    if (me == NULL)
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    // 调用底层 device 的初始化状态检查
    return bsp_device_is_initialized(&me->super) ? BSP_STATUS_OK : BSP_STATUS_NOT_INITIALIZED;
}

/**
 * @brief 初始化一个 DAC 设备实例
 * @param me 设备对象指针
 * @param config 配置参数指针
 * @return 执行状态
 */
bsp_status_t bsp_dac_init(bsp_dac_device_t *const me, const bsp_dac_config_t *const config)
{
    bsp_status_t status;
    // 参数合法性检查：对象、配置、设备句柄、驱动表、分辨率有效性（1~31位）、参考电压为正有限数、
    // 必须实现 start/stop/set_raw/get_raw 等关键接口
    if ((me == NULL) || (config == NULL) || (config->device_handle == NULL) ||
        (config->driver_ops == NULL) || (config->resolution_bits == 0U) ||
        (config->resolution_bits > 31U) || !isfinite(config->reference_voltage_v) ||
        (config->reference_voltage_v <= 0.0F) || (config->driver_ops->start == NULL) ||
        (config->driver_ops->stop == NULL) || (config->driver_ops->set_raw == NULL) ||
        (config->driver_ops->get_raw == NULL))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    // 预先标记为未初始化，避免中途失败时留下错误状态
    me->super.super.is_initialized = false;
    // 保存底层驱动操作表和逻辑通道号
    me->driver_ops = config->driver_ops;
    me->channel = config->channel;
    // 如果驱动提供了 init 回调，则调用以初始化硬件相关资源
    if (me->driver_ops->init != NULL)
    {
        status = me->driver_ops->init(config->device_handle, config->channel);
        if (status != BSP_STATUS_OK)
        {
            return status; // 底层初始化失败则直接返回
        }
    }
    // 设置回调函数和用户上下文
    me->super.callback = config->callback;
    me->super.user_context = config->user_context;
    // 保存参考电压和最大原始值（由分辨率计算）
    me->super.reference_voltage_v = config->reference_voltage_v;
    me->super.maximum_raw_value = (1UL << config->resolution_bits) - 1UL;
    // 调用 device 层的通用初始化，注册虚表并保存设备句柄
    return bsp_device_init(&me->super.super, &s_bsp_dac_device_ops.super, config->device_handle);
}

/**
 * @brief 将派生对象转为基类指针（向上转型）
 * @param me 派生对象指针
 * @return 基类指针，若输入为空则返回 NULL
 */
bsp_dac_t *bsp_dac_as_base(bsp_dac_device_t *const me)
{
    return (me != NULL) ? &me->super : NULL;
}

/**
 * @brief 设置 DAC 的事件回调函数
 * @param me 基类指针
 * @param callback 回调函数指针
 * @param user_context 用户上下文
 * @return 执行状态，若对象未初始化则失败
 */
bsp_status_t bsp_dac_set_callback(bsp_dac_t *const me, bsp_event_callback_t callback,
                                  void *user_context)
{
    bsp_status_t status = bsp_dac_validate(me);
    if (status == BSP_STATUS_OK)
    {
        me->callback = callback;         // 更新回调
        me->user_context = user_context; // 更新上下文
    }
    return status;
}

/**
 * @brief 宏：为无附加参数的公共操作函数生成封装（含校验和转发）
 */
#define BSP_DAC_PUBLIC_ACTION(name, member)                                                        \
    bsp_status_t name(bsp_dac_t *const me)                                                         \
    {                                                                                              \
        bsp_status_t status = bsp_dac_validate(me);                                                \
        return (status == BSP_STATUS_OK) ? bsp_dac_get_ops(me)->member(me) : status;               \
    }

// 生成公共接口：启动、停止、停止 DMA
BSP_DAC_PUBLIC_ACTION(bsp_dac_start, start)       // 公共启动接口
BSP_DAC_PUBLIC_ACTION(bsp_dac_stop, stop)         // 公共停止接口
BSP_DAC_PUBLIC_ACTION(bsp_dac_stop_dma, stop_dma) // 公共停止 DMA 接口

/**
 * @brief 设置原始值（公共接口）
 * @param me 基类指针
 * @param raw_value 原始码
 * @return 执行状态，若 raw_value 超出最大范围则返回 OUT_OF_RANGE
 */
bsp_status_t bsp_dac_set_raw(bsp_dac_t *const me, uint32_t raw_value)
{
    bsp_status_t status = bsp_dac_validate(me);
    // 校验对象有效后，检查原始值是否超出分辨率允许的最大值
    if ((status == BSP_STATUS_OK) && (raw_value > me->maximum_raw_value))
    {
        return BSP_STATUS_OUT_OF_RANGE;
    }
    // 转发至具体虚表函数
    return (status == BSP_STATUS_OK) ? bsp_dac_get_ops(me)->set_raw(me, raw_value) : status;
}

/**
 * @brief 获取原始值（公共接口）
 * @param me 基类指针（const）
 * @param raw_value 输出原始码
 * @return 执行状态，若读取值超过最大范围则返回 BSP_STATUS_IO_ERROR
 */
bsp_status_t bsp_dac_get_raw(const bsp_dac_t *const me, uint32_t *raw_value)
{
    bsp_status_t status = bsp_dac_validate(me);
    // 校验对象有效后，检查输出指针非空
    if ((status == BSP_STATUS_OK) && (raw_value == NULL))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    if (status != BSP_STATUS_OK)
    {
        return status;
    }
    // 通过虚表调用底层 get_raw
    status = bsp_dac_get_ops(me)->get_raw(me, raw_value);
    // 读取成功后，检查原始值是否超出分辨率允许的最大值（硬件异常保护）
    if ((status == BSP_STATUS_OK) && (*raw_value > me->maximum_raw_value))
    {
        return BSP_STATUS_IO_ERROR;
    }
    return status;
}

/**
 * @brief 设置归一化值（0~1）
 * @param me 基类指针
 * @param normalized_value 归一化浮点数（0.0~1.0）
 * @return 执行状态
 */
bsp_status_t bsp_dac_set_normalized(bsp_dac_t *const me, float normalized_value)
{
    const bsp_status_t status = bsp_dac_validate(me);
    if (status != BSP_STATUS_OK)
    {
        return status;
    }
    // 检查归一化值是否为有限数且在 [0, 1] 范围内
    if (!isfinite(normalized_value) || (normalized_value < 0.0F) || (normalized_value > 1.0F))
    {
        return BSP_STATUS_OUT_OF_RANGE;
    }
    // 归一化值 * 最大原始值 + 0.5 四舍五入取整
    return bsp_dac_set_raw(me, (uint32_t)((float)me->maximum_raw_value * normalized_value + 0.5F));
}

/**
 * @brief 设置电压值（0 ~ 参考电压）
 * @param me 基类指针
 * @param voltage_v 电压值（伏特）
 * @return 执行状态
 */
bsp_status_t bsp_dac_set_voltage(bsp_dac_t *const me, float voltage_v)
{
    const bsp_status_t status = bsp_dac_validate(me);
    if (status != BSP_STATUS_OK)
    {
        return status;
    }
    // 检查电压值是否为有限数且在 [0, reference_voltage_v] 范围内
    if (!isfinite(voltage_v) || (voltage_v < 0.0F) || (voltage_v > me->reference_voltage_v))
    {
        return BSP_STATUS_OUT_OF_RANGE;
    }
    // 电压 / 参考电压 得到归一化值，再通过归一化设置
    return bsp_dac_set_normalized(me, voltage_v / me->reference_voltage_v);
}

/**
 * @brief 启动 DMA 输出（公共接口）
 * @param me 基类指针
 * @param sample_buffer 用户提供的样本缓冲区（只读）
 * @param sample_count 样本数量
 * @return 执行状态，若缓冲区为空或数量为0返回无效参数，样本值超范围返回 OUT_OF_RANGE
 */
bsp_status_t bsp_dac_start_dma(bsp_dac_t *const me, const uint32_t *sample_buffer,
                               size_t sample_count)
{
    bsp_status_t status = bsp_dac_validate(me);
    size_t sample_index;
    // 校验对象有效后，检查缓冲区和数量合法性
    if ((status == BSP_STATUS_OK) && ((sample_buffer == NULL) || (sample_count == 0U)))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    // 校验所有样本值是否在允许范围内（防止硬件输出超出分辨率）
    if (status == BSP_STATUS_OK)
    {
        for (sample_index = 0U; sample_index < sample_count; ++sample_index)
        {
            if (sample_buffer[sample_index] > me->maximum_raw_value)
            {
                return BSP_STATUS_OUT_OF_RANGE;
            }
        }
    }
    // 转发至具体虚表函数
    return (status == BSP_STATUS_OK)
               ? bsp_dac_get_ops(me)->start_dma(me, sample_buffer, sample_count)
               : status;
}

/**
 * @brief 向应用层通知 DAC 事件（由底层驱动调用）
 * @param me 基类指针
 * @param event 事件类型
 * @param status 状态码
 * @param transferred_size 已传输的数据量（如 DMA 计数值）
 */
void bsp_dac_notify(bsp_dac_t *const me, bsp_event_t event, bsp_status_t status,
                    size_t transferred_size)
{
    // 只有在对象有效、已初始化且回调非空时才调用
    if ((me != NULL) && bsp_device_is_initialized(&me->super) && (me->callback != NULL))
    {
        me->callback(event, status, transferred_size, me->user_context);
    }
}