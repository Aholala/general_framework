/**
 * @file bsp_adc.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief ADC 通用抽象层实现
 * @version 1.0
 * @date 2026-07-21
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "bsp_adc.h" // 包含 ADC 抽象层头文件

BSP_STATIC_ASSERT_SUPER_FIRST(bsp_adc_device_t);

#include <math.h>   // 用于 isfinite() 浮点检测
#include <stddef.h> // 提供 NULL、size_t 等标准定义

/**
 * @brief 从基类指针获取派生设备对象指针
 * @param adc_base 基类 bsp_adc_t 指针
 * @return 对应的 bsp_adc_device_t 对象指针
 */
static bsp_adc_device_t *bsp_adc_get_device(bsp_adc_t *const adc_base)
{
    /* 利用 container_of 宏从基类成员地址反推出包含它的结构体地址 */
    return BSP_CONTAINER_OF(adc_base, bsp_adc_device_t, super);
}

/**
 * @brief 从基类虚表指针获取派生操作表指针（只读）
 * @param adc_base 基类 bsp_adc_t 指针
 * @return 对应的 bsp_adc_ops_t 操作表指针
 */
static const bsp_adc_ops_t *bsp_adc_get_ops(const bsp_adc_t *const adc_base)
{
    /* 从基类虚表指针反推出包含该虚表的操作表结构体地址（常量版本） */
    return BSP_CONTAINER_OF_CONST(adc_base->super.vptr, bsp_adc_ops_t, super);
}

/**
 * @brief 反初始化 ADC 设备（作为 device 层的 deinit 回调）
 * @param device_base device 基类指针
 * @return 执行状态
 */
static bsp_status_t bsp_adc_device_deinit(bsp_device_t *const device_base)
{
    /* 从 device 基类反推出 bsp_adc_t 基类地址 */
    bsp_adc_t *const adc_base = BSP_CONTAINER_OF(device_base, bsp_adc_t, super);
    /* 获取派生设备对象 */
    bsp_adc_device_t *const me = bsp_adc_get_device(adc_base);
    /* 如果驱动没有提供 deinit 接口，视为无需清理，直接成功 */
    if (me->driver_ops->deinit == NULL)
    {
        return BSP_STATUS_OK;
    }
    /* 调用底层驱动的 deinit，传入设备句柄和通道号 */
    return me->driver_ops->deinit(device_base->device_handle, me->channel);
}

/* 为 ADC 操作函数生成标准转发函数（无额外参数） */
#define BSP_ADC_FORWARD(name, member)                                                 \
    static bsp_status_t name(bsp_adc_t *const adc_base)                               \
    {                                                                                 \
        bsp_adc_device_t *const me = bsp_adc_get_device(adc_base);                    \
        return me->driver_ops->member(adc_base->super.device_handle, me->channel);    \
    }

/* 生成启动、停止的转发函数 */
BSP_ADC_FORWARD(bsp_adc_device_start, start) /* 启动 ADC 转换 */
BSP_ADC_FORWARD(bsp_adc_device_stop, stop)   /* 停止 ADC 转换 */

/**
 * @brief 停止 ADC 的 DMA 传输（转发至底层驱动）
 * @param adc_base 基类指针
 * @return 状态，若驱动不支持则返回 BSP_STATUS_UNSUPPORTED
 */
static bsp_status_t bsp_adc_device_stop_dma(bsp_adc_t *const adc_base)
{
    bsp_adc_device_t *const me = bsp_adc_get_device(adc_base);
    /* 检查底层是否实现了 stop_dma，否则返回不支持 */
    return (me->driver_ops->stop_dma != NULL)
               ? me->driver_ops->stop_dma(adc_base->super.device_handle, me->channel)
               : BSP_STATUS_UNSUPPORTED;
}

/**
 * @brief 校准 ADC（转发至底层驱动）
 * @param adc_base 基类指针
 * @return 执行状态
 */
static bsp_status_t bsp_adc_device_calibrate(bsp_adc_t *const adc_base)
{
    bsp_adc_device_t *const me = bsp_adc_get_device(adc_base);
    /* 校准只需要设备句柄，不需要通道号 */
    return me->driver_ops->calibrate(adc_base->super.device_handle);
}

/**
 * @brief 阻塞读取原始采样值（转发至底层驱动）
 * @param adc_base 基类指针
 * @param raw_value 输出原始码
 * @param timeout_ms 超时时间（毫秒）
 * @return 执行状态
 */
static bsp_status_t bsp_adc_device_read_raw(bsp_adc_t *const adc_base, uint32_t *raw_value,
                                            uint32_t timeout_ms)
{
    bsp_adc_device_t *const me = bsp_adc_get_device(adc_base);
    return me->driver_ops->read_raw(adc_base->super.device_handle, me->channel, raw_value,
                                    timeout_ms);
}

/**
 * @brief 启动 ADC 的 DMA 采样（转发至底层驱动）
 * @param adc_base 基类指针
 * @param sample_buffer 采样缓冲区
 * @param sample_count 采样个数
 * @return 若驱动不支持则返回 BSP_STATUS_UNSUPPORTED
 */
static bsp_status_t bsp_adc_device_start_dma(bsp_adc_t *const adc_base, uint32_t *sample_buffer,
                                             size_t sample_count)
{
    bsp_adc_device_t *const me = bsp_adc_get_device(adc_base);
    return (me->driver_ops->start_dma != NULL)
               ? me->driver_ops->start_dma(adc_base->super.device_handle, me->channel,
                                           sample_buffer, sample_count)
               : BSP_STATUS_UNSUPPORTED;
}

/* 定义 ADC 设备层的操作表（虚表），将所有转发函数填入 */
static const bsp_adc_ops_t s_bsp_adc_device_ops = {
    .super = {.deinit = bsp_adc_device_deinit}, /* 继承自 device 的 deinit */
    .start = bsp_adc_device_start,              /* 启动转发 */
    .stop = bsp_adc_device_stop,                /* 停止转发 */
    .calibrate = bsp_adc_device_calibrate,      /* 校准转发 */
    .read_raw = bsp_adc_device_read_raw,        /* 阻塞读原始值转发 */
    .start_dma = bsp_adc_device_start_dma,      /* DMA 启动转发 */
    .stop_dma = bsp_adc_device_stop_dma         /* DMA 停止转发 */
};

/**
 * @brief 校验 ADC 对象是否有效且已初始化
 * @param me bsp_adc_t 指针
 * @return 状态，成功则 BSP_STATUS_OK
 */
static bsp_status_t bsp_adc_validate(const bsp_adc_t *const me)
{
    if (me == NULL) /* 空指针检查 */
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    /* 调用底层 device 的初始化状态检查 */
    return bsp_device_is_initialized(&me->super) ? BSP_STATUS_OK : BSP_STATUS_NOT_INITIALIZED;
}

/**
 * @brief 初始化一个 ADC 设备实例
 * @param me 设备对象指针
 * @param config 配置参数指针
 * @return 执行状态
 */
bsp_status_t bsp_adc_init(bsp_adc_device_t *const me, const bsp_adc_config_t *const config)
{
    bsp_status_t status;
    /* 参数合法性检查：对象、配置、设备句柄、驱动操作表、分辨率有效性（1~31位）、参考电压为正有限数、
       必须实现 start/stop/calibrate/read_raw 等关键接口 */
    if ((me == NULL) || (config == NULL) || (config->device_handle == NULL) ||
        (config->driver_ops == NULL) || (config->resolution_bits == 0U) ||
        (config->resolution_bits > 31U) || !isfinite(config->reference_voltage_v) ||
        (config->reference_voltage_v <= 0.0F) || (config->driver_ops->start == NULL) ||
        (config->driver_ops->stop == NULL) || (config->driver_ops->calibrate == NULL) ||
        (config->driver_ops->read_raw == NULL))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    /* 预先标记为未初始化，避免中途失败时留下错误状态 */
    me->super.super.is_initialized = false;
    /* 保存底层驱动操作表和逻辑通道号 */
    me->driver_ops = config->driver_ops;
    me->channel = config->channel;
    /* 如果驱动提供了 init 回调，则调用以初始化硬件相关资源 */
    if (me->driver_ops->init != NULL)
    {
        status = me->driver_ops->init(config->device_handle, config->channel);
        if (status != BSP_STATUS_OK)
        {
            return status; /* 底层初始化失败则直接返回 */
        }
    }
    /* 设置回调函数和用户上下文 */
    me->super.callback = config->callback;
    me->super.user_context = config->user_context;
    /* 保存参考电压和最大原始值（由分辨率计算） */
    me->super.reference_voltage_v = config->reference_voltage_v;
    me->super.maximum_raw_value = (1UL << config->resolution_bits) - 1UL;
    /* 调用 device 层的通用初始化，注册虚表并保存设备句柄 */
    return bsp_device_init(&me->super.super, &s_bsp_adc_device_ops.super, config->device_handle);
}

/**
 * @brief 将派生对象转为基类指针（向上转型）
 * @param me 派生对象指针
 * @return 基类指针，若输入为空则返回 NULL
 */
bsp_adc_t *bsp_adc_as_base(bsp_adc_device_t *const me)
{
    return (me != NULL) ? &me->super : NULL;
}

/**
 * @brief 设置 ADC 的事件回调函数
 * @param me 基类指针
 * @param callback 回调函数指针
 * @param user_context 用户上下文
 * @return 执行状态，若对象未初始化则失败
 */
bsp_status_t bsp_adc_set_callback(bsp_adc_t *const me, bsp_event_callback_t callback,
                                  void *user_context)
{
    bsp_status_t status = bsp_adc_validate(me);
    if (status == BSP_STATUS_OK)
    {
        me->callback = callback;         /* 更新回调 */
        me->user_context = user_context; /* 更新上下文 */
    }
    return status;
}

/* 宏：为无附加参数的公共操作函数生成封装（含校验和转发） */
#define BSP_ADC_PUBLIC_ACTION(name, member)                                                        \
    bsp_status_t name(bsp_adc_t *const me)                                                         \
    {                                                                                              \
        bsp_status_t status = bsp_adc_validate(me);                                                \
        return (status == BSP_STATUS_OK) ? bsp_adc_get_ops(me)->member(me) : status;               \
    }

/* 生成公共接口：启动、停止、校准、停止 DMA */
BSP_ADC_PUBLIC_ACTION(bsp_adc_start, start)         /* 公共启动接口 */
BSP_ADC_PUBLIC_ACTION(bsp_adc_stop, stop)           /* 公共停止接口 */
BSP_ADC_PUBLIC_ACTION(bsp_adc_calibrate, calibrate) /* 公共校准接口 */
BSP_ADC_PUBLIC_ACTION(bsp_adc_stop_dma, stop_dma)   /* 公共停止 DMA 接口 */

/**
 * @brief 阻塞读取原始采样值（公共接口）
 * @param me 基类指针
 * @param raw_value 输出原始码
 * @param timeout_ms 超时时间
 * @return 执行状态，若读取值超过最大范围则返回 BSP_STATUS_IO_ERROR
 */
bsp_status_t bsp_adc_read_raw(bsp_adc_t *const me, uint32_t *raw_value, uint32_t timeout_ms)
{
    bsp_status_t status = bsp_adc_validate(me);           /* 校验对象有效性 */
    if ((status == BSP_STATUS_OK) && (raw_value == NULL)) /* 输出指针非空检查 */
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    if (status != BSP_STATUS_OK) /* 对象无效则直接返回错误码 */
    {
        return status;
    }
    /* 通过虚表调用底层 read_raw */
    status = bsp_adc_get_ops(me)->read_raw(me, raw_value, timeout_ms);
    /* 读取成功后，检查原始值是否超出分辨率允许的最大值（硬件异常保护） */
    if ((status == BSP_STATUS_OK) && (*raw_value > me->maximum_raw_value))
    {
        return BSP_STATUS_IO_ERROR;
    }
    return status;
}

/**
 * @brief 读取归一化值（0~1）
 * @param me 基类指针
 * @param normalized_value 输出归一化浮点数
 * @param timeout_ms 超时时间
 * @return 执行状态
 */
bsp_status_t bsp_adc_read_normalized(bsp_adc_t *const me, float *normalized_value,
                                     uint32_t timeout_ms)
{
    uint32_t raw_value;
    bsp_status_t status;
    if (normalized_value == NULL) /* 输出指针非空检查 */
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    /* 先读取原始值 */
    status = bsp_adc_read_raw(me, &raw_value, timeout_ms);
    if (status == BSP_STATUS_OK)
    {
        /* 计算归一化：原始值 / 最大原始值（浮点运算） */
        *normalized_value = (float)raw_value / (float)me->maximum_raw_value;
    }
    return status;
}

/**
 * @brief 读取电压值（根据参考电压换算）
 * @param me 基类指针
 * @param voltage_v 输出电压值（单位伏特）
 * @param timeout_ms 超时时间
 * @return 执行状态
 */
bsp_status_t bsp_adc_read_voltage(bsp_adc_t *const me, float *voltage_v, uint32_t timeout_ms)
{
    float normalized_value;
    bsp_status_t status;
    if (voltage_v == NULL) /* 输出指针非空检查 */
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    /* 先读取归一化值 */
    status = bsp_adc_read_normalized(me, &normalized_value, timeout_ms);
    if (status == BSP_STATUS_OK)
    {
        /* 电压 = 归一化值 * 参考电压 */
        *voltage_v = normalized_value * me->reference_voltage_v;
    }
    return status;
}

/**
 * @brief 启动 DMA 采样（公共接口）
 * @param me 基类指针
 * @param sample_buffer 用户提供的缓冲区
 * @param sample_count 样本数量
 * @return 执行状态，若缓冲区为空或数量为0返回无效参数
 */
bsp_status_t bsp_adc_start_dma(bsp_adc_t *const me, uint32_t *sample_buffer, size_t sample_count)
{
    bsp_status_t status = bsp_adc_validate(me);
    /* 校验对象有效后，检查缓冲区和数量合法性 */
    if ((status == BSP_STATUS_OK) && ((sample_buffer == NULL) || (sample_count == 0U)))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    /* 转发至具体虚表函数 */
    return (status == BSP_STATUS_OK)
               ? bsp_adc_get_ops(me)->start_dma(me, sample_buffer, sample_count)
               : status;
}

/**
 * @brief 向应用层通知 ADC 事件（由底层驱动调用）
 * @param me 基类指针
 * @param event 事件类型
 * @param status 状态码
 * @param transferred_size 已传输的数据量（如 DMA 计数）
 */
void bsp_adc_notify(bsp_adc_t *const me, bsp_event_t event, bsp_status_t status,
                    size_t transferred_size)
{
    /* 只有在对象有效、已初始化且回调非空时才调用 */
    if ((me != NULL) && bsp_device_is_initialized(&me->super) && (me->callback != NULL))
    {
        me->callback(event, status, transferred_size, me->user_context);
    }
}
