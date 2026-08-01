/**
 * @file bsp_timebase.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 时间基准通用抽象层实现
 * @version 1.0
 * @date 2026-07-27
 * @copyright Copyright (c) 2026
 *
 * @note 基于自由运行周期计数器的单调时间基准，可由 Cortex DWT、
 *       通用 32 位定时器或其他硬件实现。提供时间点、回绕安全的
 *       耗时计算、周期/微秒转换和短延时。
 */

#include "bsp_timebase.h" // 包含时间基准抽象层头文件

BSP_STATIC_ASSERT_SUPER_FIRST(bsp_timebase_device_t);
#include <stddef.h>       // 提供 NULL

/** 每秒微秒数（用于周期与时间的换算） */
#define BSP_TIMEBASE_MICROSECONDS_PER_SECOND (1000000ULL)

/**
 * @brief 最大安全延时周期数
 * @note 为 UINT32_MAX / 2，防止忙等待循环溢出或误判
 *       超过此值的延时建议使用 RTOS 延时或其他机制
 */
#define BSP_TIMEBASE_MAXIMUM_SAFE_DELAY_CYCLES (UINT32_MAX / 2U)

/**
 * @brief 从基类指针获取派生设备对象（非常量版本）
 * @param me bsp_timebase_t 基类指针
 * @return 对应的 bsp_timebase_device_t 对象指针
 */
static bsp_timebase_device_t *bsp_timebase_get_device(bsp_timebase_t *const me)
{
    // 利用 container_of 宏从基类成员地址反推出包含它的结构体地址
    return BSP_CONTAINER_OF(me, bsp_timebase_device_t, super);
}

/**
 * @brief 从基类指针获取派生设备对象（常量版本）
 * @param me const bsp_timebase_t 指针
 * @return 对应的 const bsp_timebase_device_t 指针
 */
static const bsp_timebase_device_t *bsp_timebase_get_device_const(const bsp_timebase_t *const me)
{
    return BSP_CONTAINER_OF_CONST(me, bsp_timebase_device_t, super);
}

/**
 * @brief 从基类虚表指针获取高层操作表
 * @param me const bsp_timebase_t 指针
 * @return 对应的 bsp_timebase_ops_t 操作表指针（只读）
 */
static const bsp_timebase_ops_t *bsp_timebase_get_ops(const bsp_timebase_t *const me)
{
    // 基类 bsp_device_t 的 vptr 指向 bsp_timebase_ops_t 中的 super 成员
    return BSP_CONTAINER_OF_CONST(me->super.vptr, bsp_timebase_ops_t, super);
}

/**
 * @brief 时间基准设备反初始化（作为 device 层的 deinit 回调）
 * @param device_base bsp_device_t 基类指针
 * @return 执行状态
 */
static bsp_status_t bsp_timebase_device_deinit(bsp_device_t *const device_base)
{
    // 从 device 基类反推出 bsp_timebase_t 基类地址
    bsp_timebase_t *const timebase_base = BSP_CONTAINER_OF(device_base, bsp_timebase_t, super);
    bsp_timebase_device_t *const me = bsp_timebase_get_device(timebase_base);
    // 如果驱动没有提供 deinit，视为无需清理，直接成功
    return (me->driver_ops->deinit != NULL) ? me->driver_ops->deinit(device_base->device_handle)
                                            : BSP_STATUS_OK;
}

/**
 * @brief 复位周期计数器（转发至底层驱动，可选）
 * @param timebase_base 基类指针
 * @return 若驱动未实现则返回 BSP_STATUS_UNSUPPORTED
 */
static bsp_status_t bsp_timebase_device_reset(bsp_timebase_t *const timebase_base)
{
    bsp_timebase_device_t *const me = bsp_timebase_get_device(timebase_base);
    return (me->driver_ops->reset != NULL)
               ? me->driver_ops->reset(timebase_base->super.device_handle)
               : BSP_STATUS_UNSUPPORTED;
}

/**
 * @brief 获取当前周期计数值（转发至底层驱动）
 * @param timebase_base 基类指针（const）
 * @param cycle_count 输出周期计数值
 * @return 执行状态
 */
static bsp_status_t bsp_timebase_device_get_cycle_count(const bsp_timebase_t *const timebase_base,
                                                        uint32_t *cycle_count)
{
    const bsp_timebase_device_t *const me = bsp_timebase_get_device_const(timebase_base);
    // 调用驱动层的 get_cycle_count，传入设备句柄和输出指针
    return me->driver_ops->get_cycle_count(timebase_base->super.device_handle, cycle_count);
}

/**
 * @brief 获取时间基准频率（转发至底层驱动）
 * @param timebase_base 基类指针（const）
 * @param frequency_hz 输出频率（Hz）
 * @return 执行状态
 */
static bsp_status_t bsp_timebase_device_get_frequency(const bsp_timebase_t *const timebase_base,
                                                      uint32_t *frequency_hz)
{
    const bsp_timebase_device_t *const me = bsp_timebase_get_device_const(timebase_base);
    return me->driver_ops->get_frequency(timebase_base->super.device_handle, frequency_hz);
}

/* 定义时间基准设备层的操作表（虚表） */
static const bsp_timebase_ops_t s_bsp_timebase_device_ops = {
    .super = {.deinit = bsp_timebase_device_deinit},        // 继承自 device 的 deinit
    .reset = bsp_timebase_device_reset,                     // 复位转发（可选）
    .get_cycle_count = bsp_timebase_device_get_cycle_count, // 获取周期计数转发
    .get_frequency = bsp_timebase_device_get_frequency,     // 获取频率转发
};

/**
 * @brief 校验时间基准对象是否有效且已初始化
 * @param me bsp_timebase_t 指针
 * @return 状态，成功则 BSP_STATUS_OK
 */
static bsp_status_t bsp_timebase_validate(const bsp_timebase_t *const me)
{
    if (me == NULL)
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    // 调用底层 device 的初始化状态检查
    return bsp_device_is_initialized(&me->super) ? BSP_STATUS_OK : BSP_STATUS_NOT_INITIALIZED;
}

/**
 * @brief 初始化时间基准设备实例
 * @param me 设备对象指针
 * @param config 配置参数指针
 * @return 执行状态
 */
bsp_status_t bsp_timebase_init(bsp_timebase_device_t *const me,
                               const bsp_timebase_config_t *const config)
{
    bsp_status_t status;

    // 参数合法性检查：对象、配置、设备句柄、驱动表、必须实现 get_cycle_count/get_frequency
    if ((me == NULL) || (config == NULL) || (config->device_handle == NULL) ||
        (config->driver_ops == NULL) || (config->driver_ops->get_cycle_count == NULL) ||
        (config->driver_ops->get_frequency == NULL))
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
    return bsp_device_init(&me->super.super, &s_bsp_timebase_device_ops.super,
                           config->device_handle);
}

/**
 * @brief 将派生对象转为基类指针（向上转型）
 * @param me 派生对象指针
 * @return 基类指针，若输入为空则返回 NULL
 */
bsp_timebase_t *bsp_timebase_as_base(bsp_timebase_device_t *const me)
{
    return (me != NULL) ? &me->super : NULL;
}

/**
 * @brief 复位周期计数器（公共接口）
 * @param me 基类指针
 * @return 执行状态，若驱动未实现则返回 BSP_STATUS_UNSUPPORTED
 */
bsp_status_t bsp_timebase_reset(bsp_timebase_t *const me)
{
    const bsp_status_t status = bsp_timebase_validate(me);
    // 校验通过后通过虚表调用 reset（可能返回 UNSUPPORTED）
    return (status == BSP_STATUS_OK) ? bsp_timebase_get_ops(me)->reset(me) : status;
}

/**
 * @brief 获取当前周期计数值（公共接口）
 * @param me 基类指针（const）
 * @param cycle_count 输出周期计数值
 * @return 执行状态
 */
bsp_status_t bsp_timebase_get_cycle_count(const bsp_timebase_t *const me, uint32_t *cycle_count)
{
    const bsp_status_t status = bsp_timebase_validate(me);

    if (status != BSP_STATUS_OK)
    {
        return status;
    }
    // 输出指针非空检查
    if (cycle_count == NULL)
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    // 通过虚表调用 get_cycle_count
    return bsp_timebase_get_ops(me)->get_cycle_count(me, cycle_count);
}

/**
 * @brief 获取时间基准频率（公共接口）
 * @param me 基类指针（const）
 * @param frequency_hz 输出频率（Hz）
 * @return 执行状态，若频率为 0 则返回 BSP_STATUS_IO_ERROR
 */
bsp_status_t bsp_timebase_get_frequency(const bsp_timebase_t *const me, uint32_t *frequency_hz)
{
    bsp_status_t status = bsp_timebase_validate(me);

    if (status != BSP_STATUS_OK)
    {
        return status;
    }
    // 输出指针非空检查
    if (frequency_hz == NULL)
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    // 通过虚表调用 get_frequency
    status = bsp_timebase_get_ops(me)->get_frequency(me, frequency_hz);
    // 频率为 0 表示硬件异常（不可能为 0）
    if ((status == BSP_STATUS_OK) && (*frequency_hz == 0U))
    {
        return BSP_STATUS_IO_ERROR;
    }
    return status;
}

/**
 * @brief 获取当前时间点（公共接口）
 * @param me 基类指针（const）
 * @param time_point 输出时间点结构体
 * @return 执行状态
 * @note 时间点本质上是当前周期计数值的快照
 */
bsp_status_t bsp_timebase_now(const bsp_timebase_t *const me, bsp_timebase_time_point_t *time_point)
{
    // 输出指针非空检查
    if (time_point == NULL)
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    // 获取当前周期计数并存入 time_point
    return bsp_timebase_get_cycle_count(me, &time_point->cycle_count);
}

/**
 * @brief 计算从起始时间点到现在的经过周期数
 * @param me 基类指针（const）
 * @param start_time 起始时间点
 * @param elapsed_cycles 输出经过的周期数
 * @return 执行状态
 * @note 使用无符号减法，正确处理一次 32 位回绕
 *       若测量时间超过完整计数周期，将无法区分多次回绕
 */
bsp_status_t bsp_timebase_elapsed_cycles(const bsp_timebase_t *const me,
                                         bsp_timebase_time_point_t start_time,
                                         uint32_t *elapsed_cycles)
{
    uint32_t current_cycle_count;
    bsp_status_t status;

    if (elapsed_cycles == NULL)
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    // 获取当前周期计数值
    status = bsp_timebase_get_cycle_count(me, &current_cycle_count);
    if (status != BSP_STATUS_OK)
    {
        return status;
    }
    // 无符号减法：current - start，正确处理 32 位回绕
    *elapsed_cycles = current_cycle_count - start_time.cycle_count;
    return BSP_STATUS_OK;
}

/**
 * @brief 将周期数转换为微秒数
 * @param me 基类指针（const）
 * @param cycle_count 周期数
 * @param time_us 输出微秒数
 * @return 执行状态，若结果超出 uint32_t 范围则返回 BSP_STATUS_OUT_OF_RANGE
 */
bsp_status_t bsp_timebase_cycles_to_us(const bsp_timebase_t *const me, uint32_t cycle_count,
                                       uint32_t *time_us)
{
    uint32_t frequency_hz;
    uint64_t calculated_time_us;
    bsp_status_t status;

    if (time_us == NULL)
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    // 获取频率
    status = bsp_timebase_get_frequency(me, &frequency_hz);
    if (status != BSP_STATUS_OK)
    {
        return status;
    }
    // 换算：微秒 = 周期数 * 1,000,000 / 频率
    calculated_time_us =
        ((uint64_t)cycle_count * BSP_TIMEBASE_MICROSECONDS_PER_SECOND) / frequency_hz;
    // 检查结果是否超出 uint32_t 范围（约 4294 秒）
    if (calculated_time_us > UINT32_MAX)
    {
        return BSP_STATUS_OUT_OF_RANGE;
    }
    *time_us = (uint32_t)calculated_time_us;
    return BSP_STATUS_OK;
}

/**
 * @brief 将微秒数转换为周期数（向上取整）
 * @param me 基类指针（const）
 * @param time_us 微秒数
 * @param cycle_count 输出周期数
 * @return 执行状态，若结果超出 uint32_t 范围则返回 BSP_STATUS_OUT_OF_RANGE
 * @note 采用向上取整（+频率-1）/频率，确保至少达到所需时间
 */
bsp_status_t bsp_timebase_us_to_cycles(const bsp_timebase_t *const me, uint32_t time_us,
                                       uint32_t *cycle_count)
{
    uint32_t frequency_hz;
    uint64_t calculated_cycle_count;
    bsp_status_t status;

    if (cycle_count == NULL)
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    // 获取频率
    status = bsp_timebase_get_frequency(me, &frequency_hz);
    if (status != BSP_STATUS_OK)
    {
        return status;
    }
    // 换算：周期数 = (微秒 * 频率 + 999999) / 1,000,000（向上取整）
    calculated_cycle_count =
        ((uint64_t)time_us * frequency_hz + BSP_TIMEBASE_MICROSECONDS_PER_SECOND - 1ULL) /
        BSP_TIMEBASE_MICROSECONDS_PER_SECOND;
    // 检查结果是否超出 uint32_t 范围
    if (calculated_cycle_count > UINT32_MAX)
    {
        return BSP_STATUS_OUT_OF_RANGE;
    }
    *cycle_count = (uint32_t)calculated_cycle_count;
    return BSP_STATUS_OK;
}

/**
 * @brief 微秒级忙等待延时
 * @param me 基类指针（const）
 * @param delay_us 延时微秒数
 * @return 执行状态
 * @note 同步忙等待，占用 CPU。适用于芯片上电、传感器复位或极短硬件时序
 *       不得在高优先级 ISR 中延迟，也不应代替 RTOS 延时
 */
bsp_status_t bsp_timebase_delay_us(const bsp_timebase_t *const me, uint32_t delay_us)
{
    bsp_timebase_time_point_t start_time;
    uint32_t required_cycles;
    uint32_t elapsed_cycles;
    bsp_status_t status;

    // 将微秒转换为周期数
    status = bsp_timebase_us_to_cycles(me, delay_us, &required_cycles);
    if (status != BSP_STATUS_OK)
    {
        return status;
    }
    // 检查延时是否超出安全范围（避免循环溢出）
    if (required_cycles > BSP_TIMEBASE_MAXIMUM_SAFE_DELAY_CYCLES)
    {
        return BSP_STATUS_OUT_OF_RANGE;
    }
    // 记录起始时间点
    status = bsp_timebase_now(me, &start_time);
    if (status != BSP_STATUS_OK)
    {
        return status;
    }
    // 忙等待循环
    do
    {
        status = bsp_timebase_elapsed_cycles(me, start_time, &elapsed_cycles);
        if (status != BSP_STATUS_OK)
        {
            return status;
        }
    } while (elapsed_cycles < required_cycles);
    return BSP_STATUS_OK;
}

/**
 * @brief 检查从起始时间点是否已经过指定微秒数
 * @param me 基类指针（const）
 * @param start_time 起始时间点
 * @param duration_us 持续时间（微秒）
 * @param has_elapsed 输出是否已超时
 * @return 执行状态
 * @note 非阻塞，用于超时判断，适合在循环中调用
 */
bsp_status_t bsp_timebase_has_elapsed_us(const bsp_timebase_t *const me,
                                         bsp_timebase_time_point_t start_time, uint32_t duration_us,
                                         bool *has_elapsed)
{
    uint32_t required_cycles;
    uint32_t elapsed_cycles;
    bsp_status_t status;

    if (has_elapsed == NULL)
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    // 将微秒转换为周期数
    status = bsp_timebase_us_to_cycles(me, duration_us, &required_cycles);
    if (status != BSP_STATUS_OK)
    {
        return status;
    }
    // 检查持续时间是否超出安全范围
    if (required_cycles > BSP_TIMEBASE_MAXIMUM_SAFE_DELAY_CYCLES)
    {
        return BSP_STATUS_OUT_OF_RANGE;
    }
    // 计算已经过的周期数
    status = bsp_timebase_elapsed_cycles(me, start_time, &elapsed_cycles);
    if (status != BSP_STATUS_OK)
    {
        return status;
    }
    // 判断是否超时
    *has_elapsed = elapsed_cycles >= required_cycles;
    return BSP_STATUS_OK;
}
