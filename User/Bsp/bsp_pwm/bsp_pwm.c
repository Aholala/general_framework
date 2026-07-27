/**
 * @file bsp_pwm.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief PWM 通用抽象层实现
 * @note 支持 PWM 的启动/停止、频率设置、脉冲宽度（比较值）和归一化占空比。
 *       定时器实例、通道号和引脚复用通过平台句柄与配置注入。
 * @version 1.0
 * @date 2026-07-25
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include "bsp_pwm.h" // 包含 PWM 抽象层头文件
#include <math.h>    // 提供 isfinite() 浮点检测
#include <stddef.h>  // 提供 NULL

/**
 * @brief 从基类指针获取派生设备对象（非常量版本）
 * @param pwm_base bsp_pwm_t 基类指针
 * @return 对应的 bsp_pwm_device_t 对象指针
 */
static bsp_pwm_device_t *bsp_pwm_get_device(bsp_pwm_t *const pwm_base)
{
    // 利用 container_of 宏从基类成员地址反推出包含它的结构体地址
    return BSP_CONTAINER_OF(pwm_base, bsp_pwm_device_t, super);
}

/**
 * @brief 从基类指针获取派生设备对象（常量版本）
 * @param pwm_base const bsp_pwm_t 指针
 * @return 对应的 const bsp_pwm_device_t 指针
 */
static const bsp_pwm_device_t *bsp_pwm_get_device_const(const bsp_pwm_t *const pwm_base)
{
    return BSP_CONTAINER_OF_CONST(pwm_base, bsp_pwm_device_t, super);
}

/**
 * @brief 从基类虚表指针获取高层操作表
 * @param pwm_base const bsp_pwm_t 指针
 * @return 对应的 bsp_pwm_ops_t 操作表指针（只读）
 */
static const bsp_pwm_ops_t *bsp_pwm_get_ops(const bsp_pwm_t *const pwm_base)
{
    // 基类 bsp_device_t 的 vptr 指向 bsp_pwm_ops_t 中的 super 成员
    return BSP_CONTAINER_OF_CONST(pwm_base->super.vptr, bsp_pwm_ops_t, super);
}

/**
 * @brief PWM 设备反初始化（作为 device 层的 deinit 回调）
 * @param device_base bsp_device_t 基类指针
 * @return 执行状态
 */
static bsp_status_t bsp_pwm_device_deinit(bsp_device_t *const device_base)
{
    // 从 device 基类反推出 bsp_pwm_t 基类地址
    bsp_pwm_t *const pwm_base = BSP_CONTAINER_OF(device_base, bsp_pwm_t, super);
    bsp_pwm_device_t *const me = bsp_pwm_get_device(pwm_base);
    // 如果驱动没有提供 deinit，视为无需清理，直接成功
    if (me->driver_ops->deinit == NULL)
    {
        return BSP_STATUS_OK;
    }
    // 调用底层驱动的 deinit，传入设备句柄和通道号
    return me->driver_ops->deinit(device_base->device_handle, me->channel);
}

/**
 * @brief 启动 PWM 输出（转发至底层驱动）
 * @param pwm_base 基类指针
 * @return 执行状态
 */
static bsp_status_t bsp_pwm_device_start(bsp_pwm_t *const pwm_base)
{
    bsp_pwm_device_t *const me = bsp_pwm_get_device(pwm_base);
    // 调用驱动层的 start，传入设备句柄和通道号
    return me->driver_ops->start(pwm_base->super.device_handle, me->channel);
}

/**
 * @brief 停止 PWM 输出（转发至底层驱动）
 * @param pwm_base 基类指针
 * @return 执行状态
 */
static bsp_status_t bsp_pwm_device_stop(bsp_pwm_t *const pwm_base)
{
    bsp_pwm_device_t *const me = bsp_pwm_get_device(pwm_base);
    return me->driver_ops->stop(pwm_base->super.device_handle, me->channel);
}

/**
 * @brief 设置 PWM 频率（转发至底层驱动）
 * @param pwm_base 基类指针
 * @param frequency_hz 频率（Hz）
 * @return 执行状态
 */
static bsp_status_t bsp_pwm_device_set_frequency(bsp_pwm_t *const pwm_base, uint32_t frequency_hz)
{
    bsp_pwm_device_t *const me = bsp_pwm_get_device(pwm_base);
    return me->driver_ops->set_frequency(pwm_base->super.device_handle, me->channel, frequency_hz);
}

/**
 * @brief 获取 PWM 频率（转发至底层驱动）
 * @param pwm_base 基类指针（const）
 * @param frequency_hz 输出频率（Hz）
 * @return 执行状态
 */
static bsp_status_t bsp_pwm_device_get_frequency(const bsp_pwm_t *const pwm_base,
                                                 uint32_t *frequency_hz)
{
    const bsp_pwm_device_t *const me = bsp_pwm_get_device_const(pwm_base);
    return me->driver_ops->get_frequency(pwm_base->super.device_handle, me->channel, frequency_hz);
}

/**
 * @brief 设置脉冲宽度（比较值，转发至底层驱动）
 * @param pwm_base 基类指针
 * @param pulse_ticks 高电平比较值（以定时器 tick 为单位）
 * @return 执行状态
 */
static bsp_status_t bsp_pwm_device_set_pulse(bsp_pwm_t *const pwm_base, uint32_t pulse_ticks)
{
    bsp_pwm_device_t *const me = bsp_pwm_get_device(pwm_base);
    return me->driver_ops->set_pulse(pwm_base->super.device_handle, me->channel, pulse_ticks);
}

/**
 * @brief 获取脉冲宽度（比较值，转发至底层驱动）
 * @param pwm_base 基类指针（const）
 * @param pulse_ticks 输出高电平比较值
 * @return 执行状态
 */
static bsp_status_t bsp_pwm_device_get_pulse(const bsp_pwm_t *const pwm_base, uint32_t *pulse_ticks)
{
    const bsp_pwm_device_t *const me = bsp_pwm_get_device_const(pwm_base);
    return me->driver_ops->get_pulse(pwm_base->super.device_handle, me->channel, pulse_ticks);
}

/**
 * @brief 获取周期值（转发至底层驱动）
 * @param pwm_base 基类指针（const）
 * @param period_ticks 输出周期值（以定时器 tick 为单位）
 * @return 执行状态
 */
static bsp_status_t bsp_pwm_device_get_period(const bsp_pwm_t *const pwm_base,
                                              uint32_t *period_ticks)
{
    const bsp_pwm_device_t *const me = bsp_pwm_get_device_const(pwm_base);
    return me->driver_ops->get_period(pwm_base->super.device_handle, me->channel, period_ticks);
}

/* 定义 PWM 设备层的操作表（虚表），将所有转发函数填入 */
static const bsp_pwm_ops_t s_bsp_pwm_device_ops = {
    .super = {.deinit = bsp_pwm_device_deinit},    // 继承自 device 的 deinit
    .start = bsp_pwm_device_start,                 // 启动转发
    .stop = bsp_pwm_device_stop,                   // 停止转发
    .set_frequency = bsp_pwm_device_set_frequency, // 设置频率转发
    .get_frequency = bsp_pwm_device_get_frequency, // 获取频率转发
    .set_pulse = bsp_pwm_device_set_pulse,         // 设置脉冲宽度转发
    .get_pulse = bsp_pwm_device_get_pulse,         // 获取脉冲宽度转发
    .get_period = bsp_pwm_device_get_period,       // 获取周期转发
};

/**
 * @brief 校验 PWM 对象是否有效且已初始化
 * @param me bsp_pwm_t 指针
 * @return 状态，成功则 BSP_STATUS_OK
 */
static bsp_status_t bsp_pwm_validate(const bsp_pwm_t *const me)
{
    if (me == NULL)
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    // 调用底层 device 的初始化状态检查
    return bsp_device_is_initialized(&me->super) ? BSP_STATUS_OK : BSP_STATUS_NOT_INITIALIZED;
}

/**
 * @brief 初始化 PWM 设备实例
 * @param me 设备对象指针
 * @param config 配置参数指针
 * @return 执行状态
 */
bsp_status_t bsp_pwm_init(bsp_pwm_device_t *const me, const bsp_pwm_config_t *const config)
{
    bsp_status_t status;
    // 参数合法性检查：对象、配置、设备句柄、驱动表、必须实现所有关键接口
    if ((me == NULL) || (config == NULL) || (config->device_handle == NULL) ||
        (config->driver_ops == NULL) || (config->driver_ops->start == NULL) ||
        (config->driver_ops->stop == NULL) || (config->driver_ops->set_frequency == NULL) ||
        (config->driver_ops->get_frequency == NULL) || (config->driver_ops->set_pulse == NULL) ||
        (config->driver_ops->get_pulse == NULL) || (config->driver_ops->get_period == NULL))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    // 预先标记为未初始化，避免中途失败时留下错误状态
    me->super.super.is_initialized = false;
    // 保存底层驱动操作表和逻辑通道号
    me->driver_ops = config->driver_ops;
    me->channel = config->channel;
    // 如果驱动提供了 init 回调，则调用以初始化硬件
    if (me->driver_ops->init != NULL)
    {
        status = me->driver_ops->init(config->device_handle, config->channel);
        if (status != BSP_STATUS_OK)
        {
            return status; // 底层初始化失败则直接返回
        }
    }
    // 调用 device 基类初始化，注册虚表并保存设备句柄
    return bsp_device_init(&me->super.super, &s_bsp_pwm_device_ops.super, config->device_handle);
}

/**
 * @brief 将派生对象转为基类指针（向上转型）
 * @param me 派生对象指针
 * @return 基类指针，若输入为空则返回 NULL
 */
bsp_pwm_t *bsp_pwm_as_base(bsp_pwm_device_t *const me)
{
    return (me != NULL) ? &me->super : NULL;
}

/**
 * @brief 宏：为无附加参数的公共操作函数生成封装（含校验和转发）
 */
#define BSP_PWM_PUBLIC_ACTION(name, member)                                                        \
    bsp_status_t name(bsp_pwm_t *const me)                                                         \
    {                                                                                              \
        bsp_status_t status = bsp_pwm_validate(me);                                                \
        return (status == BSP_STATUS_OK) ? bsp_pwm_get_ops(me)->member(me) : status;               \
    }

// 生成公共接口：启动、停止
BSP_PWM_PUBLIC_ACTION(bsp_pwm_start, start) // 公共启动接口
BSP_PWM_PUBLIC_ACTION(bsp_pwm_stop, stop)   // 公共停止接口

/**
 * @brief 设置 PWM 频率（公共接口）
 * @param me 基类指针
 * @param frequency_hz 频率（Hz）
 * @return 执行状态，频率为 0 时返回 OUT_OF_RANGE
 */
bsp_status_t bsp_pwm_set_frequency(bsp_pwm_t *const me, uint32_t frequency_hz)
{
    bsp_status_t status = bsp_pwm_validate(me);
    // 频率必须大于 0
    if ((status == BSP_STATUS_OK) && (frequency_hz == 0U))
    {
        return BSP_STATUS_OUT_OF_RANGE;
    }
    // 校验通过后通过虚表调用 set_frequency
    return (status == BSP_STATUS_OK) ? bsp_pwm_get_ops(me)->set_frequency(me, frequency_hz)
                                     : status;
}

/**
 * @brief 获取 PWM 频率（公共接口）
 * @param me 基类指针（const）
 * @param frequency_hz 输出频率（Hz）
 * @return 执行状态
 */
bsp_status_t bsp_pwm_get_frequency(const bsp_pwm_t *const me, uint32_t *frequency_hz)
{
    bsp_status_t status = bsp_pwm_validate(me);
    // 输出指针非空检查
    if ((status == BSP_STATUS_OK) && (frequency_hz == NULL))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    // 校验通过后通过虚表调用 get_frequency
    return (status == BSP_STATUS_OK) ? bsp_pwm_get_ops(me)->get_frequency(me, frequency_hz)
                                     : status;
}

/**
 * @brief 设置脉冲宽度（比较值，公共接口）
 * @param me 基类指针
 * @param pulse_ticks 高电平比较值（以定时器 tick 为单位）
 * @return 执行状态，若 pulse_ticks 超过周期值则返回 OUT_OF_RANGE
 */
bsp_status_t bsp_pwm_set_pulse(bsp_pwm_t *const me, uint32_t pulse_ticks)
{
    uint32_t period_ticks;
    bsp_status_t status = bsp_pwm_validate(me);
    if (status != BSP_STATUS_OK)
    {
        return status;
    }
    // 先获取周期值，用于校验 pulse_ticks 是否越界
    status = bsp_pwm_get_ops(me)->get_period(me, &period_ticks);
    if (status != BSP_STATUS_OK)
    {
        return status;
    }
    // 脉冲宽度不能超过周期值
    if (pulse_ticks > period_ticks)
    {
        return BSP_STATUS_OUT_OF_RANGE;
    }
    // 校验通过后通过虚表调用 set_pulse
    return bsp_pwm_get_ops(me)->set_pulse(me, pulse_ticks);
}

/**
 * @brief 获取脉冲宽度（比较值，公共接口）
 * @param me 基类指针（const）
 * @param pulse_ticks 输出高电平比较值
 * @return 执行状态
 */
bsp_status_t bsp_pwm_get_pulse(const bsp_pwm_t *const me, uint32_t *pulse_ticks)
{
    bsp_status_t status = bsp_pwm_validate(me);
    uint32_t period_ticks;
    // 输出指针非空检查
    if ((status == BSP_STATUS_OK) && (pulse_ticks == NULL))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    if (status != BSP_STATUS_OK)
    {
        return status;
    }
    // 获取周期值
    status = bsp_pwm_get_ops(me)->get_period(me, &period_ticks);
    if (status != BSP_STATUS_OK)
    {
        return status;
    }
    // 获取脉冲宽度
    status = bsp_pwm_get_ops(me)->get_pulse(me, pulse_ticks);
    // 检查读取的值是否超过周期（硬件异常保护）
    if ((status == BSP_STATUS_OK) && (*pulse_ticks > period_ticks))
    {
        return BSP_STATUS_IO_ERROR;
    }
    return status;
}

/**
 * @brief 设置归一化占空比（公共接口）
 * @param me 基类指针
 * @param duty_cycle 占空比（0.0~1.0）
 * @return 执行状态
 */
bsp_status_t bsp_pwm_set_duty_cycle(bsp_pwm_t *const me, float duty_cycle)
{
    uint32_t period_ticks;
    bsp_status_t status = bsp_pwm_validate(me);
    if (status != BSP_STATUS_OK)
    {
        return status;
    }
    // 检查占空比是否为有限数且在 [0, 1] 范围内
    if (!isfinite(duty_cycle) || (duty_cycle < 0.0F) || (duty_cycle > 1.0F))
    {
        return BSP_STATUS_OUT_OF_RANGE;
    }
    // 获取周期值
    status = bsp_pwm_get_ops(me)->get_period(me, &period_ticks);
    if (status != BSP_STATUS_OK)
    {
        return status;
    }
    // 占空比 * 周期 + 0.5 四舍五入取整，然后设置脉冲宽度
    return bsp_pwm_get_ops(me)->set_pulse(me, (uint32_t)((float)period_ticks * duty_cycle + 0.5F));
}

/**
 * @brief 获取归一化占空比（公共接口）
 * @param me 基类指针（const）
 * @param duty_cycle 输出占空比（0.0~1.0）
 * @return 执行状态
 */
bsp_status_t bsp_pwm_get_duty_cycle(const bsp_pwm_t *const me, float *duty_cycle)
{
    uint32_t period_ticks;
    uint32_t pulse_ticks;
    bsp_status_t status = bsp_pwm_validate(me);
    // 输出指针非空检查
    if ((status == BSP_STATUS_OK) && (duty_cycle == NULL))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    if (status != BSP_STATUS_OK)
    {
        return status;
    }
    // 获取周期值
    status = bsp_pwm_get_ops(me)->get_period(me, &period_ticks);
    // 周期不能为 0（否则除零）
    if ((status != BSP_STATUS_OK) || (period_ticks == 0U))
    {
        return (status != BSP_STATUS_OK) ? status : BSP_STATUS_IO_ERROR;
    }
    // 获取脉冲宽度
    status = bsp_pwm_get_ops(me)->get_pulse(me, &pulse_ticks);
    if (status == BSP_STATUS_OK)
    {
        // 占空比 = 脉冲宽度 / 周期
        *duty_cycle = (float)pulse_ticks / (float)period_ticks;
    }
    return status;
}