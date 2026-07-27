/**
 * @file bsp_encoder.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 增量编码器通用抽象层实现
 * @note 提供编码器计数、方向读取及带模数回绕处理的增量计算。
 * @version 1.0
 * @date 2026-07-27
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "bsp_encoder.h" // 包含编码器抽象层头文件
#include <limits.h>      // 提供 INT32_MAX / INT32_MIN
#include <stddef.h>      // 提供 NULL

/**
 * @brief 从基类指针获取派生设备对象（非常量版本）
 * @param encoder_base bsp_encoder_t 基类指针
 * @return 对应的 bsp_encoder_device_t 对象指针
 */
static bsp_encoder_device_t *bsp_encoder_get_device(bsp_encoder_t *const encoder_base)
{
    // 利用 container_of 宏从基类成员地址反推出包含它的结构体地址
    return BSP_CONTAINER_OF(encoder_base, bsp_encoder_device_t, super);
}

/**
 * @brief 从基类指针获取派生设备对象（常量版本）
 * @param encoder_base const bsp_encoder_t 指针
 * @return 对应的 const bsp_encoder_device_t 指针
 */
static const bsp_encoder_device_t *
bsp_encoder_get_device_const(const bsp_encoder_t *const encoder_base)
{
    return BSP_CONTAINER_OF_CONST(encoder_base, bsp_encoder_device_t, super);
}

/**
 * @brief 从基类虚表指针获取高层操作表
 * @param encoder_base const bsp_encoder_t 指针
 * @return 对应的 bsp_encoder_ops_t 操作表指针（只读）
 */
static const bsp_encoder_ops_t *bsp_encoder_get_ops(const bsp_encoder_t *const encoder_base)
{
    // 基类 bsp_device_t 的 vptr 指向 bsp_encoder_ops_t 中的 super 成员
    return BSP_CONTAINER_OF_CONST(encoder_base->super.vptr, bsp_encoder_ops_t, super);
}

/**
 * @brief 编码器设备反初始化（作为 device 层的 deinit 回调）
 * @param device_base bsp_device_t 基类指针
 * @return 执行状态
 */
static bsp_status_t bsp_encoder_device_deinit(bsp_device_t *const device_base)
{
    // 从 device 基类反推出 bsp_encoder_t 基类地址
    bsp_encoder_t *const encoder_base = BSP_CONTAINER_OF(device_base, bsp_encoder_t, super);
    bsp_encoder_device_t *const me = bsp_encoder_get_device(encoder_base);
    // 如果驱动没有提供 deinit，视为无需清理，直接成功
    if (me->driver_ops->deinit == NULL)
    {
        return BSP_STATUS_OK;
    }
    // 调用底层驱动的 deinit，传入设备句柄
    return me->driver_ops->deinit(device_base->device_handle);
}

/**
 * @brief 启动编码器计数（转发至底层驱动）
 * @param encoder_base 基类指针
 * @return 执行状态
 */
static bsp_status_t bsp_encoder_device_start(bsp_encoder_t *const encoder_base)
{
    bsp_encoder_device_t *const me = bsp_encoder_get_device(encoder_base);
    // 调用驱动层的 start，传入设备句柄
    return me->driver_ops->start(encoder_base->super.device_handle);
}

/**
 * @brief 停止编码器计数（转发至底层驱动）
 */
static bsp_status_t bsp_encoder_device_stop(bsp_encoder_t *const encoder_base)
{
    bsp_encoder_device_t *const me = bsp_encoder_get_device(encoder_base);
    return me->driver_ops->stop(encoder_base->super.device_handle);
}

/**
 * @brief 设置当前计数值（转发至底层驱动）
 */
static bsp_status_t bsp_encoder_device_set_count(bsp_encoder_t *const encoder_base, int32_t count)
{
    bsp_encoder_device_t *const me = bsp_encoder_get_device(encoder_base);
    return me->driver_ops->set_count(encoder_base->super.device_handle, count);
}

/**
 * @brief 获取当前计数值（转发至底层驱动）
 */
static bsp_status_t bsp_encoder_device_get_count(const bsp_encoder_t *const encoder_base,
                                                 int32_t *count)
{
    const bsp_encoder_device_t *const me = bsp_encoder_get_device_const(encoder_base);
    return me->driver_ops->get_count(encoder_base->super.device_handle, count);
}

/**
 * @brief 获取旋转方向（转发至底层驱动）
 */
static bsp_status_t bsp_encoder_device_get_direction(const bsp_encoder_t *const encoder_base,
                                                     bsp_encoder_direction_t *direction)
{
    const bsp_encoder_device_t *const me = bsp_encoder_get_device_const(encoder_base);
    return me->driver_ops->get_direction(encoder_base->super.device_handle, direction);
}

/* 定义编码器设备层的操作表（虚表），将所有转发函数填入 */
static const bsp_encoder_ops_t s_bsp_encoder_device_ops = {
    .super = {.deinit = bsp_encoder_device_deinit},   // 继承自 device 的 deinit
    .start = bsp_encoder_device_start,                // 启动转发
    .stop = bsp_encoder_device_stop,                  // 停止转发
    .set_count = bsp_encoder_device_set_count,        // 设置计数值转发
    .get_count = bsp_encoder_device_get_count,        // 获取计数值转发
    .get_direction = bsp_encoder_device_get_direction // 获取方向转发
};

/**
 * @brief 校验编码器对象是否有效且已初始化
 * @param me bsp_encoder_t 指针
 * @return 状态，成功则 BSP_STATUS_OK
 */
static bsp_status_t bsp_encoder_validate(const bsp_encoder_t *const me)
{
    if (me == NULL)
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    // 调用底层 device 的初始化状态检查
    return bsp_device_is_initialized(&me->super) ? BSP_STATUS_OK : BSP_STATUS_NOT_INITIALIZED;
}

/**
 * @brief 初始化编码器设备实例
 * @param me 设备对象指针
 * @param config 配置参数指针
 * @return 执行状态
 */
bsp_status_t bsp_encoder_init(bsp_encoder_device_t *const me,
                              const bsp_encoder_config_t *const config)
{
    bsp_status_t status;
    // 参数合法性检查：对象、配置、设备句柄、驱动表、必须实现
    // start/stop/set_count/get_count/get_direction， 若模数非零则必须 >=2（模数 0
    // 表示无回绕，有效模数最小为 2）
    if ((me == NULL) || (config == NULL) || (config->device_handle == NULL) ||
        (config->driver_ops == NULL) || (config->driver_ops->start == NULL) ||
        (config->driver_ops->stop == NULL) || (config->driver_ops->set_count == NULL) ||
        (config->driver_ops->get_count == NULL) || (config->driver_ops->get_direction == NULL) ||
        ((config->counter_modulus != 0U) && (config->counter_modulus < 2U)))
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
    // 初始化基类字段：previous_count 设为 0，保存模数
    me->super.previous_count = 0;
    me->super.counter_modulus = config->counter_modulus;
    // 调用 device 基类初始化，注册虚表并保存设备句柄
    return bsp_device_init(&me->super.super, &s_bsp_encoder_device_ops.super,
                           config->device_handle);
}

/**
 * @brief 将派生对象转为基类指针（向上转型）
 * @param me 派生对象指针
 * @return 基类指针，若输入为空则返回 NULL
 */
bsp_encoder_t *bsp_encoder_as_base(bsp_encoder_device_t *const me)
{
    return (me != NULL) ? &me->super : NULL;
}

/**
 * @brief 启动编码器计数（公共接口）
 * @param me 基类指针
 * @return 执行状态
 */
bsp_status_t bsp_encoder_start(bsp_encoder_t *const me)
{
    bsp_status_t status = bsp_encoder_validate(me);
    return (status == BSP_STATUS_OK) ? bsp_encoder_get_ops(me)->start(me) : status;
}

/**
 * @brief 停止编码器计数（公共接口）
 */
bsp_status_t bsp_encoder_stop(bsp_encoder_t *const me)
{
    bsp_status_t status = bsp_encoder_validate(me);
    return (status == BSP_STATUS_OK) ? bsp_encoder_get_ops(me)->stop(me) : status;
}

/**
 * @brief 复位计数值为 0（公共接口）
 * @param me 基类指针
 * @return 执行状态
 * @note 同时清除内部历史计数值，避免下一次 delta 计算错误
 */
bsp_status_t bsp_encoder_reset(bsp_encoder_t *const me)
{
    bsp_status_t status = bsp_encoder_set_count(me, 0);
    if (status == BSP_STATUS_OK)
    {
        // 同时清除内部历史计数值，避免下一次 delta 计算错误
        me->previous_count = 0;
    }
    return status;
}

/**
 * @brief 设置当前计数值（公共接口）
 */
bsp_status_t bsp_encoder_set_count(bsp_encoder_t *const me, int32_t count)
{
    bsp_status_t status = bsp_encoder_validate(me);
    return (status == BSP_STATUS_OK) ? bsp_encoder_get_ops(me)->set_count(me, count) : status;
}

/**
 * @brief 获取当前计数值（公共接口）
 */
bsp_status_t bsp_encoder_get_count(const bsp_encoder_t *const me, int32_t *count)
{
    bsp_status_t status = bsp_encoder_validate(me);
    if ((status == BSP_STATUS_OK) && (count == NULL))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    return (status == BSP_STATUS_OK) ? bsp_encoder_get_ops(me)->get_count(me, count) : status;
}

/**
 * @brief 获取自上次调用以来的计数值增量（带模数回绕处理）
 * @param me 基类指针（非常量，因为会更新 previous_count）
 * @param count_delta 输出增量（有符号，范围 int32_t）
 * @return 执行状态
 * @note 该函数有状态：内部保存 previous_count，每次调用后更新为当前值。
 *       调用者需确保采样周期内真实增量不超过半个模数，否则回绕判断失效。
 */
bsp_status_t bsp_encoder_get_delta(bsp_encoder_t *const me, int32_t *count_delta)
{
    int32_t current_count;
    int64_t count_difference;
    bsp_status_t status;
    if (count_delta == NULL)
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    // 获取当前计数值
    status = bsp_encoder_get_count(me, &current_count);
    if (status == BSP_STATUS_OK)
    {
        // 计算与上次记录的差值（用 64 位防止溢出）
        count_difference = (int64_t)current_count - (int64_t)me->previous_count;
        // 如果设定了模数（>0），则处理回绕
        if (me->counter_modulus > 0U)
        {
            const int64_t modulus = (int64_t)me->counter_modulus;
            const int64_t half_modulus = modulus / 2;
            // 若差值大于半模数，认为是负向回绕（实际增量应为负）
            if (count_difference > half_modulus)
            {
                count_difference -= modulus;
            }
            // 若差值小于负半模数，认为是正向回绕（实际增量应为正）
            else if (count_difference < -half_modulus)
            {
                count_difference += modulus;
            }
            // 否则无需调整
        }
        // 检查结果是否在 int32 范围内
        if ((count_difference > INT32_MAX) || (count_difference < INT32_MIN))
        {
            return BSP_STATUS_OUT_OF_RANGE;
        }
        *count_delta = (int32_t)count_difference;
        // 更新历史值
        me->previous_count = current_count;
    }
    return status;
}

/**
 * @brief 获取旋转方向（公共接口）
 * @param me 基类指针（const）
 * @param direction 输出方向枚举
 * @return 执行状态
 */
bsp_status_t bsp_encoder_get_direction(const bsp_encoder_t *const me,
                                       bsp_encoder_direction_t *direction)
{
    bsp_status_t status = bsp_encoder_validate(me);
    if ((status == BSP_STATUS_OK) && (direction == NULL))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    return (status == BSP_STATUS_OK) ? bsp_encoder_get_ops(me)->get_direction(me, direction)
                                     : status;
}