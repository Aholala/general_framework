/**
 * @file bsp_crc.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief CRC 硬件抽象层实现
 * @note 提供 CRC 计算的多态接口，通过虚表转发到底层驱动。
 * @version 1.0
 * @date 2026-07-27
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "bsp_crc.h"

/**
 * @brief CRC 设备虚析构函数（作为 bsp_device_ops_t 的 deinit 回调）
 * @param device bsp_device_t 基类指针
 * @return 执行状态
 * @note 该函数由 bsp_device_deinit 在析构时调用
 */
static bsp_status_t bsp_crc_deinit_virtual(bsp_device_t *device)
{
    // 从基类指针获取派生设备对象
    bsp_crc_device_t *const me = BSP_CONTAINER_OF(device, bsp_crc_device_t, super.super);
    // 调用底层驱动的 deinit，传入设备句柄
    const bsp_status_t status = me->driver_ops->deinit(device->device_handle);
    // 仅在析构成功时清除初始化标志（基类会在 bsp_device_deinit 中清空其他字段）
    if (status == BSP_STATUS_OK)
    {
        device->is_initialized = false;
    }
    return status;
}

/**
 * @brief CRC 计算虚函数（作为 bsp_crc_ops_t 的 calculate 回调）
 * @param me bsp_crc_t 基类指针
 * @param data 待计算的数据指针
 * @param size 数据大小（字节）
 * @param initial_value 初始值（用于累加/链式计算）
 * @param result 输出计算结果
 * @return 执行状态
 */
static bsp_status_t bsp_crc_calculate_virtual(bsp_crc_t *me, const void *data, size_t size,
                                              uint32_t initial_value, uint32_t *result)
{
    // 从基类指针获取派生设备对象
    bsp_crc_device_t *const device = BSP_CONTAINER_OF(me, bsp_crc_device_t, super);
    // 转发到底层驱动，传入设备句柄、数据、大小、初值、结果指针
    return device->driver_ops->calculate(me->super.device_handle, data, size, initial_value,
                                         result);
}

/**
 * @brief CRC 高层虚表（静态常量）
 * @note 继承自 bsp_device_ops_t，并添加 calculate 函数
 */
static const bsp_crc_ops_t bsp_crc_ops = {
    .super = {.deinit = bsp_crc_deinit_virtual}, // 虚析构
    .calculate = bsp_crc_calculate_virtual,      // CRC 计算
};

/**
 * @brief 初始化 CRC 设备对象
 * @param me 设备对象指针（bsp_crc_device_t）
 * @param config 配置参数指针
 * @return 执行状态
 */
bsp_status_t bsp_crc_init(bsp_crc_device_t *me, const bsp_crc_config_t *config)
{
    bsp_status_t status;
    // 参数合法性检查：对象、配置、设备句柄、驱动表，以及必须实现的 init/deinit/calculate
    if ((me == NULL) || (config == NULL) || (config->device_handle == NULL) ||
        (config->driver_ops == NULL) || (config->driver_ops->init == NULL) ||
        (config->driver_ops->deinit == NULL) || (config->driver_ops->calculate == NULL))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    // 保存底层驱动操作表
    me->driver_ops = config->driver_ops;
    // 初始化基类（bsp_crc_t 的 super 是 bsp_device_t），绑定虚表并保存设备句柄
    status = bsp_device_init(&me->super.super, &bsp_crc_ops.super, config->device_handle);
    // 基类初始化成功后，调用底层驱动的 init
    if (status == BSP_STATUS_OK)
    {
        status = config->driver_ops->init(config->device_handle);
    }
    // 如果底层 init 失败，需要清除基类的初始化标志，避免对象处于半有效状态
    if (status != BSP_STATUS_OK)
    {
        me->super.super.is_initialized = false;
    }
    return status;
}

/**
 * @brief 将派生对象转为基类指针（向上转型）
 * @param me 派生对象指针
 * @return 基类指针，若输入为 NULL 则返回 NULL
 */
bsp_crc_t *bsp_crc_as_base(bsp_crc_device_t *me)
{
    return (me != NULL) ? &me->super : NULL;
}

/**
 * @brief 计算 CRC（公共接口）
 * @param me 基类指针
 * @param data 数据指针
 * @param size 数据大小（字节）
 * @param initial_value 初始值
 * @param result 输出结果
 * @return 执行状态
 */
bsp_status_t bsp_crc_calculate(bsp_crc_t *me, const void *data, size_t size, uint32_t initial_value,
                               uint32_t *result)
{
    // 参数校验：基类非空且已初始化，数据及结果指针非空，数据长度非零
    if ((me == NULL) || !bsp_device_is_initialized(&me->super) || (data == NULL) || (size == 0U) ||
        (result == NULL))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    // 通过基类的虚表指针强制转换为 bsp_crc_ops_t 类型，调用 calculate 虚函数
    return ((const bsp_crc_ops_t *)me->super.vptr)
        ->calculate(me, data, size, initial_value, result);
}