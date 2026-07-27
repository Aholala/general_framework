/**
 * @file bsp_rng.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 硬件随机数发生器（RNG）通用抽象层实现
 * @note 提供单个 32 位随机数和填充缓冲区两种操作。
 * @version 1.0
 * @date 2026-07-27
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "bsp_rng.h" // 包含 RNG 抽象层头文件

/**
 * @brief RNG 设备虚析构函数（作为 bsp_device_ops_t 的 deinit 回调）
 * @param device bsp_device_t 基类指针
 * @return 执行状态
 * @note 该函数由 bsp_device_deinit 在析构时调用
 */
static bsp_status_t bsp_rng_deinit_virtual(bsp_device_t *device)
{
    // 从基类指针获取派生设备对象（跳过 bsp_rng_t 层）
    bsp_rng_device_t *const me = BSP_CONTAINER_OF(device, bsp_rng_device_t, super.super);
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
 * @brief 获取单个 32 位随机数（虚函数实现）
 * @param me bsp_rng_t 基类指针
 * @param value 输出随机数值
 * @return 执行状态
 */
static bsp_status_t bsp_rng_get_uint32_virtual(bsp_rng_t *me, uint32_t *value)
{
    // 从基类指针获取派生设备对象
    bsp_rng_device_t *const device = BSP_CONTAINER_OF(me, bsp_rng_device_t, super);
    // 转发到底层驱动，传入设备句柄和输出指针
    return device->driver_ops->get_uint32(me->super.device_handle, value);
}

/**
 * @brief 填充缓冲区（虚函数实现）
 * @param me bsp_rng_t 基类指针
 * @param data 输出缓冲区指针
 * @param size 缓冲区大小（字节）
 * @return 执行状态
 */
static bsp_status_t bsp_rng_fill_virtual(bsp_rng_t *me, void *data, size_t size)
{
    // 从基类指针获取派生设备对象
    bsp_rng_device_t *const device = BSP_CONTAINER_OF(me, bsp_rng_device_t, super);
    // 转发到底层驱动，传入设备句柄、缓冲区和大小
    return device->driver_ops->fill(me->super.device_handle, data, size);
}

/**
 * @brief RNG 高层虚表（静态常量）
 * @note 继承自 bsp_device_ops_t，并添加 get_uint32 和 fill 函数
 */
static const bsp_rng_ops_t bsp_rng_ops = {
    .super = {.deinit = bsp_rng_deinit_virtual}, // 虚析构
    .get_uint32 = bsp_rng_get_uint32_virtual,    // 获取随机数
    .fill = bsp_rng_fill_virtual,                // 填充缓冲区
};

/**
 * @brief 初始化 RNG 设备对象
 * @param me 设备对象指针（bsp_rng_device_t）
 * @param config 配置参数指针
 * @return 执行状态
 */
bsp_status_t bsp_rng_init(bsp_rng_device_t *me, const bsp_rng_config_t *config)
{
    bsp_status_t status;
    // 参数合法性检查：对象、配置、设备句柄、驱动表，以及必须实现的 init/deinit/get_uint32/fill
    if ((me == NULL) || (config == NULL) || (config->device_handle == NULL) ||
        (config->driver_ops == NULL) || (config->driver_ops->init == NULL) ||
        (config->driver_ops->deinit == NULL) || (config->driver_ops->get_uint32 == NULL) ||
        (config->driver_ops->fill == NULL))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    // 保存底层驱动操作表
    me->driver_ops = config->driver_ops;
    // 初始化基类（bsp_rng_t 的 super 是 bsp_device_t），绑定虚表并保存设备句柄
    status = bsp_device_init(&me->super.super, &bsp_rng_ops.super, config->device_handle);
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
bsp_rng_t *bsp_rng_as_base(bsp_rng_device_t *me)
{
    return (me != NULL) ? &me->super : NULL;
}

/**
 * @brief 获取单个 32 位随机数（公共接口）
 * @param me 基类指针
 * @param value 输出随机数值
 * @return 执行状态
 */
bsp_status_t bsp_rng_get_uint32(bsp_rng_t *me, uint32_t *value)
{
    // 参数校验：基类非空且已初始化，输出指针非空
    if ((me == NULL) || !bsp_device_is_initialized(&me->super) || (value == NULL))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    // 通过基类的虚表指针强制转换为 bsp_rng_ops_t 类型，调用 get_uint32 虚函数
    return ((const bsp_rng_ops_t *)me->super.vptr)->get_uint32(me, value);
}

/**
 * @brief 填充缓冲区（公共接口）
 * @param me 基类指针
 * @param data 输出缓冲区指针
 * @param size 缓冲区大小（字节），必须大于 0
 * @return 执行状态
 */
bsp_status_t bsp_rng_fill(bsp_rng_t *me, void *data, size_t size)
{
    // 参数校验：基类非空且已初始化，数据指针非空，大小非零
    if ((me == NULL) || !bsp_device_is_initialized(&me->super) || (data == NULL) || (size == 0U))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    // 通过基类的虚表指针强制转换为 bsp_rng_ops_t 类型，调用 fill 虚函数
    return ((const bsp_rng_ops_t *)me->super.vptr)->fill(me, data, size);
}