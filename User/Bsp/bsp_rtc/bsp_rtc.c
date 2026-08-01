/**
 * @file bsp_rtc.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 实时时钟（RTC）通用抽象层实现
 * @version 1.0
 * @date 2026-07-27
 * @copyright Copyright (c) 2026
 *
 * @note 提供结构化时间的获取/设置和 Unix 时间戳获取功能。
 *       使用 bsp_device_t 基类实现多态对象管理。
 */

#include "bsp_rtc.h" // 包含 RTC 抽象层头文件

BSP_STATIC_ASSERT_SUPER_FIRST(bsp_rtc_device_t);

/**
 * @brief RTC 设备虚析构函数（作为 bsp_device_ops_t 的 deinit 回调）
 * @param device bsp_device_t 基类指针
 * @return 执行状态
 * @note 该函数由 bsp_device_deinit 在析构时调用
 *       负责调用底层驱动的 deinit 并清除初始化标志
 */
static bsp_status_t bsp_rtc_deinit_virtual(bsp_device_t *device)
{
    // 从基类指针获取派生设备对象（跳过 bsp_rtc_t 中间层）
    bsp_rtc_device_t *const me = BSP_CONTAINER_OF(device, bsp_rtc_device_t, super.super);
    // 调用底层驱动的 deinit，传入设备句柄
    const bsp_status_t status = me->driver_ops->deinit(device->device_handle);
    // 仅在析构成功时清除初始化标志
    // 基类 bsp_device_deinit 会在后续清空其他字段（vptr、device_handle 等）
    if (status == BSP_STATUS_OK)
    {
        device->is_initialized = false;
    }
    return status;
}

/**
 * @brief 获取结构化时间（虚函数实现）
 * @param me bsp_rtc_t 基类指针
 * @param time 输出时间结构体指针
 * @return 执行状态
 * @note 通过虚表转发到底层驱动
 */
static bsp_status_t bsp_rtc_get_time_virtual(bsp_rtc_t *me, bsp_rtc_time_t *time)
{
    // 从基类指针获取派生设备对象
    bsp_rtc_device_t *const device = BSP_CONTAINER_OF(me, bsp_rtc_device_t, super);
    // 转发到底层驱动，传入设备句柄和时间结构体指针
    return device->driver_ops->get_time(me->super.device_handle, time);
}

/**
 * @brief 设置结构化时间（虚函数实现）
 * @param me bsp_rtc_t 基类指针
 * @param time 时间结构体指针（只读）
 * @return 执行状态
 * @note 通过虚表转发到底层驱动
 */
static bsp_status_t bsp_rtc_set_time_virtual(bsp_rtc_t *me, const bsp_rtc_time_t *time)
{
    // 从基类指针获取派生设备对象
    bsp_rtc_device_t *const device = BSP_CONTAINER_OF(me, bsp_rtc_device_t, super);
    // 转发到底层驱动，传入设备句柄和时间结构体
    return device->driver_ops->set_time(me->super.device_handle, time);
}

/**
 * @brief 获取 Unix 时间戳（虚函数实现）
 * @param me bsp_rtc_t 基类指针
 * @param unix_time_s 输出 Unix 时间戳（秒，64 位）
 * @return 执行状态
 * @note 使用 uint64_t 避免 2038 年问题
 */
static bsp_status_t bsp_rtc_get_unix_virtual(bsp_rtc_t *me, uint64_t *unix_time_s)
{
    // 从基类指针获取派生设备对象
    bsp_rtc_device_t *const device = BSP_CONTAINER_OF(me, bsp_rtc_device_t, super);
    // 转发到底层驱动，传入设备句柄和输出指针
    return device->driver_ops->get_unix_time(me->super.device_handle, unix_time_s);
}

/**
 * @brief RTC 高层虚表（静态常量）
 * @note 继承自 bsp_device_ops_t，并添加 get_time、set_time、get_unix_time 三个虚函数
 *       所有派生类共享同一张虚表，实现多态
 */
static const bsp_rtc_ops_t bsp_rtc_ops = {
    .super = {.deinit = bsp_rtc_deinit_virtual}, // 虚析构函数
    .get_time = bsp_rtc_get_time_virtual,        // 获取结构化时间
    .set_time = bsp_rtc_set_time_virtual,        // 设置结构化时间
    .get_unix_time = bsp_rtc_get_unix_virtual,   // 获取 Unix 时间戳
};

/**
 * @brief 初始化 RTC 设备对象
 * @param me 设备对象指针（bsp_rtc_device_t）
 * @param config 配置参数指针
 * @return 执行状态
 * @note 先初始化基类，再调用驱动 init。若任意步骤失败，清除初始化标志
 */
bsp_status_t bsp_rtc_init(bsp_rtc_device_t *me, const bsp_rtc_config_t *config)
{
    bsp_status_t status;
    // 参数合法性检查：对象、配置、设备句柄、驱动表均不能为空
    // 所有驱动函数（init/deinit/get_time/set_time/get_unix_time）必须实现
    if ((me == NULL) || (config == NULL) || (config->device_handle == NULL) ||
        (config->driver_ops == NULL) || (config->driver_ops->init == NULL) ||
        (config->driver_ops->deinit == NULL) || (config->driver_ops->get_time == NULL) ||
        (config->driver_ops->set_time == NULL) || (config->driver_ops->get_unix_time == NULL))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    // 保存底层驱动操作表
    me->driver_ops = config->driver_ops;
    // 初始化基类（bsp_rtc_t 的 super 是 bsp_device_t），绑定虚表并保存设备句柄
    status = bsp_device_init(&me->super.super, &bsp_rtc_ops.super, config->device_handle);
    // 基类初始化成功后，调用底层驱动的 init（初始化硬件 RTC）
    if (status == BSP_STATUS_OK)
    {
        status = config->driver_ops->init(config->device_handle);
    }
    // 如果基类初始化或驱动 init 失败，清除基类的初始化标志
    // 避免对象处于半有效状态（部分字段已设置但硬件不可用）
    if (status != BSP_STATUS_OK)
    {
        me->super.super.is_initialized = false;
    }
    return status;
}

/**
 * @brief 将派生对象转为基类指针（向上转型）
 * @param me 派生对象指针（bsp_rtc_device_t）
 * @return 基类指针（bsp_rtc_t），若输入为 NULL 则返回 NULL
 * @note 用于将具体设备对象传递给只接受基类指针的公共 API
 */
bsp_rtc_t *bsp_rtc_as_base(bsp_rtc_device_t *me)
{
    return (me != NULL) ? &me->super : NULL;
}

/**
 * @brief 获取结构化时间（公共接口）
 * @param me 基类指针（bsp_rtc_t）
 * @param time 输出时间结构体指针
 * @return 执行状态
 * @note 调用前确保对象已初始化，time 指针非空
 *       通过虚表调用派生类的 get_time 实现
 */
bsp_status_t bsp_rtc_get_time(bsp_rtc_t *me, bsp_rtc_time_t *time)
{
    // 参数校验：基类非空且已初始化，输出指针非空
    if ((me == NULL) || !bsp_device_is_initialized(&me->super) || (time == NULL))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    // 通过基类的虚表指针强制转换为 bsp_rtc_ops_t 类型，调用 get_time 虚函数
    return ((const bsp_rtc_ops_t *)me->super.vptr)->get_time(me, time);
}

/**
 * @brief 设置结构化时间（公共接口）
 * @param me 基类指针（bsp_rtc_t）
 * @param time 时间结构体指针（只读）
 * @return 执行状态
 * @note 对 time 各字段进行基本范围校验：
 *       - 月份 1-12
 *       - 日期 1-31
 *       - 小时 0-23
 *       - 分钟 0-59
 *       - 秒 0-59
 *       - 毫秒 0-999
 *       平台驱动负责更深入的校验（如月份天数、闰年等）
 */
bsp_status_t bsp_rtc_set_time(bsp_rtc_t *me, const bsp_rtc_time_t *time)
{
    // 参数校验：基类非空且已初始化，时间指针非空
    // 各字段在合法范围内
    if ((me == NULL) || !bsp_device_is_initialized(&me->super) || (time == NULL) ||
        (time->month < 1U) || (time->month > 12U) || (time->day < 1U) || (time->day > 31U) ||
        (time->hour > 23U) || (time->minute > 59U) || (time->second > 59U) ||
        (time->millisecond > 999U))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    // 通过基类的虚表指针强制转换为 bsp_rtc_ops_t 类型，调用 set_time 虚函数
    return ((const bsp_rtc_ops_t *)me->super.vptr)->set_time(me, time);
}

/**
 * @brief 获取 Unix 时间戳（公共接口）
 * @param me 基类指针（bsp_rtc_t）
 * @param unix_time_s 输出 Unix 时间戳（秒，64 位）
 * @return 执行状态
 * @note Unix 时间戳为自 1970-01-01 00:00:00 UTC 以来的秒数
 *       使用 uint64_t 避免 2038 年问题
 *       通过虚表调用派生类的 get_unix_time 实现
 */
bsp_status_t bsp_rtc_get_unix_time(bsp_rtc_t *me, uint64_t *unix_time_s)
{
    // 参数校验：基类非空且已初始化，输出指针非空
    if ((me == NULL) || !bsp_device_is_initialized(&me->super) || (unix_time_s == NULL))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    // 通过基类的虚表指针强制转换为 bsp_rtc_ops_t 类型，调用 get_unix_time 虚函数
    return ((const bsp_rtc_ops_t *)me->super.vptr)->get_unix_time(me, unix_time_s);
}
