/**
 * @file bsp_storage.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 存储设备通用抽象层实现
 * @version 1.0
 * @date 2026-07-27
 * @copyright Copyright (c) 2026
 *
 * @note 统一非易失存储基类，覆盖片内 Flash、QSPI/OSPI NOR、EEPROM、
 *       SDMMC 块设备和主机文件模拟器。上层只依赖读、编程、擦除、
 *       同步和几何信息，不依赖具体存储控制器。
 */

#include "bsp_storage.h" // 包含存储抽象层头文件

BSP_STATIC_ASSERT_SUPER_FIRST(bsp_storage_device_t);

/**
 * @brief 从基类指针获取派生设备对象（非常量版本）
 * @param me bsp_storage_t 基类指针
 * @return 对应的 bsp_storage_device_t 对象指针
 */
static bsp_storage_device_t *bsp_storage_get_device(bsp_storage_t *me)
{
    // 利用 container_of 宏从基类成员地址反推出包含它的结构体地址
    return BSP_CONTAINER_OF(me, bsp_storage_device_t, super);
}

/**
 * @brief 从基类指针获取派生设备对象（常量版本）
 * @param me const bsp_storage_t 指针
 * @return 对应的 const bsp_storage_device_t 指针
 */
static const bsp_storage_device_t *bsp_storage_get_const_device(const bsp_storage_t *me)
{
    return BSP_CONTAINER_OF_CONST(me, bsp_storage_device_t, super);
}

/**
 * @brief 存储设备虚析构函数（作为 bsp_device_ops_t 的 deinit 回调）
 * @param device bsp_device_t 基类指针
 * @return 执行状态
 * @note 该函数由 bsp_device_deinit 在析构时调用
 */
static bsp_status_t bsp_storage_deinit_virtual(bsp_device_t *device)
{
    // 从基类指针获取派生设备对象（跳过 bsp_storage_t 中间层）
    bsp_storage_device_t *const me = BSP_CONTAINER_OF(device, bsp_storage_device_t, super.super);
    // 调用底层驱动的 deinit，传入设备句柄
    const bsp_status_t status = me->driver_ops->deinit(device->device_handle);
    // 仅在析构成功时清除初始化标志
    if (status == BSP_STATUS_OK)
    {
        device->is_initialized = false;
    }
    return status;
}

/**
 * @brief 从存储设备读取数据（虚函数实现）
 * @param me bsp_storage_t 基类指针
 * @param address 起始地址（偏移量，从 0 开始）
 * @param data 输出数据缓冲区指针
 * @param size 读取大小（字节）
 * @return 执行状态
 */
static bsp_status_t bsp_storage_read_virtual(bsp_storage_t *me, uint64_t address, void *data,
                                             size_t size)
{
    // 从基类指针获取派生设备对象
    bsp_storage_device_t *const device = bsp_storage_get_device(me);
    // 转发到底层驱动，传入设备句柄、地址、缓冲区和大小
    return device->driver_ops->read(me->super.device_handle, address, data, size);
}

/**
 * @brief 编程（写入）数据到存储设备（虚函数实现）
 * @param me bsp_storage_t 基类指针
 * @param address 起始地址（偏移量，从 0 开始）
 * @param data 待写入数据指针
 * @param size 写入大小（字节）
 * @return 执行状态
 * @note 某些存储介质需要先擦除才能写入，由驱动根据几何信息处理
 */
static bsp_status_t bsp_storage_program_virtual(bsp_storage_t *me, uint64_t address,
                                                const void *data, size_t size)
{
    bsp_storage_device_t *const device = bsp_storage_get_device(me);
    return device->driver_ops->program(me->super.device_handle, address, data, size);
}

/**
 * @brief 擦除存储区域（虚函数实现）
 * @param me bsp_storage_t 基类指针
 * @param address 起始地址（偏移量，从 0 开始）
 * @param size 擦除大小（字节）
 * @return 执行状态
 * @note 擦除粒度由几何信息中的 erase_block_bytes 决定
 */
static bsp_status_t bsp_storage_erase_virtual(bsp_storage_t *me, uint64_t address, size_t size)
{
    bsp_storage_device_t *const device = bsp_storage_get_device(me);
    return device->driver_ops->erase(me->super.device_handle, address, size);
}

/**
 * @brief 同步存储设备（虚函数实现）
 * @param me bsp_storage_t 基类指针
 * @return 执行状态
 * @note 对于有缓存的存储介质，确保数据已持久化到物理介质
 */
static bsp_status_t bsp_storage_sync_virtual(bsp_storage_t *me)
{
    bsp_storage_device_t *const device = bsp_storage_get_device(me);
    return device->driver_ops->sync(me->super.device_handle);
}

/**
 * @brief 获取存储设备几何信息（虚函数实现）
 * @param me bsp_storage_t 基类指针（const）
 * @param geometry 输出几何信息结构体指针
 * @return 执行状态
 */
static bsp_status_t bsp_storage_geometry_virtual(const bsp_storage_t *me,
                                                 bsp_storage_geometry_t *geometry)
{
    const bsp_storage_device_t *const device = bsp_storage_get_const_device(me);
    return device->driver_ops->get_geometry(me->super.device_handle, geometry);
}

/**
 * @brief 存储设备高层虚表（静态常量）
 * @note 继承自 bsp_device_ops_t，并添加 read、program、erase、sync、get_geometry
 */
static const bsp_storage_ops_t bsp_storage_ops = {
    .super = {.deinit = bsp_storage_deinit_virtual}, // 虚析构
    .read = bsp_storage_read_virtual,                // 读取数据
    .program = bsp_storage_program_virtual,          // 编程数据
    .erase = bsp_storage_erase_virtual,              // 擦除区域
    .sync = bsp_storage_sync_virtual,                // 同步
    .get_geometry = bsp_storage_geometry_virtual,    // 获取几何信息
};

/**
 * @brief 初始化存储设备对象
 * @param me 设备对象指针（bsp_storage_device_t）
 * @param config 配置参数指针
 * @return 执行状态
 * @note 所有驱动函数（init/deinit/read/program/erase/sync/get_geometry）均必须实现
 */
bsp_status_t bsp_storage_init(bsp_storage_device_t *me, const bsp_storage_config_t *config)
{
    bsp_status_t status;
    // 参数合法性检查：对象、配置、设备句柄、驱动表均不能为空
    // 所有驱动函数必须实现（存储设备不允许可选操作）
    if ((me == NULL) || (config == NULL) || (config->device_handle == NULL) ||
        (config->driver_ops == NULL) || (config->driver_ops->init == NULL) ||
        (config->driver_ops->deinit == NULL) || (config->driver_ops->read == NULL) ||
        (config->driver_ops->program == NULL) || (config->driver_ops->erase == NULL) ||
        (config->driver_ops->sync == NULL) || (config->driver_ops->get_geometry == NULL))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    // 保存底层驱动操作表
    me->driver_ops = config->driver_ops;
    // 初始化基类（bsp_storage_t 的 super 是 bsp_device_t），绑定虚表并保存设备句柄
    status = bsp_device_init(&me->super.super, &bsp_storage_ops.super, config->device_handle);
    // 基类初始化成功后，调用底层驱动的 init（初始化硬件存储控制器）
    if (status == BSP_STATUS_OK)
    {
        status = config->driver_ops->init(config->device_handle);
    }
    // 如果基类初始化或驱动 init 失败，清除基类的初始化标志
    // 避免对象处于半有效状态
    if (status != BSP_STATUS_OK)
    {
        me->super.super.is_initialized = false;
    }
    return status;
}

/**
 * @brief 将派生对象转为基类指针（向上转型）
 * @param me 派生对象指针（bsp_storage_device_t）
 * @return 基类指针（bsp_storage_t）
 */
bsp_storage_t *bsp_storage_as_base(bsp_storage_device_t *me)
{
    return (me != NULL) ? &me->super : NULL;
}

/**
 * @brief 从存储设备读取数据（公共接口）
 * @param me 基类指针
 * @param address 起始地址（偏移量，从 0 开始）
 * @param data 输出数据缓冲区指针
 * @param size 读取大小（字节），必须大于 0
 * @return 执行状态
 */
bsp_status_t bsp_storage_read(bsp_storage_t *me, uint64_t address, void *data, size_t size)
{
    // 参数校验：基类非空且已初始化，数据指针非空，大小非零
    if ((me == NULL) || !bsp_device_is_initialized(&me->super) || (data == NULL) || (size == 0U))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    // 通过虚表调用 read
    return ((const bsp_storage_ops_t *)me->super.vptr)->read(me, address, data, size);
}

/**
 * @brief 编程（写入）数据到存储设备（公共接口）
 * @param me 基类指针
 * @param address 起始地址（偏移量，从 0 开始）
 * @param data 待写入数据指针
 * @param size 写入大小（字节），必须大于 0
 * @return 执行状态
 */
bsp_status_t bsp_storage_program(bsp_storage_t *me, uint64_t address, const void *data, size_t size)
{
    // 参数校验：基类非空且已初始化，数据指针非空，大小非零
    if ((me == NULL) || !bsp_device_is_initialized(&me->super) || (data == NULL) || (size == 0U))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    // 通过虚表调用 program
    return ((const bsp_storage_ops_t *)me->super.vptr)->program(me, address, data, size);
}

/**
 * @brief 擦除存储区域（公共接口）
 * @param me 基类指针
 * @param address 起始地址（偏移量，从 0 开始）
 * @param size 擦除大小（字节），必须大于 0
 * @return 执行状态
 * @note 驱动应确保 address 和 size 对齐到擦除块边界
 */
bsp_status_t bsp_storage_erase(bsp_storage_t *me, uint64_t address, size_t size)
{
    // 参数校验：基类非空且已初始化，大小非零
    if ((me == NULL) || !bsp_device_is_initialized(&me->super) || (size == 0U))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    // 通过虚表调用 erase
    return ((const bsp_storage_ops_t *)me->super.vptr)->erase(me, address, size);
}

/**
 * @brief 同步存储设备（公共接口）
 * @param me 基类指针
 * @return 执行状态
 * @note 确保所有挂起的写入已持久化到物理介质
 */
bsp_status_t bsp_storage_sync(bsp_storage_t *me)
{
    // 参数校验：基类非空且已初始化
    if ((me == NULL) || !bsp_device_is_initialized(&me->super))
    {
        return BSP_STATUS_NOT_INITIALIZED;
    }
    // 通过虚表调用 sync
    return ((const bsp_storage_ops_t *)me->super.vptr)->sync(me);
}

/**
 * @brief 获取存储设备几何信息（公共接口）
 * @param me 基类指针（const）
 * @param geometry 输出几何信息结构体指针
 * @return 执行状态
 */
bsp_status_t bsp_storage_get_geometry(const bsp_storage_t *me, bsp_storage_geometry_t *geometry)
{
    // 参数校验：基类非空且已初始化，输出指针非空
    if ((me == NULL) || !bsp_device_is_initialized(&me->super) || (geometry == NULL))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    // 通过虚表调用 get_geometry
    return ((const bsp_storage_ops_t *)me->super.vptr)->get_geometry(me, geometry);
}
