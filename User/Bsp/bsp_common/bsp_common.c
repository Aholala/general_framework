/**
 * @file bsp_common.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief BSP 通用基础设施实现
 * @note 提供设备基类的初始化、反初始化、状态查询及工具函数。
 * @version 1.0
 * @date 2026-07-27
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "bsp_common.h" // 包含公共头文件
#include <stddef.h>     // 提供 NULL 和 size_t

/**
 * @brief 设备对象魔数（Magic Number），用于校验对象有效性
 * @note 该值为 'BSP_' 的 ASCII 编码，用于区分已初始化与未初始化对象
 */
#define BSP_DEVICE_OBJECT_MAGIC (0x4253504FU) // 对应 "BSP\0" 的小端表示

/**
 * @brief 初始化设备基类
 * @param me 设备对象指针
 * @param vptr 设备操作虚表指针（不可变）
 * @param device_handle 平台相关的不透明句柄
 * @return 执行状态
 * @note 初始化后 is_initialized 置为 true，object_magic 写入魔数
 */
bsp_status_t bsp_device_init(bsp_device_t *const me, const bsp_device_ops_t *const vptr,
                             void *const device_handle)
{
    // 参数校验：对象、虚表、句柄均不能为空
    if ((me == NULL) || (vptr == NULL) || (device_handle == NULL))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    me->vptr = vptr;                            // 绑定虚表
    me->device_handle = device_handle;          // 保存句柄
    me->object_magic = BSP_DEVICE_OBJECT_MAGIC; // 写入魔数，标识有效对象
    me->is_initialized = true;                  // 标记为已初始化
    return BSP_STATUS_OK;
}

/**
 * @brief 反初始化设备基类（析构）
 * @param me 设备对象指针
 * @return 执行状态
 * @note 会调用虚表中的 deinit（若存在），然后清空所有字段
 */
bsp_status_t bsp_device_deinit(bsp_device_t *const me)
{
    bsp_status_t status;

    if (me == NULL)
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    // 检查对象是否有效（魔数、状态等）
    if (!bsp_device_is_initialized(me))
    {
        return BSP_STATUS_NOT_INITIALIZED;
    }
    // 如果虚表提供了 deinit 则调用，否则直接成功
    status = (me->vptr->deinit != NULL) ? me->vptr->deinit(me) : BSP_STATUS_OK;
    // 仅在析构成功时清空对象字段，防止部分失败导致状态不一致
    if (status == BSP_STATUS_OK)
    {
        me->vptr = NULL;
        me->device_handle = NULL;
        me->object_magic = 0U;
        me->is_initialized = false;
    }
    return status;
}

/**
 * @brief 检查设备对象是否已初始化且有效
 * @param me 设备对象指针
 * @return true 表示对象有效且已初始化
 * @note 同时检查魔数、初始化标志、虚表和句柄均非空
 */
bool bsp_device_is_initialized(const bsp_device_t *const me)
{
    return (me != NULL) && (me->object_magic == BSP_DEVICE_OBJECT_MAGIC) && me->is_initialized &&
           (me->vptr != NULL) && (me->device_handle != NULL);
}

/**
 * @brief 校验传输模式是否合法
 * @param transfer_mode 传输模式枚举值
 * @return true 表示合法（阻塞、中断、DMA 之一）
 */
bool bsp_transfer_mode_is_valid(bsp_transfer_mode_t transfer_mode)
{
    return (transfer_mode == BSP_TRANSFER_MODE_BLOCKING) ||
           (transfer_mode == BSP_TRANSFER_MODE_INTERRUPT) ||
           (transfer_mode == BSP_TRANSFER_MODE_DMA);
}

/**
 * @brief 获取设备句柄（只读）
 * @param me 设备对象指针
 * @return 设备句柄，若对象无效则返回 NULL
 */
void *bsp_device_get_handle(const bsp_device_t *const me)
{
    return bsp_device_is_initialized(me) ? me->device_handle : NULL;
}