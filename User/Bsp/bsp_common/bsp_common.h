/**
 * @file bsp_common.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief BSP 公共基础设施头文件
 * @note 包含所有 BSP 模块共享的类型、枚举、宏和基类定义。
 * @version 1.0
 * @date 2026-07-27
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef BSP_COMMON_H
#define BSP_COMMON_H

#include <stdbool.h>          // 布尔类型
#include <stddef.h>           // size_t 等
#include <stdint.h>           // 固定宽度整数类型

#ifdef __cplusplus
extern "C"
{
#endif

    /* ---------- 状态码枚举 ---------- */
    /**
     * @brief BSP 操作状态码
     * @note 所有 BSP 函数统一使用此枚举返回结果
     */
    typedef enum
    {
        BSP_STATUS_OK = 0,                // 操作成功
        BSP_STATUS_INVALID_ARGUMENT,      // 参数非法（如空指针、超范围）
        BSP_STATUS_OUT_OF_RANGE,          // 数值超出硬件或协议允许范围
        BSP_STATUS_NOT_INITIALIZED,       // 对象未初始化或已销毁
        BSP_STATUS_BUSY,                  // 资源忙（如异步操作进行中）
        BSP_STATUS_TIMEOUT,               // 等待超时
        BSP_STATUS_IO_ERROR,              // 底层 I/O 错误（如总线故障）
        BSP_STATUS_NO_RESOURCE,           // 资源不足（如队列满、路由表满）
        BSP_STATUS_UNSUPPORTED            // 可选操作未实现
    } bsp_status_t;

    /* ---------- 传输模式枚举 ---------- */
    /**
     * @brief 数据传输模式
     * @note 用于配置外设的读写方式
     */
    typedef enum
    {
        BSP_TRANSFER_MODE_BLOCKING = 0,   // 阻塞模式（轮询）
        BSP_TRANSFER_MODE_INTERRUPT,      // 中断模式
        BSP_TRANSFER_MODE_DMA             // DMA 模式
    } bsp_transfer_mode_t;

    /* ---------- 事件类型枚举 ---------- */
    /**
     * @brief BSP 事件类型（用于回调通知）
     */
    typedef enum
    {
        BSP_EVENT_TRANSMIT_COMPLETE = 0,  // 发送完成
        BSP_EVENT_RECEIVE_COMPLETE,       // 接收完成
        BSP_EVENT_TRANSFER_COMPLETE,      // 传输完成（通用）
        BSP_EVENT_RECEIVE_PENDING,        // 有数据待接收（如 FIFO 非空）
        BSP_EVENT_ABORT_COMPLETE,         // 中止操作完成
        BSP_EVENT_ERROR                   // 发生错误
    } bsp_event_t;

    /* ---------- 事件回调函数类型 ---------- */
    /**
     * @brief 事件回调函数原型
     * @param event 事件类型
     * @param status 状态码
     * @param transferred_size 已传输的数据量（如字节数或帧数）
     * @param user_context 用户自定义上下文
     * @note 回调函数必须在非阻塞上下文中执行（不应包含延时或信号量获取）
     */
    typedef void (*bsp_event_callback_t)(bsp_event_t event, bsp_status_t status,
                                         size_t transferred_size, void *user_context);

    /* ---------- container_of 宏 ---------- */
    /**
     * @brief 通过结构体成员指针获取包含该成员的结构体首地址（非常量版本）
     * @param pointer 成员指针
     * @param type 父结构体类型
     * @param member 成员名称
     * @return 父结构体指针
     * @note 使用 offsetof 计算偏移，确保类型安全
     */
#define BSP_CONTAINER_OF(pointer, type, member)                                                    \
    ((type *)((uint8_t *)(pointer) - offsetof(type, member)))

    /**
     * @brief 通过结构体成员指针获取包含该成员的结构体首地址（常量版本）
     * @param pointer 成员指针（const）
     * @param type 父结构体类型（const）
     * @param member 成员名称
     * @return 父结构体常量指针
     */
#define BSP_CONTAINER_OF_CONST(pointer, type, member)                                              \
    ((const type *)((const uint8_t *)(pointer) - offsetof(type, member)))

    /**
     * @brief 编译期检查派生对象是否将 super 放在首成员
     * @param derived_type 完整的派生对象类型
     * @note 每个 bsp_device_t 派生类都应在实现文件中调用一次。
     */
#define BSP_STATIC_ASSERT_SUPER_FIRST(derived_type)                                               \
    _Static_assert(offsetof(derived_type, super) == 0U, #derived_type " must place super first")

    /* ---------- 设备基类 ---------- */
    /**
     * @brief 设备基类结构体（所有 BSP 外设对象的公共头）
     * @note 派生类必须将 super 作为第一个成员
     */
    typedef struct bsp_device bsp_device_t;

    /**
     * @brief 设备操作虚表（目前仅包含 deinit）
     * @note 派生类可扩展此表，但必须保持 super 为第一个成员
     */
    typedef struct
    {
        bsp_status_t (*deinit)(bsp_device_t *const me);  // 析构函数（虚）
    } bsp_device_ops_t;

    /**
     * @brief 设备基类定义
     */
    struct bsp_device
    {
        const bsp_device_ops_t *vptr;      // 虚表指针（只读）
        void *device_handle;               // 平台相关句柄（不透明）
        uint32_t object_magic;             // 魔数，用于验证对象有效性
        bool is_initialized;               // 初始化完成标志
    };

    /* ---------- 公共 API 声明 ---------- */

    /**
     * @brief 初始化设备基类
     * @param me 设备对象指针
     * @param vptr 虚表指针
     * @param device_handle 句柄
     * @return 状态码
     */
    bsp_status_t bsp_device_init(bsp_device_t *const me, const bsp_device_ops_t *const vptr,
                                 void *const device_handle);

    /**
     * @brief 反初始化设备基类
     * @param me 设备对象指针
     * @return 状态码
     */
    bsp_status_t bsp_device_deinit(bsp_device_t *const me);

    /**
     * @brief 检查设备是否已初始化且有效
     * @param me 设备对象指针
     * @return true 有效
     */
    bool bsp_device_is_initialized(const bsp_device_t *const me);

    /**
     * @brief 校验传输模式是否合法
     * @param transfer_mode 模式枚举
     * @return true 合法
     */
    bool bsp_transfer_mode_is_valid(bsp_transfer_mode_t transfer_mode);

    /**
     * @brief 获取设备句柄（只读）
     * @param me 设备对象指针
     * @return 句柄指针，若无效则返回 NULL
     */
    void *bsp_device_get_handle(const bsp_device_t *const me);

#ifdef __cplusplus
}
#endif

#endif /* BSP_COMMON_H */
