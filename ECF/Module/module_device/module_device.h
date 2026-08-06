/**
 * @file module_device.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 模块设备统一基类头文件
 * @version 1.0
 * @date 2026-07-28
 * @copyright Copyright (c) 2026
 *
 * @note 提供所有非电机模块的统一 C11 对象基类。派生对象将 super 作为第一个成员，
 *       并在各自的 .c 文件中持有 static const module_device_ops_t 虚表。
 *       基类不分配内存，也不依赖 MCU 或厂商 HAL。
 */

#ifndef MODULE_DEVICE_H
#define MODULE_DEVICE_H

#include <stdbool.h> // bool
#include <stddef.h>  // size_t, offsetof
#include <stdint.h>  // uint32_t

#ifdef __cplusplus
extern "C"
{
#endif

/* ======================== 容器宏 ======================== */

/**
 * @brief 通过结构体成员指针获取包含该成员的父结构体指针（非常量）
 * @param member_pointer 成员指针
 * @param parent_type 父结构体类型
 * @param member_name 成员名称
 * @return 父结构体指针
 */
#define MODULE_CONTAINER_OF(member_pointer, parent_type, member_name)                              \
    ((parent_type *)((uint8_t *)(member_pointer) - offsetof(parent_type, member_name)))

/**
 * @brief 通过结构体成员指针获取包含该成员的父结构体指针（常量版本）
 */
#define MODULE_CONTAINER_OF_CONST(member_pointer, parent_type, member_name)                        \
    ((const parent_type *)((const uint8_t *)(member_pointer) - offsetof(parent_type, member_name)))

/**
 * @brief 编译期检查派生对象是否将 super 放在首成员
 * @param derived_type 完整的派生对象类型
 * @note 每个 module_device_t 派生类都应在实现文件中调用一次。
 */
#define MODULE_STATIC_ASSERT_SUPER_FIRST(derived_type)                                            \
    _Static_assert(offsetof(derived_type, super) == 0U, #derived_type " must place super first")

/**
 * @brief 对象魔数（'MDEV' 的 ASCII 编码，用于识别有效对象）
 */
#define MODULE_DEVICE_OBJECT_MAGIC (0x4D444556UL)

    /* ======================== 前向声明 ======================== */

    typedef struct module_device module_device_t;

    /* ======================== 状态码枚举 ======================== */

    /**
     * @brief 模块设备基类状态码
     */
    typedef enum
    {
        MODULE_DEVICE_STATUS_OK = 0,              // 操作成功
        MODULE_DEVICE_STATUS_INVALID_ARGUMENT,    // 参数非法（空指针等）
        MODULE_DEVICE_STATUS_NOT_INITIALIZED,     // 对象未初始化
        MODULE_DEVICE_STATUS_ALREADY_INITIALIZED, // 对象已初始化（重复初始化）
        MODULE_DEVICE_STATUS_UNSUPPORTED,         // 虚操作未实现（start/stop/update 为 NULL）
        MODULE_DEVICE_STATUS_OPERATION_FAILED     // 派生类具体操作失败
    } module_device_status_t;

    /* ======================== 虚操作表 ======================== */

    /**
     * @brief 设备虚操作表（由派生类在 .c 中静态定义）
     * @note start/stop/update 构成 module_device_t 的统一生命周期契约，均为必须操作。
     *       不需要实际动作的派生类应提供返回 OK 的空实现，而不是填写 NULL。
     */
    typedef struct
    {
        module_device_status_t (*start)(module_device_t *const me);
        module_device_status_t (*stop)(module_device_t *const me);
        module_device_status_t (*update)(module_device_t *const me, uint32_t elapsed_time_ms);
    } module_device_ops_t;

    /* ======================== 基类结构体 ======================== */

    /**
     * @brief 模块设备基类
     * @note 派生类必须将 super 作为第一个成员
     */
    struct module_device
    {
        const module_device_ops_t *vptr; // 虚表指针（只读）
        const char *logical_name;        // 逻辑名称（便于日志诊断）
        uint32_t registration_key;       // 注册键值（稳定数字标识）
        uint32_t object_magic;           // 魔数，用于对象有效性检查
        bool is_initialized;             // 初始化完成标志
    };

    /* ======================== 公共 API ======================== */

    /**
     * @brief 初始化基类（第一阶段构造）
     * @param me 设备对象
     * @param vptr 虚表指针
     * @param logical_name 逻辑名称
     * @param registration_key 注册键值
     * @return 执行状态
     * @note 只填充基类字段，不标记为已初始化。
     *       vptr 中 start/stop/update 任一缺失都会返回 INVALID_ARGUMENT。
     *       派生类完成自己的资源初始化后调用 module_device_complete_init 提交。
     */
    module_device_status_t module_device_init_base(module_device_t *const me,
                                                   const module_device_ops_t *const vptr,
                                                   const char *const logical_name,
                                                   uint32_t registration_key);

    /**
     * @brief 完成初始化（第二阶段构造）
     * @param me 设备对象
     * @return 执行状态
     * @note 设置 is_initialized = true，提交对象为有效状态。
     *       调用前必须确保派生类资源已成功初始化。
     */
    module_device_status_t module_device_complete_init(module_device_t *const me);

    /**
     * @brief 中止初始化（清理状态）
     * @param me 设备对象
     * @note 清除所有字段，留下可识别的未初始化对象。
     *       在派生类资源初始化失败时调用。
     */
    void module_device_abort_init(module_device_t *const me);

    /**
     * @brief 启动设备（调用虚表 start）
     * @param me 设备对象
     * @return 执行状态
     */
    module_device_status_t module_device_start(module_device_t *const me);

    /**
     * @brief 停止设备（调用虚表 stop）
     * @param me 设备对象
     * @return 执行状态
     */
    module_device_status_t module_device_stop(module_device_t *const me);

    /**
     * @brief 更新设备（调用虚表 update）
     * @param me 设备对象
     * @param elapsed_time_ms 距上次更新的时间（毫秒）
     * @return 执行状态
     */
    module_device_status_t module_device_update(module_device_t *const me,
                                                uint32_t elapsed_time_ms);

    /**
     * @brief 检查设备是否已初始化
     * @param me 设备对象
     * @return true=已初始化且有效
     */
    bool module_device_is_initialized(const module_device_t *const me);

    /**
     * @brief 获取逻辑名称
     * @param me 设备对象
     * @return 逻辑名称指针，若未初始化则返回 NULL
     */
    const char *module_device_get_logical_name(const module_device_t *const me);

    /**
     * @brief 获取注册键值
     * @param me 设备对象
     * @return 注册键值，若未初始化则返回 0
     */
    uint32_t module_device_get_registration_key(const module_device_t *const me);

#ifdef __cplusplus
}
#endif

#endif /* MODULE_DEVICE_H */
