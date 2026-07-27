/**
 * @file bsp_exti.h
 * @brief 外部中断通用抽象层头文件
 * @note 定义 EXTI 中断源的启用/禁用、回调和事件通知接口。
 */

#ifndef BSP_EXTI_H
#define BSP_EXTI_H

#include "bsp_common.h" // 包含基础类型、状态码、设备基类等

#ifdef __cplusplus
extern "C"
{
#endif

    /* 前向声明，避免循环依赖 */
    typedef struct bsp_exti bsp_exti_t;
    typedef struct bsp_exti_device bsp_exti_device_t;

    /**
     * @brief EXTI 中断回调函数类型
     * @param me 触发中断的 EXTI 对象指针
     * @param user_context 注册时传入的用户上下文
     * @note 回调在 ISR 上下文中执行，必须快速返回，不可阻塞
     */
    typedef void (*bsp_exti_callback_t)(bsp_exti_t *const me, void *user_context);

    /* ---------- 高层虚表（面向应用层） ---------- */
    /**
     * @brief EXTI 操作虚表，继承自 bsp_device_ops_t
     * @note 派生类必须保持 super 为第一成员
     */
    typedef struct
    {
        bsp_device_ops_t super;                        // 父类虚表（含 deinit）
        bsp_status_t (*enable)(bsp_exti_t *const me);  // 启用中断
        bsp_status_t (*disable)(bsp_exti_t *const me); // 禁用中断
    } bsp_exti_ops_t;

    /* ---------- 基类 ---------- */
    /**
     * @brief EXTI 基类结构体
     */
    struct bsp_exti
    {
        bsp_device_t super;           // 设备基类
        bsp_exti_callback_t callback; // 中断回调函数
        void *user_context;           // 回调用户上下文
    };

    /* ---------- 底层驱动操作表（平台实现） ---------- */
    /**
     * @brief 平台相关 EXTI 驱动操作表
     * @note 所有函数接收 device_handle 作为第一个参数
     */
    typedef struct
    {
        bsp_status_t (*init)(void *device_handle);    // 初始化（可选，配置引脚、边沿、NVIC）
        bsp_status_t (*deinit)(void *device_handle);  // 反初始化（可选）
        bsp_status_t (*enable)(void *device_handle);  // 使能中断（NVIC 使能）
        bsp_status_t (*disable)(void *device_handle); // 禁用中断（NVIC 禁用）
    } bsp_exti_driver_ops_t;

    /* ---------- 派生设备对象 ---------- */
    /**
     * @brief EXTI 设备对象（派生类）
     */
    struct bsp_exti_device
    {
        bsp_exti_t super;                        // 基类实例
        const bsp_exti_driver_ops_t *driver_ops; // 底层驱动操作表
    };

    /* ---------- 配置结构 ---------- */
    /**
     * @brief EXTI 初始化配置
     */
    typedef struct
    {
        void *device_handle;                     // 平台设备句柄
        const bsp_exti_driver_ops_t *driver_ops; // 底层驱动表
        bsp_exti_callback_t callback;            // 中断回调（可为 NULL）
        void *user_context;                      // 回调用户上下文
    } bsp_exti_config_t;

    /* ---------- 公共 API 声明 ---------- */

    /**
     * @brief 初始化 EXTI 设备
     * @param me 设备对象指针
     * @param config 配置参数
     * @return 执行状态
     */
    bsp_status_t bsp_exti_init(bsp_exti_device_t *const me, const bsp_exti_config_t *const config);

    /**
     * @brief 将派生对象转为基类指针（向上转型）
     */
    bsp_exti_t *bsp_exti_as_base(bsp_exti_device_t *const me);

    /**
     * @brief 设置 EXTI 事件回调（运行时更换）
     * @param me 基类指针
     * @param callback 回调函数指针（可为 NULL）
     * @param user_context 用户上下文
     * @return 执行状态
     * @note 若中断可能并发发生，更换前建议先禁用中断
     */
    bsp_status_t bsp_exti_set_callback(bsp_exti_t *const me, bsp_exti_callback_t callback,
                                       void *user_context);

    /**
     * @brief 启用外部中断
     */
    bsp_status_t bsp_exti_enable(bsp_exti_t *const me);

    /**
     * @brief 禁用外部中断
     */
    bsp_status_t bsp_exti_disable(bsp_exti_t *const me);

    /**
     * @brief 中断通知函数（由平台 ISR 调用）
     * @param me 基类指针
     * @note 该函数在 ISR 上下文中被调用，会触发用户回调
     */
    void bsp_exti_notify(bsp_exti_t *const me);

#ifdef __cplusplus
}
#endif

#endif /* BSP_EXTI_H */