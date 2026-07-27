/**
 * @file bsp_fdcan_classic_adapter.h
 * @brief FDCAN Classic 适配器头文件
 * @note 将 FDCAN 的 Classic 帧能力适配为 bsp_can_t 接口。
 */

#ifndef BSP_FDCAN_CLASSIC_ADAPTER_H
#define BSP_FDCAN_CLASSIC_ADAPTER_H

#include "bsp_can.h"   // 作为目标接口
#include "bsp_fdcan.h" // 依赖 FDCAN 对象

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief 适配器对象（派生自 bsp_can_device_t）
     */
    typedef struct
    {
        bsp_can_device_t super; // 基类（bsp_can_t）
        bsp_fdcan_t *fdcan;     // 组合的 FDCAN 对象（不拥有）
    } bsp_fdcan_classic_adapter_t;

    /**
     * @brief 适配器配置
     */
    typedef struct
    {
        bsp_fdcan_t *fdcan;            // 已初始化的 FDCAN 对象
        bsp_event_callback_t callback; // 用户回调（转发给 Classic CAN）
        void *user_context;
    } bsp_fdcan_classic_adapter_config_t;

    /**
     * @brief 初始化适配器
     * @param me 适配器对象
     * @param config 配置
     * @return 状态码
     */
    bsp_status_t
    bsp_fdcan_classic_adapter_init(bsp_fdcan_classic_adapter_t *const me,
                                   const bsp_fdcan_classic_adapter_config_t *const config);

    /**
     * @brief 获取 Classic CAN 基类指针
     * @param me 适配器对象
     * @return bsp_can_t*，可用于 bsp_can_dispatcher 或模块
     */
    bsp_can_t *bsp_fdcan_classic_adapter_as_can(bsp_fdcan_classic_adapter_t *const me);

#ifdef __cplusplus
}
#endif

#endif /* BSP_FDCAN_CLASSIC_ADAPTER_H */