/**
 * @file bsp_can_dispatcher.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief CAN 帧分发器头文件
 * @version 1.0
 * @date 2026-07-27
 * @copyright Copyright (c) 2026
 */

#ifndef BSP_CAN_DISPATCHER_H
#define BSP_CAN_DISPATCHER_H

#include "bsp_can.h" // 依赖 CAN 基类

#ifdef __cplusplus
extern "C"
{
#endif

    /* 前向声明 */
    typedef struct bsp_can_dispatcher bsp_can_dispatcher_t;

    /**
     * @brief CAN 帧回调函数类型（用户实现）
     * @param frame 接收到的帧指针（只读）
     * @param user_context 注册时传入的上下文
     * @note 该回调会在 bsp_can_dispatcher_process 中被调用，属于任务上下文，不可阻塞。
     */
    typedef void (*bsp_can_frame_callback_t)(const bsp_can_frame_t *frame, void *user_context);

    /**
     * @brief 路由表项
     */
    typedef struct
    {
        uint32_t identifier;               // 匹配的标识符
        uint32_t mask;                     // 掩码（1 表示关心）
        bsp_can_id_type_t id_type;         // 标准/扩展
        bsp_can_frame_callback_t callback; // 匹配后调用的回调
        void *user_context;                // 回调上下文
        bool is_enabled;                   // 是否启用（可动态开关）
    } bsp_can_route_t;

    /**
     * @brief 分发器配置结构
     */
    typedef struct
    {
        bsp_can_t *can;                      // 关联的 CAN 基类指针（必须已初始化）
        bsp_can_receive_fifo_t receive_fifo; // 从哪个 FIFO 读取
        bsp_can_route_t *route_storage;      // 路由存储数组（由调用者分配）
        size_t route_capacity;               // 数组最大容量
        size_t maximum_frames_per_process;   // 单次 process 最多处理的帧数
    } bsp_can_dispatcher_config_t;

    /**
     * @brief 分发器对象结构
     * @note 成员可直接访问，但建议通过 API 修改
     */
    struct bsp_can_dispatcher
    {
        bsp_can_t *can;                      // 关联的 CAN 对象
        bsp_can_receive_fifo_t receive_fifo; // 使用的 FIFO
        bsp_can_route_t *route_storage;      // 路由存储指针
        size_t route_capacity;               // 存储容量
        size_t route_count;                  // 当前路由数量
        size_t maximum_frames_per_process;   // 每轮最大处理帧数
        uint32_t received_frame_count;       // 累计接收帧数（统计）
        uint32_t unmatched_frame_count;      // 未匹配帧数（统计）
        uint32_t receive_error_count;        // 接收错误次数（统计）
        volatile bool receive_pending;       // 有待处理帧标志（中断中设置）
        bool is_processing;                  // 是否正在处理（防重入）
        bool is_initialized;                 // 初始化标志
    };

    /* ---------- 公共 API ---------- */

    /**
     * @brief 初始化分发器
     * @param me 分发器对象
     * @param config 配置参数
     * @return 执行状态
     */
    bsp_status_t bsp_can_dispatcher_init(bsp_can_dispatcher_t *const me,
                                         const bsp_can_dispatcher_config_t *const config);

    /**
     * @brief 添加路由
     * @param me 分发器对象
     * @param identifier 匹配 ID
     * @param mask 掩码
     * @param id_type 标准/扩展
     * @param callback 匹配回调
     * @param user_context 回调上下文
     * @param route_index 输出路由索引（可选）
     * @return 执行状态
     */
    bsp_status_t bsp_can_dispatcher_add_route(bsp_can_dispatcher_t *const me, uint32_t identifier,
                                              uint32_t mask, bsp_can_id_type_t id_type,
                                              bsp_can_frame_callback_t callback, void *user_context,
                                              size_t *route_index);

    /**
     * @brief 移除路由（按索引）
     */
    bsp_status_t bsp_can_dispatcher_remove_route(bsp_can_dispatcher_t *const me,
                                                 size_t route_index);

    /**
     * @brief 清空所有路由
     */
    bsp_status_t bsp_can_dispatcher_clear_routes(bsp_can_dispatcher_t *const me);

    /**
     * @brief 反初始化分发器
     */
    bsp_status_t bsp_can_dispatcher_deinit(bsp_can_dispatcher_t *const me);

    /**
     * @brief 启用或禁用路由
     */
    bsp_status_t bsp_can_dispatcher_set_route_enabled(bsp_can_dispatcher_t *const me,
                                                      size_t route_index, bool is_enabled);

    /**
     * @brief 处理接收帧（在任务上下文中调用）
     * @param me 分发器对象
     * @param processed_frame_count 输出本次处理的帧数（可选）
     * @return 执行状态
     */
    bsp_status_t bsp_can_dispatcher_process(bsp_can_dispatcher_t *const me,
                                            size_t *processed_frame_count);

    /**
     * @brief 查询是否有待处理帧
     */
    bool bsp_can_dispatcher_has_pending_receive(const bsp_can_dispatcher_t *const me);

#ifdef __cplusplus
}
#endif

#endif /* BSP_CAN_DISPATCHER_H */