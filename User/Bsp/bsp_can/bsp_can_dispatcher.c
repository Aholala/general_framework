/**
 * @file bsp_can_dispatcher.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief CAN 帧分发器实现
 * @version 1.0
 * @date 2026-07-27
 * @copyright Copyright (c) 2026
 */

#include "bsp_can_dispatcher.h" // 包含分发器头文件
#include <stddef.h>             // 提供 NULL

/**
 * @brief 检查路由是否匹配当前帧
 * @param route 路由项
 * @param frame 接收到的帧
 * @return true 匹配
 */
static bool bsp_can_dispatcher_route_matches(const bsp_can_route_t *const route,
                                             const bsp_can_frame_t *const frame)
{
    // 路由必须启用，ID 类型相同，且 (帧ID & 掩码) == (路由ID & 掩码)
    return route->is_enabled && (route->id_type == frame->id_type) &&
           ((frame->identifier & route->mask) == (route->identifier & route->mask));
}

/**
 * @brief 分发器内部事件回调（由 bsp_can 基类调用）
 * @note 此函数在中断上下文中执行，只设置标志和累加计数器，不执行用户回调
 */
static void bsp_can_dispatcher_event_callback(bsp_event_t event, bsp_status_t status,
                                              size_t transferred_size, void *user_context)
{
    bsp_can_dispatcher_t *const me = (bsp_can_dispatcher_t *)user_context;
    (void)transferred_size; // 该参数未使用

    if (me == NULL)
        return;
    // 接收完成或待处理事件 => 标记有待处理帧
    if ((event == BSP_EVENT_RECEIVE_COMPLETE) || (event == BSP_EVENT_RECEIVE_PENDING))
        me->receive_pending = true;
    // 错误事件或状态非 OK => 错误计数增加
    if ((event == BSP_EVENT_ERROR) || (status != BSP_STATUS_OK))
        ++me->receive_error_count;
}

/**
 * @brief 初始化分发器
 * @param me 分发器对象指针
 * @param config 配置参数
 * @return 执行状态
 */
bsp_status_t bsp_can_dispatcher_init(bsp_can_dispatcher_t *const me,
                                     const bsp_can_dispatcher_config_t *const config)
{
    size_t route_index;
    bsp_status_t status;

    // 参数合法性检查：对象、配置、CAN 基类必须已初始化、路由存储非空、容量>0、每轮最大帧数>0、FIFO
    // 合法
    if ((me == NULL) || (config == NULL) || (config->can == NULL) ||
        !bsp_device_is_initialized(&config->can->super) || (config->route_storage == NULL) ||
        (config->route_capacity == 0U) || (config->maximum_frames_per_process == 0U) ||
        ((config->receive_fifo != BSP_CAN_RX_FIFO_0) &&
         (config->receive_fifo != BSP_CAN_RX_FIFO_1)))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }

    // 清零分发器状态
    me->is_initialized = false;
    me->can = config->can;
    me->receive_fifo = config->receive_fifo;
    me->route_storage = config->route_storage;
    me->route_capacity = config->route_capacity;
    me->route_count = 0U;
    me->maximum_frames_per_process = config->maximum_frames_per_process;
    me->received_frame_count = 0U;
    me->unmatched_frame_count = 0U;
    me->receive_error_count = 0U;
    me->receive_pending = false;
    me->is_processing = false;

    // 清空路由存储（全部置零）
    for (route_index = 0U; route_index < me->route_capacity; ++route_index)
        me->route_storage[route_index] = (bsp_can_route_t){0};

    // 将分发器自身的回调注册到 CAN 基类
    status = bsp_can_set_callback(config->can, bsp_can_dispatcher_event_callback, me);
    if (status == BSP_STATUS_OK)
        me->is_initialized = true;
    return status;
}

/**
 * @brief 移除指定索引的路由（后续路由前移）
 * @param me 分发器对象
 * @param route_index 要移除的索引
 * @return 执行状态
 */
bsp_status_t bsp_can_dispatcher_remove_route(bsp_can_dispatcher_t *const me, size_t route_index)
{
    size_t move_index;

    if (me == NULL)
        return BSP_STATUS_INVALID_ARGUMENT;
    if (!me->is_initialized)
        return BSP_STATUS_NOT_INITIALIZED;
    if (me->is_processing)
        return BSP_STATUS_BUSY; // 处理中不允许修改路由表
    if (route_index >= me->route_count)
        return BSP_STATUS_OUT_OF_RANGE;

    // 从删除位置开始，后面的路由前移
    for (move_index = route_index; (move_index + 1U) < me->route_count; ++move_index)
        me->route_storage[move_index] = me->route_storage[move_index + 1U];
    --me->route_count;
    // 清除最后一个位置
    me->route_storage[me->route_count] = (bsp_can_route_t){0};
    return BSP_STATUS_OK;
}

/**
 * @brief 清空所有路由
 * @param me 分发器对象
 * @return 执行状态
 */
bsp_status_t bsp_can_dispatcher_clear_routes(bsp_can_dispatcher_t *const me)
{
    size_t route_index;

    if (me == NULL)
        return BSP_STATUS_INVALID_ARGUMENT;
    if (!me->is_initialized)
        return BSP_STATUS_NOT_INITIALIZED;
    if (me->is_processing)
        return BSP_STATUS_BUSY;

    for (route_index = 0U; route_index < me->route_count; ++route_index)
        me->route_storage[route_index] = (bsp_can_route_t){0};
    me->route_count = 0U;
    return BSP_STATUS_OK;
}

/**
 * @brief 反初始化分发器（清空路由，注销回调）
 * @param me 分发器对象
 * @return 执行状态
 */
bsp_status_t bsp_can_dispatcher_deinit(bsp_can_dispatcher_t *const me)
{
    bsp_status_t status;

    if (me == NULL)
        return BSP_STATUS_INVALID_ARGUMENT;
    if (!me->is_initialized)
        return BSP_STATUS_NOT_INITIALIZED;
    if (me->is_processing)
        return BSP_STATUS_BUSY;

    // 注销 CAN 回调
    status = bsp_can_set_callback(me->can, NULL, NULL);
    if (status != BSP_STATUS_OK)
        return status;

    // 清空路由
    (void)bsp_can_dispatcher_clear_routes(me);
    me->receive_pending = false;
    me->is_initialized = false;
    me->can = NULL;
    return BSP_STATUS_OK;
}

/**
 * @brief 添加一条路由（匹配 ID+掩码，注册回调）
 * @param me 分发器对象
 * @param identifier 要匹配的 ID
 * @param mask 掩码
 * @param id_type 标准/扩展
 * @param callback 匹配后调用的用户函数
 * @param user_context 回调用户上下文
 * @param route_index 输出路由索引（可为 NULL）
 * @return 执行状态
 */
bsp_status_t bsp_can_dispatcher_add_route(bsp_can_dispatcher_t *const me, uint32_t identifier,
                                          uint32_t mask, bsp_can_id_type_t id_type,
                                          bsp_can_frame_callback_t callback, void *user_context,
                                          size_t *route_index)
{
    bsp_can_route_t *route;

    if ((me == NULL) || (callback == NULL))
        return BSP_STATUS_INVALID_ARGUMENT;
    if (!me->is_initialized)
        return BSP_STATUS_NOT_INITIALIZED;
    // 校验 ID 和掩码是否在合法范围内
    if ((id_type > BSP_CAN_ID_EXTENDED) ||
        ((id_type == BSP_CAN_ID_STANDARD) && ((identifier > 0x7FFU) || (mask > 0x7FFU))) ||
        ((id_type == BSP_CAN_ID_EXTENDED) && ((identifier > 0x1FFFFFFFU) || (mask > 0x1FFFFFFFU))))
    {
        return BSP_STATUS_OUT_OF_RANGE;
    }
    if (me->route_count >= me->route_capacity)
        return BSP_STATUS_NO_RESOURCE;

    route = &me->route_storage[me->route_count];
    *route = (bsp_can_route_t){
        .identifier = identifier,
        .mask = mask,
        .id_type = id_type,
        .callback = callback,
        .user_context = user_context,
        .is_enabled = true,
    };
    if (route_index != NULL)
        *route_index = me->route_count;
    ++me->route_count;
    return BSP_STATUS_OK;
}

/**
 * @brief 启用或禁用某条路由
 * @param me 分发器对象
 * @param route_index 路由索引
 * @param is_enabled true 启用，false 禁用
 * @return 执行状态
 */
bsp_status_t bsp_can_dispatcher_set_route_enabled(bsp_can_dispatcher_t *const me,
                                                  size_t route_index, bool is_enabled)
{
    if (me == NULL)
        return BSP_STATUS_INVALID_ARGUMENT;
    if (!me->is_initialized)
        return BSP_STATUS_NOT_INITIALIZED;
    if (route_index >= me->route_count)
        return BSP_STATUS_OUT_OF_RANGE;
    me->route_storage[route_index].is_enabled = is_enabled;
    return BSP_STATUS_OK;
}

/**
 * @brief 处理接收到的帧（在任务上下文中调用）
 * @param me 分发器对象
 * @param processed_frame_count 输出实际处理的帧数（可为 NULL）
 * @return 若至少处理了一帧返回 BSP_STATUS_OK，否则返回底层接收错误码
 * @note 该函数不可重入，内部有防重入保护
 */
bsp_status_t bsp_can_dispatcher_process(bsp_can_dispatcher_t *const me,
                                        size_t *processed_frame_count)
{
    size_t frame_index;
    size_t route_index;
    size_t processed_count = 0U;
    bsp_status_t receive_status = BSP_STATUS_OK;

    if (me == NULL)
        return BSP_STATUS_INVALID_ARGUMENT;
    if (!me->is_initialized)
        return BSP_STATUS_NOT_INITIALIZED;
    if (me->is_processing)
        return BSP_STATUS_BUSY; // 防止重入

    me->is_processing = true;
    me->receive_pending = false; // 本轮开始前清除待处理标志

    // 循环读取 FIFO，最多读取 maximum_frames_per_process 帧
    for (frame_index = 0U; frame_index < me->maximum_frames_per_process; ++frame_index)
    {
        bsp_can_frame_t frame;
        bool was_matched = false;

        receive_status = bsp_can_receive(me->can, me->receive_fifo, &frame);
        if (receive_status != BSP_STATUS_OK)
            break; // FIFO 空或出错则退出循环

        ++processed_count;
        ++me->received_frame_count;

        // 遍历所有路由，匹配则调用回调
        for (route_index = 0U; route_index < me->route_count; ++route_index)
        {
            const bsp_can_route_t *const route = &me->route_storage[route_index];
            if (bsp_can_dispatcher_route_matches(route, &frame))
            {
                route->callback(&frame, route->user_context);
                was_matched = true;
            }
        }
        if (!was_matched)
            ++me->unmatched_frame_count;
    }

    if (processed_frame_count != NULL)
        *processed_frame_count = processed_count;

    // 如果处理了帧且达到上限，说明 FIFO 可能还有帧，保留 receive_pending = true
    if ((processed_count > 0U) && (processed_count == me->maximum_frames_per_process))
        me->receive_pending = true;

    me->is_processing = false;
    return (processed_count > 0U) ? BSP_STATUS_OK : receive_status;
}

/**
 * @brief 查询是否有待处理帧（由中断标志指示）
 * @param me 分发器对象
 * @return true 有待处理帧
 */
bool bsp_can_dispatcher_has_pending_receive(const bsp_can_dispatcher_t *const me)
{
    return (me != NULL) && me->is_initialized && me->receive_pending;
}