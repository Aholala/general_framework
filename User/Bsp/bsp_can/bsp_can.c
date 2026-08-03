/**
 * @file bsp_can.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief CAN 通用抽象层实现
 * @version 1.0
 * @date 2026-07-22
 * @copyright Copyright (c) 2026
 */

#include "bsp_can.h" // 包含 CAN 抽象层头文件

BSP_STATIC_ASSERT_SUPER_FIRST(bsp_can_device_t);
#include <stddef.h>  // 提供 NULL 和 size_t

/**
 * @brief 从基类指针获取派生设备对象（非常量版本）
 * @param can_base bsp_can_t 基类指针
 * @return 对应的 bsp_can_device_t 对象指针
 */
static bsp_can_device_t *bsp_can_get_device(bsp_can_t *const can_base)
{
    // 利用 container_of 宏从基类成员地址反推出包含它的结构体地址
    return BSP_CONTAINER_OF(can_base, bsp_can_device_t, super);
}

/**
 * @brief 从基类指针获取派生设备对象（常量版本）
 * @param can_base const bsp_can_t 指针
 * @return 对应的 const bsp_can_device_t 指针
 */
static const bsp_can_device_t *bsp_can_get_device_const(const bsp_can_t *const can_base)
{
    // 常量版本，用于只读操作
    return BSP_CONTAINER_OF_CONST(can_base, bsp_can_device_t, super);
}

/**
 * @brief 从基类虚表指针获取高层操作表
 * @param can_base const bsp_can_t 指针
 * @return 对应的 bsp_can_ops_t 操作表指针（只读）
 */
static const bsp_can_ops_t *bsp_can_get_ops(const bsp_can_t *const can_base)
{
    // 基类 bsp_device_t 的 vptr 指向 bsp_can_ops_t 中的 super 成员
    return BSP_CONTAINER_OF_CONST(can_base->super.vptr, bsp_can_ops_t, super);
}

/**
 * @brief CAN 设备反初始化（作为 device 层的 deinit 回调）
 * @param device_base bsp_device_t 基类指针
 * @return 执行状态
 */
static bsp_status_t bsp_can_device_deinit(bsp_device_t *const device_base)
{
    // 从 device 基类反推出 bsp_can_t 基类地址
    bsp_can_t *const can_base = BSP_CONTAINER_OF(device_base, bsp_can_t, super);
    bsp_can_device_t *const me = bsp_can_get_device(can_base);
    // 如果驱动提供了 deinit 则调用，否则直接成功
    return (me->driver_ops->deinit != NULL) ? me->driver_ops->deinit(device_base->device_handle)
                                            : BSP_STATUS_OK;
}

/**
 * @brief 启动 CAN（转发至底层驱动）
 * @param can_base 基类指针
 * @return 执行状态
 */
static bsp_status_t bsp_can_device_start(bsp_can_t *const can_base)
{
    bsp_can_device_t *const me = bsp_can_get_device(can_base);
    // 调用驱动层的 start，传入设备句柄
    return me->driver_ops->start(can_base->super.device_handle);
}

/**
 * @brief 停止 CAN（转发至底层驱动）
 */
static bsp_status_t bsp_can_device_stop(bsp_can_t *const can_base)
{
    bsp_can_device_t *const me = bsp_can_get_device(can_base);
    return me->driver_ops->stop(can_base->super.device_handle);
}

/**
 * @brief 配置硬件过滤器（转发）
 */
static bsp_status_t bsp_can_device_configure_filter(bsp_can_t *const can_base,
                                                    const bsp_can_filter_t *filter_config)
{
    bsp_can_device_t *const me = bsp_can_get_device(can_base);
    return me->driver_ops->configure_filter(can_base->super.device_handle, filter_config);
}

/**
 * @brief 发送帧（转发）
 */
static bsp_status_t bsp_can_device_transmit(bsp_can_t *const can_base, const bsp_can_frame_t *frame,
                                            uint32_t timeout_ms)
{
    bsp_can_device_t *const me = bsp_can_get_device(can_base);
    return me->driver_ops->transmit(can_base->super.device_handle, frame, timeout_ms);
}

/**
 * @brief 接收帧（转发）
 */
static bsp_status_t bsp_can_device_receive(bsp_can_t *const can_base,
                                           bsp_can_receive_fifo_t receive_fifo,
                                           bsp_can_frame_t *frame)
{
    bsp_can_device_t *const me = bsp_can_get_device(can_base);
    return me->driver_ops->receive(can_base->super.device_handle, receive_fifo, frame);
}

/**
 * @brief 获取发送邮箱空闲数（转发）
 */
static bsp_status_t bsp_can_device_get_transmit_free_level(const bsp_can_t *const can_base,
                                                           uint32_t *free_level)
{
    const bsp_can_device_t *const me = bsp_can_get_device_const(can_base);
    return me->driver_ops->get_tx_free_level(can_base->super.device_handle, free_level);
}

/* 定义高层虚表，所有函数指针指向上述转发函数 */
static const bsp_can_ops_t s_bsp_can_device_ops = {
    .super = {.deinit = bsp_can_device_deinit},                 // 父类 deinit
    .start = bsp_can_device_start,                              // 启动转发
    .stop = bsp_can_device_stop,                                // 停止转发
    .configure_filter = bsp_can_device_configure_filter,        // 配置过滤器转发
    .transmit = bsp_can_device_transmit,                        // 发送转发
    .receive = bsp_can_device_receive,                          // 接收转发
    .get_tx_free_level = bsp_can_device_get_transmit_free_level // 获取空闲数转发
};

/**
 * @brief 初始化 CAN 设备对象
 * @param me 设备对象指针（bsp_can_device_t）
 * @param config 配置参数指针
 * @return 状态码
 */
bsp_status_t bsp_can_init(bsp_can_device_t *const me, const bsp_can_config_t *const config)
{
    bsp_status_t status;
    // 参数合法性检查：对象、配置、设备句柄、驱动表，以及必须实现的
    // start/stop/configure_filter/transmit/receive
    if ((me == NULL) || (config == NULL) || (config->device_handle == NULL) ||
        (config->driver_ops == NULL) || (config->driver_ops->start == NULL) ||
        (config->driver_ops->stop == NULL) || (config->driver_ops->configure_filter == NULL) ||
        (config->driver_ops->transmit == NULL) || (config->driver_ops->receive == NULL))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    // 先清空对象，确保任意初始化失败路径都留下确定的未初始化状态。
    *me = (bsp_can_device_t){0};
    // 如果驱动提供了 init，则调用以初始化硬件
    if (config->driver_ops->init != NULL)
    {
        status = config->driver_ops->init(config->device_handle);
        if (status != BSP_STATUS_OK) {
            return status;
}
    }
    me->driver_ops = config->driver_ops; // 保存驱动操作表
    // 保存回调与用户上下文
    me->super.callback = config->callback;
    me->super.user_context = config->user_context;
    // 调用 device 基类初始化，注册虚表并保存设备句柄
    return bsp_device_init(&me->super.super, &s_bsp_can_device_ops.super, config->device_handle);
}

/**
 * @brief 将派生对象转为基类指针（向上转型）
 * @param me 派生对象指针
 * @return 基类指针，若输入为 NULL 则返回 NULL
 */
bsp_can_t *bsp_can_as_base(bsp_can_device_t *const me)
{
    return (me != NULL) ? &me->super : NULL;
}

/**
 * @brief 校验 CAN 对象是否有效且已初始化（内部函数）
 * @param me bsp_can_t 指针
 * @return 状态码
 */
static bsp_status_t bsp_can_validate(const bsp_can_t *const me)
{
    if (me == NULL) {
        return BSP_STATUS_INVALID_ARGUMENT;
}
    return bsp_device_is_initialized(&me->super) ? BSP_STATUS_OK : BSP_STATUS_NOT_INITIALIZED;
}

/**
 * @brief 检查 CAN ID 是否在合法范围内
 * @param id 标识符值
 * @param type 标准/扩展
 * @return true 合法
 */
static bool bsp_can_is_id_valid(uint32_t id, bsp_can_id_type_t type)
{
    // 标准 ID 最大 0x7FF，扩展 ID 最大 0x1FFFFFFF
    return (type == BSP_CAN_ID_STANDARD) ? (id <= 0x7FFU)
                                         : ((type == BSP_CAN_ID_EXTENDED) && (id <= 0x1FFFFFFFU));
}

/**
 * @brief 校验 CAN 帧是否合法（ID、数据长度、帧类型）
 * @param frame 帧指针
 * @return true 合法
 */
static bool bsp_can_is_frame_valid(const bsp_can_frame_t *frame)
{
    return (frame != NULL) && (frame->data_length <= 8U) &&
           ((frame->frame_type == BSP_CAN_FRAME_DATA) ||
            (frame->frame_type == BSP_CAN_FRAME_REMOTE)) &&
           bsp_can_is_id_valid(frame->identifier, frame->id_type);
}

/**
 * @brief 设置事件回调函数
 * @param me 基类指针
 * @param callback 回调函数指针
 * @param user_context 用户上下文
 * @return 执行状态
 */
bsp_status_t bsp_can_set_callback(bsp_can_t *const me, bsp_event_callback_t callback,
                                  void *user_context)
{
    const bsp_status_t status = bsp_can_validate(me);
    if (status != BSP_STATUS_OK) {
        return status;
}
    me->callback = callback;
    me->user_context = user_context;
    return BSP_STATUS_OK;
}

/**
 * @brief 启动 CAN（公共接口）
 * @param me 基类指针
 * @return 执行状态
 */
bsp_status_t bsp_can_start(bsp_can_t *const me)
{
    const bsp_status_t status = bsp_can_validate(me);
    return (status == BSP_STATUS_OK) ? bsp_can_get_ops(me)->start(me) : status;
}

/**
 * @brief 停止 CAN（公共接口）
 */
bsp_status_t bsp_can_stop(bsp_can_t *const me)
{
    const bsp_status_t status = bsp_can_validate(me);
    return (status == BSP_STATUS_OK) ? bsp_can_get_ops(me)->stop(me) : status;
}

/**
 * @brief 配置硬件过滤器（公共接口）
 * @param me 基类指针
 * @param filter_config 过滤器配置
 * @return 执行状态，会额外检查 ID、掩码和 FIFO 合法性
 */
bsp_status_t bsp_can_configure_filter(bsp_can_t *const me, const bsp_can_filter_t *filter_config)
{
    const bsp_status_t status = bsp_can_validate(me);
    if (status != BSP_STATUS_OK) {
        return status;
}
    // 参数合法性：过滤器配置非空，ID、掩码有效，FIFO 选择正确
    if ((filter_config == NULL) ||
        !bsp_can_is_id_valid(filter_config->identifier, filter_config->id_type) ||
        !bsp_can_is_id_valid(filter_config->mask, filter_config->id_type) ||
        ((filter_config->receive_fifo != BSP_CAN_RX_FIFO_0) &&
         (filter_config->receive_fifo != BSP_CAN_RX_FIFO_1)))
    {
        return BSP_STATUS_OUT_OF_RANGE;
    }
    return bsp_can_get_ops(me)->configure_filter(me, filter_config);
}

/**
 * @brief 发送帧（公共接口）
 * @param me 基类指针
 * @param frame 帧指针
 * @param timeout_ms 超时时间（毫秒）
 * @return 执行状态
 */
bsp_status_t bsp_can_transmit(bsp_can_t *const me, const bsp_can_frame_t *frame,
                              uint32_t timeout_ms)
{
    const bsp_status_t status = bsp_can_validate(me);
    if (status != BSP_STATUS_OK) {
        return status;
}
    // 校验帧合法性
    if (!bsp_can_is_frame_valid(frame)) {
        return BSP_STATUS_OUT_OF_RANGE;
}
    return bsp_can_get_ops(me)->transmit(me, frame, timeout_ms);
}

/**
 * @brief 接收帧（公共接口）
 * @param me 基类指针
 * @param receive_fifo 从哪个 FIFO 读取
 * @param frame 输出帧指针
 * @return 执行状态，读取后会再次校验帧合法性
 */
bsp_status_t bsp_can_receive(bsp_can_t *const me, bsp_can_receive_fifo_t receive_fifo,
                             bsp_can_frame_t *frame)
{
    const bsp_status_t status = bsp_can_validate(me);
    if (status != BSP_STATUS_OK) {
        return status;
}
    // 检查输出指针和 FIFO 合法性
    if ((frame == NULL) ||
        ((receive_fifo != BSP_CAN_RX_FIFO_0) && (receive_fifo != BSP_CAN_RX_FIFO_1))) {
        return BSP_STATUS_INVALID_ARGUMENT;
}
    // 调用底层接收
    const bsp_status_t receive_status = bsp_can_get_ops(me)->receive(me, receive_fifo, frame);
    if (receive_status != BSP_STATUS_OK) {
        return receive_status;
}
    // 再次校验帧，防止底层返回畸形数据
    return bsp_can_is_frame_valid(frame) ? BSP_STATUS_OK : BSP_STATUS_IO_ERROR;
}

/**
 * @brief 获取发送邮箱空闲数量（公共接口）
 * @param me 基类指针（const）
 * @param free_level 输出空闲数
 * @return 执行状态，若底层未实现则返回 BSP_STATUS_UNSUPPORTED
 */
bsp_status_t bsp_can_get_transmit_free_level(const bsp_can_t *const me, uint32_t *free_level)
{
    const bsp_status_t status = bsp_can_validate(me);
    if (status != BSP_STATUS_OK) {
        return status;
}
    if (free_level == NULL) {
        return BSP_STATUS_INVALID_ARGUMENT;
}
    // 检查底层驱动是否实现了该函数
    if (bsp_can_get_device_const(me)->driver_ops->get_tx_free_level == NULL) {
        return BSP_STATUS_UNSUPPORTED;
}
    return bsp_can_get_ops(me)->get_tx_free_level(me, free_level);
}

/**
 * @brief 事件通知函数（由底层驱动在中断中调用）
 * @param me 基类指针
 * @param event 事件类型
 * @param status 状态码
 * @param transferred_size 传输数量（例如帧数）
 * @note 仅在对象已初始化且回调非空时调用
 */
void bsp_can_notify(bsp_can_t *const me, bsp_event_t event, bsp_status_t status,
                    size_t transferred_size)
{
    if ((me != NULL) && bsp_device_is_initialized(&me->super) && (me->callback != NULL))
    {
        me->callback(event, status, transferred_size, me->user_context);
    }
}
