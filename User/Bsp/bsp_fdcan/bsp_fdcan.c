/**
 * @file bsp_fdcan.c
 * @brief CAN FD 通用抽象层实现
 * @note 支持 Classic CAN、CAN FD（无BRS/带BRS），提供发送、接收、过滤、状态查询等接口。
 */

#include "bsp_fdcan.h" // 包含 FDCAN 抽象层头文件

BSP_STATIC_ASSERT_SUPER_FIRST(bsp_fdcan_device_t);
#include <stddef.h>    // 提供 NULL

/**
 * @brief 从基类指针获取派生设备对象（非常量）
 */
static bsp_fdcan_device_t *bsp_fdcan_get_device(bsp_fdcan_t *const fdcan_base)
{
    return BSP_CONTAINER_OF(fdcan_base, bsp_fdcan_device_t, super);
}

/**
 * @brief 从基类指针获取派生设备对象（常量版本）
 */
static const bsp_fdcan_device_t *bsp_fdcan_get_device_const(const bsp_fdcan_t *const fdcan_base)
{
    return BSP_CONTAINER_OF_CONST(fdcan_base, bsp_fdcan_device_t, super);
}

/**
 * @brief 从基类虚表获取高层操作表
 */
static const bsp_fdcan_ops_t *bsp_fdcan_get_ops(const bsp_fdcan_t *const fdcan_base)
{
    return BSP_CONTAINER_OF_CONST(fdcan_base->super.vptr, bsp_fdcan_ops_t, super);
}

/**
 * @brief 检查数据长度是否为 CAN FD 有效长度（0~8、12、16、20、24、32、48、64）
 */
static bool bsp_fdcan_is_data_length_valid(uint8_t data_length)
{
    return (data_length <= 8U) || (data_length == 12U) || (data_length == 16U) ||
           (data_length == 20U) || (data_length == 24U) || (data_length == 32U) ||
           (data_length == 48U) || (data_length == 64U);
}

/**
 * @brief 校验 FDCAN 帧是否合法（ID、类型、格式、长度）
 */
static bool bsp_fdcan_is_frame_valid(const bsp_fdcan_frame_t *const frame)
{
    // 帧指针和数据长度合法性
    if ((frame == NULL) || !bsp_fdcan_is_data_length_valid(frame->data_length)) {
        return false;
}
    // 枚举值合法性
    if (((frame->id_type != BSP_CAN_ID_STANDARD) && (frame->id_type != BSP_CAN_ID_EXTENDED)) ||
        ((frame->frame_type != BSP_CAN_FRAME_DATA) &&
         (frame->frame_type != BSP_CAN_FRAME_REMOTE)) ||
        ((frame->format != BSP_FDCAN_FORMAT_CLASSIC) &&
         (frame->format != BSP_FDCAN_FORMAT_FD_NO_BRS) &&
         (frame->format != BSP_FDCAN_FORMAT_FD_BRS))) {
        return false;
}
    // Classic 帧长度不能超过 8 字节
    if ((frame->format == BSP_FDCAN_FORMAT_CLASSIC) && (frame->data_length > 8U)) {
        return false;
    }
    // CAN FD 协议不支持 RTR 远程帧，远程帧只能使用 Classic CAN 格式。
    if ((frame->frame_type == BSP_CAN_FRAME_REMOTE) &&
        (frame->format != BSP_FDCAN_FORMAT_CLASSIC)) {
        return false;
    }
    // ID 范围检查
    if ((frame->id_type == BSP_CAN_ID_STANDARD) && (frame->identifier > 0x7FFU)) {
        return false;
}
    return !((frame->id_type == BSP_CAN_ID_EXTENDED) && (frame->identifier > 0x1FFFFFFFU));
}

/**
 * @brief 设备反初始化（虚析构）
 */
static bsp_status_t bsp_fdcan_device_deinit(bsp_device_t *const device_base)
{
    bsp_fdcan_t *const fdcan_base = BSP_CONTAINER_OF(device_base, bsp_fdcan_t, super);
    bsp_fdcan_device_t *const me = bsp_fdcan_get_device(fdcan_base);
    if (me->driver_ops->deinit == NULL) {
        return BSP_STATUS_OK;
}
    return me->driver_ops->deinit(device_base->device_handle);
}

/**
 * @brief 宏：生成无额外参数的转发函数（start/stop）
 */
#define BSP_FDCAN_FORWARD_MUTABLE(name, member)                                                    \
    static bsp_status_t name(bsp_fdcan_t *const fdcan_base)                                        \
    {                                                                                              \
        bsp_fdcan_device_t *const me = bsp_fdcan_get_device(fdcan_base);                           \
        return me->driver_ops->member(fdcan_base->super.device_handle);                            \
    }

// 生成启动和停止转发函数
BSP_FDCAN_FORWARD_MUTABLE(bsp_fdcan_device_start, start)
BSP_FDCAN_FORWARD_MUTABLE(bsp_fdcan_device_stop, stop)

/**
 * @brief 配置过滤器（转发）
 */
static bsp_status_t bsp_fdcan_device_configure_filter(bsp_fdcan_t *const fdcan_base,
                                                      const bsp_can_filter_t *filter_config)
{
    bsp_fdcan_device_t *const me = bsp_fdcan_get_device(fdcan_base);
    return me->driver_ops->configure_filter(fdcan_base->super.device_handle, filter_config);
}

/**
 * @brief 发送 FDCAN 帧（转发）
 */
static bsp_status_t bsp_fdcan_device_transmit(bsp_fdcan_t *const fdcan_base,
                                              const bsp_fdcan_frame_t *frame, uint32_t timeout_ms)
{
    bsp_fdcan_device_t *const me = bsp_fdcan_get_device(fdcan_base);
    return me->driver_ops->transmit(fdcan_base->super.device_handle, frame, timeout_ms);
}

/**
 * @brief 接收 FDCAN 帧（转发）
 */
static bsp_status_t bsp_fdcan_device_receive(bsp_fdcan_t *const fdcan_base,
                                             bsp_can_receive_fifo_t receive_fifo,
                                             bsp_fdcan_frame_t *frame)
{
    bsp_fdcan_device_t *const me = bsp_fdcan_get_device(fdcan_base);
    return me->driver_ops->receive(fdcan_base->super.device_handle, receive_fifo, frame);
}

/**
 * @brief 获取协议状态（转发）
 */
static bsp_status_t
bsp_fdcan_device_get_protocol_status(const bsp_fdcan_t *const fdcan_base,
                                     bsp_fdcan_protocol_status_t *protocol_status)
{
    const bsp_fdcan_device_t *const me = bsp_fdcan_get_device_const(fdcan_base);
    return me->driver_ops->get_protocol_status(fdcan_base->super.device_handle, protocol_status);
}

/**
 * @brief 获取发送邮箱空闲数（转发）
 */
static bsp_status_t bsp_fdcan_device_get_transmit_free_level(const bsp_fdcan_t *const fdcan_base,
                                                             uint32_t *free_level)
{
    const bsp_fdcan_device_t *const me = bsp_fdcan_get_device_const(fdcan_base);
    return me->driver_ops->get_transmit_free_level(fdcan_base->super.device_handle, free_level);
}

/* 虚表，所有函数指向转发函数 */
static const bsp_fdcan_ops_t s_bsp_fdcan_device_ops = {
    .super = {.deinit = bsp_fdcan_device_deinit},
    .start = bsp_fdcan_device_start,
    .stop = bsp_fdcan_device_stop,
    .configure_filter = bsp_fdcan_device_configure_filter,
    .transmit = bsp_fdcan_device_transmit,
    .receive = bsp_fdcan_device_receive,
    .get_protocol_status = bsp_fdcan_device_get_protocol_status,
    .get_transmit_free_level = bsp_fdcan_device_get_transmit_free_level};

/**
 * @brief 初始化 FDCAN 设备实例
 */
bsp_status_t bsp_fdcan_init(bsp_fdcan_device_t *const me, const bsp_fdcan_config_t *const config)
{
    bsp_status_t status;
    // 参数检查：必须实现 start/stop/configure_filter/transmit/receive
    if ((me == NULL) || (config == NULL) || (config->device_handle == NULL) ||
        (config->driver_ops == NULL) || (config->driver_ops->start == NULL) ||
        (config->driver_ops->stop == NULL) || (config->driver_ops->configure_filter == NULL) ||
        (config->driver_ops->transmit == NULL) || (config->driver_ops->receive == NULL)) {
        return BSP_STATUS_INVALID_ARGUMENT;
}

    *me = (bsp_fdcan_device_t){0};
    if (config->driver_ops->init != NULL)
    {
        status = config->driver_ops->init(config->device_handle);
        if (status != BSP_STATUS_OK) {
            return status;
}
    }
    me->driver_ops = config->driver_ops;
    me->super.callback = config->callback;
    me->super.user_context = config->user_context;
    return bsp_device_init(&me->super.super, &s_bsp_fdcan_device_ops.super, config->device_handle);
}

/**
 * @brief 向上转型为基类指针
 */
bsp_fdcan_t *bsp_fdcan_as_base(bsp_fdcan_device_t *const me)
{
    return (me != NULL) ? &me->super : NULL;
}

/**
 * @brief 校验对象是否有效
 */
static bsp_status_t bsp_fdcan_validate(const bsp_fdcan_t *const me)
{
    if (me == NULL) {
        return BSP_STATUS_INVALID_ARGUMENT;
}
    return bsp_device_is_initialized(&me->super) ? BSP_STATUS_OK : BSP_STATUS_NOT_INITIALIZED;
}

/**
 * @brief 设置事件回调
 */
bsp_status_t bsp_fdcan_set_callback(bsp_fdcan_t *const me, bsp_event_callback_t callback,
                                    void *user_context)
{
    bsp_status_t status = bsp_fdcan_validate(me);
    if (status == BSP_STATUS_OK)
    {
        me->callback = callback;
        me->user_context = user_context;
    }
    return status;
}

/**
 * @brief 启动 CAN FD 总线
 */
bsp_status_t bsp_fdcan_start(bsp_fdcan_t *const me)
{
    bsp_status_t status = bsp_fdcan_validate(me);
    return (status == BSP_STATUS_OK) ? bsp_fdcan_get_ops(me)->start(me) : status;
}

/**
 * @brief 停止 CAN FD 总线
 */
bsp_status_t bsp_fdcan_stop(bsp_fdcan_t *const me)
{
    bsp_status_t status = bsp_fdcan_validate(me);
    return (status == BSP_STATUS_OK) ? bsp_fdcan_get_ops(me)->stop(me) : status;
}

/**
 * @brief 配置硬件过滤器（公共接口）
 */
bsp_status_t bsp_fdcan_configure_filter(bsp_fdcan_t *const me,
                                        const bsp_can_filter_t *filter_config)
{
    bsp_status_t status = bsp_fdcan_validate(me);
    if (status != BSP_STATUS_OK) {
        return status;
}
    // 校验 filter_config 参数
    if ((filter_config == NULL) ||
        ((filter_config->id_type != BSP_CAN_ID_STANDARD) &&
         (filter_config->id_type != BSP_CAN_ID_EXTENDED)) ||
        ((filter_config->receive_fifo != BSP_CAN_RX_FIFO_0) &&
         (filter_config->receive_fifo != BSP_CAN_RX_FIFO_1)) ||
        ((filter_config->id_type == BSP_CAN_ID_STANDARD) &&
         ((filter_config->identifier > 0x7FFU) || (filter_config->mask > 0x7FFU))) ||
        ((filter_config->id_type == BSP_CAN_ID_EXTENDED) &&
         ((filter_config->identifier > 0x1FFFFFFFU) || (filter_config->mask > 0x1FFFFFFFU)))) {
        return BSP_STATUS_INVALID_ARGUMENT;
}
    return bsp_fdcan_get_ops(me)->configure_filter(me, filter_config);
}

/**
 * @brief 发送 FDCAN 帧（公共接口）
 */
bsp_status_t bsp_fdcan_transmit(bsp_fdcan_t *const me, const bsp_fdcan_frame_t *frame,
                                uint32_t timeout_ms)
{
    bsp_status_t status = bsp_fdcan_validate(me);
    if (status != BSP_STATUS_OK) {
        return status;
}
    if (!bsp_fdcan_is_frame_valid(frame)) {
        return BSP_STATUS_OUT_OF_RANGE;
}
    return bsp_fdcan_get_ops(me)->transmit(me, frame, timeout_ms);
}

/**
 * @brief 接收 FDCAN 帧（公共接口）
 */
bsp_status_t bsp_fdcan_receive(bsp_fdcan_t *const me, bsp_can_receive_fifo_t receive_fifo,
                               bsp_fdcan_frame_t *frame)
{
    bsp_status_t status = bsp_fdcan_validate(me);
    if (status != BSP_STATUS_OK) {
        return status;
}
    if ((frame == NULL) ||
        ((receive_fifo != BSP_CAN_RX_FIFO_0) && (receive_fifo != BSP_CAN_RX_FIFO_1))) {
        return BSP_STATUS_INVALID_ARGUMENT;
}
    status = bsp_fdcan_get_ops(me)->receive(me, receive_fifo, frame);
    if (status != BSP_STATUS_OK) {
        return status;
}
    // 接收后再次校验帧有效性
    return bsp_fdcan_is_frame_valid(frame) ? BSP_STATUS_OK : BSP_STATUS_IO_ERROR;
}

/**
 * @brief 获取协议状态（公共接口）
 */
bsp_status_t bsp_fdcan_get_protocol_status(const bsp_fdcan_t *const me,
                                           bsp_fdcan_protocol_status_t *protocol_status)
{
    bsp_status_t status = bsp_fdcan_validate(me);
    if (status != BSP_STATUS_OK) {
        return status;
}
    if (protocol_status == NULL) {
        return BSP_STATUS_INVALID_ARGUMENT;
}
    if (bsp_fdcan_get_device_const(me)->driver_ops->get_protocol_status == NULL) {
        return BSP_STATUS_UNSUPPORTED;
}
    return bsp_fdcan_get_ops(me)->get_protocol_status(me, protocol_status);
}

/**
 * @brief 获取发送邮箱空闲数（公共接口）
 */
bsp_status_t bsp_fdcan_get_transmit_free_level(const bsp_fdcan_t *const me, uint32_t *free_level)
{
    bsp_status_t status = bsp_fdcan_validate(me);
    if (status != BSP_STATUS_OK) {
        return status;
}
    if (free_level == NULL) {
        return BSP_STATUS_INVALID_ARGUMENT;
}
    if (bsp_fdcan_get_device_const(me)->driver_ops->get_transmit_free_level == NULL) {
        return BSP_STATUS_UNSUPPORTED;
}
    return bsp_fdcan_get_ops(me)->get_transmit_free_level(me, free_level);
}

/**
 * @brief 事件通知（由平台 ISR 调用）
 */
void bsp_fdcan_notify(bsp_fdcan_t *const me, bsp_event_t event, bsp_status_t status,
                      size_t transferred_size)
{
    if ((me != NULL) && bsp_device_is_initialized(&me->super) && (me->callback != NULL)) {
        me->callback(event, status, transferred_size, me->user_context);
}
}
