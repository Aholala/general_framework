#include "module_dr16.h"
#include <stddef.h>
#include <string.h>

/** @brief 摇杆最小值 */
#define MODULE_DR16_CHANNEL_MINIMUM (364)
/** @brief 摇杆中位值 */
#define MODULE_DR16_CHANNEL_CENTER (1024)
/** @brief 摇杆最大值 */
#define MODULE_DR16_CHANNEL_MAXIMUM (1684)
/** @brief 摇杆归一化分母 */
#define MODULE_DR16_CHANNEL_SPAN (660.0F)

/**
 * @brief 解码通道 0（从帧字节 0-1 提取 11 位）
 * @param frame 18 字节帧数据
 * @return 解码后的 11 位通道值
 */
static uint16_t module_dr16_decode_channel0(const uint8_t frame[MODULE_DR16_FRAME_SIZE])
{
    const uint32_t packed_value = (uint32_t)frame[0] | ((uint32_t)frame[1] << 8U);
    return (uint16_t)(packed_value & 0x07FFU);
}

/**
 * @brief 解码通道 1（从帧字节 1-2 提取 11 位）
 */
static uint16_t module_dr16_decode_channel1(const uint8_t frame[MODULE_DR16_FRAME_SIZE])
{
    const uint32_t packed_value = ((uint32_t)frame[1] >> 3U) | ((uint32_t)frame[2] << 5U);
    return (uint16_t)(packed_value & 0x07FFU);
}

/**
 * @brief 解码通道 2（从帧字节 2-4 提取 11 位）
 */
static uint16_t module_dr16_decode_channel2(const uint8_t frame[MODULE_DR16_FRAME_SIZE])
{
    const uint32_t packed_value =
        ((uint32_t)frame[2] >> 6U) | ((uint32_t)frame[3] << 2U) | ((uint32_t)frame[4] << 10U);
    return (uint16_t)(packed_value & 0x07FFU);
}

/**
 * @brief 解码通道 3（从帧字节 4-5 提取 11 位）
 */
static uint16_t module_dr16_decode_channel3(const uint8_t frame[MODULE_DR16_FRAME_SIZE])
{
    const uint32_t packed_value = ((uint32_t)frame[4] >> 1U) | ((uint32_t)frame[5] << 7U);
    return (uint16_t)(packed_value & 0x07FFU);
}

/**
 * @brief 小端序解码 16 位有符号整数
 * @param data 2 字节数据
 * @return 解码后的值
 */
static int16_t module_dr16_decode_signed16(const uint8_t *const data)
{
    return (int16_t)(((uint16_t)data[1] << 8U) | data[0]);
}

/**
 * @brief 检查开关值是否合法
 * @param switch_value 从帧中提取的 2 位开关值
 * @return true=合法
 */
static bool module_dr16_is_switch_valid(uint8_t switch_value)
{
    return (switch_value == MODULE_DR16_SWITCH_UP) || (switch_value == MODULE_DR16_SWITCH_DOWN) ||
           (switch_value == MODULE_DR16_SWITCH_MIDDLE);
}

/**
 * @brief 校验一帧数据的合法性（通道范围 + 开关值检查）
 * @param frame 18 字节帧数据
 * @return true=帧有效
 */
static bool module_dr16_is_frame_valid(const uint8_t frame[MODULE_DR16_FRAME_SIZE])
{
    const uint16_t channels[MODULE_DR16_CHANNEL_COUNT] = {
        module_dr16_decode_channel0(frame), module_dr16_decode_channel1(frame),
        module_dr16_decode_channel2(frame), module_dr16_decode_channel3(frame)};
    const uint8_t right_switch = (uint8_t)((frame[5] >> 4U) & 0x03U);
    const uint8_t left_switch = (uint8_t)((frame[5] >> 6U) & 0x03U);
    size_t channel_index;

    for (channel_index = 0U; channel_index < MODULE_DR16_CHANNEL_COUNT; ++channel_index)
    {
        if ((channels[channel_index] < MODULE_DR16_CHANNEL_MINIMUM) ||
            (channels[channel_index] > MODULE_DR16_CHANNEL_MAXIMUM))
        {
            return false;
        }
    }
    return module_dr16_is_switch_valid(left_switch) && module_dr16_is_switch_valid(right_switch);
}

/**
 * @brief 对通道值应用死区
 * @param channel_value 原始值
 * @param deadband 死区值
 * @return 死区内返回 0，否则返回原值
 */
static int16_t module_dr16_apply_deadband(int16_t channel_value, int16_t deadband)
{
    return ((channel_value > -deadband) && (channel_value < deadband)) ? 0 : channel_value;
}

/**
 * @brief 将通道值归一化到 [-1.0, 1.0]
 * @param channel_value 已去中心的通道值
 * @return 归一化后的值（钳位到 [-1, 1]）
 */
float module_dr16_normalize_channel_value(int16_t channel_value)
{
    float normalized_value = (float)channel_value / MODULE_DR16_CHANNEL_SPAN;
    if (normalized_value > 1.0F)
    {
        normalized_value = 1.0F;
    }
    if (normalized_value < -1.0F)
    {
        normalized_value = -1.0F;
    }
    return normalized_value;
}

/**
 * @brief 清空控制数据（离线时置为安全状态）
 * @param me DR16 设备对象
 */
static void module_dr16_clear_control_data(module_dr16_t *const me)
{
    size_t channel_index;
    for (channel_index = 0U; channel_index < MODULE_DR16_CHANNEL_COUNT; ++channel_index)
    {
        me->data.channel[channel_index] = 0;
        me->data.normalized_channel[channel_index] = 0.0F;
    }
    me->data.left_switch = MODULE_DR16_SWITCH_INVALID;   // 开关置为无效
    me->data.right_switch = MODULE_DR16_SWITCH_INVALID;  // 开关置为无效
    me->data.mouse_x = 0;
    me->data.mouse_y = 0;
    me->data.mouse_z = 0;
    me->data.mouse_left_pressed = false;
    me->data.mouse_right_pressed = false;
    me->data.keyboard = 0U;
    me->data.dial = 0;
    me->data.normalized_dial = 0.0F;
}

/**
 * @brief 解析一帧数据并更新 data 结构
 * @param me DR16 设备对象
 * @param frame 18 字节有效帧
 */
static void module_dr16_parse_frame(module_dr16_t *const me,
                                    const uint8_t frame[MODULE_DR16_FRAME_SIZE])
{
    const uint16_t raw_channels[MODULE_DR16_CHANNEL_COUNT] = {
        module_dr16_decode_channel0(frame), module_dr16_decode_channel1(frame),
        module_dr16_decode_channel2(frame), module_dr16_decode_channel3(frame)};
    size_t channel_index;

    for (channel_index = 0U; channel_index < MODULE_DR16_CHANNEL_COUNT; ++channel_index)
    {
        me->data.channel[channel_index] = module_dr16_apply_deadband(
            (int16_t)((int32_t)raw_channels[channel_index] - (int32_t)MODULE_DR16_CHANNEL_CENTER),
            me->channel_deadband);
        me->data.normalized_channel[channel_index] =
            module_dr16_normalize_channel_value(me->data.channel[channel_index]);
    }
    me->data.right_switch = (module_dr16_switch_t)((frame[5] >> 4U) & 0x03U);
    me->data.left_switch = (module_dr16_switch_t)((frame[5] >> 6U) & 0x03U);
    me->data.mouse_x = module_dr16_decode_signed16(&frame[6]);
    me->data.mouse_y = module_dr16_decode_signed16(&frame[8]);
    me->data.mouse_z = module_dr16_decode_signed16(&frame[10]);
    me->data.mouse_left_pressed = (frame[12] != 0U);
    me->data.mouse_right_pressed = (frame[13] != 0U);
    me->data.keyboard = (uint16_t)(((uint16_t)frame[15] << 8U) | frame[14]);
    me->data.dial = module_dr16_apply_deadband(
        (int16_t)(module_dr16_decode_signed16(&frame[16]) - MODULE_DR16_CHANNEL_CENTER),
        me->channel_deadband);
    me->data.normalized_dial = module_dr16_normalize_channel_value(me->data.dial);
    me->data.is_online = true;    // 标记在线
    me->time_since_frame_ms = 0U;
    ++me->data.valid_frame_count;
    if (me->frame_callback != NULL)
    {
        me->frame_callback(&me->data, me->user_context);
    }
}

static void module_dr16_usart_callback(bsp_event_t event, bsp_status_t status,
                                       size_t transferred_size, void *user_context)
{
    module_dr16_t *const me = (module_dr16_t *)user_context;
    (void)transferred_size;
    if ((me == NULL) || !module_device_is_initialized(&me->super))
    {
        return;
    }
    if ((event == BSP_EVENT_ERROR) || (status != BSP_STATUS_OK))
    {
        if (me->data.transport_error_count != UINT32_MAX)
        {
            ++me->data.transport_error_count;
        }
    }
}

/**
 * @brief DMA M0/M1 缓冲区完成回调（ISR 上下文）。
 * @note 平台端已在进入本函数前切换 DMA 目标并重装传输计数。
 */
static void module_dr16_double_buffer_callback(uint8_t completed_buffer_index,
                                               size_t received_size, void *user_context)
{
    module_dr16_t *const me = (module_dr16_t *)user_context;

    if ((me == NULL) || !module_device_is_initialized(&me->super) || !me->is_receiving)
    {
        return;
    }
    if ((completed_buffer_index >= 2U) || (received_size == 0U) ||
        (received_size > MODULE_DR16_DMA_BUFFER_SIZE))
    {
        if (me->data.transport_error_count != UINT32_MAX)
        {
            ++me->data.transport_error_count;
        }
        return;
    }
    if (me->is_receive_pending)
    {
        if (me->data.receive_overrun_count != UINT32_MAX)
        {
            ++me->data.receive_overrun_count;
        }
        return;
    }
    (void)memcpy(me->pending_buffer, me->dma_receive_buffer[completed_buffer_index], received_size);
    me->pending_receive_size = received_size;
    me->is_receive_pending = true;
}

/**
 * @brief 设备启动回调（转发至 module_dr16_start）
 */
static module_device_status_t module_dr16_device_start(module_device_t *const device_base)
{
    module_dr16_t *const me = MODULE_CONTAINER_OF(device_base, module_dr16_t, super);
    return (module_dr16_start(me) == MODULE_DR16_STATUS_OK) ? MODULE_DEVICE_STATUS_OK
                                                            : MODULE_DEVICE_STATUS_OPERATION_FAILED;
}

/**
 * @brief 设备停止回调（转发至 module_dr16_stop）
 */
static module_device_status_t module_dr16_device_stop(module_device_t *const device_base)
{
    module_dr16_t *const me = MODULE_CONTAINER_OF(device_base, module_dr16_t, super);
    return (module_dr16_stop(me) == MODULE_DR16_STATUS_OK) ? MODULE_DEVICE_STATUS_OK
                                                           : MODULE_DEVICE_STATUS_OPERATION_FAILED;
}

/**
 * @brief 设备更新回调（处理接收 + 更新超时）
 */
static module_device_status_t module_dr16_device_update(module_device_t *const device_base,
                                                        uint32_t elapsed_time_ms)
{
    module_dr16_t *const me = MODULE_CONTAINER_OF(device_base, module_dr16_t, super);
    const module_dr16_status_t process_status = module_dr16_process(me);

    if ((process_status != MODULE_DR16_STATUS_OK) &&
        (process_status != MODULE_DR16_STATUS_INVALID_FRAME))
    {
        return MODULE_DEVICE_STATUS_OPERATION_FAILED;
    }
    module_dr16_update_time(me, elapsed_time_ms);
    return MODULE_DEVICE_STATUS_OK;
}

static const module_device_ops_t s_module_dr16_device_ops = {
    .start = module_dr16_device_start,
    .stop = module_dr16_device_stop,
    .update = module_dr16_device_update,
};

/**
 * @brief 初始化 DR16 设备
 *        保存配置、注册 USART 回调、执行两阶段构造
 * @param me DR16 设备对象
 * @param config 初始化配置
 * @return 执行状态
 */
module_dr16_status_t module_dr16_init(module_dr16_t *const me,
                                      const module_dr16_config_t *const config)
{
    if ((me == NULL) || (config == NULL) || (config->usart == NULL) ||
        (config->dma_receive_buffer == NULL) ||
        !bsp_device_is_initialized(&config->usart->super) || (config->channel_deadband < 0) ||
        (config->channel_deadband > 660) || (config->offline_timeout_ms == 0U))
    {
        return MODULE_DR16_STATUS_INVALID_ARGUMENT;
    }
    if (module_device_init_base(&me->super, &s_module_dr16_device_ops,
                                (config->logical_name != NULL) ? config->logical_name : "dr16",
                                config->registration_key) != MODULE_DEVICE_STATUS_OK)
    {
        return MODULE_DR16_STATUS_INVALID_ARGUMENT;
    }
    me->usart = config->usart;
    me->dma_receive_buffer = config->dma_receive_buffer;
    me->data = (module_dr16_data_t){0};
    me->frame_callback = config->frame_callback;
    me->user_context = config->user_context;
    (void)memset(me->dma_receive_buffer, 0, MODULE_DR16_DMA_BUFFER_SIZE * 2U);
    (void)memset(me->pending_buffer, 0, sizeof(me->pending_buffer));
    (void)memset(me->stream_window, 0, sizeof(me->stream_window));
    me->stream_size = 0U;
    me->channel_deadband = config->channel_deadband;
    me->offline_timeout_ms = config->offline_timeout_ms;
    me->time_since_frame_ms = config->offline_timeout_ms;
    me->pending_receive_size = 0U;
    me->is_receive_pending = false;
    me->is_receiving = false;
    module_dr16_clear_control_data(me);
    if (bsp_usart_set_callback(me->usart, module_dr16_usart_callback, me) != BSP_STATUS_OK)
    {
        module_device_abort_init(&me->super);
        return MODULE_DR16_STATUS_TRANSPORT_ERROR;
    }
    if (module_device_complete_init(&me->super) != MODULE_DEVICE_STATUS_OK)
    {
        (void)bsp_usart_set_callback(me->usart, NULL, NULL);
        module_device_abort_init(&me->super);
        return MODULE_DR16_STATUS_INVALID_ARGUMENT;
    }
    return MODULE_DR16_STATUS_OK;
}

/**
 * @brief 启动 DR16 DMA 空闲中断接收
 * @param me DR16 设备对象
 * @return 执行状态
 */
module_dr16_status_t module_dr16_start(module_dr16_t *const me)
{
    if ((me == NULL) || !module_device_is_initialized(&me->super))
    {
        return (me == NULL) ? MODULE_DR16_STATUS_INVALID_ARGUMENT
                            : MODULE_DR16_STATUS_NOT_INITIALIZED;
    }
    me->is_receiving = true;
    me->pending_receive_size = 0U;
    me->is_receive_pending = false;
    if (bsp_usart_receive_to_idle_double_buffer(
            me->usart, me->dma_receive_buffer[0], me->dma_receive_buffer[1],
            MODULE_DR16_DMA_BUFFER_SIZE, module_dr16_double_buffer_callback,
            me) != BSP_STATUS_OK)
    {
        me->is_receiving = false;
        return MODULE_DR16_STATUS_TRANSPORT_ERROR;
    }
    return MODULE_DR16_STATUS_OK;
}

/**
 * @brief 停止 DR16 接收
 * @param me DR16 设备对象
 * @return 执行状态
 */
module_dr16_status_t module_dr16_stop(module_dr16_t *const me)
{
    if ((me == NULL) || !module_device_is_initialized(&me->super))
    {
        return (me == NULL) ? MODULE_DR16_STATUS_INVALID_ARGUMENT
                            : MODULE_DR16_STATUS_NOT_INITIALIZED;
    }
    me->is_receiving = false;
    me->is_receive_pending = false;
    me->pending_receive_size = 0U;
    return (bsp_usart_abort(me->usart) == BSP_STATUS_OK) ? MODULE_DR16_STATUS_OK
                                                         : MODULE_DR16_STATUS_TRANSPORT_ERROR;
}

/**
 * @brief 处理待接收数据
 *        从 pending_buffer 拷贝到临时缓冲区并调用 feed_data 解析
 * @param me DR16 设备对象
 * @return 执行状态
 */
module_dr16_status_t module_dr16_process(module_dr16_t *const me)
{
    uint8_t received_data[MODULE_DR16_FRAME_SIZE * 2U];
    size_t received_size;

    if (me == NULL)
    {
        return MODULE_DR16_STATUS_INVALID_ARGUMENT;
    }
    if (!module_device_is_initialized(&me->super))
    {
        return MODULE_DR16_STATUS_NOT_INITIALIZED;
    }
    if (!me->is_receive_pending)
    {
        return MODULE_DR16_STATUS_OK;
    }

    received_size = me->pending_receive_size;
    if ((received_size == 0U) || (received_size > sizeof(received_data)))
    {
        me->pending_receive_size = 0U;
        me->is_receive_pending = false;
        if (me->data.transport_error_count != UINT32_MAX)
        {
            ++me->data.transport_error_count;
        }
        return MODULE_DR16_STATUS_TRANSPORT_ERROR;
    }
    (void)memcpy(received_data, me->pending_buffer, received_size);
    me->pending_receive_size = 0U;
    me->is_receive_pending = false;
    return module_dr16_feed_data(me, received_data, received_size);
}

/**
 * @brief 注入数据到流式窗口并尝试解析帧
 * @param me DR16 设备对象
 * @param receive_data 接收数据
 * @param data_size 数据大小
 * @return 至少解析出一帧返回 OK，否则返回 INVALID_FRAME
 */
module_dr16_status_t module_dr16_feed_data(module_dr16_t *const me, const uint8_t *receive_data,
                                           size_t data_size)
{
    size_t data_index;
    bool parsed_frame = false;
    if ((me == NULL) || (receive_data == NULL) || (data_size == 0U))
    {
        return MODULE_DR16_STATUS_INVALID_ARGUMENT;
    }
    if (!module_device_is_initialized(&me->super))
    {
        return MODULE_DR16_STATUS_NOT_INITIALIZED;
    }
    for (data_index = 0U; data_index < data_size; ++data_index)
    {
        me->stream_window[me->stream_size] = receive_data[data_index];
        ++me->stream_size;
        if (me->stream_size == MODULE_DR16_FRAME_SIZE)
        {
            if (module_dr16_is_frame_valid(me->stream_window))
            {
                module_dr16_parse_frame(me, me->stream_window);
                me->stream_size = 0U;
                parsed_frame = true;
            }
            else
            {
                (void)memmove(&me->stream_window[0], &me->stream_window[1],
                              MODULE_DR16_FRAME_SIZE - 1U);
                me->stream_size = MODULE_DR16_FRAME_SIZE - 1U;
                ++me->data.invalid_frame_count;
            }
        }
    }
    return parsed_frame ? MODULE_DR16_STATUS_OK : MODULE_DR16_STATUS_INVALID_FRAME;
}

/**
 * @brief 更新超时计时，超时则置为离线
 * @param me DR16 设备对象
 * @param elapsed_time_ms 距上次调用的时间 (ms)
 */
void module_dr16_update_time(module_dr16_t *const me, uint32_t elapsed_time_ms)
{
    if ((me == NULL) || !module_device_is_initialized(&me->super) || !me->data.is_online)
    {
        return;
    }
    if (elapsed_time_ms > (UINT32_MAX - me->time_since_frame_ms))
    {
        me->time_since_frame_ms = UINT32_MAX;
    }
    else
    {
        me->time_since_frame_ms += elapsed_time_ms;
    }
    if (me->time_since_frame_ms >= me->offline_timeout_ms)
    {
        module_dr16_clear_control_data(me);
    me->data.is_online = false;   // 标记离线
    }
}

/**
 * @brief 获取当前遥控器数据指针
 * @param me DR16 设备对象
 * @return 数据指针，未初始化返回 NULL
 */
const module_dr16_data_t *module_dr16_get_data(const module_dr16_t *const me)
{
    return ((me != NULL) && module_device_is_initialized(&me->super)) ? &me->data : NULL;
}

/**
 * @brief 检查按键是否按下
 * @param me DR16 设备对象
 * @param key 按键枚举值
 * @return true=按下
 */
bool module_dr16_is_key_pressed(const module_dr16_t *const me, module_dr16_key_t key)
{
    return (me != NULL) && module_device_is_initialized(&me->super) &&
           ((me->data.keyboard & (uint16_t)key) != 0U);
}

/**
 * @brief 获取 device 基类指针
 * @param me DR16 设备对象
 * @return module_device_t 指针
 */
module_device_t *module_dr16_as_device(module_dr16_t *const me)
{
    return (me != NULL) ? &me->super : NULL;
}
