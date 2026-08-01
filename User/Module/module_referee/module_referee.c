/**
 * @file module_referee.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief RoboMaster 裁判系统协议解码模块实现
 *        支持帧接收、CRC8/CRC16 校验、命令路由分发和帧发送
 * @version 1.0
 * @date 2026-07-28
 * @copyright Copyright (c) 2026
 *
 * @note 接收路径：
 *        USART DMA/idle ISR -> 拷贝到 processing_buffer -> 重启 DMA -> 设置 pending
 *        task: module_referee_update -> 追加到 stream_buffer -> 查找 0xA5 -> CRC8 校验
 *        -> 等待完整帧 -> CRC16 校验 -> 路由分发
 */

#include "module_referee.h"

#include "module_referee_crc.h" // CRC 校验函数

MODULE_STATIC_ASSERT_SUPER_FIRST(module_referee_t);

#include <stddef.h> // NULL, size_t
#include <string.h> // memcpy, memmove

/**
 * @brief 递增计数器（溢出保护）
 * @param counter 计数器指针
 */
static void module_referee_increment_counter(uint32_t *counter)
{
    // 防止溢出回绕（到达 UINT32_MAX 后停止递增）
    if (*counter != UINT32_MAX)
    {
        ++(*counter);
    }
}

/**
 * @brief 丢弃流缓冲区前缀数据
 * @param me 裁判系统对象
 * @param discarded_size 要丢弃的字节数
 */
static void module_referee_discard_stream_prefix(module_referee_t *me, size_t discarded_size)
{
    // 丢弃大小 >= 有效数据大小 => 清空缓冲区
    if (discarded_size >= me->stream_size)
    {
        me->stream_size = 0U;
        return;
    }
    // 将剩余数据前移
    memmove(me->stream_buffer, &me->stream_buffer[discarded_size],
            me->stream_size - discarded_size);
    me->stream_size -= discarded_size;
}

/**
 * @brief 分发一帧到已注册的命令处理器
 * @param me 裁判系统对象
 * @param command_id 命令 ID
 * @param payload 负载数据
 * @param payload_size 负载大小
 * @param sequence 序列号
 */
static void module_referee_dispatch_frame(module_referee_t *me, uint16_t command_id,
                                          const uint8_t *payload, size_t payload_size,
                                          uint8_t sequence)
{
    size_t route_index;

    // 遍历路由表，查找匹配的命令 ID
    for (route_index = 0U; route_index < me->route_count; ++route_index)
    {
        if ((me->routes[route_index].command_id == command_id) &&
            (me->routes[route_index].handler != NULL))
        {
            // 调用匹配的处理器
            me->routes[route_index].handler(command_id, payload, payload_size, sequence,
                                            me->routes[route_index].user_context);
            module_referee_increment_counter(&me->statistics.handled_frame_count);
            return;
        }
    }
    // 无匹配路由，尝试默认处理器
    if (me->default_handler != NULL)
    {
        me->default_handler(command_id, payload, payload_size, sequence, me->default_user_context);
        module_referee_increment_counter(&me->statistics.handled_frame_count);
    }
    else
    {
        // 无默认处理器，计数未知命令
        module_referee_increment_counter(&me->statistics.unknown_command_count);
    }
}

/**
 * @brief 处理流缓冲区中的所有完整帧
 *        依次检查 SOF、CRC8、帧大小、CRC16 并分发
 * @param me 裁判系统对象
 * @return 至少处理一帧返回 FRAME_HANDLED，否则返回 OK
 */
static module_referee_status_t module_referee_process_stream(module_referee_t *me)
{
    bool handled_frame = false;

    // 循环处理，直到缓冲区不足一个帧头
    while (me->stream_size >= MODULE_REFEREE_HEADER_SIZE)
    {
        uint16_t payload_size;
        size_t frame_size;
        uint16_t command_id;
        uint8_t sequence;

        // 1. 查找 SOF（起始字节 0xA5）
        if (me->stream_buffer[0] != MODULE_REFEREE_START_OF_FRAME)
        {
            module_referee_discard_stream_prefix(me, 1U); // 丢弃非法字节
            module_referee_increment_counter(&me->statistics.discarded_byte_count);
            continue;
        }

        // 2. 验证帧头 CRC8
        if (!module_referee_crc8_verify(me->stream_buffer, MODULE_REFEREE_HEADER_SIZE))
        {
            module_referee_discard_stream_prefix(me, 1U); // CRC8 失败，丢弃 SOF
            module_referee_increment_counter(&me->statistics.crc8_error_count);
            continue;
        }

        // 3. 读取负载大小（小端序，帧头偏移 1）
        payload_size = module_referee_read_uint16_le(&me->stream_buffer[1]);
        frame_size = MODULE_REFEREE_FRAME_SIZE(payload_size);

        // 4. 检查帧大小是否超出流缓冲区容量
        if (frame_size > me->stream_capacity)
        {
            module_referee_discard_stream_prefix(me, 1U);
            module_referee_increment_counter(&me->statistics.oversize_frame_count);
            continue;
        }

        // 5. 等待完整帧到达
        if (me->stream_size < frame_size)
        {
            break; // 数据不足，等待更多数据
        }

        // 6. 验证整帧 CRC16
        if (!module_referee_crc16_verify(me->stream_buffer, frame_size))
        {
            module_referee_discard_stream_prefix(me, 1U);
            module_referee_increment_counter(&me->statistics.crc16_error_count);
            continue;
        }

        // 7. 提取序列号（帧头偏移 3）和命令 ID（帧头偏移 5）
        sequence = me->stream_buffer[3];
        command_id = module_referee_read_uint16_le(&me->stream_buffer[MODULE_REFEREE_HEADER_SIZE]);

        // 8. 统计并分发
        module_referee_increment_counter(&me->statistics.received_frame_count);
        module_referee_dispatch_frame(
            me, command_id,
            &me->stream_buffer[MODULE_REFEREE_HEADER_SIZE + MODULE_REFEREE_COMMAND_ID_SIZE],
            payload_size, sequence);

        // 9. 丢弃已处理的帧
        module_referee_discard_stream_prefix(me, frame_size);
        me->receive_elapsed_time_ms = 0U;
        me->is_online = true;
        handled_frame = true;
    }
    return handled_frame ? MODULE_REFEREE_STATUS_FRAME_HANDLED : MODULE_REFEREE_STATUS_OK;
}

/**
 * @brief USART 中断回调（ISR 上下文）
 *        将接收数据拷贝到 processing_buffer 并置位标志
 * @note ISR 中不解析协议，仅拷贝数据和重启接收
 */
static void module_referee_usart_callback(bsp_event_t event, bsp_status_t status,
                                          size_t transferred_size, void *user_context)
{
    module_referee_t *const me = (module_referee_t *)user_context;

    if (me == NULL)
    {
        return;
    }

    // 接收完成或待接收事件
    if ((event == BSP_EVENT_RECEIVE_COMPLETE) || (event == BSP_EVENT_RECEIVE_PENDING))
    {
        // 检查状态和数据有效性
        if ((status == BSP_STATUS_OK) && (transferred_size > 0U) &&
            (transferred_size <= me->receive_capacity) &&
            (transferred_size <= me->processing_capacity))
        {
            // 如果上次数据已被处理，拷贝新数据
            if (!me->is_receive_pending)
            {
                (void)memcpy(me->processing_buffer, me->receive_buffer, transferred_size);
                me->pending_receive_size = transferred_size;
                me->is_receive_pending = true;
            }
            else
            {
                // 上次数据还未处理，发生覆盖
                module_referee_increment_counter(&me->statistics.receive_overrun_count);
            }
        }
        else
        {
            // 数据无效，计数错误
            module_referee_increment_counter(&me->statistics.receive_overrun_count);
        }

        // 立即重启接收（DMA 或中断模式）
        if (me->is_started &&
            (bsp_usart_receive_to_idle(me->usart, me->receive_buffer, me->receive_capacity,
                                       me->receive_mode, me->receive_timeout_ms) != BSP_STATUS_OK))
        {
            module_referee_increment_counter(&me->statistics.receive_restart_error_count);
        }
    }
    // 发送完成/中止/错误事件 -> 清除发送忙标志
    else if ((event == BSP_EVENT_TRANSMIT_COMPLETE) || (event == BSP_EVENT_ABORT_COMPLETE) ||
             (event == BSP_EVENT_ERROR))
    {
        me->is_transmit_busy = false;
    }
}

/**
 * @brief 设备启动回调（转发至 module_referee_start）
 */
static module_device_status_t module_referee_device_start(module_device_t *const device_base)
{
    module_referee_t *const me = MODULE_CONTAINER_OF(device_base, module_referee_t, super);
    return (module_referee_start(me) == MODULE_REFEREE_STATUS_OK)
               ? MODULE_DEVICE_STATUS_OK
               : MODULE_DEVICE_STATUS_OPERATION_FAILED;
}

/**
 * @brief 设备停止回调（转发至 module_referee_stop）
 */
static module_device_status_t module_referee_device_stop(module_device_t *const device_base)
{
    module_referee_t *const me = MODULE_CONTAINER_OF(device_base, module_referee_t, super);
    return (module_referee_stop(me) == MODULE_REFEREE_STATUS_OK)
               ? MODULE_DEVICE_STATUS_OK
               : MODULE_DEVICE_STATUS_OPERATION_FAILED;
}

/**
 * @brief 设备更新回调（转发至 module_referee_update）
 */
static module_device_status_t module_referee_device_update(module_device_t *const device_base,
                                                           uint32_t elapsed_time_ms)
{
    module_referee_t *const me = MODULE_CONTAINER_OF(device_base, module_referee_t, super);
    return (module_referee_update(me, elapsed_time_ms) == MODULE_REFEREE_STATUS_OK)
               ? MODULE_DEVICE_STATUS_OK
               : MODULE_DEVICE_STATUS_OPERATION_FAILED;
}

/** 模块设备操作表 */
static const module_device_ops_t s_module_referee_ops = {
    .start = module_referee_device_start,
    .stop = module_referee_device_stop,
    .update = module_referee_device_update,
};

/**
 * @brief 初始化裁判系统模块
 *        校验配置参数、检查路由表重复项、执行两阶段构造
 * @param me 裁判系统对象
 * @param config 配置参数
 * @return 执行状态
 */
module_referee_status_t module_referee_init(module_referee_t *me,
                                            const module_referee_config_t *config)
{
    size_t route_index;

    /* -------- 参数校验 -------- */
    // 基本参数
    if ((me == NULL) || (config == NULL) || (config->usart == NULL) ||
        !bsp_device_is_initialized(&config->usart->super) || (config->receive_buffer == NULL) ||
        (config->receive_capacity == 0U) || (config->processing_buffer == NULL) ||
        (config->processing_capacity < config->receive_capacity) ||
        (config->stream_buffer == NULL) ||
        (config->stream_capacity < MODULE_REFEREE_FRAME_OVERHEAD_SIZE) ||
        (config->transmit_buffer == NULL) ||
        (config->transmit_capacity < MODULE_REFEREE_FRAME_OVERHEAD_SIZE) ||
        !bsp_transfer_mode_is_valid(config->receive_mode) ||
        (config->receive_mode == BSP_TRANSFER_MODE_BLOCKING) || // 接收必须异步
        ((config->route_count > 0U) && (config->routes == NULL)) ||
        (config->offline_timeout_ms == 0U))
    {
        return MODULE_REFEREE_STATUS_INVALID_ARGUMENT;
    }

    // 检查路由表：handler 非空，命令 ID 不重复
    for (route_index = 0U; route_index < config->route_count; ++route_index)
    {
        size_t comparison_index;
        if (config->routes[route_index].handler == NULL)
        {
            return MODULE_REFEREE_STATUS_INVALID_ARGUMENT;
        }
        for (comparison_index = route_index + 1U; comparison_index < config->route_count;
             ++comparison_index)
        {
            if (config->routes[route_index].command_id ==
                config->routes[comparison_index].command_id)
            {
                return MODULE_REFEREE_STATUS_INVALID_ARGUMENT; // 重复命令 ID
            }
        }
    }

    /* -------- 初始化对象 -------- */
    *me = (module_referee_t){0}; // 清零所有字段

    // 复制配置参数到对象
    me->usart = config->usart;
    me->receive_buffer = config->receive_buffer;
    me->receive_capacity = config->receive_capacity;
    me->processing_buffer = config->processing_buffer;
    me->processing_capacity = config->processing_capacity;
    me->stream_buffer = config->stream_buffer;
    me->stream_capacity = config->stream_capacity;
    me->transmit_buffer = config->transmit_buffer;
    me->transmit_capacity = config->transmit_capacity;
    me->routes = config->routes;
    me->route_count = config->route_count;
    me->default_handler = config->default_handler;
    me->default_user_context = config->default_user_context;
    me->receive_timeout_ms = config->receive_timeout_ms;
    me->transmit_timeout_ms = config->transmit_timeout_ms;
    me->offline_timeout_ms = config->offline_timeout_ms;
    me->receive_mode = config->receive_mode;

    // 两阶段设备初始化
    if (module_device_init_base(&me->super, &s_module_referee_ops, config->logical_name,
                                config->registration_key) != MODULE_DEVICE_STATUS_OK)
    {
        return MODULE_REFEREE_STATUS_INVALID_ARGUMENT;
    }
    if (module_device_complete_init(&me->super) != MODULE_DEVICE_STATUS_OK)
    {
        module_device_abort_init(&me->super);
        return MODULE_REFEREE_STATUS_INVALID_ARGUMENT;
    }
    return MODULE_REFEREE_STATUS_OK;
}

/**
 * @brief 启动裁判系统接收
 *        注册 USART 回调并启动 DMA/中断空闲接收
 * @param me 裁判系统对象
 * @return 执行状态
 */
module_referee_status_t module_referee_start(module_referee_t *me)
{
    // 状态检查
    if ((me == NULL) || !module_device_is_initialized(&me->super))
    {
        return MODULE_REFEREE_STATUS_NOT_INITIALIZED;
    }

    // 注册 USART 回调
    if (bsp_usart_set_callback(me->usart, module_referee_usart_callback, me) != BSP_STATUS_OK)
    {
        return MODULE_REFEREE_STATUS_TRANSPORT_ERROR;
    }

    // 启动空闲接收（DMA 或中断模式）
    if (bsp_usart_receive_to_idle(me->usart, me->receive_buffer, me->receive_capacity,
                                  me->receive_mode, me->receive_timeout_ms) != BSP_STATUS_OK)
    {
        (void)bsp_usart_set_callback(me->usart, NULL, NULL); // 回滚
        return MODULE_REFEREE_STATUS_TRANSPORT_ERROR;
    }

    // 重置状态
    me->stream_size = 0U;
    me->receive_elapsed_time_ms = 0U;
    me->is_online = false;
    me->is_receive_pending = false;
    me->pending_receive_size = 0U;
    me->is_transmit_busy = false;
    me->is_started = true;
    return MODULE_REFEREE_STATUS_OK;
}

/**
 * @brief 停止裁判系统接收
 * @param me 裁判系统对象
 * @return 执行状态
 */
module_referee_status_t module_referee_stop(module_referee_t *me)
{
    if ((me == NULL) || !module_device_is_initialized(&me->super))
    {
        return MODULE_REFEREE_STATUS_NOT_INITIALIZED;
    }

    // 中止 USART 传输
    (void)bsp_usart_abort(me->usart);

    // 注销回调
    if (bsp_usart_set_callback(me->usart, NULL, NULL) != BSP_STATUS_OK)
    {
        return MODULE_REFEREE_STATUS_TRANSPORT_ERROR;
    }

    // 重置状态
    me->is_started = false;
    me->is_online = false;
    me->is_receive_pending = false;
    me->pending_receive_size = 0U;
    me->is_transmit_busy = false;
    return MODULE_REFEREE_STATUS_OK;
}

/**
 * @brief 注入接收数据到流缓冲区
 *        自动丢弃超出容量的旧数据，然后调用 process_stream 解析
 * @param me 裁判系统对象
 * @param receive_data 接收数据
 * @param data_size 数据大小
 * @return 执行状态
 */
module_referee_status_t module_referee_feed_data(module_referee_t *me, const uint8_t *receive_data,
                                                 size_t data_size)
{
    // 参数校验
    if ((me == NULL) || (receive_data == NULL) || (data_size == 0U))
    {
        return MODULE_REFEREE_STATUS_INVALID_ARGUMENT;
    }
    if (!module_device_is_initialized(&me->super))
    {
        return MODULE_REFEREE_STATUS_NOT_INITIALIZED;
    }

    // 如果流缓冲区剩余空间不足，丢弃最旧的数据
    if (data_size > me->stream_capacity - me->stream_size)
    {
        const size_t required_space = data_size - (me->stream_capacity - me->stream_size);
        if (required_space >= me->stream_size)
        {
            me->stream_size = 0U; // 全部丢弃
        }
        else
        {
            module_referee_discard_stream_prefix(me, required_space);
        }
        module_referee_increment_counter(&me->statistics.oversize_frame_count);
    }

    // 如果数据大于流缓冲区容量，只保留尾部数据
    if (data_size > me->stream_capacity)
    {
        receive_data += data_size - me->stream_capacity;
        data_size = me->stream_capacity;
    }

    // 追加数据到流缓冲区
    memcpy(&me->stream_buffer[me->stream_size], receive_data, data_size);
    me->stream_size += data_size;

    // 尝试解析帧
    return module_referee_process_stream(me);
}

/**
 * @brief 构建一帧裁判系统数据（含帧头、CRC8、命令 ID、负载、CRC16）
 * @param[out] frame_buffer 帧缓冲区
 * @param frame_capacity 缓冲区容量
 * @param sequence 序列号
 * @param command_id 命令 ID
 * @param payload 负载
 * @param payload_size 负载大小
 * @param[out] frame_size 帧大小
 * @return 执行状态
 */
module_referee_status_t module_referee_build_frame(uint8_t *frame_buffer, size_t frame_capacity,
                                                   uint8_t sequence, uint16_t command_id,
                                                   const uint8_t *payload, size_t payload_size,
                                                   size_t *frame_size)
{
    const size_t required_size = MODULE_REFEREE_FRAME_SIZE(payload_size);

    // 参数校验
    if ((frame_buffer == NULL) || (frame_size == NULL) ||
        ((payload_size > 0U) && (payload == NULL)) || (payload_size > UINT16_MAX))
    {
        return MODULE_REFEREE_STATUS_INVALID_ARGUMENT;
    }
    if (frame_capacity < required_size)
    {
        return MODULE_REFEREE_STATUS_BUFFER_TOO_SMALL;
    }

    // 1. SOF（起始字节）
    frame_buffer[0] = MODULE_REFEREE_START_OF_FRAME;

    // 2. 负载大小（小端序）
    frame_buffer[1] = (uint8_t)payload_size;
    frame_buffer[2] = (uint8_t)(payload_size >> 8U);

    // 3. 序列号
    frame_buffer[3] = sequence;

    // 4. CRC8 占位（先写 0，由 crc8_append 计算）
    frame_buffer[4] = 0U;
    (void)module_referee_crc8_append(frame_buffer, MODULE_REFEREE_HEADER_SIZE);

    // 5. 命令 ID（小端序）
    frame_buffer[5] = (uint8_t)command_id;
    frame_buffer[6] = (uint8_t)(command_id >> 8U);

    // 6. 负载数据
    if (payload_size > 0U)
    {
        memcpy(&frame_buffer[7], payload, payload_size);
    }

    // 7. CRC16（自动追加到末尾）
    (void)module_referee_crc16_append(frame_buffer, required_size);

    *frame_size = required_size;
    return MODULE_REFEREE_STATUS_OK;
}

/**
 * @brief 发送一帧裁判系统数据
 *        构建帧并通过 USART 发送（支持阻塞/非阻塞模式）
 * @param me 裁判系统对象
 * @param command_id 命令 ID
 * @param payload 负载
 * @param payload_size 负载大小
 * @param transfer_mode 传输模式
 * @return 执行状态
 * @note 非阻塞模式下 is_transmit_busy 会阻止并发发送
 */
module_referee_status_t module_referee_transmit(module_referee_t *me, uint16_t command_id,
                                                const uint8_t *payload, size_t payload_size,
                                                bsp_transfer_mode_t transfer_mode)
{
    size_t frame_size;
    module_referee_status_t status;

    // 参数校验
    if (me == NULL)
    {
        return MODULE_REFEREE_STATUS_INVALID_ARGUMENT;
    }
    if (!module_device_is_initialized(&me->super))
    {
        return MODULE_REFEREE_STATUS_NOT_INITIALIZED;
    }
    if (!me->is_started)
    {
        return MODULE_REFEREE_STATUS_NOT_STARTED;
    }

    // 检查发送忙
    if (me->is_transmit_busy)
    {
        return MODULE_REFEREE_STATUS_BUSY;
    }

    // 构建帧
    status = module_referee_build_frame(me->transmit_buffer, me->transmit_capacity,
                                        me->transmit_sequence, command_id, payload, payload_size,
                                        &frame_size);
    if (status != MODULE_REFEREE_STATUS_OK)
    {
        return status;
    }

    // 非阻塞模式标记忙
    me->is_transmit_busy = transfer_mode != BSP_TRANSFER_MODE_BLOCKING;

    // 发送
    if (bsp_usart_transmit(me->usart, me->transmit_buffer, frame_size, transfer_mode,
                           me->transmit_timeout_ms) != BSP_STATUS_OK)
    {
        me->is_transmit_busy = false;
        return MODULE_REFEREE_STATUS_TRANSPORT_ERROR;
    }

    // 递增序列号
    ++me->transmit_sequence;
    return MODULE_REFEREE_STATUS_OK;
}

/**
 * @brief 周期更新
 *        处理 pending 数据 + 更新在线超时计时
 * @param me 裁判系统对象
 * @param elapsed_time_ms 距上次更新的时间 (ms)
 * @return 执行状态
 */
module_referee_status_t module_referee_update(module_referee_t *me, uint32_t elapsed_time_ms)
{
    // 状态检查
    if ((me == NULL) || !module_device_is_initialized(&me->super))
    {
        return MODULE_REFEREE_STATUS_NOT_INITIALIZED;
    }
    if (!me->is_started)
    {
        return MODULE_REFEREE_STATUS_NOT_STARTED;
    }

    // 处理待接收数据（由 ISR 拷贝到 processing_buffer）
    if (me->is_receive_pending)
    {
        const size_t received_size = me->pending_receive_size;
        (void)module_referee_feed_data(me, me->processing_buffer, received_size);
        me->pending_receive_size = 0U;
        me->is_receive_pending = false;
    }

    // 更新在线超时计时
    if (UINT32_MAX - me->receive_elapsed_time_ms < elapsed_time_ms)
    {
        me->receive_elapsed_time_ms = UINT32_MAX; // 饱和
    }
    else
    {
        me->receive_elapsed_time_ms += elapsed_time_ms;
    }

    // 超时则置离线
    if (me->receive_elapsed_time_ms >= me->offline_timeout_ms)
    {
        me->is_online = false;
    }
    return MODULE_REFEREE_STATUS_OK;
}

/**
 * @brief 检查裁判系统是否在线
 * @param me 裁判系统对象
 * @return true=在线且已启动
 */
bool module_referee_is_online(const module_referee_t *me)
{
    return (me != NULL) && module_device_is_initialized(&me->super) && me->is_started &&
           me->is_online;
}

/**
 * @brief 获取运行统计信息
 * @param me 裁判系统对象
 * @param[out] statistics 统计结构体
 * @return 执行状态
 */
module_referee_status_t module_referee_get_statistics(const module_referee_t *me,
                                                      module_referee_statistics_t *statistics)
{
    if ((me == NULL) || (statistics == NULL))
    {
        return MODULE_REFEREE_STATUS_INVALID_ARGUMENT;
    }
    if (!module_device_is_initialized(&me->super))
    {
        return MODULE_REFEREE_STATUS_NOT_INITIALIZED;
    }
    *statistics = me->statistics;
    return MODULE_REFEREE_STATUS_OK;
}

/**
 * @brief 小端序解码 uint16
 * @param data 2 字节数据
 * @return 解码后的值
 */
uint16_t module_referee_read_uint16_le(const uint8_t *data)
{
    uint16_t value = 0U;
    if (data != NULL)
    {
        value = (uint16_t)((uint16_t)data[0] | (uint16_t)((uint16_t)data[1] << 8U));
    }
    return value;
}

/**
 * @brief 小端序解码 uint32
 * @param data 4 字节数据
 * @return 解码后的值
 */
uint32_t module_referee_read_uint32_le(const uint8_t *data)
{
    return (data != NULL) ? (uint32_t)data[0] | ((uint32_t)data[1] << 8U) |
                                ((uint32_t)data[2] << 16U) | ((uint32_t)data[3] << 24U)
                          : 0U;
}

/**
 * @brief 小端序解码 float（通过 memcpy 保持二进制兼容）
 * @param data 4 字节数据
 * @return 解码后的浮点值
 */
float module_referee_read_float_le(const uint8_t *data)
{
    uint32_t raw_value = module_referee_read_uint32_le(data);
    float value;
    memcpy(&value, &raw_value, sizeof(value));
    return value;
}
