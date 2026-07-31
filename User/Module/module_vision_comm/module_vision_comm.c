/**
 * @file module_vision_comm.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 视觉通信模块实现
 * @version 1.0
 * @date 2026-07-28
 * @copyright Copyright (c) 2026
 *
 * @note 视觉设备与主控之间使用固定 5 字节二进制帧，通过 USB CDC 虚拟串口双向传输。
 *       帧格式：0xA5, 0x5A, data1, data2, CRC8(前四字节)
 *       支持流式解析（分包、粘包、帧前噪声）。
 */

#include "module_vision_comm.h"

#include <stddef.h> // NULL, size_t
#include <string.h> // memset, memcpy

/**
 * @brief 计算 CRC8 校验值
 * @param frame_data 数据缓冲区指针
 * @param data_size 数据大小（字节）
 * @return CRC8 校验值
 * @note 初值 0xFF，多项式 0x8C，LSB-first（右移）
 *       用于对帧头 + 两个数据字节（共 4 字节）计算校验
 */
uint8_t module_vision_comm_crc8(const uint8_t *frame_data, size_t data_size)
{
    uint8_t crc = 0xFFU; // 初始值 0xFF
    size_t byte_index;   // 字节索引

    // 数据为空但长度非零 => 参数非法，返回 0
    if ((frame_data == NULL) && (data_size > 0U))
    {
        return 0U;
    }
    // 逐字节处理
    for (byte_index = 0U; byte_index < data_size; ++byte_index)
    {
        uint8_t bit_index;
        crc ^= frame_data[byte_index]; // 与当前字节异或
        for (bit_index = 0U; bit_index < 8U; ++bit_index)
        {
            // 检查最低位，右移并根据情况异或多项式 0x8C
            crc = ((crc & 0x01U) != 0U) ? (uint8_t)((crc >> 1U) ^ 0x8CU) : (uint8_t)(crc >> 1U);
        }
    }
    return crc;
}

/**
 * @brief 构建发送帧（内部函数）
 * @param[out] frame 输出 5 字节帧缓冲区
 * @param data_first 第一个数据字节
 * @param data_second 第二个数据字节
 * @note 组装帧头 + 数据 + CRC8，CRC8 对前 4 字节计算
 */
static void module_vision_comm_build_frame(uint8_t frame[MODULE_VISION_COMM_FRAME_SIZE], uint8_t data_first,
                                      uint8_t data_second)
{
    frame[0] = MODULE_VISION_COMM_FRAME_HEADER_FIRST;                        // 帧头 0xA5
    frame[1] = MODULE_VISION_COMM_FRAME_HEADER_SECOND;                       // 帧头 0x5A
    frame[2] = data_first;                                              // 数据 1
    frame[3] = data_second;                                             // 数据 2
    frame[4] = module_vision_comm_crc8(frame, MODULE_VISION_COMM_CRC_INPUT_SIZE); // CRC8
}

/**
 * @brief 校验接收到的帧是否有效（内部函数）
 * @param frame 5 字节帧缓冲区
 * @return true=有效，false=无效
 * @note 检查帧头是否匹配，CRC8 是否正确
 */
static bool module_vision_comm_frame_is_valid(const uint8_t frame[MODULE_VISION_COMM_FRAME_SIZE])
{
    return (frame[0] == MODULE_VISION_COMM_FRAME_HEADER_FIRST) &&
           (frame[1] == MODULE_VISION_COMM_FRAME_HEADER_SECOND) &&
           (frame[4] == module_vision_comm_crc8(frame, MODULE_VISION_COMM_CRC_INPUT_SIZE));
}

/**
 * @brief 重新同步流解析器（内部函数）
 * @param me 视觉模块对象
 * @note 检查流缓冲区的最后一个字节是否为帧头 0xA5，若成立则保留，否则清空
 */
static void module_vision_comm_resynchronize(module_vision_comm_t *const me)
{
    // 若最后一个字节是 0xA5，可能是下一帧的开始，保留它
    if (me->stream_buffer[MODULE_VISION_COMM_FRAME_SIZE - 1U] == MODULE_VISION_COMM_FRAME_HEADER_FIRST)
    {
        me->stream_buffer[0] = MODULE_VISION_COMM_FRAME_HEADER_FIRST;
        me->stream_size = 1U;
    }
    else
    {
        me->stream_size = 0U;
    }
}

/**
 * @brief 初始化视觉通信模块
 * @param me 视觉模块对象
 * @param config 配置参数
 * @return 执行状态
 */
module_vision_comm_status_t module_vision_comm_init(module_vision_comm_t *const me,
                                          const module_vision_comm_config_t *const config)
{
    bool is_busy;

    // 参数校验：对象、配置、USB VCP 基类（必须已初始化）
    if ((me == NULL) || (config == NULL) || (config->usb_vcp == NULL) ||
        !bsp_device_is_initialized(&config->usb_vcp->super))
    {
        return MODULE_VISION_COMM_STATUS_INVALID_ARGUMENT;
    }
    // 检查 USB VCP 是否可访问（验证句柄有效性）
    if (bsp_usb_vcp_get_busy(config->usb_vcp, &is_busy) != BSP_STATUS_OK)
    {
        return MODULE_VISION_COMM_STATUS_INVALID_ARGUMENT;
    }

    // 清零对象
    (void)memset(me, 0, sizeof(*me));
    me->usb_vcp = config->usb_vcp;
    me->transmit_timeout_ms = config->transmit_timeout_ms;
    me->is_initialized = true;
    return MODULE_VISION_COMM_STATUS_OK;
}

/**
 * @brief 发送两个数据字节到视觉设备
 * @param me 视觉模块对象
 * @param data_first 第一个数据字节
 * @param data_second 第二个数据字节
 * @return 执行状态
 * @note USB 忙时返回 BUSY，不会覆盖正在发送的数据
 *       发送缓冲区在对象内部，调用后数据会被立即复制
 */
module_vision_comm_status_t module_vision_comm_send(module_vision_comm_t *const me,
                                                    uint8_t data_first, uint8_t data_second)
{
    bool is_busy;

    // 状态检查
    if (me == NULL)
    {
        return MODULE_VISION_COMM_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return MODULE_VISION_COMM_STATUS_NOT_INITIALIZED;
    }

    // 查询 USB 是否忙
    if (bsp_usb_vcp_get_busy(me->usb_vcp, &is_busy) != BSP_STATUS_OK)
    {
        return MODULE_VISION_COMM_STATUS_TRANSPORT_ERROR;
    }
    if (is_busy)
    {
        return MODULE_VISION_COMM_STATUS_BUSY;
    }

    // 构建帧并发送
    module_vision_comm_build_frame(me->transmit_buffer, data_first, data_second);
    return (bsp_usb_vcp_transmit(me->usb_vcp, me->transmit_buffer, sizeof(me->transmit_buffer),
                                 me->transmit_timeout_ms) == BSP_STATUS_OK)
               ? MODULE_VISION_COMM_STATUS_OK
               : MODULE_VISION_COMM_STATUS_TRANSPORT_ERROR;
}

/**
 * @brief 注入接收数据流并解析帧
 * @param me 视觉模块对象
 * @param receive_data 接收数据缓冲区
 * @param data_size 数据大小
 * @return 执行状态
 * @note 支持分包、粘包和帧前噪声。
 *       收到有效帧时更新 received_data，并递增 update_count。
 *       若收到无效帧则尝试重新同步。
 */
module_vision_comm_status_t module_vision_comm_feed_data(module_vision_comm_t *const me,
                                               const uint8_t *receive_data, size_t data_size)
{
    size_t byte_index;
    bool valid_frame_received = false;   // 是否收到有效帧
    bool invalid_frame_received = false; // 是否收到无效帧

    // 参数校验
    if ((me == NULL) || ((receive_data == NULL) && (data_size > 0U)))
    {
        return MODULE_VISION_COMM_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return MODULE_VISION_COMM_STATUS_NOT_INITIALIZED;
    }

    // 逐字节处理接收数据流
    for (byte_index = 0U; byte_index < data_size; ++byte_index)
    {
        const uint8_t received_byte = receive_data[byte_index];

        /* -------- 状态 0：等待帧头 0xA5 -------- */
        if (me->stream_size == 0U)
        {
            if (received_byte == MODULE_VISION_COMM_FRAME_HEADER_FIRST)
            {
                me->stream_buffer[0] = received_byte;
                me->stream_size = 1U;
            }
            continue;
        }

        /* -------- 状态 1：等待帧头 0x5A -------- */
        if (me->stream_size == 1U)
        {
            if (received_byte == MODULE_VISION_COMM_FRAME_HEADER_SECOND)
            {
                me->stream_buffer[1] = received_byte;
                me->stream_size = 2U;
            }
            else if (received_byte != MODULE_VISION_COMM_FRAME_HEADER_FIRST)
            {
                // 不是期望的第二个帧头，也不是 0xA5，重置
                me->stream_size = 0U;
            }
            // 若收到 0xA5，保持流大小不变（等待第二个 0xA5 作为新帧开始？）
            // 实际上这里逻辑：如果收到 0xA5，不会进入 else if，也不会清空。
            // 因为 received_byte == MODULE_VISION_COMM_FRAME_HEADER_FIRST 时，什么也不做，
            // stream_size 仍为 1，stream_buffer[0] 已经是 0xA5。
            continue;
        }

        /* -------- 状态 >=2：接收数据字节和 CRC -------- */
        me->stream_buffer[me->stream_size] = received_byte;
        ++me->stream_size;

        /* -------- 满 5 字节时校验帧 -------- */
        if (me->stream_size == MODULE_VISION_COMM_FRAME_SIZE)
        {
            if (module_vision_comm_frame_is_valid(me->stream_buffer))
            {
                // 帧有效：更新接收数据
                me->received_data.data_first = me->stream_buffer[2];
                me->received_data.data_second = me->stream_buffer[3];
                ++me->received_data.update_count;
                me->received_data.is_valid = true;
                me->stream_size = 0U;
                valid_frame_received = true;
            }
            else
            {
                // 帧无效：重新同步
                module_vision_comm_resynchronize(me);
                invalid_frame_received = true;
            }
        }
    }

    // 返回结果：有效帧优先，其次无效帧，最后 OK
    if (valid_frame_received)
    {
        return MODULE_VISION_COMM_STATUS_OK;
    }
    return invalid_frame_received ? MODULE_VISION_COMM_STATUS_INVALID_FRAME : MODULE_VISION_COMM_STATUS_OK;
}

/**
 * @brief 获取最新接收到的有效帧数据
 * @param me 视觉模块对象
 * @param[out] received_data 输出接收数据
 * @return 执行状态
 * @note 若从未收到有效帧，返回 NO_DATA
 */
module_vision_comm_status_t module_vision_comm_get_data(const module_vision_comm_t *const me,
                                              module_vision_comm_data_t *const received_data)
{
    // 参数校验
    if ((me == NULL) || (received_data == NULL))
    {
        return MODULE_VISION_COMM_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return MODULE_VISION_COMM_STATUS_NOT_INITIALIZED;
    }
    if (!me->received_data.is_valid)
    {
        return MODULE_VISION_COMM_STATUS_NO_DATA;
    }
    // 拷贝数据
    *received_data = me->received_data;
    return MODULE_VISION_COMM_STATUS_OK;
}
