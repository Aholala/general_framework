/**
 * @file module_vision_comm.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 视觉通信模块头文件
 * @version 1.0
 * @date 2026-07-28
 * @copyright Copyright (c) 2026
 *
 * @note 视觉设备与主控之间使用固定 5 字节二进制帧，通过 USB CDC 虚拟串口双向传输。
 *       帧格式：0xA5, 0x5A, data1, data2, CRC8(前四字节)
 *       CRC8 参数：初值 0xFF，多项式 0x8C，LSB first
 */

#ifndef MODULE_VISION_COMM_H
#define MODULE_VISION_COMM_H

#include "bsp_usb_vcp.h" // USB 虚拟串口 BSP 抽象层

#include <stdbool.h> // bool
#include <stddef.h>  // size_t
#include <stdint.h>  // uint8_t, uint32_t

#ifdef __cplusplus
extern "C"
{
#endif

/* ======================== 帧格式常量 ======================== */

/** @brief 帧头第一个字节（0xA5） */
#define MODULE_VISION_COMM_FRAME_HEADER_FIRST (0xA5U)
/** @brief 帧头第二个字节（0x5A） */
#define MODULE_VISION_COMM_FRAME_HEADER_SECOND (0x5AU)
/** @brief 帧总大小（5 字节） */
#define MODULE_VISION_COMM_FRAME_SIZE (5U)
/** @brief CRC8 输入大小（帧头 2 + 数据 2 = 4 字节） */
#define MODULE_VISION_COMM_CRC_INPUT_SIZE (4U)

    /* ======================== 状态码枚举 ======================== */

    /**
     * @brief 视觉模块状态码
     */
    typedef enum
    {
        MODULE_VISION_COMM_STATUS_OK = 0,           // 操作成功
        MODULE_VISION_COMM_STATUS_INVALID_ARGUMENT, // 参数非法
        MODULE_VISION_COMM_STATUS_NOT_INITIALIZED,  // 对象未初始化
        MODULE_VISION_COMM_STATUS_TRANSPORT_ERROR,  // USB 传输错误
        MODULE_VISION_COMM_STATUS_BUSY,             // USB 发送忙
        MODULE_VISION_COMM_STATUS_INVALID_FRAME,    // 收到无效帧（CRC 错误等）
        MODULE_VISION_COMM_STATUS_NO_DATA           // 没有有效数据（未收到帧）
    } module_vision_comm_status_t;

    /* ======================== 接收数据结构 ======================== */

    /**
     * @brief 接收到的视觉数据
     * @note 固定通信帧中的两个数据字节
     */
    typedef struct
    {
        uint8_t data_first;    // 第一个数据字节
        uint8_t data_second;   // 第二个数据字节
        uint32_t update_count; // 更新计数（每次收到有效帧递增）
        bool is_valid;         // 是否有效（是否收到过有效帧）
    } module_vision_comm_process_data_t;

    /* ======================== 配置结构 ======================== */

    /**
     * @brief 视觉模块初始化配置
     */
    typedef struct
    {
        bsp_usb_vcp_t *usb_vcp;       // USB VCP BSP 基类
        uint32_t transmit_timeout_ms; // 发送超时（毫秒）
    } module_vision_comm_config_t;

    /* ======================== 对象结构 ======================== */

    /**
     * @brief 视觉通信模块对象
     */
    typedef struct
    {
        bsp_usb_vcp_t *usb_vcp;                            // USB VCP BSP 基类
        uint32_t transmit_timeout_ms;                      // 发送超时（毫秒）
        uint8_t transmit_buffer[MODULE_VISION_COMM_FRAME_SIZE]; // 发送缓冲区
        uint8_t stream_buffer[MODULE_VISION_COMM_FRAME_SIZE];   // 流解析缓冲区
        size_t stream_size;                                // 流缓冲区有效字节数
        module_vision_comm_process_data_t received_data;                // 最新接收数据
        bool is_initialized;                               // 是否已初始化
    } module_vision_comm_t;

    /* ======================== 公共 API ======================== */

    /**
     * @brief 初始化视觉通信模块
     * @param me 视觉模块对象
     * @param config 配置参数
     * @return 执行状态
     */
    module_vision_comm_status_t module_vision_comm_init(module_vision_comm_t *const me,
                                              const module_vision_comm_config_t *const config);

    /**
     * @brief 发送两个数据字节到视觉设备
     * @param me 视觉模块对象
     * @param data_first 第一个数据字节
     * @param data_second 第二个数据字节
     * @return 执行状态
     */
    module_vision_comm_status_t module_vision_comm_send(module_vision_comm_t *const me,
                                                       uint8_t data_first, uint8_t data_second);

    /**
     * @brief 注入接收数据流并解析帧
     * @param me 视觉模块对象
     * @param receive_data 接收数据缓冲区
     * @param data_size 数据大小
     * @return 执行状态
     */
    module_vision_comm_status_t module_vision_comm_feed_data(module_vision_comm_t *const me,
                                                   const uint8_t *receive_data, size_t data_size);

    /**
     * @brief 获取最新接收到的有效帧数据
     * @param me 视觉模块对象
     * @param[out] received_data 输出接收数据
     * @return 执行状态
     */
    module_vision_comm_status_t module_vision_comm_get_data(const module_vision_comm_t *const me,
                                                  module_vision_comm_process_data_t *const received_data);

    /**
     * @brief 计算 CRC8
     * @param frame_data 数据缓冲区
     * @param data_size 数据大小
     * @return CRC8 值
     * @note 初值 0xFF，多项式 0x8C，LSB first
     */
    uint8_t module_vision_comm_crc8(const uint8_t *frame_data, size_t data_size);

#ifdef __cplusplus
}
#endif

#endif /* MODULE_VISION_COMM_H */
