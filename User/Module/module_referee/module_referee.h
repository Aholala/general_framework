/**
 * @file module_referee.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief RoboMaster 裁判系统协议解码模块头文件
 *        支持帧接收、CRC 校验、命令路由和帧发送
 * @version 1.0
 * @date 2026-07-28
 * @copyright Copyright (c) 2026
 *
 * @note 本模块提供裁判系统串口协议的流式收发框架：
 *       - 帧同步（0xA5 起始字节）
 *       - CRC8 帧头校验 + CRC16 整帧校验
 *       - 分包/粘包处理
 *       - 命令路由分发
 *       - 在线超时检测
 *       - 发送序号管理
 *       - 完整运行统计
 */

#ifndef MODULE_REFEREE_H
#define MODULE_REFEREE_H

#include "bsp_usart.h"     // USART BSP 抽象层
#include "module_device.h" // 模块设备基类

#ifdef __cplusplus
extern "C"
{
#endif

/* ======================== 帧格式常量 ======================== */

/** @brief 裁判系统帧头起始字节（SOF = Start Of Frame） */
#define MODULE_REFEREE_START_OF_FRAME (0xA5U)

/** @brief 帧头长度：SOF(1) + 数据长度(2) + 序列号(1) + CRC8(1) = 5 字节 */
#define MODULE_REFEREE_HEADER_SIZE (5U)

/** @brief 命令 ID 长度：2 字节（小端序） */
#define MODULE_REFEREE_COMMAND_ID_SIZE (2U)

/** @brief CRC16 校验长度：2 字节 */
#define MODULE_REFEREE_CRC16_SIZE (2U)

/**
 * @brief 帧固定开销：帧头(5) + 命令ID(2) + CRC16(2) = 9 字节
 */
#define MODULE_REFEREE_FRAME_OVERHEAD_SIZE                                                         \
    (MODULE_REFEREE_HEADER_SIZE + MODULE_REFEREE_COMMAND_ID_SIZE + MODULE_REFEREE_CRC16_SIZE)

/**
 * @brief 计算完整帧大小
 * @param payload_size 负载大小（字节）
 * @return 完整帧大小 = 负载大小 + 固定开销
 */
#define MODULE_REFEREE_FRAME_SIZE(payload_size)                                                    \
    ((size_t)(payload_size) + MODULE_REFEREE_FRAME_OVERHEAD_SIZE)

    /* ======================== 状态码枚举 ======================== */

    /**
     * @brief 裁判系统模块状态码
     */
    typedef enum
    {
        MODULE_REFEREE_STATUS_OK = 0,           // 操作成功
        MODULE_REFEREE_STATUS_FRAME_HANDLED,    // 已处理一帧（非错误）
        MODULE_REFEREE_STATUS_BUSY,             // 发送忙（上一帧还未发送完成）
        MODULE_REFEREE_STATUS_INVALID_ARGUMENT, // 参数非法
        MODULE_REFEREE_STATUS_NOT_INITIALIZED,  // 未初始化
        MODULE_REFEREE_STATUS_NOT_STARTED,      // 未启动
        MODULE_REFEREE_STATUS_BUFFER_TOO_SMALL, // 缓冲区太小
        MODULE_REFEREE_STATUS_INVALID_FRAME,    // 无效帧（CRC 不通过等）
        MODULE_REFEREE_STATUS_TRANSPORT_ERROR   // 串口传输错误
    } module_referee_status_t;

    /* ======================== 命令处理器类型 ======================== */

    /**
     * @brief 命令处理器函数类型
     * @param command_id 命令 ID
     * @param payload 负载数据指针（仅在回调期间有效）
     * @param payload_size 负载大小
     * @param sequence 序列号
     * @param user_context 用户上下文
     * @note 回调在任务上下文（module_referee_update）中执行，但应保持轻量
     *       需要长期保存的数据必须复制到调用者状态结构中
     */
    typedef void (*module_referee_command_handler_t)(uint16_t command_id, const uint8_t *payload,
                                                     size_t payload_size, uint8_t sequence,
                                                     void *user_context);

    /* ======================== 路由表项 ======================== */

    /**
     * @brief 命令路由表项
     */
    typedef struct
    {
        uint16_t command_id;                      // 命令 ID
        module_referee_command_handler_t handler; // 命令处理回调
        void *user_context;                       // 回调用户上下文
    } module_referee_route_t;

    /* ======================== 运行统计 ======================== */

    /**
     * @brief 裁判系统运行统计
     */
    typedef struct
    {
        uint32_t received_frame_count;        // 成功接收的帧数
        uint32_t handled_frame_count;         // 已分发的帧数（包括默认处理器）
        uint32_t unknown_command_count;       // 未知命令数（无路由匹配且无默认处理器）
        uint32_t crc8_error_count;            // 帧头 CRC8 错误数
        uint32_t crc16_error_count;           // 帧体 CRC16 错误数
        uint32_t oversize_frame_count;        // 超大帧数（超过流缓冲区容量）
        uint32_t discarded_byte_count;        // 丢弃的非法字节数（非 0xA5 起始）
        uint32_t receive_overrun_count;       // 接收覆盖次数（ISR 拷贝时 pending 未清）
        uint32_t receive_restart_error_count; // 重启 DMA 接收失败次数
    } module_referee_statistics_t;

    /* ======================== 配置结构 ======================== */

    /**
     * @brief 裁判系统初始化配置
     * @note 所有缓冲区由调用者静态分配并保持生命周期
     *       receive_capacity <= processing_capacity
     *       stream_capacity >= 最大帧大小
     *       transmit_capacity >= 最大帧大小
     */
    typedef struct
    {
        bsp_usart_t *usart;                               // USART BSP 基类
        uint8_t *receive_buffer;                          // DMA 接收缓冲区（USART 写入）
        size_t receive_capacity;                          // 接收缓冲区大小
        uint8_t *processing_buffer;                       // 中断到任务的拷贝缓冲区
        size_t processing_capacity;                       // 拷贝缓冲区大小（>= receive_capacity）
        uint8_t *stream_buffer;                           // 流式解析缓冲区
        size_t stream_capacity;                           // 流缓冲区大小
        uint8_t *transmit_buffer;                         // 发送缓冲区
        size_t transmit_capacity;                         // 发送缓冲区大小
        const module_referee_route_t *routes;             // 命令路由表（可为 NULL）
        size_t route_count;                               // 路由表项数
        module_referee_command_handler_t default_handler; // 默认命令处理器（可为 NULL）
        void *default_user_context;                       // 默认处理器用户上下文
        uint32_t receive_timeout_ms;                      // 接收超时（用于 USART 接收）
        uint32_t transmit_timeout_ms;                     // 发送超时
        uint32_t offline_timeout_ms;                      // 离线超时（无帧接收则置离线）
        bsp_transfer_mode_t receive_mode; // 接收模式（仅 INTERRUPT 或 DMA，不支持 BLOCKING）
        const char *logical_name;         // 设备逻辑名称
        uint32_t registration_key;        // 模块注册键值
    } module_referee_config_t;

    /* ======================== 对象结构 ======================== */

    /**
     * @brief 裁判系统设备对象
     */
    typedef struct
    {
        module_device_t super;                            // 设备基类
        bsp_usart_t *usart;                               // USART BSP 基类
        uint8_t *receive_buffer;                          // DMA 接收缓冲区
        size_t receive_capacity;                          // 接收缓冲区大小
        uint8_t *processing_buffer;                       // 中断拷贝缓冲区
        size_t processing_capacity;                       // 拷贝缓冲区大小
        uint8_t *stream_buffer;                           // 流解析缓冲区
        size_t stream_capacity;                           // 流缓冲区大小
        size_t stream_size;                               // 流缓冲区有效字节数
        uint8_t *transmit_buffer;                         // 发送缓冲区
        size_t transmit_capacity;                         // 发送缓冲区大小
        const module_referee_route_t *routes;             // 命令路由表
        size_t route_count;                               // 路由表项数
        module_referee_command_handler_t default_handler; // 默认命令处理器
        void *default_user_context;                       // 默认处理器上下文
        uint32_t receive_timeout_ms;                      // 接收超时 (ms)
        uint32_t transmit_timeout_ms;                     // 发送超时 (ms)
        uint32_t offline_timeout_ms;                      // 离线超时 (ms)
        uint32_t receive_elapsed_time_ms;                 // 距上次接收的时间 (ms)
        volatile size_t pending_receive_size;             // 待处理数据大小（ISR 写入）
        uint8_t transmit_sequence;                        // 发送序列号（递增）
        bsp_transfer_mode_t receive_mode;                 // 接收模式
        module_referee_statistics_t statistics;           // 运行统计
        bool is_online;                                   // 是否在线
        volatile bool is_receive_pending;                 // 是否有待处理数据（ISR 设置）
        volatile bool is_transmit_busy;                   // 发送是否忙
        bool is_started;                                  // 是否已启动
    } module_referee_t;

    /* ======================== 公共 API ======================== */

    /**
     * @brief 初始化裁判系统模块
     * @param me 裁判系统对象
     * @param config 初始化配置
     * @return 执行状态
     */
    module_referee_status_t module_referee_init(module_referee_t *me,
                                                const module_referee_config_t *config);

    /**
     * @brief 启动裁判系统接收（注册回调 + 启动 DMA/中断 接收）
     * @param me 裁判系统对象
     * @return 执行状态
     */
    module_referee_status_t module_referee_start(module_referee_t *me);

    /**
     * @brief 停止裁判系统接收
     * @param me 裁判系统对象
     * @return 执行状态
     */
    module_referee_status_t module_referee_stop(module_referee_t *me);

    /**
     * @brief 注入接收数据到流缓冲区并尝试解析帧
     * @param me 裁判系统对象
     * @param receive_data 接收数据
     * @param data_size 数据大小
     * @return 执行状态
     * @note 通常由 module_referee_update 内部调用，也可用于测试
     */
    module_referee_status_t module_referee_feed_data(module_referee_t *me,
                                                     const uint8_t *receive_data, size_t data_size);

    /**
     * @brief 发送一帧裁判系统数据
     * @param me 裁判系统对象
     * @param command_id 命令 ID
     * @param payload 负载数据
     * @param payload_size 负载大小
     * @param transfer_mode 传输模式（阻塞/中断/DMA）
     * @return 执行状态
     * @note 非阻塞模式下 is_transmit_busy 会阻止并发发送
     */
    module_referee_status_t module_referee_transmit(module_referee_t *me, uint16_t command_id,
                                                    const uint8_t *payload, size_t payload_size,
                                                    bsp_transfer_mode_t transfer_mode);

    /**
     * @brief 构建一帧裁判系统数据（含帧头、CRC 校验）
     * @param[out] frame_buffer 帧缓冲区
     * @param frame_capacity 缓冲区大小
     * @param sequence 序列号
     * @param command_id 命令 ID
     * @param payload 负载
     * @param payload_size 负载大小
     * @param[out] frame_size 实际帧大小
     * @return 执行状态
     * @note 独立函数，可用于无 USART 场景（如单元测试）
     */
    module_referee_status_t module_referee_build_frame(uint8_t *frame_buffer, size_t frame_capacity,
                                                       uint8_t sequence, uint16_t command_id,
                                                       const uint8_t *payload, size_t payload_size,
                                                       size_t *frame_size);

    /**
     * @brief 周期更新（处理待接收数据 + 更新在线超时）
     * @param me 裁判系统对象
     * @param elapsed_time_ms 距上次更新的时间 (ms)
     * @return 执行状态
     * @note 应在主循环或任务中周期性调用（推荐 1ms~10ms）
     */
    module_referee_status_t module_referee_update(module_referee_t *me, uint32_t elapsed_time_ms);

    /**
     * @brief 检查裁判系统是否在线
     * @param me 裁判系统对象
     * @return true=在线（已启动且在 offline_timeout_ms 内收到过有效帧）
     */
    bool module_referee_is_online(const module_referee_t *me);

    /**
     * @brief 获取运行统计
     * @param me 裁判系统对象
     * @param[out] statistics 统计结构体
     * @return 执行状态
     */
    module_referee_status_t module_referee_get_statistics(const module_referee_t *me,
                                                          module_referee_statistics_t *statistics);

    /* ======================== 工具函数 ======================== */

    /**
     * @brief 小端序读取 uint16
     * @param data 2 字节数据
     * @return 解码后的值
     */
    uint16_t module_referee_read_uint16_le(const uint8_t *data);

    /**
     * @brief 小端序读取 uint32
     * @param data 4 字节数据
     * @return 解码后的值
     */
    uint32_t module_referee_read_uint32_le(const uint8_t *data);

    /**
     * @brief 小端序读取 float（二进制兼容）
     * @param data 4 字节数据
     * @return 解码后的 float 值
     */
    float module_referee_read_float_le(const uint8_t *data);

#ifdef __cplusplus
}
#endif

#endif /* MODULE_REFEREE_H */