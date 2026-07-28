/**
 * @file module_dr16.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief DR16 遥控器接收机解码模块头文件
 *        解析 DR16 的 18 字节遥控数据帧，包含摇杆通道、开关、鼠标和键盘状态
 * @version 1.0
 * @date 2026-07-28
 * @copyright Copyright (c) 2026
 */

#ifndef MODULE_DR16_H
#define MODULE_DR16_H

#include "bsp_usart.h"      // USART BSP 基类
#include "module_device.h"  // 设备基类

#ifdef __cplusplus
extern "C"
{
#endif

/** @brief DR16 单帧长度（18 字节） */
#define MODULE_DR16_FRAME_SIZE (18U)
/** @brief 每个 DMA 存储区容量（两帧长度）。 */
#define MODULE_DR16_DMA_BUFFER_SIZE (MODULE_DR16_FRAME_SIZE * 2U)
/** @brief 摇杆通道数（4 个） */
#define MODULE_DR16_CHANNEL_COUNT (4U)

    /**
     * @brief DR16 模块状态码
     */
    typedef enum
    {
        MODULE_DR16_STATUS_OK = 0,              // 操作成功
        MODULE_DR16_STATUS_INVALID_ARGUMENT,     // 参数非法
        MODULE_DR16_STATUS_NOT_INITIALIZED,      // 对象未初始化
        MODULE_DR16_STATUS_TRANSPORT_ERROR,      // 串口传输错误
        MODULE_DR16_STATUS_INVALID_FRAME         // 无效帧（数据校验失败）
    } module_dr16_status_t;

    /**
     * @brief 三段开关状态
     */
    typedef enum
    {
        MODULE_DR16_SWITCH_INVALID = 0, // 无效/未知
        MODULE_DR16_SWITCH_UP = 1,      // 上
        MODULE_DR16_SWITCH_DOWN = 2,    // 下
        MODULE_DR16_SWITCH_MIDDLE = 3   // 中
    } module_dr16_switch_t;

    /**
     * @brief 键盘按键位掩码
     */
    typedef enum
    {
        MODULE_DR16_KEY_W       = (1U << 0),   // W 键
        MODULE_DR16_KEY_S       = (1U << 1),   // S 键
        MODULE_DR16_KEY_A       = (1U << 2),   // A 键
        MODULE_DR16_KEY_D       = (1U << 3),   // D 键
        MODULE_DR16_KEY_SHIFT   = (1U << 4),   // Shift 键
        MODULE_DR16_KEY_CONTROL = (1U << 5),   // Ctrl 键
        MODULE_DR16_KEY_Q       = (1U << 6),   // Q 键
        MODULE_DR16_KEY_E       = (1U << 7),   // E 键
        MODULE_DR16_KEY_R       = (1U << 8),   // R 键
        MODULE_DR16_KEY_F       = (1U << 9),   // F 键
        MODULE_DR16_KEY_G       = (1U << 10),  // G 键
        MODULE_DR16_KEY_Z       = (1U << 11),  // Z 键
        MODULE_DR16_KEY_X       = (1U << 12),  // X 键
        MODULE_DR16_KEY_C       = (1U << 13),  // C 键
        MODULE_DR16_KEY_V       = (1U << 14),  // V 键
        MODULE_DR16_KEY_B       = (1U << 15)   // B 键
    } module_dr16_key_t;

    /**
     * @brief DR16 解码后的数据
     */
    typedef struct
    {
        int16_t channel[MODULE_DR16_CHANNEL_COUNT];           // 四路摇杆原始值（已去中心）
        float normalized_channel[MODULE_DR16_CHANNEL_COUNT];  // 四路摇杆归一化值 [-1.0, 1.0]
        module_dr16_switch_t left_switch;                     // 左侧三段开关状态
        module_dr16_switch_t right_switch;                    // 右侧三段开关状态
        int16_t mouse_x;                                      // 鼠标 X 位移
        int16_t mouse_y;                                      // 鼠标 Y 位移
        int16_t mouse_z;                                      // 鼠标滚轮位移
        bool mouse_left_pressed;                              // 鼠标左键按下
        bool mouse_right_pressed;                             // 鼠标右键按下
        uint16_t keyboard;                                    // 键盘按键位掩码（按位或）
        int16_t dial;                                         // 拨轮原始值（已去中心）
        float normalized_dial;                                // 拨轮归一化值 [-1.0, 1.0]
        uint32_t valid_frame_count;                           // 有效帧计数
        uint32_t invalid_frame_count;                         // 无效帧计数
        uint32_t receive_overrun_count;                       // 接收覆盖次数
        uint32_t transport_error_count;                       // 传输错误次数
        bool is_online;                                       // 是否在线
    } module_dr16_data_t;

    /**
     * @brief 帧回调函数指针（每帧解码完成后调用）
     * @param data 解码后的数据指针
     * @param user_context 用户上下文
     */
    typedef void (*module_dr16_frame_callback_t)(const module_dr16_data_t *data,
                                                 void *user_context);

    /**
     * @brief DR16 初始化配置
     */
    typedef struct
    {
        const char *logical_name;                   // 设备逻辑名称
        uint32_t registration_key;                  // 注册键值
        bsp_usart_t *usart;                         // USART BSP 基类指针
        uint8_t (*dma_receive_buffer)[MODULE_DR16_DMA_BUFFER_SIZE];
        int16_t channel_deadband;                   // 摇杆死区值
        uint32_t offline_timeout_ms;                // 离线超时时间 (ms)
        module_dr16_frame_callback_t frame_callback; // 帧回调函数（可为 NULL）
        void *user_context;                          // 回调用户上下文
    } module_dr16_config_t;

    /**
     * @brief DR16 设备对象
     */
    typedef struct
    {
        module_device_t super;                    // 设备基类
        bsp_usart_t *usart;                       // USART BSP 基类指针
        module_dr16_data_t data;                  // 解码后的数据
        module_dr16_frame_callback_t frame_callback; // 帧回调函数
        void *user_context;                       // 回调用户上下文
        uint8_t (*dma_receive_buffer)[MODULE_DR16_DMA_BUFFER_SIZE]; // DMA M0/M1 双缓冲区
        uint8_t pending_buffer[MODULE_DR16_DMA_BUFFER_SIZE];     // 中断中拷贝的待处理缓冲区
        uint8_t stream_window[MODULE_DR16_FRAME_SIZE];        // 流式滑动窗口
        size_t stream_size;                        // 滑动窗口中有效字节数
        int16_t channel_deadband;                  // 摇杆死区
        uint32_t offline_timeout_ms;               // 离线超时时间 (ms)
        uint32_t time_since_frame_ms;              // 距上一帧的时间 (ms)
        volatile size_t pending_receive_size;      // 待处理数据大小（ISR 中写入）
        volatile bool is_receive_pending;           // 是否有待处理数据（ISR 中置位）
        bool is_receiving;                          // 是否正在接收
    } module_dr16_t;

    /**
     * @brief 初始化 DR16 设备
     * @param me DR16 设备对象
     * @param config 初始化配置
     * @return 执行状态
     */
    module_dr16_status_t module_dr16_init(module_dr16_t *const me,
                                          const module_dr16_config_t *const config);
    /**
     * @brief 启动 DR16 接收（开始 DMA 接收）
     * @param me DR16 设备对象
     * @return 执行状态
     */
    module_dr16_status_t module_dr16_start(module_dr16_t *const me);
    /**
     * @brief 停止 DR16 接收
     * @param me DR16 设备对象
     * @return 执行状态
     */
    module_dr16_status_t module_dr16_stop(module_dr16_t *const me);
    /**
     * @brief 处理待接收数据（从 pending_buffer 解析帧）
     * @param me DR16 设备对象
     * @return 执行状态
     */
    module_dr16_status_t module_dr16_process(module_dr16_t *const me);
    /**
     * @brief 手动注入接收数据（用于非中断模式）
     * @param me DR16 设备对象
     * @param receive_data 接收数据缓冲区
     * @param data_size 数据大小
     * @return 执行状态
     */
    module_dr16_status_t module_dr16_feed_data(module_dr16_t *const me, const uint8_t *receive_data,
                                               size_t data_size);
    /**
     * @brief 更新超时计时
     * @param me DR16 设备对象
     * @param elapsed_time_ms 距上一次调用的时间 (ms)
     */
    void module_dr16_update_time(module_dr16_t *const me, uint32_t elapsed_time_ms);
    /**
     * @brief 获取当前遥控器数据
     * @param me DR16 设备对象
     * @return 数据指针，未初始化返回 NULL
     */
    const module_dr16_data_t *module_dr16_get_data(const module_dr16_t *const me);
    /**
     * @brief 检查指定键盘按键是否被按下
     * @param me DR16 设备对象
     * @param key 按键枚举
     * @return true=按下, false=未按下
     */
    bool module_dr16_is_key_pressed(const module_dr16_t *const me, module_dr16_key_t key);
    /**
     * @brief 归一化摇杆通道值到 [-1.0, 1.0]
     * @param channel_value 原始通道值（已去中心）
     * @return 归一化后的值
     */
    float module_dr16_normalize_channel_value(int16_t channel_value);
    /**
     * @brief 获取 module_device_t 基类指针
     * @param me DR16 设备对象
     * @return module_device_t 指针
     */
    module_device_t *module_dr16_as_device(module_dr16_t *const me);

#ifdef __cplusplus
}
#endif

#endif
