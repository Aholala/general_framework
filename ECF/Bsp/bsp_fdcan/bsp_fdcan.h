/**
 * @file bsp_fdcan.h
 * @brief CAN FD 通用抽象层头文件
 * @note 定义 FDCAN 帧结构、协议状态、虚表和公共 API。
 */

#ifndef BSP_FDCAN_H
#define BSP_FDCAN_H

#include "bsp_can.h" // 复用 CAN 的 ID、帧类型、FIFO 枚举

#ifdef __cplusplus
extern "C"
{
#endif

    /* 前向声明 */
    typedef struct bsp_fdcan bsp_fdcan_t;
    typedef struct bsp_fdcan_device bsp_fdcan_device_t;

    /**
     * @brief FDCAN 帧格式（Classic / FD 无BRS / FD 带BRS）
     */
    typedef enum
    {
        BSP_FDCAN_FORMAT_CLASSIC = 0, // 经典 CAN（最大 8 字节）
        BSP_FDCAN_FORMAT_FD_NO_BRS,   // CAN FD，数据段与仲裁段速率相同
        BSP_FDCAN_FORMAT_FD_BRS       // CAN FD，数据段速率提升（BRS）
    } bsp_fdcan_format_t;

    /**
     * @brief FDCAN 帧结构
     */
    typedef struct
    {
        uint32_t identifier;             // 11/29 位 ID
        bsp_can_id_type_t id_type;       // 标准/扩展
        bsp_can_frame_type_t frame_type; // 数据/远程
        bsp_fdcan_format_t format;       // 帧格式
        uint8_t data_length;             // 有效字节数（0~8,12,16,20,24,32,48,64）
        uint8_t data[64];                // 数据负载
    } bsp_fdcan_frame_t;

    /**
     * @brief CAN FD 协议状态
     */
    typedef struct
    {
        bool is_bus_off;              // 是否处于 Bus-Off
        bool is_error_passive;        // 是否 Error Passive
        bool has_warning;             // 是否达到警告阈值
        uint8_t transmit_error_count; // 发送错误计数
        uint8_t receive_error_count;  // 接收错误计数
        uint32_t last_error_code;     // 平台相关错误码
    } bsp_fdcan_protocol_status_t;

    /* ---------- 高层虚表 ---------- */
    typedef struct
    {
        bsp_device_ops_t super;
        bsp_status_t (*start)(bsp_fdcan_t *const me);
        bsp_status_t (*stop)(bsp_fdcan_t *const me);
        bsp_status_t (*configure_filter)(bsp_fdcan_t *const me,
                                         const bsp_can_filter_t *filter_config);
        bsp_status_t (*transmit)(bsp_fdcan_t *const me, const bsp_fdcan_frame_t *frame,
                                 uint32_t timeout_ms);
        bsp_status_t (*receive)(bsp_fdcan_t *const me, bsp_can_receive_fifo_t receive_fifo,
                                bsp_fdcan_frame_t *frame);
        bsp_status_t (*get_protocol_status)(const bsp_fdcan_t *const me,
                                            bsp_fdcan_protocol_status_t *protocol_status);
        bsp_status_t (*get_transmit_free_level)(const bsp_fdcan_t *const me, uint32_t *free_level);
    } bsp_fdcan_ops_t;

    /* ---------- 基类 ---------- */
    struct bsp_fdcan
    {
        bsp_device_t super;
        bsp_event_callback_t callback;
        void *user_context;
    };

    /* ---------- 底层驱动操作表 ---------- */
    typedef struct
    {
        bsp_status_t (*init)(void *device_handle);
        bsp_status_t (*deinit)(void *device_handle);
        bsp_status_t (*start)(void *device_handle);
        bsp_status_t (*stop)(void *device_handle);
        bsp_status_t (*configure_filter)(void *device_handle,
                                         const bsp_can_filter_t *filter_config);
        bsp_status_t (*transmit)(void *device_handle, const bsp_fdcan_frame_t *frame,
                                 uint32_t timeout_ms);
        bsp_status_t (*receive)(void *device_handle, bsp_can_receive_fifo_t receive_fifo,
                                bsp_fdcan_frame_t *frame);
        bsp_status_t (*get_protocol_status)(const void *device_handle,
                                            bsp_fdcan_protocol_status_t *protocol_status);
        bsp_status_t (*get_transmit_free_level)(const void *device_handle, uint32_t *free_level);
    } bsp_fdcan_driver_ops_t;

    /* ---------- 派生设备对象 ---------- */
    struct bsp_fdcan_device
    {
        bsp_fdcan_t super;
        const bsp_fdcan_driver_ops_t *driver_ops;
    };

    /* ---------- 配置结构 ---------- */
    typedef struct
    {
        void *device_handle;
        const bsp_fdcan_driver_ops_t *driver_ops;
        bsp_event_callback_t callback;
        void *user_context;
    } bsp_fdcan_config_t;

    /* ---------- 公共 API ---------- */
    bsp_status_t bsp_fdcan_init(bsp_fdcan_device_t *const me,
                                const bsp_fdcan_config_t *const config);
    bsp_fdcan_t *bsp_fdcan_as_base(bsp_fdcan_device_t *const me);
    bsp_status_t bsp_fdcan_set_callback(bsp_fdcan_t *const me, bsp_event_callback_t callback,
                                        void *user_context);
    bsp_status_t bsp_fdcan_start(bsp_fdcan_t *const me);
    bsp_status_t bsp_fdcan_stop(bsp_fdcan_t *const me);
    bsp_status_t bsp_fdcan_configure_filter(bsp_fdcan_t *const me,
                                            const bsp_can_filter_t *filter_config);
    bsp_status_t bsp_fdcan_transmit(bsp_fdcan_t *const me, const bsp_fdcan_frame_t *frame,
                                    uint32_t timeout_ms);
    bsp_status_t bsp_fdcan_receive(bsp_fdcan_t *const me, bsp_can_receive_fifo_t receive_fifo,
                                   bsp_fdcan_frame_t *frame);
    bsp_status_t bsp_fdcan_get_protocol_status(const bsp_fdcan_t *const me,
                                               bsp_fdcan_protocol_status_t *protocol_status);
    bsp_status_t bsp_fdcan_get_transmit_free_level(const bsp_fdcan_t *const me,
                                                   uint32_t *free_level);
    void bsp_fdcan_notify(bsp_fdcan_t *const me, bsp_event_t event, bsp_status_t status,
                          size_t transferred_size);

#ifdef __cplusplus
}
#endif

#endif /* BSP_FDCAN_H */