/**
 * @file module_board_comm.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 云台板与底盘板之间的板间通信协议头文件
 * @version 1.0
 * @date 2026-07-28
 * @copyright Copyright (c) 2026
 *
 * @note 传输通用遥控输入、云台、底盘和发射机构关键数据。
 *       从 base_identifier 开始连续分配 7 个消息类型。
 *       支持分片组装（同一数据组的多个分片共享序列号）。
 */

#ifndef MODULE_BOARD_COMM_H
#define MODULE_BOARD_COMM_H

#include "bsp_can.h" // CAN BSP 抽象层

#include <stdbool.h> // bool
#include <stdint.h>  // uint32_t

#define MODULE_BOARD_COMM_REMOTE_CHANNEL_COUNT (4U)

#ifdef __cplusplus
extern "C"
{
#endif

    /* ======================== 状态码枚举 ======================== */

    /**
     * @brief Robot Link 模块状态码
     */
    typedef enum
    {
        MODULE_BOARD_COMM_STATUS_OK = 0,           // 操作成功
        MODULE_BOARD_COMM_STATUS_INVALID_ARGUMENT, // 参数非法
        MODULE_BOARD_COMM_STATUS_NOT_INITIALIZED,  // 对象未初始化
        MODULE_BOARD_COMM_STATUS_TRANSPORT_ERROR,  // CAN 传输错误
        MODULE_BOARD_COMM_STATUS_INVALID_FRAME     // 无效帧（ID 越界等）
    } module_board_comm_status_t;

    /* ======================== 消息类型枚举 ======================== */

    /**
     * @brief Robot Link 协议消息类型
     * @note 从 base_identifier 开始连续分配
     */
    typedef enum
    {
        MODULE_BOARD_COMM_MESSAGE_REMOTE_CHANNELS_PRIMARY = 0, // 遥控主通道（channel 0~2）
        MODULE_BOARD_COMM_MESSAGE_REMOTE_CHANNELS_AUXILIARY,   // 遥控辅助通道（channel
                                                               // 3、拨轮、键盘）
        MODULE_BOARD_COMM_MESSAGE_REMOTE_INPUT,                // 遥控输入（鼠标+开关+按键）
        MODULE_BOARD_COMM_MESSAGE_GIMBAL_PRIMARY,              // 云台主数据（角度+偏航角速度）
        MODULE_BOARD_COMM_MESSAGE_GIMBAL_AUXILIARY,            // 云台辅助数据（俯仰角速度）
        MODULE_BOARD_COMM_MESSAGE_CHASSIS,                     // 底盘数据（速度+状态）
        MODULE_BOARD_COMM_MESSAGE_SHOOTER,                     // 发射机构数据
        MODULE_BOARD_COMM_MESSAGE_COUNT                        // 消息总数（=7）
    } module_board_comm_message_t;

    /* ======================== 数据结构体 ======================== */

    /**
     * @brief 板间协议使用的三段开关值
     * @note 数值与线上协议一致，不依赖具体遥控器驱动的枚举类型。
     */
    typedef enum
    {
        MODULE_BOARD_COMM_SWITCH_INVALID = 0,
        MODULE_BOARD_COMM_SWITCH_UP = 1,
        MODULE_BOARD_COMM_SWITCH_DOWN = 2,
        MODULE_BOARD_COMM_SWITCH_MIDDLE = 3
    } module_board_comm_switch_t;

    /**
     * @brief 板间传输的遥控输入数据
     * @note 只包含线上协议实际传输的字段。DR16 或其他输入设备由 App 映射到本结构体。
     */
    typedef struct
    {
        int16_t channel[MODULE_BOARD_COMM_REMOTE_CHANNEL_COUNT];
        module_board_comm_switch_t left_switch;
        module_board_comm_switch_t right_switch;
        int16_t mouse_x;
        int16_t mouse_y;
        int16_t mouse_z;
        bool mouse_left_pressed;
        bool mouse_right_pressed;
        uint16_t keyboard;
        int16_t dial;
        uint32_t update_count;
        bool is_online;
    } module_board_comm_remote_process_data_t;

    /**
     * @brief 云台数据
     */
    typedef struct
    {
        float yaw_rad;                  // 偏航角（弧度）
        float pitch_rad;                // 俯仰角（弧度）
        float yaw_velocity_rad_per_s;   // 偏航角速度（rad/s）
        float pitch_velocity_rad_per_s; // 俯仰角速度（rad/s）
        bool imu_valid;                 // IMU 数据是否有效
        bool motors_online;             // 电机是否在线
    } module_board_comm_gimbal_process_data_t;

    /**
     * @brief 底盘数据
     */
    typedef struct
    {
        float velocity_x_m_per_s;         // X 方向速度（m/s）
        float velocity_y_m_per_s;         // Y 方向速度（m/s）
        float angular_velocity_rad_per_s; // 角速度（rad/s）
        bool motors_online;               // 电机是否在线
        bool self_lock_active;            // 自锁是否激活
    } module_board_comm_chassis_process_data_t;

    /**
     * @brief 发射机构数据
     */
    typedef struct
    {
        uint8_t state;           // 发射机构状态
        uint8_t jam_retry_count; // 卡弹重试次数
        bool friction_ready;     // 摩擦轮是否稳定到速
        bool fire_permission;    // 当前自瞄火控许可
    } module_board_comm_shooter_process_data_t;

    /* ======================== 配置结构体 ======================== */

    /**
     * @brief Robot Link 初始化配置
     */
    typedef struct
    {
        bsp_can_t *can;               // CAN BSP 基类
        uint32_t base_identifier;     // CAN ID 基址（消息从此开始连续分配）
        uint32_t transmit_timeout_ms; // 发送超时（毫秒）
        uint32_t offline_timeout_ms;  // 离线超时（毫秒）
    } module_board_comm_config_t;

    /* ======================== 对象结构体 ======================== */

    /**
     * @brief Robot Link 设备对象
     */
    typedef struct
    {
        bsp_can_t *can;                                         // CAN BSP 基类
        uint32_t base_identifier;                               // CAN ID 基址
        uint32_t transmit_timeout_ms;                           // 发送超时
        uint32_t offline_timeout_ms;                            // 离线超时
        module_board_comm_remote_process_data_t remote_data;    // 遥控器已提交数据
        module_board_comm_remote_process_data_t remote_staging; // 遥控器分片暂存数据
        module_board_comm_gimbal_process_data_t gimbal_data;    // 云台已提交数据
        module_board_comm_gimbal_process_data_t gimbal_staging; // 云台暂存数据
        module_board_comm_chassis_process_data_t chassis_data;  // 底盘数据（单帧）
        module_board_comm_shooter_process_data_t shooter_data;  // 发射机构数据（单帧）
        uint32_t remote_elapsed_time_ms;                        // 遥控器距上次接收的时间（ms）
        uint32_t gimbal_elapsed_time_ms;                        // 云台距上次接收的时间（ms）
        uint32_t chassis_elapsed_time_ms;                       // 底盘距上次接收的时间（ms）
        uint32_t shooter_elapsed_time_ms;                       // 发射机构距上次接收的时间（ms）
        uint8_t transmit_sequence;                              // 发送序列号（递增）
        uint8_t remote_receive_mask;                            // 遥控器接收掩码（位0~2）
        uint8_t remote_assembly_sequence;                       // 遥控器当前组装序列号
        uint8_t gimbal_receive_mask;                            // 云台接收掩码（位0~1）
        uint8_t gimbal_assembly_sequence;                       // 云台当前组装序列号
        bool remote_online;                                     // 遥控器是否在线
        bool gimbal_online;                                     // 云台是否在线
        bool chassis_online;                                    // 底盘是否在线
        bool shooter_online;                                    // 发射机构是否在线
        bool is_initialized;                                    // 是否已初始化
    } module_board_comm_t;

    /* ======================== 公共 API ======================== */

    /**
     * @brief 初始化 Robot Link 模块
     * @param me Robot Link 对象
     * @param config 配置参数
     * @return 执行状态
     */
    module_board_comm_status_t module_board_comm_init(module_board_comm_t *me,
                                                      const module_board_comm_config_t *config);

    /**
     * @brief 反初始化 Robot Link 模块
     * @param me Robot Link 对象
     * @return 执行状态
     */
    module_board_comm_status_t module_board_comm_deinit(module_board_comm_t *me);

    /**
     * @brief 发送遥控器数据（三帧分片）
     * @param me Robot Link 对象
     * @param remote_data 遥控器数据
     * @return 执行状态
     */
    module_board_comm_status_t
    module_board_comm_send_remote(module_board_comm_t *me,
                                  const module_board_comm_remote_process_data_t *remote_data);

    /**
     * @brief 发送云台数据（两帧分片）
     * @param me Robot Link 对象
     * @param gimbal_data 云台数据
     * @return 执行状态
     */
    module_board_comm_status_t
    module_board_comm_send_gimbal(module_board_comm_t *me,
                                  const module_board_comm_gimbal_process_data_t *gimbal_data);

    /**
     * @brief 发送底盘数据（单帧）
     * @param me Robot Link 对象
     * @param chassis_data 底盘数据
     * @return 执行状态
     */
    module_board_comm_status_t
    module_board_comm_send_chassis(module_board_comm_t *me,
                                   const module_board_comm_chassis_process_data_t *chassis_data);

    /**
     * @brief 发送发射机构数据（单帧）
     * @param me Robot Link 对象
     * @param shooter_data 发射机构数据
     * @return 执行状态
     */
    module_board_comm_status_t
    module_board_comm_send_shooter(module_board_comm_t *me,
                                   const module_board_comm_shooter_process_data_t *shooter_data);

    /**
     * @brief 处理接收到的 CAN 帧
     * @param me Robot Link 对象
     * @param frame CAN 帧
     * @return 执行状态
     */
    module_board_comm_status_t module_board_comm_handle_frame(module_board_comm_t *me,
                                                              const bsp_can_frame_t *frame);

    /**
     * @brief 更新各数据组的在线超时计时
     * @param me Robot Link 对象
     * @param elapsed_time_ms 距上次更新的时间（毫秒）
     */
    void module_board_comm_update_time(module_board_comm_t *me, uint32_t elapsed_time_ms);

    /**
     * @brief 获取遥控器数据（只读）
     * @param me Robot Link 对象
     * @return 遥控器数据指针，若离线则返回 NULL
     */
    const module_board_comm_remote_process_data_t *
    module_board_comm_get_remote(const module_board_comm_t *me);

    /**
     * @brief 获取云台数据（只读）
     * @param me Robot Link 对象
     * @return 云台数据指针，若离线则返回 NULL
     */
    const module_board_comm_gimbal_process_data_t *
    module_board_comm_get_gimbal(const module_board_comm_t *me);

    /**
     * @brief 获取底盘数据（只读）
     * @param me Robot Link 对象
     * @return 底盘数据指针，若离线则返回 NULL
     */
    const module_board_comm_chassis_process_data_t *
    module_board_comm_get_chassis(const module_board_comm_t *me);

    /**
     * @brief 获取发射机构数据（只读）
     * @param me Robot Link 对象
     * @return 发射机构数据指针，若离线则返回 NULL
     */
    const module_board_comm_shooter_process_data_t *
    module_board_comm_get_shooter(const module_board_comm_t *me);

#ifdef __cplusplus
}
#endif

#endif /* MODULE_BOARD_COMM_H */
