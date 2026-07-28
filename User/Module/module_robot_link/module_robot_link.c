/**
 * @file module_robot_link.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 云台板与底盘板之间的 CAN 数据协议实现
 * @version 1.0
 * @date 2026-07-28
 * @copyright Copyright (c) 2026
 *
 * @note 传输 DR16、云台、底盘、发射机构和心跳关键数据。
 *       支持分片组装（同一数据组的多个分片共享序列号），
 *       各数据组独立超时检测。
 */

#include "module_robot_link.h"

#include <math.h>
#include <stddef.h> // NULL
#include <string.h> // memcpy, memset

/** @brief 浮点数缩放因子（int16 范围 -32768~32767，缩放到 ±32.767） */
#define MODULE_ROBOT_LINK_SCALE (1000.0F)

/**
 * @brief 将 int16 编码为小端序 2 字节
 * @param value 要编码的值
 * @param output 输出缓冲区（2 字节）
 */
static void module_robot_link_encode_int16(int16_t value, uint8_t output[2])
{
    const uint16_t unsigned_value = (uint16_t)value;
    output[0] = (uint8_t)unsigned_value;         // 低字节
    output[1] = (uint8_t)(unsigned_value >> 8U); // 高字节
}

/**
 * @brief 从小端序 2 字节解码 int16
 * @param input 输入缓冲区（2 字节）
 * @return 解码后的值
 */
static int16_t module_robot_link_decode_int16(const uint8_t input[2])
{
    return (int16_t)((uint16_t)input[0] | ((uint16_t)input[1] << 8U));
}

/**
 * @brief 将浮点数缩放并编码为 int16
 * @param value 浮点值
 * @return 缩放并钳位后的 int16 值
 * @note 缩放因子 1000，范围 ±32.767
 */
static int16_t module_robot_link_encode_scaled(float value)
{
    float scaled_value = value * MODULE_ROBOT_LINK_SCALE;

    // 钳位到 int16 范围（-32768~32767）
    if (scaled_value > 32767.0F)
    {
        scaled_value = 32767.0F;
    }
    else if (scaled_value < -32768.0F)
    {
        scaled_value = -32768.0F;
    }
    // lrintf 四舍五入到最近的整数
    return (int16_t)lrintf(scaled_value);
}

/**
 * @brief 从小端序 2 字节解码并反缩放为浮点数
 * @param input 输入缓冲区（2 字节）
 * @return 解码后的浮点值
 */
static float module_robot_link_decode_scaled(const uint8_t input[2])
{
    return (float)module_robot_link_decode_int16(input) / MODULE_ROBOT_LINK_SCALE;
}

/**
 * @brief 准备接收遥控器数据的事务（分片组装用）
 * @param me Robot Link 对象
 * @param sequence 当前分片的序列号
 * @note 若序列号与当前组装事务相同，保留已有 staging 数据；
 *       否则重置 staging 并更新序列号。
 */
static void module_robot_link_prepare_remote_transaction(module_robot_link_t *me, uint8_t sequence)
{
    // 如果已有数据且序列号相同，继续当前事务
    if ((me->remote_receive_mask != 0U) && (me->remote_assembly_sequence == sequence))
    {
        return;
    }
    // 新事务：重置 staging，保留计数器的当前值
    me->remote_staging = (module_dr16_data_t){0};
    me->remote_staging.valid_frame_count = me->remote_data.valid_frame_count;
    me->remote_staging.invalid_frame_count = me->remote_data.invalid_frame_count;
    me->remote_assembly_sequence = sequence;
    me->remote_receive_mask = 0U;
}

/**
 * @brief 准备接收云台数据的事务（分片组装用）
 * @param me Robot Link 对象
 * @param sequence 当前分片的序列号
 */
static void module_robot_link_prepare_gimbal_transaction(module_robot_link_t *me, uint8_t sequence)
{
    if ((me->gimbal_receive_mask != 0U) && (me->gimbal_assembly_sequence == sequence))
    {
        return;
    }
    me->gimbal_staging = (module_robot_link_gimbal_data_t){0};
    me->gimbal_assembly_sequence = sequence;
    me->gimbal_receive_mask = 0U;
}

/**
 * @brief 发送 CAN 帧
 * @param me Robot Link 对象
 * @param message 消息类型
 * @param payload 6 字节负载
 * @param flags 标志字节
 * @param sequence 序列号
 * @return 执行状态
 */
static module_robot_link_status_t module_robot_link_transmit(module_robot_link_t *me,
                                                             module_robot_link_message_t message,
                                                             const uint8_t payload[6],
                                                             uint8_t flags, uint8_t sequence)
{
    bsp_can_frame_t frame = {
        .identifier = me->base_identifier + (uint32_t)message, // CAN ID = 基址 + 消息类型
        .id_type = BSP_CAN_ID_STANDARD,
        .frame_type = BSP_CAN_FRAME_DATA,
        .data_length = 8U,
    };

    // 帧格式：字节0=序列号，字节1=标志，字节2~7=负载（6字节）
    frame.data[0] = sequence;
    frame.data[1] = flags;
    (void)memcpy(&frame.data[2], payload, 6U);
    return (bsp_can_transmit(me->can, &frame, me->transmit_timeout_ms) == BSP_STATUS_OK)
               ? MODULE_ROBOT_LINK_STATUS_OK
               : MODULE_ROBOT_LINK_STATUS_TRANSPORT_ERROR;
}

/**
 * @brief 安全累加时间（防溢出）
 * @param elapsed_time_ms 累加计数器指针
 * @param increment_ms 要累加的时间
 */
static void module_robot_link_increment_elapsed(uint32_t *elapsed_time_ms, uint32_t increment_ms)
{
    if (*elapsed_time_ms > (UINT32_MAX - increment_ms))
    {
        *elapsed_time_ms = UINT32_MAX; // 饱和
    }
    else
    {
        *elapsed_time_ms += increment_ms;
    }
}

/* ======================== 公共 API ======================== */

/**
 * @brief 初始化 Robot Link 模块
 * @param me Robot Link 对象
 * @param config 配置参数
 * @return 执行状态
 */
module_robot_link_status_t module_robot_link_init(module_robot_link_t *me,
                                                  const module_robot_link_config_t *config)
{
    // 参数校验：对象、配置、CAN 基类（已初始化）、基址不能超出 CAN ID 范围
    if ((me == NULL) || (config == NULL) || (config->can == NULL) ||
        !bsp_device_is_initialized(&config->can->super) ||
        (config->base_identifier > (0x7FFU - (uint32_t)MODULE_ROBOT_LINK_MESSAGE_COUNT)))
    {
        return MODULE_ROBOT_LINK_STATUS_INVALID_ARGUMENT;
    }
    (void)memset(me, 0, sizeof(*me)); // 清零对象
    me->can = config->can;
    me->base_identifier = config->base_identifier;
    me->transmit_timeout_ms = config->transmit_timeout_ms;
    me->offline_timeout_ms = config->offline_timeout_ms;
    me->is_initialized = true;
    return MODULE_ROBOT_LINK_STATUS_OK;
}

/**
 * @brief 发送遥控器数据（三帧分片）
 * @param me Robot Link 对象
 * @param remote_data 遥控器数据
 * @return 执行状态
 * @note 分三帧发送：主通道、辅助通道、按键/鼠标
 *       三帧使用相同的序列号，接收端组装
 */
module_robot_link_status_t module_robot_link_send_remote(module_robot_link_t *me,
                                                         const module_dr16_data_t *remote_data)
{
    uint8_t payload[6];
    uint8_t flags;
    uint8_t sequence;
    module_robot_link_status_t status;

    // 参数校验
    if ((me == NULL) || (remote_data == NULL))
    {
        return MODULE_ROBOT_LINK_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return MODULE_ROBOT_LINK_STATUS_NOT_INITIALIZED;
    }

    sequence = me->transmit_sequence++; // 获取并递增序列号

    /* -------- 第一帧：主通道（channel 0~2） -------- */
    module_robot_link_encode_int16(remote_data->channel[0], &payload[0]);
    module_robot_link_encode_int16(remote_data->channel[1], &payload[2]);
    module_robot_link_encode_int16(remote_data->channel[2], &payload[4]);
    status = module_robot_link_transmit(me, MODULE_ROBOT_LINK_MESSAGE_REMOTE_CHANNELS_PRIMARY,
                                        payload, 0U, sequence);
    if (status != MODULE_ROBOT_LINK_STATUS_OK)
    {
        return status;
    }

    /* -------- 第二帧：辅助通道（channel 3 + dial + keyboard） -------- */
    module_robot_link_encode_int16(remote_data->channel[3], &payload[0]);
    module_robot_link_encode_int16(remote_data->dial, &payload[2]);
    module_robot_link_encode_int16((int16_t)remote_data->keyboard, &payload[4]);
    status = module_robot_link_transmit(me, MODULE_ROBOT_LINK_MESSAGE_REMOTE_CHANNELS_AUXILIARY,
                                        payload, 0U, sequence);
    if (status != MODULE_ROBOT_LINK_STATUS_OK)
    {
        return status;
    }

    /* -------- 第三帧：输入事件（鼠标 + 开关/按键状态） -------- */
    module_robot_link_encode_int16(remote_data->mouse_x, &payload[0]);
    module_robot_link_encode_int16(remote_data->mouse_y, &payload[2]);
    module_robot_link_encode_int16(remote_data->mouse_z, &payload[4]);
    // 标志位：左开关(bit0-1)、右开关(bit2-3)、鼠标左键(bit4)、鼠标右键(bit5)、在线状态(bit7)
    flags =
        (uint8_t)((uint8_t)remote_data->left_switch | ((uint8_t)remote_data->right_switch << 2U) |
                  (remote_data->mouse_left_pressed ? (1U << 4U) : 0U) |
                  (remote_data->mouse_right_pressed ? (1U << 5U) : 0U) |
                  (remote_data->is_online ? (1U << 7U) : 0U));
    return module_robot_link_transmit(me, MODULE_ROBOT_LINK_MESSAGE_REMOTE_INPUT, payload, flags,
                                      sequence);
}

/**
 * @brief 发送云台数据（两帧分片）
 * @param me Robot Link 对象
 * @param gimbal_data 云台数据
 * @return 执行状态
 * @note 分两帧发送：主数据（角度+角速度）、辅助数据
 */
module_robot_link_status_t
module_robot_link_send_gimbal(module_robot_link_t *me,
                              const module_robot_link_gimbal_data_t *gimbal_data)
{
    uint8_t payload[6];
    uint8_t flags;
    module_robot_link_status_t status;
    uint8_t sequence;

    // 参数校验：指针非空，所有浮点值有限
    if ((me == NULL) || (gimbal_data == NULL) || !isfinite(gimbal_data->yaw_rad) ||
        !isfinite(gimbal_data->pitch_rad) || !isfinite(gimbal_data->yaw_velocity_rad_per_s) ||
        !isfinite(gimbal_data->pitch_velocity_rad_per_s))
    {
        return MODULE_ROBOT_LINK_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return MODULE_ROBOT_LINK_STATUS_NOT_INITIALIZED;
    }

    sequence = me->transmit_sequence++;

    /* -------- 第一帧：主数据（偏航角、俯仰角、偏航角速度） -------- */
    module_robot_link_encode_int16(module_robot_link_encode_scaled(gimbal_data->yaw_rad),
                                   &payload[0]);
    module_robot_link_encode_int16(module_robot_link_encode_scaled(gimbal_data->pitch_rad),
                                   &payload[2]);
    module_robot_link_encode_int16(
        module_robot_link_encode_scaled(gimbal_data->yaw_velocity_rad_per_s), &payload[4]);
    // 标志位：IMU 有效(bit0)、电机在线(bit1)
    flags = (gimbal_data->imu_valid ? 1U : 0U) | (gimbal_data->motors_online ? 2U : 0U);
    status = module_robot_link_transmit(me, MODULE_ROBOT_LINK_MESSAGE_GIMBAL_PRIMARY, payload,
                                        flags, sequence);
    if (status != MODULE_ROBOT_LINK_STATUS_OK)
    {
        return status;
    }

    /* -------- 第二帧：辅助数据（俯仰角速度） -------- */
    (void)memset(payload, 0, sizeof(payload));
    module_robot_link_encode_int16(
        module_robot_link_encode_scaled(gimbal_data->pitch_velocity_rad_per_s), &payload[0]);
    return module_robot_link_transmit(me, MODULE_ROBOT_LINK_MESSAGE_GIMBAL_AUXILIARY, payload,
                                      flags, sequence);
}

/**
 * @brief 发送底盘数据（单帧）
 * @param me Robot Link 对象
 * @param chassis_data 底盘数据
 * @return 执行状态
 */
module_robot_link_status_t
module_robot_link_send_chassis(module_robot_link_t *me,
                               const module_robot_link_chassis_data_t *chassis_data)
{
    uint8_t payload[6];
    uint8_t flags;
    uint8_t sequence;

    // 参数校验
    if ((me == NULL) || (chassis_data == NULL) || !isfinite(chassis_data->velocity_x_m_per_s) ||
        !isfinite(chassis_data->velocity_y_m_per_s) ||
        !isfinite(chassis_data->angular_velocity_rad_per_s))
    {
        return MODULE_ROBOT_LINK_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return MODULE_ROBOT_LINK_STATUS_NOT_INITIALIZED;
    }

    sequence = me->transmit_sequence++;
    // 编码：X 速度、Y 速度、角速度
    module_robot_link_encode_int16(
        module_robot_link_encode_scaled(chassis_data->velocity_x_m_per_s), &payload[0]);
    module_robot_link_encode_int16(
        module_robot_link_encode_scaled(chassis_data->velocity_y_m_per_s), &payload[2]);
    module_robot_link_encode_int16(
        module_robot_link_encode_scaled(chassis_data->angular_velocity_rad_per_s), &payload[4]);
    // 标志位：电机在线(bit0)、自锁激活(bit1)
    flags = (chassis_data->motors_online ? 1U : 0U) | (chassis_data->self_lock_active ? 2U : 0U);
    return module_robot_link_transmit(me, MODULE_ROBOT_LINK_MESSAGE_CHASSIS, payload, flags,
                                      sequence);
}

/**
 * @brief 发送发射机构数据（单帧）
 * @param me Robot Link 对象
 * @param shooter_data 发射机构数据
 * @return 执行状态
 */
module_robot_link_status_t
module_robot_link_send_shooter(module_robot_link_t *me,
                               const module_robot_link_shooter_data_t *shooter_data)
{
    uint8_t payload[6] = {0U};
    uint8_t sequence;

    // 参数校验
    if ((me == NULL) || (shooter_data == NULL) ||
        !isfinite(shooter_data->friction_velocity_rad_per_s) ||
        !isfinite(shooter_data->feeder_position_rad))
    {
        return MODULE_ROBOT_LINK_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return MODULE_ROBOT_LINK_STATUS_NOT_INITIALIZED;
    }

    sequence = me->transmit_sequence++;
    // 编码：摩擦轮速度、拨弹盘位置
    module_robot_link_encode_int16(
        module_robot_link_encode_scaled(shooter_data->friction_velocity_rad_per_s), &payload[0]);
    module_robot_link_encode_int16(
        module_robot_link_encode_scaled(shooter_data->feeder_position_rad), &payload[2]);
    // 状态和卡弹重试次数直接作为字节传输（不缩放）
    payload[4] = shooter_data->state;
    payload[5] = shooter_data->jam_retry_count;
    return module_robot_link_transmit(me, MODULE_ROBOT_LINK_MESSAGE_SHOOTER, payload, 0U, sequence);
}

/**
 * @brief 发送心跳帧
 * @param me Robot Link 对象
 * @param board_role 板卡角色（如 0=云台板，1=底盘板）
 * @param uptime_ms 运行时间（毫秒）
 * @return 执行状态
 */
module_robot_link_status_t module_robot_link_send_heartbeat(module_robot_link_t *me,
                                                            uint8_t board_role, uint32_t uptime_ms)
{
    uint8_t payload[6] = {
        board_role,
        0U,
        (uint8_t)uptime_ms,
        (uint8_t)(uptime_ms >> 8U),
        (uint8_t)(uptime_ms >> 16U),
        (uint8_t)(uptime_ms >> 24U),
    };
    uint8_t sequence;

    if (me == NULL)
    {
        return MODULE_ROBOT_LINK_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return MODULE_ROBOT_LINK_STATUS_NOT_INITIALIZED;
    }
    sequence = me->transmit_sequence++;
    return module_robot_link_transmit(me, MODULE_ROBOT_LINK_MESSAGE_HEARTBEAT, payload, 0U,
                                      sequence);
}

/**
 * @brief 处理接收到的 CAN 帧
 * @param me Robot Link 对象
 * @param frame CAN 帧
 * @return 执行状态
 * @note 路由到对应消息类型，执行分片组装和提交
 */
module_robot_link_status_t module_robot_link_handle_frame(module_robot_link_t *me,
                                                          const bsp_can_frame_t *frame)
{
    module_robot_link_message_t message;
    const uint8_t *payload;
    uint8_t sequence;

    // 参数校验
    if ((me == NULL) || (frame == NULL))
    {
        return MODULE_ROBOT_LINK_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return MODULE_ROBOT_LINK_STATUS_NOT_INITIALIZED;
    }
    // 帧校验：标准帧、数据帧、8字节、ID 在有效范围内
    if ((frame->id_type != BSP_CAN_ID_STANDARD) || (frame->frame_type != BSP_CAN_FRAME_DATA) ||
        (frame->data_length != 8U) || (frame->identifier < me->base_identifier) ||
        (frame->identifier >= (me->base_identifier + (uint32_t)MODULE_ROBOT_LINK_MESSAGE_COUNT)))
    {
        return MODULE_ROBOT_LINK_STATUS_INVALID_FRAME;
    }

    message = (module_robot_link_message_t)(frame->identifier - me->base_identifier);
    payload = &frame->data[2]; // 负载从第 3 字节开始（前 2 字节为序列号和标志）
    sequence = frame->data[0];

    // ---- 根据消息类型处理 ----
    switch (message)
    {
    /* 遥控器主通道：channel 0~2 */
    case MODULE_ROBOT_LINK_MESSAGE_REMOTE_CHANNELS_PRIMARY:
        module_robot_link_prepare_remote_transaction(me, sequence);
        me->remote_staging.channel[0] = module_robot_link_decode_int16(&payload[0]);
        me->remote_staging.channel[1] = module_robot_link_decode_int16(&payload[2]);
        me->remote_staging.channel[2] = module_robot_link_decode_int16(&payload[4]);
        me->remote_receive_mask |= 1U; // 标记已收到
        break;

    /* 遥控器辅助通道：channel 3 + dial + keyboard */
    case MODULE_ROBOT_LINK_MESSAGE_REMOTE_CHANNELS_AUXILIARY:
        module_robot_link_prepare_remote_transaction(me, sequence);
        me->remote_staging.channel[3] = module_robot_link_decode_int16(&payload[0]);
        me->remote_staging.dial = module_robot_link_decode_int16(&payload[2]);
        me->remote_staging.keyboard = (uint16_t)module_robot_link_decode_int16(&payload[4]);
        me->remote_receive_mask |= 2U;
        break;

    /* 遥控器输入事件：鼠标 + 开关/按键 */
    case MODULE_ROBOT_LINK_MESSAGE_REMOTE_INPUT:
        module_robot_link_prepare_remote_transaction(me, sequence);
        me->remote_staging.mouse_x = module_robot_link_decode_int16(&payload[0]);
        me->remote_staging.mouse_y = module_robot_link_decode_int16(&payload[2]);
        me->remote_staging.mouse_z = module_robot_link_decode_int16(&payload[4]);
        // 从标志字节提取开关和按键状态
        me->remote_staging.left_switch = (module_dr16_switch_t)(frame->data[1] & 0x03U);
        me->remote_staging.right_switch = (module_dr16_switch_t)((frame->data[1] >> 2U) & 0x03U);
        me->remote_staging.mouse_left_pressed = (frame->data[1] & (1U << 4U)) != 0U;
        me->remote_staging.mouse_right_pressed = (frame->data[1] & (1U << 5U)) != 0U;
        me->remote_staging.is_online = (frame->data[1] & (1U << 7U)) != 0U;
        me->remote_receive_mask |= 4U;
        break;

    /* 云台主数据：角度 + 偏航角速度 */
    case MODULE_ROBOT_LINK_MESSAGE_GIMBAL_PRIMARY:
        module_robot_link_prepare_gimbal_transaction(me, sequence);
        me->gimbal_staging.yaw_rad = module_robot_link_decode_scaled(&payload[0]);
        me->gimbal_staging.pitch_rad = module_robot_link_decode_scaled(&payload[2]);
        me->gimbal_staging.yaw_velocity_rad_per_s = module_robot_link_decode_scaled(&payload[4]);
        me->gimbal_staging.imu_valid = (frame->data[1] & 1U) != 0U;
        me->gimbal_staging.motors_online = (frame->data[1] & 2U) != 0U;
        me->gimbal_receive_mask |= 1U;
        break;

    /* 云台辅助数据：俯仰角速度 */
    case MODULE_ROBOT_LINK_MESSAGE_GIMBAL_AUXILIARY:
        module_robot_link_prepare_gimbal_transaction(me, sequence);
        me->gimbal_staging.pitch_velocity_rad_per_s = module_robot_link_decode_scaled(&payload[0]);
        me->gimbal_receive_mask |= 2U;
        break;

    /* 底盘数据（单帧，直接提交） */
    case MODULE_ROBOT_LINK_MESSAGE_CHASSIS:
        me->chassis_data.velocity_x_m_per_s = module_robot_link_decode_scaled(&payload[0]);
        me->chassis_data.velocity_y_m_per_s = module_robot_link_decode_scaled(&payload[2]);
        me->chassis_data.angular_velocity_rad_per_s = module_robot_link_decode_scaled(&payload[4]);
        me->chassis_data.motors_online = (frame->data[1] & 1U) != 0U;
        me->chassis_data.self_lock_active = (frame->data[1] & 2U) != 0U;
        me->chassis_elapsed_time_ms = 0U;
        me->chassis_online = true;
        break;

    /* 发射机构数据（单帧，直接提交） */
    case MODULE_ROBOT_LINK_MESSAGE_SHOOTER:
        me->shooter_data.friction_velocity_rad_per_s = module_robot_link_decode_scaled(&payload[0]);
        me->shooter_data.feeder_position_rad = module_robot_link_decode_scaled(&payload[2]);
        me->shooter_data.state = payload[4];
        me->shooter_data.jam_retry_count = payload[5];
        me->shooter_elapsed_time_ms = 0U;
        me->shooter_online = true;
        break;

    /* 心跳帧 - 暂不处理 */
    case MODULE_ROBOT_LINK_MESSAGE_HEARTBEAT:
    case MODULE_ROBOT_LINK_MESSAGE_COUNT:
    default:
        break;
    }

    /* -------- 遥控器分片组装：收到全部 3 个分片后提交 -------- */
    if (me->remote_receive_mask == 7U) // 二进制 111
    {
        size_t channel_index;
        // 计算归一化通道值
        for (channel_index = 0U; channel_index < MODULE_DR16_CHANNEL_COUNT; ++channel_index)
        {
            me->remote_staging.normalized_channel[channel_index] =
                module_dr16_normalize_channel_value(me->remote_staging.channel[channel_index]);
        }
        me->remote_staging.normalized_dial =
            module_dr16_normalize_channel_value(me->remote_staging.dial);
        // 递增有效帧计数
        if (me->remote_staging.valid_frame_count != UINT32_MAX)
        {
            ++me->remote_staging.valid_frame_count;
        }
        me->remote_data = me->remote_staging; // 原子提交
        me->remote_elapsed_time_ms = 0U;
        me->remote_online = me->remote_data.is_online;
        me->remote_receive_mask = 0U; // 清空掩码，准备下次接收
    }

    /* -------- 云台分片组装：收到全部 2 个分片后提交 -------- */
    if (me->gimbal_receive_mask == 3U) // 二进制 11
    {
        me->gimbal_data = me->gimbal_staging; // 原子提交
        me->gimbal_elapsed_time_ms = 0U;
        me->gimbal_online = true;
        me->gimbal_receive_mask = 0U;
    }

    return MODULE_ROBOT_LINK_STATUS_OK;
}

/* ======================== 数据获取接口 ======================== */

/**
 * @brief 获取遥控器数据（只读）
 * @param me Robot Link 对象
 * @return 遥控器数据指针，若离线或未初始化则返回 NULL
 */
const module_dr16_data_t *module_robot_link_get_remote(const module_robot_link_t *me)
{
    return ((me != NULL) && me->is_initialized && me->remote_online) ? &me->remote_data : NULL;
}

/**
 * @brief 获取云台数据（只读）
 * @param me Robot Link 对象
 * @return 云台数据指针，若离线或未初始化则返回 NULL
 */
const module_robot_link_gimbal_data_t *module_robot_link_get_gimbal(const module_robot_link_t *me)
{
    return ((me != NULL) && me->is_initialized && me->gimbal_online) ? &me->gimbal_data : NULL;
}

/**
 * @brief 获取底盘数据（只读）
 * @param me Robot Link 对象
 * @return 底盘数据指针，若离线或未初始化则返回 NULL
 */
const module_robot_link_chassis_data_t *module_robot_link_get_chassis(const module_robot_link_t *me)
{
    return ((me != NULL) && me->is_initialized && me->chassis_online) ? &me->chassis_data : NULL;
}

/**
 * @brief 获取发射机构数据（只读）
 * @param me Robot Link 对象
 * @return 发射机构数据指针，若离线或未初始化则返回 NULL
 */
const module_robot_link_shooter_data_t *module_robot_link_get_shooter(const module_robot_link_t *me)
{
    return ((me != NULL) && me->is_initialized && me->shooter_online) ? &me->shooter_data : NULL;
}

/**
 * @brief 更新各数据组的在线超时计时
 * @param me Robot Link 对象
 * @param elapsed_time_ms 距上次更新的时间（毫秒）
 * @note 应由一个周期任务调用，且只能有一个时间所有者
 */
void module_robot_link_update_time(module_robot_link_t *me, uint32_t elapsed_time_ms)
{
    if ((me == NULL) || !me->is_initialized)
    {
        return;
    }
    // 累加各组超时计数
    module_robot_link_increment_elapsed(&me->remote_elapsed_time_ms, elapsed_time_ms);
    module_robot_link_increment_elapsed(&me->gimbal_elapsed_time_ms, elapsed_time_ms);
    module_robot_link_increment_elapsed(&me->chassis_elapsed_time_ms, elapsed_time_ms);
    module_robot_link_increment_elapsed(&me->shooter_elapsed_time_ms, elapsed_time_ms);

    // 超时检测：超过 offline_timeout_ms 则置离线
    if (me->offline_timeout_ms > 0U)
    {
        me->remote_online =
            me->remote_online && (me->remote_elapsed_time_ms <= me->offline_timeout_ms);
        me->gimbal_online =
            me->gimbal_online && (me->gimbal_elapsed_time_ms <= me->offline_timeout_ms);
        me->chassis_online =
            me->chassis_online && (me->chassis_elapsed_time_ms <= me->offline_timeout_ms);
        me->shooter_online =
            me->shooter_online && (me->shooter_elapsed_time_ms <= me->offline_timeout_ms);
    }
}