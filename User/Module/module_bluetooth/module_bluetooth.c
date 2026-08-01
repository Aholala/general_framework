/**
 * @file module_bluetooth.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 通用蓝牙串口模块实现
 * @version 1.0
 * @date 2026-07-28
 * @copyright Copyright (c) 2026
 *
 * @note 基于 bsp_usart_t 的通用蓝牙串口模块，负责异步接收、双缓冲转交、
 *       在线超时、原始数据发送和 AT 命令发送。不绑定特定蓝牙芯片协议。
 */

#include "module_bluetooth.h"

#include <stddef.h> // NULL, size_t
#include <string.h> // memcpy, strlen

MODULE_STATIC_ASSERT_SUPER_FIRST(module_bluetooth_t);

/**
 * @brief USART 中断回调（ISR 上下文）
 *        将接收数据拷贝到 processing_buffer 并置位标志，立即重启 DMA
 * @param event BSP 事件类型
 * @param status 传输状态
 * @param transferred_size 传输大小
 * @param user_context 用户上下文（module_bluetooth_t 对象）
 * @note ISR 中仅拷贝数据，不解析协议，不执行用户回调
 */
static void module_bluetooth_usart_callback(bsp_event_t event, bsp_status_t status,
                                            size_t transferred_size, void *user_context)
{
    module_bluetooth_t *const me = (module_bluetooth_t *)user_context;

    if (me == NULL)
    {
        return;
    }

    // 接收完成或待接收事件（空闲中断）
    if ((event == BSP_EVENT_RECEIVE_COMPLETE) || (event == BSP_EVENT_RECEIVE_PENDING))
    {
        // 检查状态和数据有效性
        if ((status == BSP_STATUS_OK) && (transferred_size > 0U) &&
            (transferred_size <= me->receive_capacity) &&
            (transferred_size <= me->processing_capacity))
        {
            // 如果上次数据已被处理，拷贝新数据到 pending_buffer
            if (!me->is_receive_pending)
            {
                (void)memcpy(me->processing_buffer, me->receive_buffer, transferred_size);
                me->pending_receive_size = transferred_size;
                me->is_receive_pending = true;
            }
            else
            {
                // 上次数据还未处理，发生覆盖，递增覆盖计数
                if (me->receive_overrun_count != UINT32_MAX)
                {
                    ++me->receive_overrun_count;
                }
            }
        }
        else
        {
            // 数据无效或传输错误，也计入覆盖错误（可视为接收失败）
            if (me->receive_overrun_count != UINT32_MAX)
            {
                ++me->receive_overrun_count;
            }
        }

        // 立即重启 DMA 空闲接收（若模块已启动）
        if (me->is_started &&
            (bsp_usart_receive_to_idle(me->usart, me->receive_buffer, me->receive_capacity,
                                       me->receive_mode, me->receive_timeout_ms) != BSP_STATUS_OK))
        {
            if (me->receive_restart_error_count != UINT32_MAX)
            {
                ++me->receive_restart_error_count;
            }
        }
    }
}

/**
 * @brief 设备启动回调（转发至 module_bluetooth_start）
 */
static module_device_status_t module_bluetooth_device_start(module_device_t *const device_base)
{
    module_bluetooth_t *const me = MODULE_CONTAINER_OF(device_base, module_bluetooth_t, super);
    return (module_bluetooth_start(me) == MODULE_BLUETOOTH_STATUS_OK)
               ? MODULE_DEVICE_STATUS_OK
               : MODULE_DEVICE_STATUS_OPERATION_FAILED;
}

/**
 * @brief 设备停止回调（转发至 module_bluetooth_stop）
 */
static module_device_status_t module_bluetooth_device_stop(module_device_t *const device_base)
{
    module_bluetooth_t *const me = MODULE_CONTAINER_OF(device_base, module_bluetooth_t, super);
    return (module_bluetooth_stop(me) == MODULE_BLUETOOTH_STATUS_OK)
               ? MODULE_DEVICE_STATUS_OK
               : MODULE_DEVICE_STATUS_OPERATION_FAILED;
}

/**
 * @brief 设备更新回调（转发至 module_bluetooth_update）
 */
static module_device_status_t module_bluetooth_device_update(module_device_t *const device_base,
                                                             uint32_t elapsed_time_ms)
{
    module_bluetooth_t *const me = MODULE_CONTAINER_OF(device_base, module_bluetooth_t, super);
    return (module_bluetooth_update(me, elapsed_time_ms) == MODULE_BLUETOOTH_STATUS_OK)
               ? MODULE_DEVICE_STATUS_OK
               : MODULE_DEVICE_STATUS_OPERATION_FAILED;
}

/** 蓝牙模块的设备操作表 */
static const module_device_ops_t s_module_bluetooth_ops = {
    .start = module_bluetooth_device_start,
    .stop = module_bluetooth_device_stop,
    .update = module_bluetooth_device_update,
};

/**
 * @brief 初始化蓝牙模块
 *        校验参数、保存配置、执行两阶段构造
 * @param me 蓝牙对象
 * @param config 配置参数
 * @return 执行状态
 */
module_bluetooth_status_t module_bluetooth_init(module_bluetooth_t *me,
                                                const module_bluetooth_config_t *config)
{
    // 参数校验：对象、配置、USART、缓冲区、容量、接收模式（不能是阻塞）、离线超时等
    if ((me == NULL) || (config == NULL) || (config->usart == NULL) ||
        !bsp_device_is_initialized(&config->usart->super) || (config->receive_buffer == NULL) ||
        (config->receive_capacity == 0U) || (config->processing_buffer == NULL) ||
        (config->processing_capacity < config->receive_capacity) ||
        !bsp_transfer_mode_is_valid(config->receive_mode) ||
        (config->receive_mode == BSP_TRANSFER_MODE_BLOCKING) || (config->offline_timeout_ms == 0U))
    {
        return MODULE_BLUETOOTH_STATUS_INVALID_ARGUMENT;
    }

    // 清零对象
    *me = (module_bluetooth_t){0};

    // 复制配置到对象
    me->usart = config->usart;
    me->receive_buffer = config->receive_buffer;
    me->receive_capacity = config->receive_capacity;
    me->processing_buffer = config->processing_buffer;
    me->processing_capacity = config->processing_capacity;
    me->transmit_timeout_ms = config->transmit_timeout_ms;
    me->receive_timeout_ms = config->receive_timeout_ms;
    me->offline_timeout_ms = config->offline_timeout_ms;
    me->receive_mode = config->receive_mode;
    me->receive_callback = config->receive_callback;
    me->user_context = config->user_context;

    // 初始化设备基类（两阶段构造）
    if (module_device_init_base(&me->super, &s_module_bluetooth_ops, config->logical_name,
                                config->registration_key) != MODULE_DEVICE_STATUS_OK)
    {
        return MODULE_BLUETOOTH_STATUS_INVALID_ARGUMENT;
    }
    if (module_device_complete_init(&me->super) != MODULE_DEVICE_STATUS_OK)
    {
        module_device_abort_init(&me->super);
        return MODULE_BLUETOOTH_STATUS_INVALID_ARGUMENT;
    }
    return MODULE_BLUETOOTH_STATUS_OK;
}

/**
 * @brief 启动蓝牙模块（注册回调并启动空闲接收）
 * @param me 蓝牙对象
 * @return 执行状态
 */
module_bluetooth_status_t module_bluetooth_start(module_bluetooth_t *me)
{
    if ((me == NULL) || !module_device_is_initialized(&me->super))
    {
        return MODULE_BLUETOOTH_STATUS_NOT_INITIALIZED;
    }

    // 注册 USART 回调
    if (bsp_usart_set_callback(me->usart, module_bluetooth_usart_callback, me) != BSP_STATUS_OK)
    {
        return MODULE_BLUETOOTH_STATUS_TRANSPORT_ERROR;
    }

    // 启动 DMA 空闲接收（异步模式）
    if (bsp_usart_receive_to_idle(me->usart, me->receive_buffer, me->receive_capacity,
                                  me->receive_mode, me->receive_timeout_ms) != BSP_STATUS_OK)
    {
        (void)bsp_usart_set_callback(me->usart, NULL, NULL); // 回滚
        return MODULE_BLUETOOTH_STATUS_TRANSPORT_ERROR;
    }

    // 重置状态
    me->receive_elapsed_time_ms = 0U;
    me->is_online = false;
    me->is_receive_pending = false;
    me->pending_receive_size = 0U;
    me->is_started = true;
    return MODULE_BLUETOOTH_STATUS_OK;
}

/**
 * @brief 停止蓝牙模块（中止接收、注销回调）
 * @param me 蓝牙对象
 * @return 执行状态
 */
module_bluetooth_status_t module_bluetooth_stop(module_bluetooth_t *me)
{
    if ((me == NULL) || !module_device_is_initialized(&me->super))
    {
        return MODULE_BLUETOOTH_STATUS_NOT_INITIALIZED;
    }

    // 中止 USART 传输
    (void)bsp_usart_abort(me->usart);

    // 注销回调
    if (bsp_usart_set_callback(me->usart, NULL, NULL) != BSP_STATUS_OK)
    {
        return MODULE_BLUETOOTH_STATUS_TRANSPORT_ERROR;
    }

    // 重置状态
    me->is_started = false;
    me->is_online = false;
    me->is_receive_pending = false;
    me->pending_receive_size = 0U;
    return MODULE_BLUETOOTH_STATUS_OK;
}

/**
 * @brief 发送原始二进制数据
 * @param me 蓝牙对象
 * @param transmit_data 发送数据指针
 * @param data_size 数据大小（字节）
 * @param transfer_mode 传输模式（BLOCKING/INTERRUPT/DMA）
 * @return 执行状态
 */
module_bluetooth_status_t module_bluetooth_transmit(module_bluetooth_t *me,
                                                    const uint8_t *transmit_data, size_t data_size,
                                                    bsp_transfer_mode_t transfer_mode)
{
    if ((me == NULL) || (transmit_data == NULL) || (data_size == 0U))
    {
        return MODULE_BLUETOOTH_STATUS_INVALID_ARGUMENT;
    }
    if (!module_device_is_initialized(&me->super))
    {
        return MODULE_BLUETOOTH_STATUS_NOT_INITIALIZED;
    }
    if (!me->is_started)
    {
        return MODULE_BLUETOOTH_STATUS_NOT_STARTED;
    }

    // 调用 BSP USART 发送
    return (bsp_usart_transmit(me->usart, transmit_data, data_size, transfer_mode,
                               me->transmit_timeout_ms) == BSP_STATUS_OK)
               ? MODULE_BLUETOOTH_STATUS_OK
               : MODULE_BLUETOOTH_STATUS_TRANSPORT_ERROR;
}

/**
 * @brief 发送以 NULL 结尾的 AT 命令字符串（阻塞发送）
 * @param me 蓝牙对象
 * @param command 命令字符串（以 '\0' 结尾）
 * @return 执行状态
 */
module_bluetooth_status_t module_bluetooth_send_command(module_bluetooth_t *me, const char *command)
{
    if (command == NULL)
    {
        return MODULE_BLUETOOTH_STATUS_INVALID_ARGUMENT;
    }
    // 发送整个字符串（不包含结束符），使用阻塞模式以确保发送完成
    return module_bluetooth_transmit(me, (const uint8_t *)command, strlen(command),
                                     BSP_TRANSFER_MODE_BLOCKING);
}

/**
 * @brief 周期更新（处理待接收数据 + 更新在线超时）
 * @param me 蓝牙对象
 * @param elapsed_time_ms 距上次更新的时间 (ms)
 * @return 执行状态
 */
module_bluetooth_status_t module_bluetooth_update(module_bluetooth_t *me, uint32_t elapsed_time_ms)
{
    if ((me == NULL) || !module_device_is_initialized(&me->super))
    {
        return MODULE_BLUETOOTH_STATUS_NOT_INITIALIZED;
    }
    if (!me->is_started)
    {
        return MODULE_BLUETOOTH_STATUS_NOT_STARTED;
    }

    // 处理待接收数据
    if (me->is_receive_pending)
    {
        const size_t received_size = me->pending_receive_size;
        me->receive_elapsed_time_ms = 0U; // 收到数据重置超时计时
        me->is_online = true;             // 标记在线

        // 调用用户回调（若有）
        if ((me->receive_callback != NULL) && (received_size > 0U))
        {
            me->receive_callback(me->processing_buffer, received_size, me->user_context);
        }

        // 清除待处理标志
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

    return MODULE_BLUETOOTH_STATUS_OK;
}

/**
 * @brief 检查蓝牙模块是否在线
 * @param me 蓝牙对象
 * @return true=在线（已启动且最近收到数据）
 */
bool module_bluetooth_is_online(const module_bluetooth_t *me)
{
    return (me != NULL) && module_device_is_initialized(&me->super) && me->is_started &&
           me->is_online;
}
