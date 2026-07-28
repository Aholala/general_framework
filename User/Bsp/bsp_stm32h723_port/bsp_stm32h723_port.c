/**
 * @file bsp_stm32h723_port.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief STM32H723 芯片 BSP 端口实现
 * @version 1.0
 * @date 2026-07-27
 * @copyright Copyright (c) 2026
 *
 * @note 这是通用 BSP 与 STM32H723 HAL 之间唯一的芯片适配层。
 *       包含 HAL 句柄映射、驱动操作表、静态板级对象和 HAL 回调路由。
 *       通用 BSP 头文件（bsp_can.h 等）不包含 HAL 头文件。
 */

#include "bsp_stm32h723_port.h"

#include "board_config.h" // 板级配置（时钟频率等）
#include "fdcan.h"        // FDCAN HAL 句柄
#include "main.h"         // 主头文件（包含所有 HAL 句柄）
#include "spi.h"          // SPI HAL 句柄
#include "tim.h"          // TIM HAL 句柄
#include "usart.h"        // USART HAL 句柄
#include "usb_device.h"   // USB 设备 HAL
#include "usbd_cdc_if.h"  // USB CDC 接口（虚拟串口）

#include <string.h> // 提供 memcpy

/** USB 接收缓冲区大小（512 字节） */
#define BSP_STM32H723_USB_RECEIVE_CAPACITY (512U)

/** 外部 USB 设备句柄（由 USB 协议栈定义） */
extern USBD_HandleTypeDef hUsbDeviceFS;

/**
 * @brief EXTI 上下文结构体
 * @note 存储引脚号和中断号，用于将 HAL GPIO 回调路由到具体 EXTI 对象
 */
typedef struct
{
    uint16_t pin;               // GPIO 引脚号
    IRQn_Type interrupt_number; // 中断向量号
} bsp_stm32h723_exti_context_t;

/**
 * @brief 看门狗上下文结构体
 * @note 存储超时时间和复位检测标志
 */
typedef struct
{
    uint32_t timeout_ms; // 看门狗超时时间（毫秒）
    bool reset_detected; // 是否检测到看门狗复位
} bsp_stm32h723_watchdog_context_t;

/**
 * @brief USB 虚拟串口上下文结构体
 * @note 存储接收缓冲区和状态，用于 USB CDC 接收数据缓存
 */
typedef struct
{
    uint8_t receive_buffer[BSP_STM32H723_USB_RECEIVE_CAPACITY]; // 接收缓冲区
    volatile size_t receive_size;                               // 已接收数据大小
    volatile bool receive_pending;                              // 是否有待接收数据
} bsp_stm32h723_usb_context_t;

/* ---------- 静态设备对象（所有外设实例） ---------- */
static bsp_can_device_t bsp_stm32h723_can_devices[BSP_STM32H723_CAN_COUNT];
static bsp_usart_device_t bsp_stm32h723_usart_devices[BSP_STM32H723_USART_COUNT];
static bsp_spi_device_t bsp_stm32h723_bmi088_spi_device;
static bsp_exti_device_t bsp_stm32h723_exti_devices[BSP_STM32H723_EXTI_COUNT];
static bsp_pwm_device_t bsp_stm32h723_pwm_devices[BSP_STM32H723_PWM_COUNT];
static bsp_usb_vcp_device_t bsp_stm32h723_usb_device;
static bsp_timebase_device_t bsp_stm32h723_timebase_device;
static bsp_watchdog_device_t bsp_stm32h723_watchdog_device;

/* ---------- 上下文对象 ---------- */
static bsp_stm32h723_exti_context_t bsp_stm32h723_exti_contexts[BSP_STM32H723_EXTI_COUNT];
static bsp_stm32h723_usb_context_t bsp_stm32h723_usb_context;
static bsp_stm32h723_watchdog_context_t bsp_stm32h723_watchdog_context;

/* ---------- 初始化状态标志 ---------- */
static bool bsp_stm32h723_initialized;          // 端口是否已初始化
static bool bsp_stm32h723_watchdog_initialized; // 看门狗是否已初始化

/**
 * @brief 将 HAL 状态码转换为 BSP 状态码
 * @param status HAL 状态码（HAL_StatusTypeDef）
 * @return BSP 状态码
 */
static bsp_status_t bsp_stm32h723_status(HAL_StatusTypeDef status)
{
    switch (status)
    {
    case HAL_OK:
        return BSP_STATUS_OK;
    case HAL_BUSY:
        return BSP_STATUS_BUSY;
    case HAL_TIMEOUT:
        return BSP_STATUS_TIMEOUT;
    default:
        return BSP_STATUS_IO_ERROR;
    }
}

/**
 * @brief 空操作 init（用于不需要初始化的驱动）
 * @param handle 设备句柄
 * @return 若 handle 非空则返回 OK
 */
static bsp_status_t bsp_stm32h723_noop_init(void *handle)
{
    return (handle != NULL) ? BSP_STATUS_OK : BSP_STATUS_INVALID_ARGUMENT;
}

/**
 * @brief 空操作 deinit（用于不需要反初始化的驱动）
 * @param handle 设备句柄
 * @return 若 handle 非空则返回 OK
 */
static bsp_status_t bsp_stm32h723_noop_deinit(void *handle)
{
    return (handle != NULL) ? BSP_STATUS_OK : BSP_STATUS_INVALID_ARGUMENT;
}

/* ---------- CAN (FDCAN) 驱动实现 ---------- */

/**
 * @brief 将字节长度转换为 FDCAN DLC 编码
 * @param data_length 数据长度（字节）
 * @return FDCAN DLC 编码值
 * @note 0~8 字节直接映射，其他返回 0（实际不会被调用）
 */
static uint32_t bsp_stm32h723_fdcan_data_length(uint8_t data_length)
{
    static const uint32_t lengths[] = {
        FDCAN_DLC_BYTES_0, FDCAN_DLC_BYTES_1, FDCAN_DLC_BYTES_2,
        FDCAN_DLC_BYTES_3, FDCAN_DLC_BYTES_4, FDCAN_DLC_BYTES_5,
        FDCAN_DLC_BYTES_6, FDCAN_DLC_BYTES_7, FDCAN_DLC_BYTES_8,
    };
    return (data_length <= 8U) ? lengths[data_length] : FDCAN_DLC_BYTES_0;
}

/**
 * @brief 将 FDCAN DLC 编码解码为字节长度
 * @param data_length FDCAN DLC 编码值
 * @return 字节长度
 */
static uint8_t bsp_stm32h723_fdcan_decode_length(uint32_t data_length)
{
    return (uint8_t)((data_length >> 16U) & 0x0FU);
}

/**
 * @brief 启动 FDCAN 外设
 * @param handle FDCAN_HandleTypeDef* 句柄
 * @return 执行状态
 */
static bsp_status_t bsp_stm32h723_can_start(void *handle)
{
    FDCAN_HandleTypeDef *const fdcan = (FDCAN_HandleTypeDef *)handle;
    HAL_StatusTypeDef status;
    // 配置全局过滤器：所有帧都接收（不拒绝）
    status = HAL_FDCAN_ConfigGlobalFilter(fdcan, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0,
                                          FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
    if (status == HAL_OK)
    {
        // 激活中断通知：FIFO0/1 新消息、错误警告、错误被动、Bus-Off
        status = HAL_FDCAN_ActivateNotification(
            fdcan,
            FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_RX_FIFO1_NEW_MESSAGE | FDCAN_IT_ERROR_WARNING |
                FDCAN_IT_ERROR_PASSIVE | FDCAN_IT_BUS_OFF,
            0U);
    }
    if (status == HAL_OK)
    {
        // 启动 FDCAN
        status = HAL_FDCAN_Start(fdcan);
    }
    return bsp_stm32h723_status(status);
}

/**
 * @brief 停止 FDCAN 外设
 * @param handle FDCAN_HandleTypeDef* 句柄
 * @return 执行状态
 */
static bsp_status_t bsp_stm32h723_can_stop(void *handle)
{
    return bsp_stm32h723_status(HAL_FDCAN_Stop((FDCAN_HandleTypeDef *)handle));
}

/**
 * @brief 配置 FDCAN 硬件过滤器
 * @param handle FDCAN_HandleTypeDef* 句柄
 * @param filter 过滤器配置
 * @return 执行状态
 */
static bsp_status_t bsp_stm32h723_can_filter(void *handle, const bsp_can_filter_t *filter)
{
    FDCAN_FilterTypeDef hal_filter = {0};
    if (filter == NULL)
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    // ID 类型：标准/扩展
    hal_filter.IdType =
        (filter->id_type == BSP_CAN_ID_STANDARD) ? FDCAN_STANDARD_ID : FDCAN_EXTENDED_ID;
    // 过滤器索引（由平台端分配）
    hal_filter.FilterIndex = filter->filter_index;
    // 过滤器类型：掩码模式
    hal_filter.FilterType = FDCAN_FILTER_MASK;
    // 匹配帧路由到 FIFO0 或 FIFO1
    hal_filter.FilterConfig = (filter->receive_fifo == BSP_CAN_RX_FIFO_0) ? FDCAN_FILTER_TO_RXFIFO0
                                                                          : FDCAN_FILTER_TO_RXFIFO1;
    hal_filter.FilterID1 = filter->identifier; // 匹配 ID
    hal_filter.FilterID2 = filter->mask;       // 掩码
    return bsp_stm32h723_status(HAL_FDCAN_ConfigFilter((FDCAN_HandleTypeDef *)handle, &hal_filter));
}

/**
 * @brief 发送 CAN 帧（阻塞，带超时）
 * @param handle FDCAN_HandleTypeDef* 句柄
 * @param frame CAN 帧指针
 * @param timeout_ms 超时时间
 * @return 执行状态
 */
static bsp_status_t bsp_stm32h723_can_transmit(void *handle, const bsp_can_frame_t *frame,
                                               uint32_t timeout_ms)
{
    FDCAN_TxHeaderTypeDef header = {0};
    uint32_t started_at_ms;
    // 参数校验：帧非空，数据长度 <= 8（Classic CAN）
    if ((frame == NULL) || (frame->data_length > 8U))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    // 填充发送头
    header.Identifier = frame->identifier;
    header.IdType = (frame->id_type == BSP_CAN_ID_STANDARD) ? FDCAN_STANDARD_ID : FDCAN_EXTENDED_ID;
    header.TxFrameType =
        (frame->frame_type == BSP_CAN_FRAME_DATA) ? FDCAN_DATA_FRAME : FDCAN_REMOTE_FRAME;
    header.DataLength = bsp_stm32h723_fdcan_data_length(frame->data_length);
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch = FDCAN_BRS_OFF; // Classic CAN 不使用 BRS
    header.FDFormat = FDCAN_CLASSIC_CAN;  // Classic CAN 格式
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    header.MessageMarker = 0U;
    // 等待发送 FIFO 有空闲（超时检测）
    started_at_ms = HAL_GetTick();
    while (HAL_FDCAN_GetTxFifoFreeLevel((FDCAN_HandleTypeDef *)handle) == 0U)
    {
        if ((HAL_GetTick() - started_at_ms) >= timeout_ms)
        {
            return BSP_STATUS_TIMEOUT;
        }
    }
    // 将消息加入发送 FIFO
    return bsp_stm32h723_status(HAL_FDCAN_AddMessageToTxFifoQ((FDCAN_HandleTypeDef *)handle,
                                                              &header, (uint8_t *)frame->data));
}

/**
 * @brief 接收 CAN 帧（从指定 FIFO）
 * @param handle FDCAN_HandleTypeDef* 句柄
 * @param receive_fifo FIFO 选择（0 或 1）
 * @param frame 输出 CAN 帧
 * @return 执行状态
 */
static bsp_status_t bsp_stm32h723_can_receive(void *handle, bsp_can_receive_fifo_t receive_fifo,
                                              bsp_can_frame_t *frame)
{
    FDCAN_RxHeaderTypeDef header = {0};
    const uint32_t fifo = (receive_fifo == BSP_CAN_RX_FIFO_0) ? FDCAN_RX_FIFO0 : FDCAN_RX_FIFO1;
    HAL_StatusTypeDef status;
    if (frame == NULL)
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    // 从指定 FIFO 读取消息
    status = HAL_FDCAN_GetRxMessage((FDCAN_HandleTypeDef *)handle, fifo, &header, frame->data);
    if (status == HAL_OK)
    {
        // 解析接收头并填充 frame
        frame->identifier = header.Identifier;
        frame->id_type =
            (header.IdType == FDCAN_STANDARD_ID) ? BSP_CAN_ID_STANDARD : BSP_CAN_ID_EXTENDED;
        frame->frame_type =
            (header.RxFrameType == FDCAN_DATA_FRAME) ? BSP_CAN_FRAME_DATA : BSP_CAN_FRAME_REMOTE;
        frame->data_length = bsp_stm32h723_fdcan_decode_length(header.DataLength);
    }
    return bsp_stm32h723_status(status);
}

/**
 * @brief 获取发送 FIFO 空闲数量
 * @param handle FDCAN_HandleTypeDef* 句柄
 * @param free_level 输出空闲数量
 * @return 执行状态
 */
static bsp_status_t bsp_stm32h723_can_free_level(const void *handle, uint32_t *free_level)
{
    if ((handle == NULL) || (free_level == NULL))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    *free_level = HAL_FDCAN_GetTxFifoFreeLevel((FDCAN_HandleTypeDef *)(uintptr_t)handle);
    return BSP_STATUS_OK;
}

/** CAN 驱动操作表（FDCAN） */
static const bsp_can_driver_ops_t bsp_stm32h723_can_driver_ops = {
    .init = bsp_stm32h723_noop_init,
    .deinit = bsp_stm32h723_noop_deinit,
    .start = bsp_stm32h723_can_start,
    .stop = bsp_stm32h723_can_stop,
    .configure_filter = bsp_stm32h723_can_filter,
    .transmit = bsp_stm32h723_can_transmit,
    .receive = bsp_stm32h723_can_receive,
    .get_tx_free_level = bsp_stm32h723_can_free_level,
};

/* ---------- USART (UART) 驱动实现 ---------- */

/**
 * @brief USART 发送（支持三种模式）
 * @param handle UART_HandleTypeDef* 句柄
 * @param data 发送数据指针
 * @param size 数据大小
 * @param mode 传输模式（阻塞/中断/DMA）
 * @param timeout_ms 超时时间（仅阻塞模式有效）
 * @return 执行状态
 */
static bsp_status_t bsp_stm32h723_usart_transmit(void *handle, const uint8_t *data, size_t size,
                                                 bsp_transfer_mode_t mode, uint32_t timeout_ms)
{
    // 参数校验：数据非空，大小在有效范围内
    if ((data == NULL) || (size == 0U) || (size > UINT16_MAX))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    if (mode == BSP_TRANSFER_MODE_BLOCKING)
    {
        return bsp_stm32h723_status(
            HAL_UART_Transmit((UART_HandleTypeDef *)handle, data, (uint16_t)size, timeout_ms));
    }
    if (mode == BSP_TRANSFER_MODE_INTERRUPT)
    {
        return bsp_stm32h723_status(
            HAL_UART_Transmit_IT((UART_HandleTypeDef *)handle, data, (uint16_t)size));
    }
    // DMA 模式
    return bsp_stm32h723_status(
        HAL_UART_Transmit_DMA((UART_HandleTypeDef *)handle, data, (uint16_t)size));
}

/**
 * @brief USART 接收（支持三种模式）
 * @param handle UART_HandleTypeDef* 句柄
 * @param data 接收缓冲区指针
 * @param size 数据大小
 * @param mode 传输模式（阻塞/中断/DMA）
 * @param timeout_ms 超时时间（仅阻塞模式有效）
 * @return 执行状态
 */
static bsp_status_t bsp_stm32h723_usart_receive(void *handle, uint8_t *data, size_t size,
                                                bsp_transfer_mode_t mode, uint32_t timeout_ms)
{
    if ((data == NULL) || (size == 0U) || (size > UINT16_MAX))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    if (mode == BSP_TRANSFER_MODE_BLOCKING)
    {
        return bsp_stm32h723_status(
            HAL_UART_Receive((UART_HandleTypeDef *)handle, data, (uint16_t)size, timeout_ms));
    }
    if (mode == BSP_TRANSFER_MODE_INTERRUPT)
    {
        return bsp_stm32h723_status(
            HAL_UART_Receive_IT((UART_HandleTypeDef *)handle, data, (uint16_t)size));
    }
    return bsp_stm32h723_status(
        HAL_UART_Receive_DMA((UART_HandleTypeDef *)handle, data, (uint16_t)size));
}

/**
 * @brief USART 接收直到空闲（支持三种模式）
 * @param handle UART_HandleTypeDef* 句柄
 * @param data 接收缓冲区指针
 * @param capacity 缓冲区容量
 * @param mode 传输模式
 * @param timeout_ms 超时时间
 * @return 执行状态
 */
static bsp_status_t bsp_stm32h723_usart_receive_to_idle(void *handle, uint8_t *data,
                                                        size_t capacity, bsp_transfer_mode_t mode,
                                                        uint32_t timeout_ms)
{
    if ((data == NULL) || (capacity == 0U) || (capacity > UINT16_MAX))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    if (mode == BSP_TRANSFER_MODE_BLOCKING)
    {
        uint16_t received_size = 0U;
        return bsp_stm32h723_status(HAL_UARTEx_ReceiveToIdle(
            (UART_HandleTypeDef *)handle, data, (uint16_t)capacity, &received_size, timeout_ms));
    }
    if (mode == BSP_TRANSFER_MODE_INTERRUPT)
    {
        return bsp_stm32h723_status(
            HAL_UARTEx_ReceiveToIdle_IT((UART_HandleTypeDef *)handle, data, (uint16_t)capacity));
    }
    return bsp_stm32h723_status(
        HAL_UARTEx_ReceiveToIdle_DMA((UART_HandleTypeDef *)handle, data, (uint16_t)capacity));
}

static bsp_status_t bsp_stm32h723_usart_receive_to_idle_double_buffer(
    void *handle, uint8_t *first_buffer, uint8_t *second_buffer, size_t buffer_capacity)
{
    UART_HandleTypeDef *const uart = (UART_HandleTypeDef *)handle;
    HAL_StatusTypeDef hal_status;

    if ((uart == NULL) || (uart->hdmarx == NULL) || (first_buffer == NULL) ||
        (second_buffer == NULL) || (first_buffer == second_buffer) || (buffer_capacity == 0U) ||
        (buffer_capacity > UINT16_MAX))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    if (uart->RxState != HAL_UART_STATE_READY)
    {
        return BSP_STATUS_BUSY;
    }

    uart->ReceptionType = HAL_UART_RECEPTION_TOIDLE;
    uart->RxEventType = HAL_UART_RXEVENT_IDLE;
    uart->pRxBuffPtr = first_buffer;
    uart->RxXferSize = (uint16_t)buffer_capacity;
    uart->RxXferCount = (uint16_t)buffer_capacity;
    uart->ErrorCode = HAL_UART_ERROR_NONE;
    uart->RxState = HAL_UART_STATE_BUSY_RX;

    __HAL_UART_CLEAR_FLAG(uart, UART_CLEAR_IDLEF);
    __HAL_UART_ENABLE_IT(uart, UART_IT_IDLE);
    hal_status = HAL_DMAEx_MultiBufferStart(
        uart->hdmarx, (uint32_t)(uintptr_t)&uart->Instance->RDR,
        (uint32_t)(uintptr_t)first_buffer, (uint32_t)(uintptr_t)second_buffer,
        (uint32_t)buffer_capacity);
    if (hal_status != HAL_OK)
    {
        __HAL_UART_DISABLE_IT(uart, UART_IT_IDLE);
        uart->RxState = HAL_UART_STATE_READY;
        return bsp_stm32h723_status(hal_status);
    }
    SET_BIT(uart->Instance->CR3, USART_CR3_DMAR);
    return BSP_STATUS_OK;
}

/**
 * @brief USART 中止当前传输
 * @param handle UART_HandleTypeDef* 句柄
 * @return 执行状态
 */
static bsp_status_t bsp_stm32h723_usart_abort(void *handle)
{
    return bsp_stm32h723_status(HAL_UART_Abort((UART_HandleTypeDef *)handle));
}

/**
 * @brief 查询 USART 是否忙
 * @param handle UART_HandleTypeDef* 句柄
 * @param is_busy 输出是否忙
 * @return 执行状态
 */
static bsp_status_t bsp_stm32h723_usart_busy(const void *handle, bool *is_busy)
{
    if ((handle == NULL) || (is_busy == NULL))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    *is_busy = HAL_UART_GetState((UART_HandleTypeDef *)(uintptr_t)handle) != HAL_UART_STATE_READY;
    return BSP_STATUS_OK;
}

/** USART 驱动操作表 */
static const bsp_usart_driver_ops_t bsp_stm32h723_usart_driver_ops = {
    .init = bsp_stm32h723_noop_init,
    .deinit = bsp_stm32h723_noop_deinit,
    .transmit = bsp_stm32h723_usart_transmit,
    .receive = bsp_stm32h723_usart_receive,
    .receive_to_idle = bsp_stm32h723_usart_receive_to_idle,
    .receive_to_idle_double_buffer = bsp_stm32h723_usart_receive_to_idle_double_buffer,
    .abort = bsp_stm32h723_usart_abort,
    .get_busy = bsp_stm32h723_usart_busy,
};

/* ---------- SPI 驱动实现 ---------- */

/**
 * @brief SPI 发送（支持三种模式）
 */
static bsp_status_t bsp_stm32h723_spi_transmit(void *handle, const uint8_t *data, size_t size,
                                               bsp_transfer_mode_t mode, uint32_t timeout_ms)
{
    if ((data == NULL) || (size == 0U) || (size > UINT16_MAX))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    if (mode == BSP_TRANSFER_MODE_BLOCKING)
    {
        return bsp_stm32h723_status(
            HAL_SPI_Transmit((SPI_HandleTypeDef *)handle, data, (uint16_t)size, timeout_ms));
    }
    if (mode == BSP_TRANSFER_MODE_INTERRUPT)
    {
        return bsp_stm32h723_status(
            HAL_SPI_Transmit_IT((SPI_HandleTypeDef *)handle, data, (uint16_t)size));
    }
    return bsp_stm32h723_status(
        HAL_SPI_Transmit_DMA((SPI_HandleTypeDef *)handle, data, (uint16_t)size));
}

/**
 * @brief SPI 接收（支持三种模式）
 */
static bsp_status_t bsp_stm32h723_spi_receive(void *handle, uint8_t *data, size_t size,
                                              bsp_transfer_mode_t mode, uint32_t timeout_ms)
{
    if ((data == NULL) || (size == 0U) || (size > UINT16_MAX))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    if (mode == BSP_TRANSFER_MODE_BLOCKING)
    {
        return bsp_stm32h723_status(
            HAL_SPI_Receive((SPI_HandleTypeDef *)handle, data, (uint16_t)size, timeout_ms));
    }
    if (mode == BSP_TRANSFER_MODE_INTERRUPT)
    {
        return bsp_stm32h723_status(
            HAL_SPI_Receive_IT((SPI_HandleTypeDef *)handle, data, (uint16_t)size));
    }
    return bsp_stm32h723_status(
        HAL_SPI_Receive_DMA((SPI_HandleTypeDef *)handle, data, (uint16_t)size));
}

/**
 * @brief SPI 全双工交换（支持三种模式）
 */
static bsp_status_t bsp_stm32h723_spi_exchange(void *handle, const uint8_t *transmit_data,
                                               uint8_t *receive_data, size_t size,
                                               bsp_transfer_mode_t mode, uint32_t timeout_ms)
{
    if ((transmit_data == NULL) || (receive_data == NULL) || (size == 0U) || (size > UINT16_MAX))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    if (mode == BSP_TRANSFER_MODE_BLOCKING)
    {
        return bsp_stm32h723_status(HAL_SPI_TransmitReceive(
            (SPI_HandleTypeDef *)handle, transmit_data, receive_data, (uint16_t)size, timeout_ms));
    }
    if (mode == BSP_TRANSFER_MODE_INTERRUPT)
    {
        return bsp_stm32h723_status(HAL_SPI_TransmitReceive_IT(
            (SPI_HandleTypeDef *)handle, transmit_data, receive_data, (uint16_t)size));
    }
    return bsp_stm32h723_status(HAL_SPI_TransmitReceive_DMA(
        (SPI_HandleTypeDef *)handle, transmit_data, receive_data, (uint16_t)size));
}

/**
 * @brief SPI 中止当前传输
 */
static bsp_status_t bsp_stm32h723_spi_abort(void *handle)
{
    return bsp_stm32h723_status(HAL_SPI_Abort((SPI_HandleTypeDef *)handle));
}

/**
 * @brief 查询 SPI 是否忙
 */
static bsp_status_t bsp_stm32h723_spi_busy(const void *handle, bool *is_busy)
{
    if ((handle == NULL) || (is_busy == NULL))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    *is_busy = HAL_SPI_GetState((SPI_HandleTypeDef *)(uintptr_t)handle) != HAL_SPI_STATE_READY;
    return BSP_STATUS_OK;
}

/** SPI 驱动操作表 */
static const bsp_spi_driver_ops_t bsp_stm32h723_spi_driver_ops = {
    .init = bsp_stm32h723_noop_init,
    .deinit = bsp_stm32h723_noop_deinit,
    .transmit = bsp_stm32h723_spi_transmit,
    .receive = bsp_stm32h723_spi_receive,
    .exchange = bsp_stm32h723_spi_exchange,
    .abort = bsp_stm32h723_spi_abort,
    .get_busy = bsp_stm32h723_spi_busy,
};

/* ---------- EXTI 驱动实现 ---------- */

/**
 * @brief EXTI 初始化（仅验证 handle 非空）
 */
static bsp_status_t bsp_stm32h723_exti_init(void *handle)
{
    return (handle != NULL) ? BSP_STATUS_OK : BSP_STATUS_INVALID_ARGUMENT;
}

/**
 * @brief 使能 EXTI 中断（使能 NVIC）
 */
static bsp_status_t bsp_stm32h723_exti_enable(void *handle)
{
    const bsp_stm32h723_exti_context_t *const context =
        (const bsp_stm32h723_exti_context_t *)handle;
    HAL_NVIC_EnableIRQ(context->interrupt_number);
    return BSP_STATUS_OK;
}

/**
 * @brief 禁用 EXTI 中断（禁用 NVIC）
 */
static bsp_status_t bsp_stm32h723_exti_disable(void *handle)
{
    const bsp_stm32h723_exti_context_t *const context =
        (const bsp_stm32h723_exti_context_t *)handle;
    HAL_NVIC_DisableIRQ(context->interrupt_number);
    return BSP_STATUS_OK;
}

/** EXTI 驱动操作表 */
static const bsp_exti_driver_ops_t bsp_stm32h723_exti_driver_ops = {
    .init = bsp_stm32h723_exti_init,
    .deinit = bsp_stm32h723_noop_deinit,
    .enable = bsp_stm32h723_exti_enable,
    .disable = bsp_stm32h723_exti_disable,
};

/* ---------- PWM 驱动实现 ---------- */

/**
 * @brief 将逻辑通道号映射为 HAL 通道宏
 * @param channel 逻辑通道号（1~4）
 * @return HAL TIM 通道宏（TIM_CHANNEL_1~4）
 */
static uint32_t bsp_stm32h723_pwm_channel(uint32_t channel)
{
    static const uint32_t channels[] = {0U, TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3,
                                        TIM_CHANNEL_4};
    return (channel <= 4U) ? channels[channel] : 0U;
}

/**
 * @brief PWM 初始化
 */
static bsp_status_t bsp_stm32h723_pwm_init(void *handle, uint32_t channel)
{
    return ((handle != NULL) && (bsp_stm32h723_pwm_channel(channel) != 0U))
               ? BSP_STATUS_OK
               : BSP_STATUS_INVALID_ARGUMENT;
}

/**
 * @brief 启动 PWM 输出
 */
static bsp_status_t bsp_stm32h723_pwm_start(void *handle, uint32_t channel)
{
    return bsp_stm32h723_status(
        HAL_TIM_PWM_Start((TIM_HandleTypeDef *)handle, bsp_stm32h723_pwm_channel(channel)));
}

/**
 * @brief 停止 PWM 输出
 */
static bsp_status_t bsp_stm32h723_pwm_stop(void *handle, uint32_t channel)
{
    return bsp_stm32h723_status(
        HAL_TIM_PWM_Stop((TIM_HandleTypeDef *)handle, bsp_stm32h723_pwm_channel(channel)));
}

/**
 * @brief 设置 PWM 频率
 * @param handle TIM_HandleTypeDef* 句柄
 * @param channel 通道号（未使用，频率对定时器所有通道生效）
 * @param frequency_hz 目标频率（Hz）
 * @return 执行状态
 * @note 修改频率会影响同一定时器的所有 PWM 通道
 */
static bsp_status_t bsp_stm32h723_pwm_set_frequency(void *handle, uint32_t channel,
                                                    uint32_t frequency_hz)
{
    TIM_HandleTypeDef *const timer = (TIM_HandleTypeDef *)handle;
    const uint32_t timer_clock_hz = BOARD_CONFIG_APB_FREQUENCY_HZ * 2UL;
    uint32_t period_ticks;
    (void)channel;
    if (frequency_hz == 0U)
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    // 计算周期值：period = 时钟 / (预分频器 + 1) / 频率
    period_ticks = timer_clock_hz / ((timer->Init.Prescaler + 1U) * frequency_hz);
    if ((period_ticks == 0U) || (period_ticks > 65536U))
    {
        return BSP_STATUS_OUT_OF_RANGE;
    }
    // 设置自动重载值（ARR = period - 1）
    __HAL_TIM_SET_AUTORELOAD(timer, period_ticks - 1U);
    __HAL_TIM_SET_COUNTER(timer, 0U);
    return BSP_STATUS_OK;
}

/**
 * @brief 获取 PWM 频率
 */
static bsp_status_t bsp_stm32h723_pwm_get_frequency(const void *handle, uint32_t channel,
                                                    uint32_t *frequency_hz)
{
    const TIM_HandleTypeDef *const timer = (const TIM_HandleTypeDef *)handle;
    (void)channel;
    if ((handle == NULL) || (frequency_hz == NULL))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    *frequency_hz = (BOARD_CONFIG_APB_FREQUENCY_HZ * 2UL) /
                    ((timer->Init.Prescaler + 1U) * (timer->Instance->ARR + 1U));
    return BSP_STATUS_OK;
}

/**
 * @brief 设置脉冲宽度（比较值）
 */
static bsp_status_t bsp_stm32h723_pwm_set_pulse(void *handle, uint32_t channel,
                                                uint32_t pulse_ticks)
{
    TIM_HandleTypeDef *const timer = (TIM_HandleTypeDef *)handle;
    // 脉冲宽度不能超过周期值
    if (pulse_ticks > timer->Instance->ARR + 1U)
    {
        return BSP_STATUS_OUT_OF_RANGE;
    }
    __HAL_TIM_SET_COMPARE(timer, bsp_stm32h723_pwm_channel(channel), pulse_ticks);
    return BSP_STATUS_OK;
}

/**
 * @brief 获取脉冲宽度（比较值）
 */
static bsp_status_t bsp_stm32h723_pwm_get_pulse(const void *handle, uint32_t channel,
                                                uint32_t *pulse_ticks)
{
    if ((handle == NULL) || (pulse_ticks == NULL))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    *pulse_ticks = __HAL_TIM_GET_COMPARE((TIM_HandleTypeDef *)(uintptr_t)handle,
                                         bsp_stm32h723_pwm_channel(channel));
    return BSP_STATUS_OK;
}

/**
 * @brief 获取周期值
 */
static bsp_status_t bsp_stm32h723_pwm_get_period(const void *handle, uint32_t channel,
                                                 uint32_t *period_ticks)
{
    (void)channel;
    if ((handle == NULL) || (period_ticks == NULL))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    *period_ticks = ((const TIM_HandleTypeDef *)handle)->Instance->ARR + 1U;
    return BSP_STATUS_OK;
}

/** PWM 驱动操作表 */
static const bsp_pwm_driver_ops_t bsp_stm32h723_pwm_driver_ops = {
    .init = bsp_stm32h723_pwm_init,
    .deinit = bsp_stm32h723_pwm_stop,
    .start = bsp_stm32h723_pwm_start,
    .stop = bsp_stm32h723_pwm_stop,
    .set_frequency = bsp_stm32h723_pwm_set_frequency,
    .get_frequency = bsp_stm32h723_pwm_get_frequency,
    .set_pulse = bsp_stm32h723_pwm_set_pulse,
    .get_pulse = bsp_stm32h723_pwm_get_pulse,
    .get_period = bsp_stm32h723_pwm_get_period,
};

/* ---------- Timebase 驱动实现 ---------- */

/**
 * @brief 初始化时间基准（使用 DWT 周期计数器）
 */
static bsp_status_t bsp_stm32h723_timebase_init(void *handle)
{
    (void)handle;
    // 使能 DWT 跟踪（需要先使能 TRCENA）
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    return ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) != 0U) ? BSP_STATUS_OK : BSP_STATUS_UNSUPPORTED;
}

/**
 * @brief 复位周期计数器
 */
static bsp_status_t bsp_stm32h723_timebase_reset(void *handle)
{
    (void)handle;
    DWT->CYCCNT = 0U;
    return BSP_STATUS_OK;
}

/**
 * @brief 获取当前周期计数
 */
static bsp_status_t bsp_stm32h723_timebase_cycles(const void *handle, uint32_t *cycle_count)
{
    (void)handle;
    if (cycle_count == NULL)
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    *cycle_count = DWT->CYCCNT;
    return BSP_STATUS_OK;
}

/**
 * @brief 获取时间基准频率（CPU 主频）
 */
static bsp_status_t bsp_stm32h723_timebase_frequency(const void *handle, uint32_t *frequency_hz)
{
    (void)handle;
    if (frequency_hz == NULL)
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    *frequency_hz = SystemCoreClock;
    return BSP_STATUS_OK;
}

/** Timebase 驱动操作表 */
static const bsp_timebase_driver_ops_t bsp_stm32h723_timebase_driver_ops = {
    .init = bsp_stm32h723_timebase_init,
    .deinit = bsp_stm32h723_noop_deinit,
    .reset = bsp_stm32h723_timebase_reset,
    .get_cycle_count = bsp_stm32h723_timebase_cycles,
    .get_frequency = bsp_stm32h723_timebase_frequency,
};

/* ---------- Watchdog 驱动实现 ---------- */

/**
 * @brief 初始化独立看门狗（IWDG1）
 * @param handle 看门狗上下文指针
 * @return 执行状态
 */
static bsp_status_t bsp_stm32h723_watchdog_init(void *handle)
{
    bsp_stm32h723_watchdog_context_t *const context = (bsp_stm32h723_watchdog_context_t *)handle;
    // 检测是否由看门狗复位（IWDG1 复位标志）
    context->reset_detected = (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDG1RST) != 0U);
    __HAL_RCC_CLEAR_RESET_FLAGS();
    // 配置 IWDG：使能写访问，设置预分频器（4 = 256 分频），设置重载值（1000）
    IWDG1->KR = 0x5555U; // 使能写访问
    IWDG1->PR = 4U;      // 预分频器 = 256（约 2 秒超时，取决于 LSI）
    IWDG1->RLR = 1000U;  // 重载值
    IWDG1->WINR = 4095U; // 窗口值（禁用窗口模式）
    while (IWDG1->SR != 0U)
    {
    }                    // 等待寄存器更新完成
    IWDG1->KR = 0xCCCCU; // 启动看门狗
    return BSP_STATUS_OK;
}

/**
 * @brief 刷新看门狗（喂狗）
 */
static bsp_status_t bsp_stm32h723_watchdog_refresh(void *handle)
{
    (void)handle;
    IWDG1->KR = 0xAAAAU; // 重载计数器
    return BSP_STATUS_OK;
}

/**
 * @brief 获取看门狗超时时间
 */
static bsp_status_t bsp_stm32h723_watchdog_timeout(const void *handle, uint32_t *timeout_ms)
{
    if ((handle == NULL) || (timeout_ms == NULL))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    *timeout_ms = ((const bsp_stm32h723_watchdog_context_t *)handle)->timeout_ms;
    return BSP_STATUS_OK;
}

/**
 * @brief 检测是否由看门狗复位
 */
static bsp_status_t bsp_stm32h723_watchdog_reset_detected(const void *handle, bool *reset_detected)
{
    if ((handle == NULL) || (reset_detected == NULL))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    *reset_detected = ((const bsp_stm32h723_watchdog_context_t *)handle)->reset_detected;
    return BSP_STATUS_OK;
}

/** Watchdog 驱动操作表 */
static const bsp_watchdog_driver_ops_t bsp_stm32h723_watchdog_driver_ops = {
    .init = bsp_stm32h723_watchdog_init,
    .deinit = bsp_stm32h723_noop_deinit,
    .refresh = bsp_stm32h723_watchdog_refresh,
    .get_timeout_ms = bsp_stm32h723_watchdog_timeout,
    .get_reset_detected = bsp_stm32h723_watchdog_reset_detected,
};

/* ---------- USB VCP 驱动实现 ---------- */

/**
 * @brief USB 虚拟串口发送
 * @param handle USB 上下文指针
 * @param transmit_data 发送数据指针
 * @param data_size 数据大小
 * @param timeout_ms 超时时间
 * @return 执行状态
 */
static bsp_status_t bsp_stm32h723_usb_transmit(void *handle, const uint8_t *transmit_data,
                                               size_t data_size, uint32_t timeout_ms)
{
    uint32_t started_at_ms = HAL_GetTick();
    uint8_t usb_status;
    (void)handle;
    if ((transmit_data == NULL) || (data_size == 0U) || (data_size > UINT16_MAX))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    // 循环尝试发送直到成功或超时
    do
    {
        usb_status = usb_cdc_transmit((uint8_t *)(uintptr_t)transmit_data, (uint16_t)data_size);
        if (usb_status == USBD_OK)
        {
            return BSP_STATUS_OK;
        }
        if (usb_status != USBD_BUSY)
        {
            return BSP_STATUS_IO_ERROR;
        }
        if ((HAL_GetTick() - started_at_ms) >= timeout_ms)
        {
            return BSP_STATUS_TIMEOUT;
        }
    } while (true);
}

/**
 * @brief USB 虚拟串口接收
 * @param handle USB 上下文指针
 * @param receive_data 接收缓冲区指针
 * @param data_capacity 缓冲区容量
 * @return 执行状态
 */
static bsp_status_t bsp_stm32h723_usb_receive(void *handle, uint8_t *receive_data,
                                              size_t data_capacity)
{
    bsp_stm32h723_usb_context_t *const context = (bsp_stm32h723_usb_context_t *)handle;
    size_t copy_size;
    if ((receive_data == NULL) || (data_capacity == 0U))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    if (!context->receive_pending)
    {
        return BSP_STATUS_BUSY;
    }
    // 复制数据到用户缓冲区
    copy_size = (context->receive_size < data_capacity) ? context->receive_size : data_capacity;
    memcpy(receive_data, context->receive_buffer, copy_size);
    context->receive_pending = false;
    context->receive_size = 0U;
    return BSP_STATUS_OK;
}

/**
 * @brief USB 中止接收
 */
static bsp_status_t bsp_stm32h723_usb_abort(void *handle)
{
    bsp_stm32h723_usb_context_t *const context = (bsp_stm32h723_usb_context_t *)handle;
    context->receive_pending = false;
    context->receive_size = 0U;
    return BSP_STATUS_OK;
}

/**
 * @brief 查询 USB 是否已连接（已配置）
 */
static bsp_status_t bsp_stm32h723_usb_connected(const void *handle, bool *is_connected)
{
    (void)handle;
    if (is_connected == NULL)
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    *is_connected = hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED;
    return BSP_STATUS_OK;
}

/**
 * @brief 查询 USB 发送是否忙
 */
static bsp_status_t bsp_stm32h723_usb_busy(const void *handle, bool *is_busy)
{
    const USBD_CDC_HandleTypeDef *class_data;
    (void)handle;
    if (is_busy == NULL)
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    class_data = (const USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
    *is_busy = (class_data != NULL) && (class_data->TxState != 0U);
    return BSP_STATUS_OK;
}

/** USB VCP 驱动操作表 */
static const bsp_usb_vcp_driver_ops_t bsp_stm32h723_usb_driver_ops = {
    .init = bsp_stm32h723_noop_init,
    .deinit = bsp_stm32h723_noop_deinit,
    .transmit = bsp_stm32h723_usb_transmit,
    .receive = bsp_stm32h723_usb_receive,
    .abort = bsp_stm32h723_usb_abort,
    .get_connected = bsp_stm32h723_usb_connected,
    .get_busy = bsp_stm32h723_usb_busy,
};

/* ---------- 初始化函数 ---------- */

/**
 * @brief 初始化所有 CAN 设备
 * @return 执行状态
 */
static bsp_status_t bsp_stm32h723_init_can_devices(void)
{
    FDCAN_HandleTypeDef *handles[BSP_STM32H723_CAN_COUNT] = {
        &hfdcan1,
        &hfdcan2,
        &hfdcan3,
    };
    size_t index;
    for (index = 0U; index < BSP_STM32H723_CAN_COUNT; ++index)
    {
        const bsp_can_config_t config = {
            .device_handle = handles[index],
            .driver_ops = &bsp_stm32h723_can_driver_ops,
        };
        if (bsp_can_init(&bsp_stm32h723_can_devices[index], &config) != BSP_STATUS_OK)
        {
            return BSP_STATUS_IO_ERROR;
        }
    }
    return BSP_STATUS_OK;
}

/**
 * @brief 初始化所有 USART 设备
 * @return 执行状态
 */
static bsp_status_t bsp_stm32h723_init_usart_devices(void)
{
    UART_HandleTypeDef *handles[BSP_STM32H723_USART_COUNT] = {
        &huart2, &huart6, &huart7, &huart8, &huart5,
    };
    size_t index;
    for (index = 0U; index < BSP_STM32H723_USART_COUNT; ++index)
    {
        const bsp_usart_config_t config = {
            .device_handle = handles[index],
            .driver_ops = &bsp_stm32h723_usart_driver_ops,
        };
        if (bsp_usart_init(&bsp_stm32h723_usart_devices[index], &config) != BSP_STATUS_OK)
        {
            return BSP_STATUS_IO_ERROR;
        }
    }
    return BSP_STATUS_OK;
}

/**
 * @brief 端口初始化主函数
 * @param config 端口配置
 * @return 执行状态
 */
bsp_status_t bsp_stm32h723_port_init(const bsp_stm32h723_port_config_t *config)
{
    static const uint32_t pwm_channels[BSP_STM32H723_PWM_COUNT] = {1U, 2U, 3U, 4U, 1U};
    TIM_HandleTypeDef *pwm_handles[BSP_STM32H723_PWM_COUNT] = {
        &htim3, &htim3, &htim3, &htim3, &htim1,
    };
    size_t index;
    // 参数校验：config 非空，且端口未初始化
    if ((config == NULL) || bsp_stm32h723_initialized)
    {
        return (config == NULL) ? BSP_STATUS_INVALID_ARGUMENT : BSP_STATUS_BUSY;
    }
    // 1. 初始化 CAN 和 USART
    if ((bsp_stm32h723_init_can_devices() != BSP_STATUS_OK) ||
        (bsp_stm32h723_init_usart_devices() != BSP_STATUS_OK))
    {
        return BSP_STATUS_IO_ERROR;
    }
    // 2. 初始化 BMI088 SPI
    {
        const bsp_spi_config_t spi_config = {
            .device_handle = &hspi2,
            .driver_ops = &bsp_stm32h723_spi_driver_ops,
        };
        if (bsp_spi_init(&bsp_stm32h723_bmi088_spi_device, &spi_config) != BSP_STATUS_OK)
        {
            return BSP_STATUS_IO_ERROR;
        }
    }
    // 3. 配置 EXTI 上下文（陀螺仪和加速度计中断引脚）
    bsp_stm32h723_exti_contexts[0] = (bsp_stm32h723_exti_context_t){
        .pin = BMI088_GYRO_INT_Pin,
        .interrupt_number = BMI088_GYRO_INT_EXTI_IRQn,
    };
    bsp_stm32h723_exti_contexts[1] = (bsp_stm32h723_exti_context_t){
        .pin = BMI088_ACCEL_INT_Pin,
        .interrupt_number = BMI088_ACCEL_INT_EXTI_IRQn,
    };
    // 4. 初始化 EXTI 设备
    for (index = 0U; index < BSP_STM32H723_EXTI_COUNT; ++index)
    {
        const bsp_exti_config_t exti_config = {
            .device_handle = &bsp_stm32h723_exti_contexts[index],
            .driver_ops = &bsp_stm32h723_exti_driver_ops,
        };
        if (bsp_exti_init(&bsp_stm32h723_exti_devices[index], &exti_config) != BSP_STATUS_OK)
        {
            return BSP_STATUS_IO_ERROR;
        }
    }
    // 5. 初始化 PWM 设备
    for (index = 0U; index < BSP_STM32H723_PWM_COUNT; ++index)
    {
        const bsp_pwm_config_t pwm_config = {
            .device_handle = pwm_handles[index],
            .driver_ops = &bsp_stm32h723_pwm_driver_ops,
            .channel = pwm_channels[index],
        };
        if (bsp_pwm_init(&bsp_stm32h723_pwm_devices[index], &pwm_config) != BSP_STATUS_OK)
        {
            return BSP_STATUS_IO_ERROR;
        }
    }
    // 6. 初始化 USB VCP 和 Timebase
    {
        const bsp_usb_vcp_config_t usb_config = {
            .device_handle = &bsp_stm32h723_usb_context,
            .driver_ops = &bsp_stm32h723_usb_driver_ops,
        };
        const bsp_timebase_config_t timebase_config = {
            .device_handle = &bsp_stm32h723_timebase_device,
            .driver_ops = &bsp_stm32h723_timebase_driver_ops,
        };
        if ((bsp_usb_vcp_init(&bsp_stm32h723_usb_device, &usb_config) != BSP_STATUS_OK) ||
            (bsp_timebase_init(&bsp_stm32h723_timebase_device, &timebase_config) != BSP_STATUS_OK))
        {
            return BSP_STATUS_IO_ERROR;
        }
    }
    // 7. 可选初始化看门狗
    if (config->initialize_watchdog)
    {
        const bsp_watchdog_config_t watchdog_config = {
            .device_handle = &bsp_stm32h723_watchdog_context,
            .driver_ops = &bsp_stm32h723_watchdog_driver_ops,
        };
        bsp_stm32h723_watchdog_context.timeout_ms = 2000U;
        if (bsp_watchdog_init(&bsp_stm32h723_watchdog_device, &watchdog_config) != BSP_STATUS_OK)
        {
            return BSP_STATUS_IO_ERROR;
        }
        bsp_stm32h723_watchdog_initialized = true;
    }
    bsp_stm32h723_initialized = true;
    return BSP_STATUS_OK;
}

/* ---------- Getter 函数 ---------- */

/**
 * @brief 获取 CAN 基类指针
 */
bsp_can_t *bsp_stm32h723_port_get_can(bsp_stm32h723_can_index_t index)
{
    return (bsp_stm32h723_initialized && (index < BSP_STM32H723_CAN_COUNT))
               ? bsp_can_as_base(&bsp_stm32h723_can_devices[index])
               : NULL;
}

/**
 * @brief 获取 USART 基类指针
 */
bsp_usart_t *bsp_stm32h723_port_get_usart(bsp_stm32h723_usart_index_t index)
{
    return (bsp_stm32h723_initialized && (index < BSP_STM32H723_USART_COUNT))
               ? bsp_usart_as_base(&bsp_stm32h723_usart_devices[index])
               : NULL;
}

/**
 * @brief 获取 BMI088 SPI 基类指针
 */
bsp_spi_t *bsp_stm32h723_port_get_bmi088_spi(void)
{
    return bsp_stm32h723_initialized ? bsp_spi_as_base(&bsp_stm32h723_bmi088_spi_device) : NULL;
}

/**
 * @brief 获取 EXTI 基类指针
 */
bsp_exti_t *bsp_stm32h723_port_get_exti(bsp_stm32h723_exti_index_t index)
{
    return (bsp_stm32h723_initialized && (index < BSP_STM32H723_EXTI_COUNT))
               ? bsp_exti_as_base(&bsp_stm32h723_exti_devices[index])
               : NULL;
}

/**
 * @brief 获取 PWM 基类指针
 */
bsp_pwm_t *bsp_stm32h723_port_get_pwm(bsp_stm32h723_pwm_index_t index)
{
    return (bsp_stm32h723_initialized && (index < BSP_STM32H723_PWM_COUNT))
               ? bsp_pwm_as_base(&bsp_stm32h723_pwm_devices[index])
               : NULL;
}

/**
 * @brief 获取 USB VCP 基类指针
 */
bsp_usb_vcp_t *bsp_stm32h723_port_get_usb_vcp(void)
{
    return bsp_stm32h723_initialized ? bsp_usb_vcp_as_base(&bsp_stm32h723_usb_device) : NULL;
}

/**
 * @brief 获取 Timebase 基类指针
 */
bsp_timebase_t *bsp_stm32h723_port_get_timebase(void)
{
    return bsp_stm32h723_initialized ? bsp_timebase_as_base(&bsp_stm32h723_timebase_device) : NULL;
}

/**
 * @brief 获取 Watchdog 基类指针
 */
bsp_watchdog_t *bsp_stm32h723_port_get_watchdog(void)
{
    return (bsp_stm32h723_initialized && bsp_stm32h723_watchdog_initialized)
               ? bsp_watchdog_as_base(&bsp_stm32h723_watchdog_device)
               : NULL;
}

/**
 * @brief 查询端口是否已初始化
 */
bool bsp_stm32h723_port_is_initialized(void)
{
    return bsp_stm32h723_initialized;
}

/* ---------- HAL 回调路由（将 HAL 回调转发到 BSP notify） ---------- */

/**
 * @brief FDCAN FIFO0 接收回调
 * @param fdcan FDCAN 句柄
 * @param interrupt_flags 中断标志
 * @note 在 ISR 中执行，只发布事件，不解析协议
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *fdcan, uint32_t interrupt_flags)
{
    size_t index;
    (void)interrupt_flags;
    // 查找对应的 CAN 设备并发送通知
    for (index = 0U; index < BSP_STM32H723_CAN_COUNT; ++index)
    {
        if (bsp_device_get_handle(&bsp_stm32h723_can_devices[index].super.super) == fdcan)
        {
            bsp_can_notify(&bsp_stm32h723_can_devices[index].super, BSP_EVENT_RECEIVE_PENDING,
                           BSP_STATUS_OK, 0U);
            break;
        }
    }
}

/**
 * @brief FDCAN FIFO1 接收回调
 */
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *fdcan, uint32_t interrupt_flags)
{
    HAL_FDCAN_RxFifo0Callback(fdcan, interrupt_flags);
}

/**
 * @brief FDCAN 错误状态回调
 */
void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *fdcan, uint32_t error_status)
{
    size_t index;
    (void)error_status;
    for (index = 0U; index < BSP_STM32H723_CAN_COUNT; ++index)
    {
        if (bsp_device_get_handle(&bsp_stm32h723_can_devices[index].super.super) == fdcan)
        {
            bsp_can_notify(&bsp_stm32h723_can_devices[index].super, BSP_EVENT_ERROR,
                           BSP_STATUS_IO_ERROR, 0U);
            break;
        }
    }
}

/**
 * @brief 根据 UART 句柄查找对应的 USART 基类指针
 */
static bsp_usart_t *bsp_stm32h723_find_usart(UART_HandleTypeDef *uart)
{
    size_t index;
    for (index = 0U; index < BSP_STM32H723_USART_COUNT; ++index)
    {
        if (bsp_device_get_handle(&bsp_stm32h723_usart_devices[index].super.super) == uart)
        {
            return &bsp_stm32h723_usart_devices[index].super;
        }
    }
    return NULL;
}

/**
 * @brief UART 发送完成回调
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *uart)
{
    bsp_usart_t *const usart = bsp_stm32h723_find_usart(uart);
    if (usart != NULL)
    {
        bsp_usart_notify(usart, BSP_EVENT_TRANSMIT_COMPLETE, BSP_STATUS_OK, uart->TxXferSize);
    }
}

/**
 * @brief UART 接收完成回调
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart)
{
    bsp_usart_t *const usart = bsp_stm32h723_find_usart(uart);
    if (usart != NULL)
    {
        bsp_usart_notify(usart, BSP_EVENT_RECEIVE_COMPLETE, BSP_STATUS_OK, uart->RxXferSize);
    }
}

/**
 * @brief UART 空闲事件回调（接收到空闲）
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *uart, uint16_t received_size)
{
    bsp_usart_t *const usart = bsp_stm32h723_find_usart(uart);
    if (usart != NULL)
    {
        DMA_Stream_TypeDef *const dma_stream =
            (uart->hdmarx != NULL) ? (DMA_Stream_TypeDef *)uart->hdmarx->Instance : NULL;
        if ((dma_stream != NULL) && ((dma_stream->CR & DMA_SxCR_DBM) != 0U))
        {
            const uint8_t completed_buffer_index =
                ((dma_stream->CR & DMA_SxCR_CT) != 0U) ? 1U : 0U;

            __HAL_DMA_DISABLE(uart->hdmarx);
            while ((dma_stream->CR & DMA_SxCR_EN) != 0U)
            {
            }
            if (completed_buffer_index == 0U)
            {
                SET_BIT(dma_stream->CR, DMA_SxCR_CT);
            }
            else
            {
                CLEAR_BIT(dma_stream->CR, DMA_SxCR_CT);
            }
            __HAL_DMA_SET_COUNTER(uart->hdmarx, uart->RxXferSize);
            __HAL_DMA_ENABLE(uart->hdmarx);
            bsp_usart_notify_double_buffer(usart, completed_buffer_index, received_size);
        }
        else
        {
            bsp_usart_notify(usart, BSP_EVENT_RECEIVE_COMPLETE, BSP_STATUS_OK, received_size);
        }
    }
}

/**
 * @brief UART 错误回调
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
    bsp_usart_t *const usart = bsp_stm32h723_find_usart(uart);
    if (usart != NULL)
    {
        bsp_usart_notify(usart, BSP_EVENT_ERROR, BSP_STATUS_IO_ERROR, 0U);
    }
}

/**
 * @brief SPI 发送/接收完成回调（仅用于 BMI088 SPI）
 */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *spi)
{
    if (spi == &hspi2)
    {
        bsp_spi_notify(&bsp_stm32h723_bmi088_spi_device.super, BSP_EVENT_TRANSFER_COMPLETE,
                       BSP_STATUS_OK, spi->TxXferSize);
    }
}

/**
 * @brief SPI 发送完成回调（仅用于 BMI088 SPI）
 */
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *spi)
{
    if (spi == &hspi2)
    {
        bsp_spi_notify(&bsp_stm32h723_bmi088_spi_device.super, BSP_EVENT_TRANSMIT_COMPLETE,
                       BSP_STATUS_OK, spi->TxXferSize);
    }
}

/**
 * @brief SPI 接收完成回调（仅用于 BMI088 SPI）
 */
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *spi)
{
    if (spi == &hspi2)
    {
        bsp_spi_notify(&bsp_stm32h723_bmi088_spi_device.super, BSP_EVENT_RECEIVE_COMPLETE,
                       BSP_STATUS_OK, spi->RxXferSize);
    }
}

/**
 * @brief GPIO EXTI 回调
 * @param pin 触发中断的引脚号
 */
void HAL_GPIO_EXTI_Callback(uint16_t pin)
{
    size_t index;
    for (index = 0U; index < BSP_STM32H723_EXTI_COUNT; ++index)
    {
        if (bsp_stm32h723_exti_contexts[index].pin == pin)
        {
            bsp_exti_notify(&bsp_stm32h723_exti_devices[index].super);
        }
    }
}

/**
 * @brief USB CDC 接收回调（由 USB 协议栈调用）
 * @param receive_data 接收数据指针
 * @param receive_size 数据大小
 */
void usb_cdc_receive_callback(const uint8_t *receive_data, uint32_t receive_size)
{
    const size_t copy_size = (receive_size < BSP_STM32H723_USB_RECEIVE_CAPACITY)
                                 ? (size_t)receive_size
                                 : BSP_STM32H723_USB_RECEIVE_CAPACITY;
    if ((receive_data == NULL) || !bsp_stm32h723_initialized)
    {
        return;
    }
    // 复制数据到 USB 上下文缓冲区
    memcpy(bsp_stm32h723_usb_context.receive_buffer, receive_data, copy_size);
    bsp_stm32h723_usb_context.receive_size = copy_size;
    bsp_stm32h723_usb_context.receive_pending = true;
    // 通知 USB VCP 有数据待接收
    bsp_usb_vcp_notify(&bsp_stm32h723_usb_device.super, BSP_EVENT_RECEIVE_PENDING, BSP_STATUS_OK,
                       copy_size);
}
