/**
 * @file board_config.c
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

#include "board_config.h"

#include "fdcan.h"
#include "main.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"
#include "usbd_def.h"
#include "cmsis_os2.h"

#include <string.h>

/** Cortex-M7 DWT 锁访问寄存器的解锁键。 */
#define BOARD_CONFIG_DWT_LOCK_ACCESS_KEY (0xC5ACCE55UL)

typedef struct
{
    uint16_t pin;
    IRQn_Type interrupt_number;
} board_config_exti_context_t;

typedef struct
{
    TIM_HandleTypeDef *timer;
    uint32_t timer_clock_hz;
} board_config_pwm_context_t;

typedef struct
{
    uint32_t timeout_ms;
    bool reset_detected;
} board_config_watchdog_context_t;

/**
 * @brief 将 HAL 状态码转换为 BSP 状态码
 * @param status HAL 状态码（HAL_StatusTypeDef）
 * @return BSP 状态码
 */
static bsp_status_t board_config_status(HAL_StatusTypeDef status)
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
static bsp_status_t board_config_noop_init(void *handle)
{
    return (handle != NULL) ? BSP_STATUS_OK : BSP_STATUS_INVALID_ARGUMENT;
}

/**
 * @brief 空操作 deinit（用于不需要反初始化的驱动）
 * @param handle 设备句柄
 * @return 若 handle 非空则返回 OK
 */
static bsp_status_t board_config_noop_deinit(void *handle)
{
    return (handle != NULL) ? BSP_STATUS_OK : BSP_STATUS_INVALID_ARGUMENT;
}

/* ---------- Cortex-M7 DWT 驱动实现 ---------- */

static bsp_status_t board_config_dwt_init(void *device_handle)
{
    (void)device_handle;
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->LAR = BOARD_CONFIG_DWT_LOCK_ACCESS_KEY;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    __DSB();
    __ISB();
    return ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) != 0U) ? BSP_STATUS_OK : BSP_STATUS_UNSUPPORTED;
}

static bsp_status_t board_config_dwt_reset(void *device_handle)
{
    (void)device_handle;
    DWT->CYCCNT = 0U;
    __DSB();
    __ISB();
    return BSP_STATUS_OK;
}

static bsp_status_t board_config_dwt_get_cycle_count(const void *device_handle,
                                                     uint32_t *cycle_count)
{
    (void)device_handle;
    if (cycle_count == NULL)
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    *cycle_count = DWT->CYCCNT;
    return BSP_STATUS_OK;
}

static bsp_status_t board_config_dwt_get_frequency_hz(const void *device_handle,
                                                      uint32_t *frequency_hz)
{
    (void)device_handle;
    if (frequency_hz == NULL)
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    if (SystemCoreClock == 0U)
    {
        return BSP_STATUS_IO_ERROR;
    }
    *frequency_hz = SystemCoreClock;
    return BSP_STATUS_OK;
}

static const bsp_dwt_driver_ops_t board_config_dwt_driver_ops = {
    .init = board_config_dwt_init,
    .reset = board_config_dwt_reset,
    .get_cycle_count = board_config_dwt_get_cycle_count,
    .get_frequency_hz = board_config_dwt_get_frequency_hz,
};

/* ---------- CAN (FDCAN) 驱动实现 ---------- */

/**
 * @brief 将字节长度转换为 FDCAN DLC 编码
 * @param data_length 数据长度（字节）
 * @return FDCAN DLC 编码值
 * @note 0~8 字节直接映射，其他返回 0（实际不会被调用）
 */
static uint32_t board_config_fdcan_data_length(uint8_t data_length)
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
static uint8_t board_config_fdcan_decode_length(uint32_t data_length)
{
    return (uint8_t)((data_length >> 16U) & 0x0FU);
}

/**
 * @brief 启动 FDCAN 外设
 * @param handle FDCAN_HandleTypeDef* 句柄
 * @return 执行状态
 */
static bsp_status_t board_config_can_start(void *handle)
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
    return board_config_status(status);
}

/**
 * @brief 停止 FDCAN 外设
 * @param handle FDCAN_HandleTypeDef* 句柄
 * @return 执行状态
 */
static bsp_status_t board_config_can_stop(void *handle)
{
    return board_config_status(HAL_FDCAN_Stop((FDCAN_HandleTypeDef *)handle));
}

/**
 * @brief 配置 FDCAN 硬件过滤器
 * @param handle FDCAN_HandleTypeDef* 句柄
 * @param filter 过滤器配置
 * @return 执行状态
 */
static bsp_status_t board_config_can_filter(void *handle, const bsp_can_filter_t *filter)
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
    return board_config_status(HAL_FDCAN_ConfigFilter((FDCAN_HandleTypeDef *)handle, &hal_filter));
}

/**
 * @brief 发送 CAN 帧（阻塞，带超时）
 * @param handle FDCAN_HandleTypeDef* 句柄
 * @param frame CAN 帧指针
 * @param timeout_ms 超时时间
 * @return 执行状态
 */
static bsp_status_t board_config_can_transmit(void *handle, const bsp_can_frame_t *frame,
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
    header.DataLength = board_config_fdcan_data_length(frame->data_length);
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
    return board_config_status(HAL_FDCAN_AddMessageToTxFifoQ((FDCAN_HandleTypeDef *)handle, &header,
                                                             (uint8_t *)frame->data));
}

/**
 * @brief 接收 CAN 帧（从指定 FIFO）
 * @param handle FDCAN_HandleTypeDef* 句柄
 * @param receive_fifo FIFO 选择（0 或 1）
 * @param frame 输出 CAN 帧
 * @return 执行状态
 */
static bsp_status_t board_config_can_receive(void *handle, bsp_can_receive_fifo_t receive_fifo,
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
        frame->data_length = board_config_fdcan_decode_length(header.DataLength);
    }
    return board_config_status(status);
}

/**
 * @brief 获取发送 FIFO 空闲数量
 * @param handle FDCAN_HandleTypeDef* 句柄
 * @param free_level 输出空闲数量
 * @return 执行状态
 */
static bsp_status_t board_config_can_free_level(const void *handle, uint32_t *free_level)
{
    if ((handle == NULL) || (free_level == NULL))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    *free_level = HAL_FDCAN_GetTxFifoFreeLevel((FDCAN_HandleTypeDef *)(uintptr_t)handle);
    return BSP_STATUS_OK;
}

/** CAN 驱动操作表（FDCAN） */
static const bsp_can_driver_ops_t board_config_can_driver_ops = {
    .init = board_config_noop_init,
    .deinit = board_config_noop_deinit,
    .start = board_config_can_start,
    .stop = board_config_can_stop,
    .configure_filter = board_config_can_filter,
    .transmit = board_config_can_transmit,
    .receive = board_config_can_receive,
    .get_tx_free_level = board_config_can_free_level,
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
static bsp_status_t board_config_usart_transmit(void *handle, const uint8_t *data, size_t size,
                                                bsp_transfer_mode_t mode, uint32_t timeout_ms)
{
    // 参数校验：数据非空，大小在有效范围内
    if ((data == NULL) || (size == 0U) || (size > UINT16_MAX))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    if (mode == BSP_TRANSFER_MODE_BLOCKING)
    {
        return board_config_status(
            HAL_UART_Transmit((UART_HandleTypeDef *)handle, data, (uint16_t)size, timeout_ms));
    }
    if (mode == BSP_TRANSFER_MODE_INTERRUPT)
    {
        return board_config_status(
            HAL_UART_Transmit_IT((UART_HandleTypeDef *)handle, data, (uint16_t)size));
    }
    // DMA 模式
    return board_config_status(
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
static bsp_status_t board_config_usart_receive(void *handle, uint8_t *data, size_t size,
                                               bsp_transfer_mode_t mode, uint32_t timeout_ms)
{
    if ((data == NULL) || (size == 0U) || (size > UINT16_MAX))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    if (mode == BSP_TRANSFER_MODE_BLOCKING)
    {
        return board_config_status(
            HAL_UART_Receive((UART_HandleTypeDef *)handle, data, (uint16_t)size, timeout_ms));
    }
    if (mode == BSP_TRANSFER_MODE_INTERRUPT)
    {
        return board_config_status(
            HAL_UART_Receive_IT((UART_HandleTypeDef *)handle, data, (uint16_t)size));
    }
    return board_config_status(
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
static bsp_status_t board_config_usart_receive_to_idle(void *handle, uint8_t *data, size_t capacity,
                                                       bsp_transfer_mode_t mode,
                                                       uint32_t timeout_ms)
{
    if ((data == NULL) || (capacity == 0U) || (capacity > UINT16_MAX))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    if (mode == BSP_TRANSFER_MODE_BLOCKING)
    {
        uint16_t received_size = 0U;
        return board_config_status(HAL_UARTEx_ReceiveToIdle(
            (UART_HandleTypeDef *)handle, data, (uint16_t)capacity, &received_size, timeout_ms));
    }
    if (mode == BSP_TRANSFER_MODE_INTERRUPT)
    {
        return board_config_status(
            HAL_UARTEx_ReceiveToIdle_IT((UART_HandleTypeDef *)handle, data, (uint16_t)capacity));
    }
    return board_config_status(
        HAL_UARTEx_ReceiveToIdle_DMA((UART_HandleTypeDef *)handle, data, (uint16_t)capacity));
}

static bsp_status_t board_config_usart_receive_to_idle_double_buffer(void *handle,
                                                                     uint8_t *first_buffer,
                                                                     uint8_t *second_buffer,
                                                                     size_t buffer_capacity)
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
        uart->hdmarx, (uint32_t)(uintptr_t)&uart->Instance->RDR, (uint32_t)(uintptr_t)first_buffer,
        (uint32_t)(uintptr_t)second_buffer, (uint32_t)buffer_capacity);
    if (hal_status != HAL_OK)
    {
        __HAL_UART_DISABLE_IT(uart, UART_IT_IDLE);
        uart->RxState = HAL_UART_STATE_READY;
        return board_config_status(hal_status);
    }
    SET_BIT(uart->Instance->CR3, USART_CR3_DMAR);
    return BSP_STATUS_OK;
}

/**
 * @brief USART 中止当前传输
 * @param handle UART_HandleTypeDef* 句柄
 * @return 执行状态
 */
static bsp_status_t board_config_usart_abort(void *handle)
{
    return board_config_status(HAL_UART_Abort((UART_HandleTypeDef *)handle));
}

/**
 * @brief 查询 USART 是否忙
 * @param handle UART_HandleTypeDef* 句柄
 * @param is_busy 输出是否忙
 * @return 执行状态
 */
static bsp_status_t board_config_usart_busy(const void *handle, bool *is_busy)
{
    if ((handle == NULL) || (is_busy == NULL))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    *is_busy = HAL_UART_GetState((UART_HandleTypeDef *)(uintptr_t)handle) != HAL_UART_STATE_READY;
    return BSP_STATUS_OK;
}

/** USART 驱动操作表 */
static const bsp_usart_driver_ops_t board_config_usart_driver_ops = {
    .init = board_config_noop_init,
    .deinit = board_config_noop_deinit,
    .transmit = board_config_usart_transmit,
    .receive = board_config_usart_receive,
    .receive_to_idle = board_config_usart_receive_to_idle,
    .receive_to_idle_double_buffer = board_config_usart_receive_to_idle_double_buffer,
    .abort = board_config_usart_abort,
    .get_busy = board_config_usart_busy,
};

/* ---------- SPI 驱动实现 ---------- */

/**
 * @brief SPI 发送（支持三种模式）
 */
static bsp_status_t board_config_spi_transmit(void *handle, const uint8_t *data, size_t size,
                                              bsp_transfer_mode_t mode, uint32_t timeout_ms)
{
    if ((data == NULL) || (size == 0U) || (size > UINT16_MAX))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    if (mode == BSP_TRANSFER_MODE_BLOCKING)
    {
        return board_config_status(
            HAL_SPI_Transmit((SPI_HandleTypeDef *)handle, data, (uint16_t)size, timeout_ms));
    }
    if (mode == BSP_TRANSFER_MODE_INTERRUPT)
    {
        return board_config_status(
            HAL_SPI_Transmit_IT((SPI_HandleTypeDef *)handle, data, (uint16_t)size));
    }
    return board_config_status(
        HAL_SPI_Transmit_DMA((SPI_HandleTypeDef *)handle, data, (uint16_t)size));
}

/**
 * @brief SPI 接收（支持三种模式）
 */
static bsp_status_t board_config_spi_receive(void *handle, uint8_t *data, size_t size,
                                             bsp_transfer_mode_t mode, uint32_t timeout_ms)
{
    if ((data == NULL) || (size == 0U) || (size > UINT16_MAX))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    if (mode == BSP_TRANSFER_MODE_BLOCKING)
    {
        return board_config_status(
            HAL_SPI_Receive((SPI_HandleTypeDef *)handle, data, (uint16_t)size, timeout_ms));
    }
    if (mode == BSP_TRANSFER_MODE_INTERRUPT)
    {
        return board_config_status(
            HAL_SPI_Receive_IT((SPI_HandleTypeDef *)handle, data, (uint16_t)size));
    }
    return board_config_status(
        HAL_SPI_Receive_DMA((SPI_HandleTypeDef *)handle, data, (uint16_t)size));
}

/**
 * @brief SPI 全双工交换（支持三种模式）
 */
static bsp_status_t board_config_spi_exchange(void *handle, const uint8_t *transmit_data,
                                              uint8_t *receive_data, size_t size,
                                              bsp_transfer_mode_t mode, uint32_t timeout_ms)
{
    if ((transmit_data == NULL) || (receive_data == NULL) || (size == 0U) || (size > UINT16_MAX))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    if (mode == BSP_TRANSFER_MODE_BLOCKING)
    {
        return board_config_status(HAL_SPI_TransmitReceive(
            (SPI_HandleTypeDef *)handle, transmit_data, receive_data, (uint16_t)size, timeout_ms));
    }
    if (mode == BSP_TRANSFER_MODE_INTERRUPT)
    {
        return board_config_status(HAL_SPI_TransmitReceive_IT(
            (SPI_HandleTypeDef *)handle, transmit_data, receive_data, (uint16_t)size));
    }
    return board_config_status(HAL_SPI_TransmitReceive_DMA(
        (SPI_HandleTypeDef *)handle, transmit_data, receive_data, (uint16_t)size));
}

/**
 * @brief SPI 中止当前传输
 */
static bsp_status_t board_config_spi_abort(void *handle)
{
    return board_config_status(HAL_SPI_Abort((SPI_HandleTypeDef *)handle));
}

/**
 * @brief 查询 SPI 是否忙
 */
static bsp_status_t board_config_spi_busy(const void *handle, bool *is_busy)
{
    if ((handle == NULL) || (is_busy == NULL))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    *is_busy = HAL_SPI_GetState((SPI_HandleTypeDef *)(uintptr_t)handle) != HAL_SPI_STATE_READY;
    return BSP_STATUS_OK;
}

static const bsp_spi_driver_ops_t board_config_spi_driver_ops = {
    .init = board_config_noop_init,
    .deinit = board_config_noop_deinit,
    .transmit = board_config_spi_transmit,
    .receive = board_config_spi_receive,
    .exchange = board_config_spi_exchange,
    .abort = board_config_spi_abort,
    .get_busy = board_config_spi_busy,
};

/* ---------- EXTI 驱动实现 ---------- */

static bsp_status_t board_config_exti_init(void *handle)
{
    return (handle != NULL) ? BSP_STATUS_OK : BSP_STATUS_INVALID_ARGUMENT;
}

static bsp_status_t board_config_exti_enable(void *handle)
{
    const board_config_exti_context_t *const context = (const board_config_exti_context_t *)handle;
    if (context == NULL)
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    HAL_NVIC_EnableIRQ(context->interrupt_number);
    return BSP_STATUS_OK;
}

static bsp_status_t board_config_exti_disable(void *handle)
{
    const board_config_exti_context_t *const context = (const board_config_exti_context_t *)handle;
    if (context == NULL)
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    HAL_NVIC_DisableIRQ(context->interrupt_number);
    return BSP_STATUS_OK;
}

static const bsp_exti_driver_ops_t board_config_exti_driver_ops = {
    .init = board_config_exti_init,
    .deinit = board_config_noop_deinit,
    .enable = board_config_exti_enable,
    .disable = board_config_exti_disable,
};

/* ---------- PWM 驱动实现 ---------- */

static uint32_t board_config_pwm_channel(uint32_t channel)
{
    static const uint32_t channels[] = {0U, TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3,
                                        TIM_CHANNEL_4};
    return (channel <= 4U) ? channels[channel] : 0U;
}

static board_config_pwm_context_t *board_config_pwm_context(void *handle)
{
    return (board_config_pwm_context_t *)handle;
}

static const board_config_pwm_context_t *board_config_pwm_const_context(const void *handle)
{
    return (const board_config_pwm_context_t *)handle;
}

static bsp_status_t board_config_pwm_init(void *handle, uint32_t channel)
{
    const board_config_pwm_context_t *const context = board_config_pwm_const_context(handle);
    return ((context != NULL) && (context->timer != NULL) && (context->timer_clock_hz > 0U) &&
            (board_config_pwm_channel(channel) != 0U))
               ? BSP_STATUS_OK
               : BSP_STATUS_INVALID_ARGUMENT;
}

static bsp_status_t board_config_pwm_start(void *handle, uint32_t channel)
{
    board_config_pwm_context_t *const context = board_config_pwm_context(handle);
    return board_config_status(
        HAL_TIM_PWM_Start(context->timer, board_config_pwm_channel(channel)));
}

static bsp_status_t board_config_pwm_stop(void *handle, uint32_t channel)
{
    board_config_pwm_context_t *const context = board_config_pwm_context(handle);
    return board_config_status(HAL_TIM_PWM_Stop(context->timer, board_config_pwm_channel(channel)));
}

static bsp_status_t board_config_pwm_set_frequency(void *handle, uint32_t channel,
                                                   uint32_t frequency_hz)
{
    board_config_pwm_context_t *const context = board_config_pwm_context(handle);
    TIM_HandleTypeDef *const timer = context->timer;
    uint32_t period_ticks;
    (void)channel;
    if (frequency_hz == 0U)
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    period_ticks = context->timer_clock_hz / ((timer->Init.Prescaler + 1U) * frequency_hz);
    if ((period_ticks == 0U) || (period_ticks > 65536U))
    {
        return BSP_STATUS_OUT_OF_RANGE;
    }
    __HAL_TIM_SET_AUTORELOAD(timer, period_ticks - 1U);
    __HAL_TIM_SET_COUNTER(timer, 0U);
    return BSP_STATUS_OK;
}

static bsp_status_t board_config_pwm_get_frequency(const void *handle, uint32_t channel,
                                                   uint32_t *frequency_hz)
{
    const board_config_pwm_context_t *const context = board_config_pwm_const_context(handle);
    const TIM_HandleTypeDef *const timer = context->timer;
    (void)channel;
    if (frequency_hz == NULL)
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    *frequency_hz =
        context->timer_clock_hz / ((timer->Init.Prescaler + 1U) * (timer->Instance->ARR + 1U));
    return BSP_STATUS_OK;
}

static bsp_status_t board_config_pwm_set_pulse(void *handle, uint32_t channel, uint32_t pulse_ticks)
{
    board_config_pwm_context_t *const context = board_config_pwm_context(handle);
    TIM_HandleTypeDef *const timer = context->timer;
    if (pulse_ticks > timer->Instance->ARR + 1U)
    {
        return BSP_STATUS_OUT_OF_RANGE;
    }
    __HAL_TIM_SET_COMPARE(timer, board_config_pwm_channel(channel), pulse_ticks);
    return BSP_STATUS_OK;
}

static bsp_status_t board_config_pwm_get_pulse(const void *handle, uint32_t channel,
                                               uint32_t *pulse_ticks)
{
    const board_config_pwm_context_t *const context = board_config_pwm_const_context(handle);
    if (pulse_ticks == NULL)
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    *pulse_ticks = __HAL_TIM_GET_COMPARE(context->timer, board_config_pwm_channel(channel));
    return BSP_STATUS_OK;
}

static bsp_status_t board_config_pwm_get_period(const void *handle, uint32_t channel,
                                                uint32_t *period_ticks)
{
    const board_config_pwm_context_t *const context = board_config_pwm_const_context(handle);
    (void)channel;
    if (period_ticks == NULL)
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    *period_ticks = context->timer->Instance->ARR + 1U;
    return BSP_STATUS_OK;
}

static const bsp_pwm_driver_ops_t board_config_pwm_driver_ops = {
    .init = board_config_pwm_init,
    .deinit = board_config_pwm_stop,
    .start = board_config_pwm_start,
    .stop = board_config_pwm_stop,
    .set_frequency = board_config_pwm_set_frequency,
    .get_frequency = board_config_pwm_get_frequency,
    .set_pulse = board_config_pwm_set_pulse,
    .get_pulse = board_config_pwm_get_pulse,
    .get_period = board_config_pwm_get_period,
};

/* ---------- Watchdog 驱动实现 ---------- */

/**
 * @brief 初始化独立看门狗（IWDG1）
 * @param handle 看门狗上下文指针
 * @return 执行状态
 */
static bsp_status_t board_config_watchdog_init(void *handle)
{
    board_config_watchdog_context_t *const context = (board_config_watchdog_context_t *)handle;
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
    } // 等待寄存器更新完成
    IWDG1->KR = 0xCCCCU; // 启动看门狗
    return BSP_STATUS_OK;
}

/**
 * @brief 刷新看门狗（喂狗）
 */
static bsp_status_t board_config_watchdog_refresh(void *handle)
{
    (void)handle;
    IWDG1->KR = 0xAAAAU; // 重载计数器
    return BSP_STATUS_OK;
}

/**
 * @brief 获取看门狗超时时间
 */
static bsp_status_t board_config_watchdog_timeout(const void *handle, uint32_t *timeout_ms)
{
    if ((handle == NULL) || (timeout_ms == NULL))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    *timeout_ms = ((const board_config_watchdog_context_t *)handle)->timeout_ms;
    return BSP_STATUS_OK;
}

/**
 * @brief 检测是否由看门狗复位
 */
static bsp_status_t board_config_watchdog_reset_detected(const void *handle, bool *reset_detected)
{
    if ((handle == NULL) || (reset_detected == NULL))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    *reset_detected = ((const board_config_watchdog_context_t *)handle)->reset_detected;
    return BSP_STATUS_OK;
}

/** Watchdog 驱动操作表 */
static const bsp_watchdog_driver_ops_t board_config_watchdog_driver_ops = {
    .init = board_config_watchdog_init,
    .deinit = board_config_noop_deinit,
    .refresh = board_config_watchdog_refresh,
    .get_timeout_ms = board_config_watchdog_timeout,
    .get_reset_detected = board_config_watchdog_reset_detected,
};

static const bsp_can_driver_ops_t *board_config_get_can_driver_ops(void)
{
    return &board_config_can_driver_ops;
}

static const bsp_usart_driver_ops_t *board_config_get_usart_driver_ops(void)
{
    return &board_config_usart_driver_ops;
}

static const bsp_spi_driver_ops_t *board_config_get_spi_driver_ops(void)
{
    return &board_config_spi_driver_ops;
}

static const bsp_exti_driver_ops_t *board_config_get_exti_driver_ops(void)
{
    return &board_config_exti_driver_ops;
}

static const bsp_pwm_driver_ops_t *board_config_get_pwm_driver_ops(void)
{
    return &board_config_pwm_driver_ops;
}

static const bsp_watchdog_driver_ops_t *board_config_get_watchdog_driver_ops(void)
{
    return &board_config_watchdog_driver_ops;
}

static const bsp_dwt_driver_ops_t *board_config_get_dwt_driver_ops(void)
{
    return &board_config_dwt_driver_ops;
}
#define BOARD_CONFIG_USB_RECEIVE_CAPACITY (512U)
#define BOARD_CONFIG_USB_RECEIVE_QUEUE_DEPTH (4U)
#define BOARD_CONFIG_WATCHDOG_TIMEOUT_MS (2000U)

typedef struct
{
    uint8_t receive_buffer[BOARD_CONFIG_USB_RECEIVE_QUEUE_DEPTH][BOARD_CONFIG_USB_RECEIVE_CAPACITY];
    size_t receive_size[BOARD_CONFIG_USB_RECEIVE_QUEUE_DEPTH];
    volatile uint8_t read_index;
    volatile uint8_t write_index;
    volatile uint8_t pending_count;
    volatile uint32_t overrun_count;
} board_config_usb_context_t;

extern USBD_HandleTypeDef hUsbDeviceHS;

static bsp_can_device_t board_config_can_devices[BOARD_CONFIG_CAN_COUNT];
static bsp_usart_device_t board_config_usart_devices[BOARD_CONFIG_USART_COUNT];
static bsp_spi_device_t board_config_bmi088_spi_device;
static bsp_exti_t board_config_exti_objects[BOARD_CONFIG_EXTI_COUNT];
static bsp_pwm_t board_config_pwm_objects[BOARD_CONFIG_PWM_COUNT];
static bsp_usb_vcp_device_t board_config_usb_device;
static bsp_watchdog_device_t board_config_watchdog_device;
static bsp_dwt_t board_config_dwt;

static board_config_exti_context_t board_config_exti_contexts[BOARD_CONFIG_EXTI_COUNT];
static board_config_pwm_context_t board_config_pwm_contexts[BOARD_CONFIG_PWM_COUNT];
static board_config_watchdog_context_t board_config_watchdog_context;
static board_config_usb_context_t board_config_usb_context;
static bool board_config_initialized;
static bool board_config_watchdog_initialized;

static bsp_status_t board_config_usb_init(void *device_handle)
{
    return (device_handle != NULL) ? BSP_STATUS_OK : BSP_STATUS_INVALID_ARGUMENT;
}

static bsp_status_t board_config_usb_deinit(void *device_handle)
{
    return board_config_usb_init(device_handle);
}

static bsp_status_t board_config_usb_transmit(void *device_handle, const uint8_t *transmit_data,
                                              size_t data_size, uint32_t timeout_ms)
{
    uint32_t started_at_ms = HAL_GetTick();
    uint8_t usb_status;
    (void)device_handle;
    if ((transmit_data == NULL) || (data_size == 0U) || (data_size > UINT16_MAX))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    do
    {
        usb_status = CDC_Transmit_HS((uint8_t *)(uintptr_t)transmit_data, (uint16_t)data_size);
        if (usb_status == USBD_OK)
        {
            return BSP_STATUS_OK;
        }
        if (usb_status != USBD_BUSY)
        {
            return BSP_STATUS_IO_ERROR;
        }
        if ((timeout_ms == 0U) || (__get_IPSR() != 0U) || (osKernelGetState() != osKernelRunning))
        {
            return BSP_STATUS_BUSY;
        }
        if ((HAL_GetTick() - started_at_ms) >= timeout_ms)
        {
            return BSP_STATUS_TIMEOUT;
        }
        (void)osDelay(1U);
    } while (true);
}

static bsp_status_t board_config_usb_receive(void *device_handle, uint8_t *receive_data,
                                             size_t data_capacity)
{
    board_config_usb_context_t *const context = (board_config_usb_context_t *)device_handle;
    uint32_t interrupt_state;
    uint8_t read_index;
    size_t copy_size;
    if ((context == NULL) || (receive_data == NULL) || (data_capacity == 0U))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    interrupt_state = __get_PRIMASK();
    __disable_irq();
    if (context->pending_count == 0U)
    {
        __set_PRIMASK(interrupt_state);
        return BSP_STATUS_BUSY;
    }
    read_index = context->read_index;
    copy_size = (context->receive_size[read_index] < data_capacity)
                    ? context->receive_size[read_index]
                    : data_capacity;
    memcpy(receive_data, context->receive_buffer[read_index], copy_size);
    context->receive_size[read_index] = 0U;
    context->read_index = (uint8_t)((read_index + 1U) % BOARD_CONFIG_USB_RECEIVE_QUEUE_DEPTH);
    --context->pending_count;
    __set_PRIMASK(interrupt_state);
    return BSP_STATUS_OK;
}

static bsp_status_t board_config_usb_abort(void *device_handle)
{
    board_config_usb_context_t *const context = (board_config_usb_context_t *)device_handle;
    if (context == NULL)
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    context->read_index = 0U;
    context->write_index = 0U;
    context->pending_count = 0U;
    (void)memset(context->receive_size, 0, sizeof(context->receive_size));
    return BSP_STATUS_OK;
}

static bsp_status_t board_config_usb_get_connected(const void *device_handle, bool *is_connected)
{
    (void)device_handle;
    if (is_connected == NULL)
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    *is_connected = hUsbDeviceHS.dev_state == USBD_STATE_CONFIGURED;
    return BSP_STATUS_OK;
}

static bsp_status_t board_config_usb_get_busy(const void *device_handle, bool *is_busy)
{
    const USBD_CDC_HandleTypeDef *class_data;
    (void)device_handle;
    if (is_busy == NULL)
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    class_data = (const USBD_CDC_HandleTypeDef *)hUsbDeviceHS.pClassData;
    *is_busy = (class_data != NULL) && (class_data->TxState != 0U);
    return BSP_STATUS_OK;
}

static const bsp_usb_vcp_driver_ops_t board_config_usb_driver_ops = {
    .init = board_config_usb_init,
    .deinit = board_config_usb_deinit,
    .transmit = board_config_usb_transmit,
    .receive = board_config_usb_receive,
    .abort = board_config_usb_abort,
    .get_connected = board_config_usb_get_connected,
    .get_busy = board_config_usb_get_busy,
};

/* ---------- 各外设回滚辅助函数 ---------- */

static void board_config_deinit_exti(void)
{
    for (size_t i = 0U; i < BOARD_CONFIG_EXTI_COUNT; ++i)
    {
        (void)bsp_exti_deinit(&board_config_exti_objects[i]);
    }
}

static void board_config_deinit_pwm(void)
{
    for (size_t i = 0U; i < BOARD_CONFIG_PWM_COUNT; ++i)
    {
        (void)bsp_pwm_deinit(&board_config_pwm_objects[i]);
    }
}

static void board_config_deinit_can(size_t initialized_count)
{
    /* 逆序停止：最后初始化的 CAN 最先停止 */
    while (initialized_count > 0U)
    {
        --initialized_count;
        (void)HAL_FDCAN_Stop((FDCAN_HandleTypeDef *)bsp_device_get_handle(
            &board_config_can_devices[initialized_count].super.super));
        (void)bsp_device_deinit(&board_config_can_devices[initialized_count].super.super);
    }
}

/**
 * @brief 板级 BSP 初始化（带回滚）
 *
 * 按顺序初始化 CAN → USART → SPI → EXTI → PWM → USB VCP → DWT → Watchdog。
 * 任一步骤失败时，逆序回滚已初始化的外设，并将错误码和步骤名写入 config。
 *
 * @param config 初始化配置（非 const，失败时写入诊断信息）
 * @return BSP_STATUS_OK 成功，其他值表示失败
 */
bsp_status_t board_config_init(board_config_init_t *config)
{
    bsp_status_t status = BSP_STATUS_OK;
    size_t can_count = 0U;
    bool usart_ok = false;
    bool spi_ok = false;
    bool exti_ok = false;
    bool pwm_ok = false;
    bool usb_ok = false;
    bool wdt_ok = false;

    if ((config == NULL) || board_config_initialized)
    {
        return (config == NULL) ? BSP_STATUS_INVALID_ARGUMENT : BSP_STATUS_BUSY;
    }
    config->last_error = BSP_STATUS_OK;
    config->failed_step = NULL;

    /* ---- CAN (x3, 完整 OOP) ---- */
    if (status == BSP_STATUS_OK)
    {
        FDCAN_HandleTypeDef *const handles[BOARD_CONFIG_CAN_COUNT] = {
            &hfdcan1, &hfdcan2, &hfdcan3,
        };
        for (size_t i = 0U; (i < BOARD_CONFIG_CAN_COUNT) && (status == BSP_STATUS_OK); ++i)
        {
            const bsp_can_config_t cfg = {
                .device_handle = handles[i],
                .driver_ops = board_config_get_can_driver_ops(),
            };
            status = bsp_can_init(&board_config_can_devices[i], &cfg);
            if (status != BSP_STATUS_OK)
            {
                config->failed_step = "can";
            }
            else
            {
                can_count = i + 1U;
            }
        }
    }

    /* ---- USART (完整 OOP) ---- */
    if (status == BSP_STATUS_OK)
    {
        const bsp_usart_config_t cfg = {
            .device_handle = &huart5,
            .driver_ops = board_config_get_usart_driver_ops(),
        };
        status = bsp_usart_init(&board_config_usart_devices[0], &cfg);
        if (status != BSP_STATUS_OK)
        {
            config->failed_step = "usart";
        }
        else
        {
            usart_ok = true;
        }
    }

    /* ---- SPI (完整 OOP) ---- */
    if (status == BSP_STATUS_OK)
    {
        const bsp_spi_config_t cfg = {
            .device_handle = &hspi2,
            .driver_ops = board_config_get_spi_driver_ops(),
        };
        status = bsp_spi_init(&board_config_bmi088_spi_device, &cfg);
        if (status != BSP_STATUS_OK)
        {
            config->failed_step = "spi";
        }
        else
        {
            spi_ok = true;
        }
    }

    /* ---- EXTI (单例 dispatcher) ---- */
    if (status == BSP_STATUS_OK)
    {
        board_config_exti_contexts[BOARD_CONFIG_EXTI_BMI088_GYROSCOPE] =
            (board_config_exti_context_t){
                .pin = BMI088_GYRO_INT_Pin,
                .interrupt_number = BMI088_GYRO_INT_EXTI_IRQn,
            };
        board_config_exti_contexts[BOARD_CONFIG_EXTI_BMI088_ACCELEROMETER] =
            (board_config_exti_context_t){
                .pin = BMI088_ACCEL_INT_Pin,
                .interrupt_number = BMI088_ACCEL_INT_EXTI_IRQn,
            };
        for (size_t i = 0U; (i < BOARD_CONFIG_EXTI_COUNT) && (status == BSP_STATUS_OK); ++i)
        {
            const bsp_exti_config_t cfg = {
                .device_handle = &board_config_exti_contexts[i],
                .driver_ops = board_config_get_exti_driver_ops(),
            };
            status = bsp_exti_init(&board_config_exti_objects[i], &cfg);
            if (status != BSP_STATUS_OK)
            {
                config->failed_step = "exti";
            }
        }
        if (status == BSP_STATUS_OK)
        {
            exti_ok = true;
        }
    }

    /* ---- PWM (单例 dispatcher，表驱动多通道) ---- */
    if (status == BSP_STATUS_OK)
    {
        /*
         * PWM 通道描述表：每个条目描述一个逻辑 PWM 外设的硬件绑定。
         * 添加舵机或其他 PWM 外设时，在此表中追加条目并更新
         * board_config.h 中的 board_config_pwm_index_t 枚举即可。
         */
        typedef struct
        {
            TIM_HandleTypeDef *timer;
            uint32_t channel;
            uint32_t timer_clock_hz;
        } board_config_pwm_entry_t;

        const board_config_pwm_entry_t entries[BOARD_CONFIG_PWM_COUNT] = {
            [BOARD_CONFIG_PWM_BUZZER] = {&htim1, 1U, BOARD_CONFIG_APB_FREQUENCY_HZ * 2UL},
            /* [BOARD_CONFIG_PWM_SERVO_1] = {&htim2, 1U, BOARD_CONFIG_APB_FREQUENCY_HZ * 2UL}, */
            /* [BOARD_CONFIG_PWM_SERVO_2] = {&htim2, 2U, BOARD_CONFIG_APB_FREQUENCY_HZ * 2UL}, */
        };
        for (size_t i = 0U; (i < BOARD_CONFIG_PWM_COUNT) && (status == BSP_STATUS_OK); ++i)
        {
            board_config_pwm_contexts[i] = (board_config_pwm_context_t){
                .timer = entries[i].timer,
                .timer_clock_hz = entries[i].timer_clock_hz,
            };
            const bsp_pwm_config_t cfg = {
                .device_handle = &board_config_pwm_contexts[i],
                .driver_ops = board_config_get_pwm_driver_ops(),
                .channel = entries[i].channel,
            };
            status = bsp_pwm_init(&board_config_pwm_objects[i], &cfg);
            if (status != BSP_STATUS_OK)
            {
                config->failed_step = "pwm";
            }
        }
        pwm_ok = (status == BSP_STATUS_OK);
    }

    /* ---- USB VCP (完整 OOP) ---- */
    if (status == BSP_STATUS_OK)
    {
        const bsp_usb_vcp_config_t cfg = {
            .device_handle = &board_config_usb_context,
            .driver_ops = &board_config_usb_driver_ops,
        };
        status = bsp_usb_vcp_init(&board_config_usb_device, &cfg);
        if (status != BSP_STATUS_OK)
        {
            config->failed_step = "usb_vcp";
        }
        else
        {
            usb_ok = true;
        }
    }

    /* ---- DWT (平铺结构体) ---- */
    if (status == BSP_STATUS_OK)
    {
        const bsp_dwt_config_t cfg = {
            .device_handle = NULL,
            .driver_ops = board_config_get_dwt_driver_ops(),
        };
        status = bsp_dwt_init(&board_config_dwt, &cfg);
        if (status != BSP_STATUS_OK)
        {
            config->failed_step = "dwt";
        }
    }

    /* ---- Watchdog (可选, 完整 OOP) ---- */
    if ((status == BSP_STATUS_OK) && config->initialize_watchdog)
    {
        const bsp_watchdog_config_t wdt_cfg = {
            .device_handle = &board_config_watchdog_context,
            .driver_ops = board_config_get_watchdog_driver_ops(),
        };
        board_config_watchdog_context.timeout_ms = BOARD_CONFIG_WATCHDOG_TIMEOUT_MS;
        status = bsp_watchdog_init(&board_config_watchdog_device, &wdt_cfg);
        if (status != BSP_STATUS_OK)
        {
            config->failed_step = "watchdog";
        }
        else
        {
            wdt_ok = true;
        }
    }

    /* ---- 成功 / 回滚 ---- */
    if (status == BSP_STATUS_OK)
    {
        board_config_watchdog_initialized = wdt_ok;
        board_config_initialized = true;
        return BSP_STATUS_OK;
    }

    config->last_error = status;
    bsp_error_record(status, config->failed_step, 0);
    /*
     * 逆序回滚已初始化的外设。
     *
     * 回滚分两层：
     *   1. HAL_Stop / HAL_Abort — 停止硬件（DMA、中断、时钟）
     *   2. bsp_device_deinit / bsp_xxx_deinit — 清理 BSP 对象状态
     *
     * 两层职责分离：driver_ops->deinit 保持为空（CubeMX 管理 HAL 生命周期），
     * 硬件停止由本文件的显式 HAL 调用来完成。
     */
    if (wdt_ok)
    {
        /* IWDG 一旦启动无法停止 */
        board_config_watchdog_initialized = false;
    }
    /* DWT 无需硬件停止 */
    board_config_dwt.is_initialized = false;
    if (usb_ok)
    {
        (void)bsp_device_deinit(&board_config_usb_device.super.super);
    }
    if (pwm_ok)
    {
        board_config_deinit_pwm();
    }
    if (exti_ok)
    {
        board_config_deinit_exti();
    }
    if (spi_ok)
    {
        (void)HAL_SPI_Abort(&hspi2);
        (void)bsp_device_deinit(&board_config_bmi088_spi_device.super.super);
    }
    if (usart_ok)
    {
        (void)HAL_UART_Abort(&huart5);
        (void)bsp_device_deinit(&board_config_usart_devices[0].super.super);
    }
    board_config_deinit_can(can_count);
    return status;
}

bsp_can_t *board_config_get_can(board_config_can_index_t index)
{
    return (board_config_initialized && (index < BOARD_CONFIG_CAN_COUNT))
               ? bsp_can_as_base(&board_config_can_devices[index])
               : NULL;
}

bsp_usart_t *board_config_get_usart(board_config_usart_index_t index)
{
    return (board_config_initialized && (index < BOARD_CONFIG_USART_COUNT))
               ? bsp_usart_as_base(&board_config_usart_devices[index])
               : NULL;
}

bsp_spi_t *board_config_get_bmi088_spi(void)
{
    return board_config_initialized ? bsp_spi_as_base(&board_config_bmi088_spi_device) : NULL;
}

bsp_exti_t *board_config_get_exti(board_config_exti_index_t index)
{
    return (board_config_initialized && (index < BOARD_CONFIG_EXTI_COUNT))
               ? &board_config_exti_objects[index]
               : NULL;
}

bsp_pwm_t *board_config_get_pwm(board_config_pwm_index_t index)
{
    return (board_config_initialized && (index < BOARD_CONFIG_PWM_COUNT))
               ? &board_config_pwm_objects[index]
               : NULL;
}

bsp_usb_vcp_t *board_config_get_usb_vcp(void)
{
    return board_config_initialized ? bsp_usb_vcp_as_base(&board_config_usb_device) : NULL;
}

bsp_watchdog_t *board_config_get_watchdog(void)
{
    return (board_config_initialized && board_config_watchdog_initialized)
               ? bsp_watchdog_as_base(&board_config_watchdog_device)
               : NULL;
}

bsp_dwt_t *board_config_get_dwt(void)
{
    return board_config_initialized ? &board_config_dwt : NULL;
}

bool board_config_is_initialized(void)
{
    return board_config_initialized;
}

static bsp_usart_t *board_config_find_usart(UART_HandleTypeDef *uart)
{
    size_t index;
    for (index = 0U; index < BOARD_CONFIG_USART_COUNT; ++index)
    {
        if (bsp_device_get_handle(&board_config_usart_devices[index].super.super) == uart)
        {
            return &board_config_usart_devices[index].super;
        }
    }
    return NULL;
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *fdcan, uint32_t interrupt_flags)
{
    size_t index;
    (void)interrupt_flags;
    for (index = 0U; index < BOARD_CONFIG_CAN_COUNT; ++index)
    {
        if (bsp_device_get_handle(&board_config_can_devices[index].super.super) == fdcan)
        {
            bsp_can_notify(&board_config_can_devices[index].super, BSP_EVENT_RECEIVE_PENDING,
                           BSP_STATUS_OK, 0U);
            break;
        }
    }
}

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *fdcan, uint32_t interrupt_flags)
{
    HAL_FDCAN_RxFifo0Callback(fdcan, interrupt_flags);
}

void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *fdcan, uint32_t error_status)
{
    size_t index;
    (void)error_status;
    for (index = 0U; index < BOARD_CONFIG_CAN_COUNT; ++index)
    {
        if (bsp_device_get_handle(&board_config_can_devices[index].super.super) == fdcan)
        {
            bsp_can_notify(&board_config_can_devices[index].super, BSP_EVENT_ERROR,
                           BSP_STATUS_IO_ERROR, 0U);
            break;
        }
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *uart)
{
    bsp_usart_t *const usart = board_config_find_usart(uart);
    if (usart != NULL)
    {
        bsp_usart_notify(usart, BSP_EVENT_TRANSMIT_COMPLETE, BSP_STATUS_OK, uart->TxXferSize);
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart)
{
    bsp_usart_t *const usart = board_config_find_usart(uart);
    if (usart != NULL)
    {
        bsp_usart_notify(usart, BSP_EVENT_RECEIVE_COMPLETE, BSP_STATUS_OK, uart->RxXferSize);
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *uart, uint16_t received_size)
{
    bsp_usart_t *const usart = board_config_find_usart(uart);
    if (usart != NULL)
    {
        DMA_Stream_TypeDef *const dma_stream =
            (uart->hdmarx != NULL) ? (DMA_Stream_TypeDef *)uart->hdmarx->Instance : NULL;
        if ((dma_stream != NULL) && ((dma_stream->CR & DMA_SxCR_DBM) != 0U))
        {
            const uint8_t completed_buffer_index = ((dma_stream->CR & DMA_SxCR_CT) != 0U) ? 1U : 0U;
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

void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
    bsp_usart_t *const usart = board_config_find_usart(uart);
    if (usart != NULL)
    {
        bsp_usart_notify(usart, BSP_EVENT_ERROR, BSP_STATUS_IO_ERROR, 0U);
    }
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *spi)
{
    if (spi == &hspi2)
    {
        bsp_spi_notify(&board_config_bmi088_spi_device.super, BSP_EVENT_TRANSFER_COMPLETE,
                       BSP_STATUS_OK, spi->TxXferSize);
    }
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *spi)
{
    if (spi == &hspi2)
    {
        bsp_spi_notify(&board_config_bmi088_spi_device.super, BSP_EVENT_TRANSMIT_COMPLETE,
                       BSP_STATUS_OK, spi->TxXferSize);
    }
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *spi)
{
    if (spi == &hspi2)
    {
        bsp_spi_notify(&board_config_bmi088_spi_device.super, BSP_EVENT_RECEIVE_COMPLETE,
                       BSP_STATUS_OK, spi->RxXferSize);
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t pin)
{
    size_t index;
    for (index = 0U; index < BOARD_CONFIG_EXTI_COUNT; ++index)
    {
        if (board_config_exti_contexts[index].pin == pin)
        {
            bsp_exti_notify(&board_config_exti_objects[index]);
        }
    }
}

void usb_cdc_receive_callback(const uint8_t *receive_data, uint32_t receive_size)
{
    size_t copy_size;
    uint8_t write_index;
    if ((receive_data == NULL) || !board_config_initialized)
    {
        return;
    }
    if (board_config_usb_context.pending_count >= BOARD_CONFIG_USB_RECEIVE_QUEUE_DEPTH)
    {
        if (board_config_usb_context.overrun_count != UINT32_MAX)
        {
            ++board_config_usb_context.overrun_count;
        }
        bsp_usb_vcp_notify(&board_config_usb_device.super, BSP_EVENT_ERROR, BSP_STATUS_NO_RESOURCE,
                           receive_size);
        return;
    }
    copy_size = (receive_size < BOARD_CONFIG_USB_RECEIVE_CAPACITY)
                    ? (size_t)receive_size
                    : BOARD_CONFIG_USB_RECEIVE_CAPACITY;
    write_index = board_config_usb_context.write_index;
    memcpy(board_config_usb_context.receive_buffer[write_index], receive_data, copy_size);
    board_config_usb_context.receive_size[write_index] = copy_size;
    board_config_usb_context.write_index =
        (uint8_t)((write_index + 1U) % BOARD_CONFIG_USB_RECEIVE_QUEUE_DEPTH);
    ++board_config_usb_context.pending_count;
    bsp_usb_vcp_notify(&board_config_usb_device.super, BSP_EVENT_RECEIVE_PENDING, BSP_STATUS_OK,
                       copy_size);
}
