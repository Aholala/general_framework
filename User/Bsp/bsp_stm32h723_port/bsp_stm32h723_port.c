#include "bsp_stm32h723_port.h"

#include "board_config.h"
#include "fdcan.h"
#include "main.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"

#include <string.h>

#define BSP_STM32H723_USB_RECEIVE_CAPACITY (512U)

extern USBD_HandleTypeDef hUsbDeviceFS;

typedef struct
{
    uint16_t pin;
    IRQn_Type interrupt_number;
} bsp_stm32h723_exti_context_t;

typedef struct
{
    uint32_t timeout_ms;
    bool reset_detected;
} bsp_stm32h723_watchdog_context_t;

typedef struct
{
    uint8_t receive_buffer[BSP_STM32H723_USB_RECEIVE_CAPACITY];
    volatile size_t receive_size;
    volatile bool receive_pending;
} bsp_stm32h723_usb_context_t;

static bsp_can_device_t bsp_stm32h723_can_devices[BSP_STM32H723_CAN_COUNT];
static bsp_usart_device_t bsp_stm32h723_usart_devices[BSP_STM32H723_USART_COUNT];
static bsp_spi_device_t bsp_stm32h723_bmi088_spi_device;
static bsp_exti_device_t bsp_stm32h723_exti_devices[BSP_STM32H723_EXTI_COUNT];
static bsp_pwm_device_t bsp_stm32h723_pwm_devices[BSP_STM32H723_PWM_COUNT];
static bsp_usb_vcp_device_t bsp_stm32h723_usb_device;
static bsp_timebase_device_t bsp_stm32h723_timebase_device;
static bsp_watchdog_device_t bsp_stm32h723_watchdog_device;
static bsp_stm32h723_exti_context_t bsp_stm32h723_exti_contexts[BSP_STM32H723_EXTI_COUNT];
static bsp_stm32h723_usb_context_t bsp_stm32h723_usb_context;
static bsp_stm32h723_watchdog_context_t bsp_stm32h723_watchdog_context;
static bool bsp_stm32h723_initialized;
static bool bsp_stm32h723_watchdog_initialized;

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

static bsp_status_t bsp_stm32h723_noop_init(void *handle)
{
    return (handle != NULL) ? BSP_STATUS_OK : BSP_STATUS_INVALID_ARGUMENT;
}

static bsp_status_t bsp_stm32h723_noop_deinit(void *handle)
{
    return (handle != NULL) ? BSP_STATUS_OK : BSP_STATUS_INVALID_ARGUMENT;
}

static uint32_t bsp_stm32h723_fdcan_data_length(uint8_t data_length)
{
    static const uint32_t lengths[] = {
        FDCAN_DLC_BYTES_0, FDCAN_DLC_BYTES_1, FDCAN_DLC_BYTES_2,
        FDCAN_DLC_BYTES_3, FDCAN_DLC_BYTES_4, FDCAN_DLC_BYTES_5,
        FDCAN_DLC_BYTES_6, FDCAN_DLC_BYTES_7, FDCAN_DLC_BYTES_8,
    };
    return (data_length <= 8U) ? lengths[data_length] : FDCAN_DLC_BYTES_0;
}

static uint8_t bsp_stm32h723_fdcan_decode_length(uint32_t data_length)
{
    return (uint8_t)((data_length >> 16U) & 0x0FU);
}

static bsp_status_t bsp_stm32h723_can_start(void *handle)
{
    FDCAN_HandleTypeDef *const fdcan = (FDCAN_HandleTypeDef *)handle;
    HAL_StatusTypeDef status;
    status = HAL_FDCAN_ConfigGlobalFilter(fdcan, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0,
                                          FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
    if (status == HAL_OK)
    {
        status = HAL_FDCAN_ActivateNotification(
            fdcan,
            FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_RX_FIFO1_NEW_MESSAGE | FDCAN_IT_ERROR_WARNING |
                FDCAN_IT_ERROR_PASSIVE | FDCAN_IT_BUS_OFF,
            0U);
    }
    if (status == HAL_OK)
    {
        status = HAL_FDCAN_Start(fdcan);
    }
    return bsp_stm32h723_status(status);
}

static bsp_status_t bsp_stm32h723_can_stop(void *handle)
{
    return bsp_stm32h723_status(HAL_FDCAN_Stop((FDCAN_HandleTypeDef *)handle));
}

static bsp_status_t bsp_stm32h723_can_filter(void *handle, const bsp_can_filter_t *filter)
{
    FDCAN_FilterTypeDef hal_filter = {0};
    if (filter == NULL)
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    hal_filter.IdType =
        (filter->id_type == BSP_CAN_ID_STANDARD) ? FDCAN_STANDARD_ID : FDCAN_EXTENDED_ID;
    hal_filter.FilterIndex = filter->filter_index;
    hal_filter.FilterType = FDCAN_FILTER_MASK;
    hal_filter.FilterConfig = (filter->receive_fifo == BSP_CAN_RX_FIFO_0) ? FDCAN_FILTER_TO_RXFIFO0
                                                                          : FDCAN_FILTER_TO_RXFIFO1;
    hal_filter.FilterID1 = filter->identifier;
    hal_filter.FilterID2 = filter->mask;
    return bsp_stm32h723_status(HAL_FDCAN_ConfigFilter((FDCAN_HandleTypeDef *)handle, &hal_filter));
}

static bsp_status_t bsp_stm32h723_can_transmit(void *handle, const bsp_can_frame_t *frame,
                                               uint32_t timeout_ms)
{
    FDCAN_TxHeaderTypeDef header = {0};
    uint32_t started_at_ms;
    if ((frame == NULL) || (frame->data_length > 8U))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    header.Identifier = frame->identifier;
    header.IdType = (frame->id_type == BSP_CAN_ID_STANDARD) ? FDCAN_STANDARD_ID : FDCAN_EXTENDED_ID;
    header.TxFrameType =
        (frame->frame_type == BSP_CAN_FRAME_DATA) ? FDCAN_DATA_FRAME : FDCAN_REMOTE_FRAME;
    header.DataLength = bsp_stm32h723_fdcan_data_length(frame->data_length);
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch = FDCAN_BRS_OFF;
    header.FDFormat = FDCAN_CLASSIC_CAN;
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    header.MessageMarker = 0U;
    started_at_ms = HAL_GetTick();
    while (HAL_FDCAN_GetTxFifoFreeLevel((FDCAN_HandleTypeDef *)handle) == 0U)
    {
        if ((HAL_GetTick() - started_at_ms) >= timeout_ms)
        {
            return BSP_STATUS_TIMEOUT;
        }
    }
    return bsp_stm32h723_status(HAL_FDCAN_AddMessageToTxFifoQ((FDCAN_HandleTypeDef *)handle,
                                                              &header, (uint8_t *)frame->data));
}

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
    status = HAL_FDCAN_GetRxMessage((FDCAN_HandleTypeDef *)handle, fifo, &header, frame->data);
    if (status == HAL_OK)
    {
        frame->identifier = header.Identifier;
        frame->id_type =
            (header.IdType == FDCAN_STANDARD_ID) ? BSP_CAN_ID_STANDARD : BSP_CAN_ID_EXTENDED;
        frame->frame_type =
            (header.RxFrameType == FDCAN_DATA_FRAME) ? BSP_CAN_FRAME_DATA : BSP_CAN_FRAME_REMOTE;
        frame->data_length = bsp_stm32h723_fdcan_decode_length(header.DataLength);
    }
    return bsp_stm32h723_status(status);
}

static bsp_status_t bsp_stm32h723_can_free_level(const void *handle, uint32_t *free_level)
{
    if ((handle == NULL) || (free_level == NULL))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    *free_level = HAL_FDCAN_GetTxFifoFreeLevel((FDCAN_HandleTypeDef *)(uintptr_t)handle);
    return BSP_STATUS_OK;
}

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

static bsp_status_t bsp_stm32h723_usart_transmit(void *handle, const uint8_t *data, size_t size,
                                                 bsp_transfer_mode_t mode, uint32_t timeout_ms)
{
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
    return bsp_stm32h723_status(
        HAL_UART_Transmit_DMA((UART_HandleTypeDef *)handle, data, (uint16_t)size));
}

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

static bsp_status_t bsp_stm32h723_usart_abort(void *handle)
{
    return bsp_stm32h723_status(HAL_UART_Abort((UART_HandleTypeDef *)handle));
}

static bsp_status_t bsp_stm32h723_usart_busy(const void *handle, bool *is_busy)
{
    if ((handle == NULL) || (is_busy == NULL))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    *is_busy = HAL_UART_GetState((UART_HandleTypeDef *)(uintptr_t)handle) != HAL_UART_STATE_READY;
    return BSP_STATUS_OK;
}

static const bsp_usart_driver_ops_t bsp_stm32h723_usart_driver_ops = {
    .init = bsp_stm32h723_noop_init,
    .deinit = bsp_stm32h723_noop_deinit,
    .transmit = bsp_stm32h723_usart_transmit,
    .receive = bsp_stm32h723_usart_receive,
    .receive_to_idle = bsp_stm32h723_usart_receive_to_idle,
    .abort = bsp_stm32h723_usart_abort,
    .get_busy = bsp_stm32h723_usart_busy,
};

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

static bsp_status_t bsp_stm32h723_spi_abort(void *handle)
{
    return bsp_stm32h723_status(HAL_SPI_Abort((SPI_HandleTypeDef *)handle));
}

static bsp_status_t bsp_stm32h723_spi_busy(const void *handle, bool *is_busy)
{
    if ((handle == NULL) || (is_busy == NULL))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    *is_busy = HAL_SPI_GetState((SPI_HandleTypeDef *)(uintptr_t)handle) != HAL_SPI_STATE_READY;
    return BSP_STATUS_OK;
}

static const bsp_spi_driver_ops_t bsp_stm32h723_spi_driver_ops = {
    .init = bsp_stm32h723_noop_init,
    .deinit = bsp_stm32h723_noop_deinit,
    .transmit = bsp_stm32h723_spi_transmit,
    .receive = bsp_stm32h723_spi_receive,
    .exchange = bsp_stm32h723_spi_exchange,
    .abort = bsp_stm32h723_spi_abort,
    .get_busy = bsp_stm32h723_spi_busy,
};

static bsp_status_t bsp_stm32h723_exti_init(void *handle)
{
    return (handle != NULL) ? BSP_STATUS_OK : BSP_STATUS_INVALID_ARGUMENT;
}

static bsp_status_t bsp_stm32h723_exti_enable(void *handle)
{
    const bsp_stm32h723_exti_context_t *const context =
        (const bsp_stm32h723_exti_context_t *)handle;
    HAL_NVIC_EnableIRQ(context->interrupt_number);
    return BSP_STATUS_OK;
}

static bsp_status_t bsp_stm32h723_exti_disable(void *handle)
{
    const bsp_stm32h723_exti_context_t *const context =
        (const bsp_stm32h723_exti_context_t *)handle;
    HAL_NVIC_DisableIRQ(context->interrupt_number);
    return BSP_STATUS_OK;
}

static const bsp_exti_driver_ops_t bsp_stm32h723_exti_driver_ops = {
    .init = bsp_stm32h723_exti_init,
    .deinit = bsp_stm32h723_noop_deinit,
    .enable = bsp_stm32h723_exti_enable,
    .disable = bsp_stm32h723_exti_disable,
};

static uint32_t bsp_stm32h723_pwm_channel(uint32_t channel)
{
    static const uint32_t channels[] = {0U, TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3,
                                        TIM_CHANNEL_4};
    return (channel <= 4U) ? channels[channel] : 0U;
}

static bsp_status_t bsp_stm32h723_pwm_init(void *handle, uint32_t channel)
{
    return ((handle != NULL) && (bsp_stm32h723_pwm_channel(channel) != 0U))
               ? BSP_STATUS_OK
               : BSP_STATUS_INVALID_ARGUMENT;
}

static bsp_status_t bsp_stm32h723_pwm_start(void *handle, uint32_t channel)
{
    return bsp_stm32h723_status(
        HAL_TIM_PWM_Start((TIM_HandleTypeDef *)handle, bsp_stm32h723_pwm_channel(channel)));
}

static bsp_status_t bsp_stm32h723_pwm_stop(void *handle, uint32_t channel)
{
    return bsp_stm32h723_status(
        HAL_TIM_PWM_Stop((TIM_HandleTypeDef *)handle, bsp_stm32h723_pwm_channel(channel)));
}

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
    period_ticks = timer_clock_hz / ((timer->Init.Prescaler + 1U) * frequency_hz);
    if ((period_ticks == 0U) || (period_ticks > 65536U))
    {
        return BSP_STATUS_OUT_OF_RANGE;
    }
    __HAL_TIM_SET_AUTORELOAD(timer, period_ticks - 1U);
    __HAL_TIM_SET_COUNTER(timer, 0U);
    return BSP_STATUS_OK;
}

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

static bsp_status_t bsp_stm32h723_pwm_set_pulse(void *handle, uint32_t channel,
                                                uint32_t pulse_ticks)
{
    TIM_HandleTypeDef *const timer = (TIM_HandleTypeDef *)handle;
    if (pulse_ticks > timer->Instance->ARR + 1U)
    {
        return BSP_STATUS_OUT_OF_RANGE;
    }
    __HAL_TIM_SET_COMPARE(timer, bsp_stm32h723_pwm_channel(channel), pulse_ticks);
    return BSP_STATUS_OK;
}

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

static bsp_status_t bsp_stm32h723_timebase_init(void *handle)
{
    (void)handle;
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    return ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) != 0U) ? BSP_STATUS_OK : BSP_STATUS_UNSUPPORTED;
}

static bsp_status_t bsp_stm32h723_timebase_reset(void *handle)
{
    (void)handle;
    DWT->CYCCNT = 0U;
    return BSP_STATUS_OK;
}

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

static const bsp_timebase_driver_ops_t bsp_stm32h723_timebase_driver_ops = {
    .init = bsp_stm32h723_timebase_init,
    .deinit = bsp_stm32h723_noop_deinit,
    .reset = bsp_stm32h723_timebase_reset,
    .get_cycle_count = bsp_stm32h723_timebase_cycles,
    .get_frequency = bsp_stm32h723_timebase_frequency,
};

static bsp_status_t bsp_stm32h723_watchdog_init(void *handle)
{
    bsp_stm32h723_watchdog_context_t *const context = (bsp_stm32h723_watchdog_context_t *)handle;
    context->reset_detected = (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDG1RST) != 0U);
    __HAL_RCC_CLEAR_RESET_FLAGS();
    IWDG1->KR = 0x5555U;
    IWDG1->PR = 4U;
    IWDG1->RLR = 1000U;
    IWDG1->WINR = 4095U;
    while (IWDG1->SR != 0U)
    {
    }
    IWDG1->KR = 0xCCCCU;
    return BSP_STATUS_OK;
}

static bsp_status_t bsp_stm32h723_watchdog_refresh(void *handle)
{
    (void)handle;
    IWDG1->KR = 0xAAAAU;
    return BSP_STATUS_OK;
}

static bsp_status_t bsp_stm32h723_watchdog_timeout(const void *handle, uint32_t *timeout_ms)
{
    if ((handle == NULL) || (timeout_ms == NULL))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    *timeout_ms = ((const bsp_stm32h723_watchdog_context_t *)handle)->timeout_ms;
    return BSP_STATUS_OK;
}

static bsp_status_t bsp_stm32h723_watchdog_reset_detected(const void *handle, bool *reset_detected)
{
    if ((handle == NULL) || (reset_detected == NULL))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    *reset_detected = ((const bsp_stm32h723_watchdog_context_t *)handle)->reset_detected;
    return BSP_STATUS_OK;
}

static const bsp_watchdog_driver_ops_t bsp_stm32h723_watchdog_driver_ops = {
    .init = bsp_stm32h723_watchdog_init,
    .deinit = bsp_stm32h723_noop_deinit,
    .refresh = bsp_stm32h723_watchdog_refresh,
    .get_timeout_ms = bsp_stm32h723_watchdog_timeout,
    .get_reset_detected = bsp_stm32h723_watchdog_reset_detected,
};

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
    copy_size = (context->receive_size < data_capacity) ? context->receive_size : data_capacity;
    memcpy(receive_data, context->receive_buffer, copy_size);
    context->receive_pending = false;
    context->receive_size = 0U;
    return BSP_STATUS_OK;
}

static bsp_status_t bsp_stm32h723_usb_abort(void *handle)
{
    bsp_stm32h723_usb_context_t *const context = (bsp_stm32h723_usb_context_t *)handle;
    context->receive_pending = false;
    context->receive_size = 0U;
    return BSP_STATUS_OK;
}

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

static const bsp_usb_vcp_driver_ops_t bsp_stm32h723_usb_driver_ops = {
    .init = bsp_stm32h723_noop_init,
    .deinit = bsp_stm32h723_noop_deinit,
    .transmit = bsp_stm32h723_usb_transmit,
    .receive = bsp_stm32h723_usb_receive,
    .abort = bsp_stm32h723_usb_abort,
    .get_connected = bsp_stm32h723_usb_connected,
    .get_busy = bsp_stm32h723_usb_busy,
};

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

bsp_status_t bsp_stm32h723_port_init(const bsp_stm32h723_port_config_t *config)
{
    static const uint32_t pwm_channels[BSP_STM32H723_PWM_COUNT] = {1U, 2U, 3U, 4U, 1U};
    TIM_HandleTypeDef *pwm_handles[BSP_STM32H723_PWM_COUNT] = {
        &htim3, &htim3, &htim3, &htim3, &htim1,
    };
    size_t index;
    if ((config == NULL) || bsp_stm32h723_initialized)
    {
        return (config == NULL) ? BSP_STATUS_INVALID_ARGUMENT : BSP_STATUS_BUSY;
    }
    if ((bsp_stm32h723_init_can_devices() != BSP_STATUS_OK) ||
        (bsp_stm32h723_init_usart_devices() != BSP_STATUS_OK))
    {
        return BSP_STATUS_IO_ERROR;
    }
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
    bsp_stm32h723_exti_contexts[0] = (bsp_stm32h723_exti_context_t){
        .pin = BMI088_GYRO_INT_Pin,
        .interrupt_number = BMI088_GYRO_INT_EXTI_IRQn,
    };
    bsp_stm32h723_exti_contexts[1] = (bsp_stm32h723_exti_context_t){
        .pin = BMI088_ACCEL_INT_Pin,
        .interrupt_number = BMI088_ACCEL_INT_EXTI_IRQn,
    };
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

bsp_can_t *bsp_stm32h723_port_get_can(bsp_stm32h723_can_index_t index)
{
    return (bsp_stm32h723_initialized && (index < BSP_STM32H723_CAN_COUNT))
               ? bsp_can_as_base(&bsp_stm32h723_can_devices[index])
               : NULL;
}

bsp_usart_t *bsp_stm32h723_port_get_usart(bsp_stm32h723_usart_index_t index)
{
    return (bsp_stm32h723_initialized && (index < BSP_STM32H723_USART_COUNT))
               ? bsp_usart_as_base(&bsp_stm32h723_usart_devices[index])
               : NULL;
}

bsp_spi_t *bsp_stm32h723_port_get_bmi088_spi(void)
{
    return bsp_stm32h723_initialized ? bsp_spi_as_base(&bsp_stm32h723_bmi088_spi_device) : NULL;
}

bsp_exti_t *bsp_stm32h723_port_get_exti(bsp_stm32h723_exti_index_t index)
{
    return (bsp_stm32h723_initialized && (index < BSP_STM32H723_EXTI_COUNT))
               ? bsp_exti_as_base(&bsp_stm32h723_exti_devices[index])
               : NULL;
}

bsp_pwm_t *bsp_stm32h723_port_get_pwm(bsp_stm32h723_pwm_index_t index)
{
    return (bsp_stm32h723_initialized && (index < BSP_STM32H723_PWM_COUNT))
               ? bsp_pwm_as_base(&bsp_stm32h723_pwm_devices[index])
               : NULL;
}

bsp_usb_vcp_t *bsp_stm32h723_port_get_usb_vcp(void)
{
    return bsp_stm32h723_initialized ? bsp_usb_vcp_as_base(&bsp_stm32h723_usb_device) : NULL;
}

bsp_timebase_t *bsp_stm32h723_port_get_timebase(void)
{
    return bsp_stm32h723_initialized ? bsp_timebase_as_base(&bsp_stm32h723_timebase_device) : NULL;
}

bsp_watchdog_t *bsp_stm32h723_port_get_watchdog(void)
{
    return (bsp_stm32h723_initialized && bsp_stm32h723_watchdog_initialized)
               ? bsp_watchdog_as_base(&bsp_stm32h723_watchdog_device)
               : NULL;
}

bool bsp_stm32h723_port_is_initialized(void)
{
    return bsp_stm32h723_initialized;
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *fdcan, uint32_t interrupt_flags)
{
    size_t index;
    (void)interrupt_flags;
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

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *fdcan, uint32_t interrupt_flags)
{
    HAL_FDCAN_RxFifo0Callback(fdcan, interrupt_flags);
}

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

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *uart)
{
    bsp_usart_t *const usart = bsp_stm32h723_find_usart(uart);
    if (usart != NULL)
    {
        bsp_usart_notify(usart, BSP_EVENT_TRANSMIT_COMPLETE, BSP_STATUS_OK, uart->TxXferSize);
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart)
{
    bsp_usart_t *const usart = bsp_stm32h723_find_usart(uart);
    if (usart != NULL)
    {
        bsp_usart_notify(usart, BSP_EVENT_RECEIVE_COMPLETE, BSP_STATUS_OK, uart->RxXferSize);
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *uart, uint16_t received_size)
{
    bsp_usart_t *const usart = bsp_stm32h723_find_usart(uart);
    if (usart != NULL)
    {
        bsp_usart_notify(usart, BSP_EVENT_RECEIVE_COMPLETE, BSP_STATUS_OK, received_size);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
    bsp_usart_t *const usart = bsp_stm32h723_find_usart(uart);
    if (usart != NULL)
    {
        bsp_usart_notify(usart, BSP_EVENT_ERROR, BSP_STATUS_IO_ERROR, 0U);
    }
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *spi)
{
    if (spi == &hspi2)
    {
        bsp_spi_notify(&bsp_stm32h723_bmi088_spi_device.super, BSP_EVENT_TRANSFER_COMPLETE,
                       BSP_STATUS_OK, spi->TxXferSize);
    }
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *spi)
{
    if (spi == &hspi2)
    {
        bsp_spi_notify(&bsp_stm32h723_bmi088_spi_device.super, BSP_EVENT_TRANSMIT_COMPLETE,
                       BSP_STATUS_OK, spi->TxXferSize);
    }
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *spi)
{
    if (spi == &hspi2)
    {
        bsp_spi_notify(&bsp_stm32h723_bmi088_spi_device.super, BSP_EVENT_RECEIVE_COMPLETE,
                       BSP_STATUS_OK, spi->RxXferSize);
    }
}

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

void usb_cdc_receive_callback(const uint8_t *receive_data, uint32_t receive_size)
{
    const size_t copy_size = (receive_size < BSP_STM32H723_USB_RECEIVE_CAPACITY)
                                 ? (size_t)receive_size
                                 : BSP_STM32H723_USB_RECEIVE_CAPACITY;
    if ((receive_data == NULL) || !bsp_stm32h723_initialized)
    {
        return;
    }
    memcpy(bsp_stm32h723_usb_context.receive_buffer, receive_data, copy_size);
    bsp_stm32h723_usb_context.receive_size = copy_size;
    bsp_stm32h723_usb_context.receive_pending = true;
    bsp_usb_vcp_notify(&bsp_stm32h723_usb_device.super, BSP_EVENT_RECEIVE_PENDING, BSP_STATUS_OK,
                       copy_size);
}
