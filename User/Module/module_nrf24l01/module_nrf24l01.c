#include "module_nrf24l01.h"

#include <stddef.h>
#include <string.h>

#define MODULE_NRF24L01_COMMAND_READ_REGISTER (0x00U)
#define MODULE_NRF24L01_COMMAND_WRITE_REGISTER (0x20U)
#define MODULE_NRF24L01_COMMAND_READ_PAYLOAD (0x61U)
#define MODULE_NRF24L01_COMMAND_WRITE_PAYLOAD (0xA0U)
#define MODULE_NRF24L01_COMMAND_FLUSH_TRANSMIT (0xE1U)
#define MODULE_NRF24L01_COMMAND_FLUSH_RECEIVE (0xE2U)
#define MODULE_NRF24L01_COMMAND_NOP (0xFFU)
#define MODULE_NRF24L01_REGISTER_CONFIG (0x00U)
#define MODULE_NRF24L01_REGISTER_ENABLE_AUTO_ACK (0x01U)
#define MODULE_NRF24L01_REGISTER_ENABLE_RECEIVE_ADDRESS (0x02U)
#define MODULE_NRF24L01_REGISTER_SETUP_ADDRESS_WIDTH (0x03U)
#define MODULE_NRF24L01_REGISTER_SETUP_RETRANSMIT (0x04U)
#define MODULE_NRF24L01_REGISTER_RADIO_FREQUENCY_CHANNEL (0x05U)
#define MODULE_NRF24L01_REGISTER_RADIO_FREQUENCY_SETUP (0x06U)
#define MODULE_NRF24L01_REGISTER_STATUS (0x07U)
#define MODULE_NRF24L01_REGISTER_OBSERVE_TRANSMIT (0x08U)
#define MODULE_NRF24L01_REGISTER_RECEIVE_ADDRESS_PIPE_0 (0x0AU)
#define MODULE_NRF24L01_REGISTER_TRANSMIT_ADDRESS (0x10U)
#define MODULE_NRF24L01_REGISTER_RECEIVE_PAYLOAD_WIDTH_PIPE_0 (0x11U)
#define MODULE_NRF24L01_CONFIG_ENABLE_CRC (1U << 3)
#define MODULE_NRF24L01_CONFIG_CRC_TWO_BYTES (1U << 2)
#define MODULE_NRF24L01_CONFIG_POWER_UP (1U << 1)
#define MODULE_NRF24L01_CONFIG_PRIMARY_RECEIVE (1U << 0)
#define MODULE_NRF24L01_STATUS_RECEIVE_DATA_READY (1U << 6)
#define MODULE_NRF24L01_STATUS_TRANSMIT_DATA_SENT (1U << 5)
#define MODULE_NRF24L01_STATUS_MAXIMUM_RETRANSMIT (1U << 4)

static module_nrf24l01_status_t module_nrf24l01_exchange(module_nrf24l01_t *me,
                                                         const uint8_t *transmit_data,
                                                         uint8_t *receive_data, size_t data_size)
{
    bsp_status_t status;

    if (bsp_gpio_write(me->chip_select_gpio, false) != BSP_STATUS_OK)
    {
        return MODULE_NRF24L01_STATUS_TRANSPORT_ERROR;
    }
    status = bsp_spi_exchange(me->spi, transmit_data, receive_data, data_size,
                              BSP_TRANSFER_MODE_BLOCKING, me->spi_timeout_ms);
    if (bsp_gpio_write(me->chip_select_gpio, true) != BSP_STATUS_OK)
    {
        return MODULE_NRF24L01_STATUS_TRANSPORT_ERROR;
    }
    return (status == BSP_STATUS_OK) ? MODULE_NRF24L01_STATUS_OK
                                     : MODULE_NRF24L01_STATUS_TRANSPORT_ERROR;
}

static module_nrf24l01_status_t module_nrf24l01_command(module_nrf24l01_t *me, uint8_t command,
                                                        uint8_t *status_register)
{
    uint8_t receive_value = 0U;
    const module_nrf24l01_status_t status =
        module_nrf24l01_exchange(me, &command, &receive_value, 1U);
    if ((status == MODULE_NRF24L01_STATUS_OK) && (status_register != NULL))
    {
        *status_register = receive_value;
    }
    return status;
}

static module_nrf24l01_status_t module_nrf24l01_read_register(module_nrf24l01_t *me,
                                                              uint8_t register_address,
                                                              uint8_t *register_data,
                                                              size_t data_size)
{
    uint8_t transmit_buffer[MODULE_NRF24L01_MAXIMUM_ADDRESS_SIZE + 1U];
    uint8_t receive_buffer[MODULE_NRF24L01_MAXIMUM_ADDRESS_SIZE + 1U];
    module_nrf24l01_status_t status;

    if ((register_data == NULL) || (data_size == 0U) ||
        (data_size > MODULE_NRF24L01_MAXIMUM_ADDRESS_SIZE))
    {
        return MODULE_NRF24L01_STATUS_INVALID_ARGUMENT;
    }
    memset(transmit_buffer, MODULE_NRF24L01_COMMAND_NOP, data_size + 1U);
    transmit_buffer[0] = MODULE_NRF24L01_COMMAND_READ_REGISTER | (register_address & 0x1FU);
    status = module_nrf24l01_exchange(me, transmit_buffer, receive_buffer, data_size + 1U);
    if (status == MODULE_NRF24L01_STATUS_OK)
    {
        memcpy(register_data, &receive_buffer[1], data_size);
    }
    return status;
}

static module_nrf24l01_status_t module_nrf24l01_write_register(module_nrf24l01_t *me,
                                                               uint8_t register_address,
                                                               const uint8_t *register_data,
                                                               size_t data_size)
{
    uint8_t transmit_buffer[MODULE_NRF24L01_MAXIMUM_ADDRESS_SIZE + 1U];
    uint8_t receive_buffer[MODULE_NRF24L01_MAXIMUM_ADDRESS_SIZE + 1U];

    if ((register_data == NULL) || (data_size == 0U) ||
        (data_size > MODULE_NRF24L01_MAXIMUM_ADDRESS_SIZE))
    {
        return MODULE_NRF24L01_STATUS_INVALID_ARGUMENT;
    }
    transmit_buffer[0] = MODULE_NRF24L01_COMMAND_WRITE_REGISTER | (register_address & 0x1FU);
    memcpy(&transmit_buffer[1], register_data, data_size);
    return module_nrf24l01_exchange(me, transmit_buffer, receive_buffer, data_size + 1U);
}

static module_nrf24l01_status_t module_nrf24l01_write_single_register(module_nrf24l01_t *me,
                                                                      uint8_t register_address,
                                                                      uint8_t register_value)
{
    return module_nrf24l01_write_register(me, register_address, &register_value, 1U);
}

static module_nrf24l01_status_t module_nrf24l01_set_mode(module_nrf24l01_t *me,
                                                         module_nrf24l01_mode_t mode)
{
    uint8_t configuration = me->configuration_register | MODULE_NRF24L01_CONFIG_POWER_UP;

    if (bsp_gpio_write(me->chip_enable_gpio, false) != BSP_STATUS_OK)
    {
        return MODULE_NRF24L01_STATUS_TRANSPORT_ERROR;
    }
    if (mode == MODULE_NRF24L01_MODE_RECEIVE)
    {
        configuration |= MODULE_NRF24L01_CONFIG_PRIMARY_RECEIVE;
    }
    else
    {
        configuration &= (uint8_t)(~MODULE_NRF24L01_CONFIG_PRIMARY_RECEIVE);
    }
    if (module_nrf24l01_write_single_register(me, MODULE_NRF24L01_REGISTER_CONFIG, configuration) !=
        MODULE_NRF24L01_STATUS_OK)
    {
        return MODULE_NRF24L01_STATUS_TRANSPORT_ERROR;
    }
    me->mode = mode;
    me->delay_us(1500U, me->delay_user_context);
    return MODULE_NRF24L01_STATUS_OK;
}

static module_device_status_t module_nrf24l01_device_start(module_device_t *const device_base)
{
    module_nrf24l01_t *const me = MODULE_CONTAINER_OF(device_base, module_nrf24l01_t, super);
    return (module_nrf24l01_start(me) == MODULE_NRF24L01_STATUS_OK)
               ? MODULE_DEVICE_STATUS_OK
               : MODULE_DEVICE_STATUS_OPERATION_FAILED;
}

static module_device_status_t module_nrf24l01_device_stop(module_device_t *const device_base)
{
    module_nrf24l01_t *const me = MODULE_CONTAINER_OF(device_base, module_nrf24l01_t, super);
    return (module_nrf24l01_stop(me) == MODULE_NRF24L01_STATUS_OK)
               ? MODULE_DEVICE_STATUS_OK
               : MODULE_DEVICE_STATUS_OPERATION_FAILED;
}

static module_device_status_t module_nrf24l01_device_update(module_device_t *const device_base,
                                                            uint32_t elapsed_time_ms)
{
    (void)device_base;
    (void)elapsed_time_ms;
    return MODULE_DEVICE_STATUS_OK;
}

static const module_device_ops_t s_module_nrf24l01_ops = {
    .start = module_nrf24l01_device_start,
    .stop = module_nrf24l01_device_stop,
    .update = module_nrf24l01_device_update,
};

module_nrf24l01_status_t module_nrf24l01_init(module_nrf24l01_t *me,
                                              const module_nrf24l01_config_t *config)
{
    uint8_t radio_frequency_setup;
    uint8_t retransmit_delay_field;

    if ((me == NULL) || (config == NULL) || (config->spi == NULL) ||
        !bsp_device_is_initialized(&config->spi->super) || (config->chip_enable_gpio == NULL) ||
        (config->chip_select_gpio == NULL) ||
        !bsp_device_is_initialized(&config->chip_enable_gpio->super) ||
        !bsp_device_is_initialized(&config->chip_select_gpio->super) || (config->channel > 125U) ||
        (config->address_size < 3U) || (config->address_size > 5U) ||
        (config->payload_size == 0U) ||
        (config->payload_size > MODULE_NRF24L01_MAXIMUM_PAYLOAD_SIZE) ||
        (config->automatic_retransmit_count > 15U) ||
        (config->automatic_retransmit_delay_us < 250U) ||
        (config->automatic_retransmit_delay_us > 4000U) ||
        ((config->automatic_retransmit_delay_us % 250U) != 0U) ||
        (config->data_rate > MODULE_NRF24L01_DATA_RATE_250_KBPS) ||
        (config->output_power > MODULE_NRF24L01_OUTPUT_POWER_0_DBM) || (config->delay_us == NULL))
    {
        return MODULE_NRF24L01_STATUS_INVALID_ARGUMENT;
    }
    *me = (module_nrf24l01_t){0};
    me->spi = config->spi;
    me->chip_enable_gpio = config->chip_enable_gpio;
    me->chip_select_gpio = config->chip_select_gpio;
    me->channel = config->channel;
    me->address_size = config->address_size;
    me->payload_size = config->payload_size;
    me->configuration_register =
        MODULE_NRF24L01_CONFIG_ENABLE_CRC | MODULE_NRF24L01_CONFIG_CRC_TWO_BYTES;
    radio_frequency_setup = (uint8_t)((uint8_t)config->output_power << 1U);
    if (config->data_rate == MODULE_NRF24L01_DATA_RATE_2_MBPS)
    {
        radio_frequency_setup |= (1U << 3);
    }
    else if (config->data_rate == MODULE_NRF24L01_DATA_RATE_250_KBPS)
    {
        radio_frequency_setup |= (1U << 5);
    }
    me->radio_frequency_setup_register = radio_frequency_setup;
    retransmit_delay_field = (uint8_t)(config->automatic_retransmit_delay_us / 250U - 1U);
    me->automatic_retransmit_setup_register =
        (uint8_t)((retransmit_delay_field << 4U) | config->automatic_retransmit_count);
    me->spi_timeout_ms = config->spi_timeout_ms;
    me->delay_us = config->delay_us;
    me->delay_user_context = config->delay_user_context;
    if (module_device_init_base(&me->super, &s_module_nrf24l01_ops, config->logical_name,
                                config->registration_key) != MODULE_DEVICE_STATUS_OK)
    {
        return MODULE_NRF24L01_STATUS_INVALID_ARGUMENT;
    }
    if (bsp_gpio_write(me->chip_enable_gpio, false) != BSP_STATUS_OK ||
        bsp_gpio_write(me->chip_select_gpio, true) != BSP_STATUS_OK)
    {
        module_device_abort_init(&me->super);
        return MODULE_NRF24L01_STATUS_TRANSPORT_ERROR;
    }
    if ((module_nrf24l01_write_single_register(me, MODULE_NRF24L01_REGISTER_CONFIG,
                                               me->configuration_register) !=
         MODULE_NRF24L01_STATUS_OK) ||
        (module_nrf24l01_write_single_register(
             me, MODULE_NRF24L01_REGISTER_ENABLE_AUTO_ACK,
             config->automatic_acknowledge_enabled ? 0x01U : 0x00U) != MODULE_NRF24L01_STATUS_OK) ||
        (module_nrf24l01_write_single_register(me, MODULE_NRF24L01_REGISTER_ENABLE_RECEIVE_ADDRESS,
                                               0x01U) != MODULE_NRF24L01_STATUS_OK) ||
        (module_nrf24l01_write_single_register(me, MODULE_NRF24L01_REGISTER_SETUP_ADDRESS_WIDTH,
                                               (uint8_t)(config->address_size - 2U)) !=
         MODULE_NRF24L01_STATUS_OK) ||
        (module_nrf24l01_write_single_register(me, MODULE_NRF24L01_REGISTER_SETUP_RETRANSMIT,
                                               me->automatic_retransmit_setup_register) !=
         MODULE_NRF24L01_STATUS_OK) ||
        (module_nrf24l01_write_single_register(me, MODULE_NRF24L01_REGISTER_RADIO_FREQUENCY_CHANNEL,
                                               me->channel) != MODULE_NRF24L01_STATUS_OK) ||
        (module_nrf24l01_write_single_register(me, MODULE_NRF24L01_REGISTER_RADIO_FREQUENCY_SETUP,
                                               me->radio_frequency_setup_register) !=
         MODULE_NRF24L01_STATUS_OK) ||
        (module_nrf24l01_write_single_register(
             me, MODULE_NRF24L01_REGISTER_RECEIVE_PAYLOAD_WIDTH_PIPE_0, me->payload_size) !=
         MODULE_NRF24L01_STATUS_OK))
    {
        module_device_abort_init(&me->super);
        return MODULE_NRF24L01_STATUS_TRANSPORT_ERROR;
    }
    if (module_device_complete_init(&me->super) != MODULE_DEVICE_STATUS_OK)
    {
        module_device_abort_init(&me->super);
        return MODULE_NRF24L01_STATUS_INVALID_ARGUMENT;
    }
    return MODULE_NRF24L01_STATUS_OK;
}

module_nrf24l01_status_t module_nrf24l01_start(module_nrf24l01_t *me)
{
    uint8_t configuration_readback;

    if ((me == NULL) || !module_device_is_initialized(&me->super))
    {
        return MODULE_NRF24L01_STATUS_NOT_INITIALIZED;
    }
    if (module_nrf24l01_read_register(me, MODULE_NRF24L01_REGISTER_CONFIG, &configuration_readback,
                                      1U) != MODULE_NRF24L01_STATUS_OK)
    {
        return MODULE_NRF24L01_STATUS_TRANSPORT_ERROR;
    }
    if (configuration_readback != me->configuration_register)
    {
        return MODULE_NRF24L01_STATUS_DEVICE_NOT_FOUND;
    }
    if ((module_nrf24l01_flush_transmit(me) != MODULE_NRF24L01_STATUS_OK) ||
        (module_nrf24l01_flush_receive(me) != MODULE_NRF24L01_STATUS_OK) ||
        (module_nrf24l01_write_single_register(
             me, MODULE_NRF24L01_REGISTER_STATUS,
             MODULE_NRF24L01_STATUS_RECEIVE_DATA_READY | MODULE_NRF24L01_STATUS_TRANSMIT_DATA_SENT |
                 MODULE_NRF24L01_STATUS_MAXIMUM_RETRANSMIT) != MODULE_NRF24L01_STATUS_OK))
    {
        return MODULE_NRF24L01_STATUS_TRANSPORT_ERROR;
    }
    me->is_started = true;
    return module_nrf24l01_set_mode(me, MODULE_NRF24L01_MODE_STANDBY);
}

module_nrf24l01_status_t module_nrf24l01_stop(module_nrf24l01_t *me)
{
    if ((me == NULL) || !module_device_is_initialized(&me->super))
    {
        return MODULE_NRF24L01_STATUS_NOT_INITIALIZED;
    }
    if (bsp_gpio_write(me->chip_enable_gpio, false) != BSP_STATUS_OK ||
        module_nrf24l01_write_single_register(me, MODULE_NRF24L01_REGISTER_CONFIG,
                                              me->configuration_register) !=
            MODULE_NRF24L01_STATUS_OK)
    {
        return MODULE_NRF24L01_STATUS_TRANSPORT_ERROR;
    }
    me->is_started = false;
    me->transmit_pending = false;
    me->mode = MODULE_NRF24L01_MODE_STANDBY;
    return MODULE_NRF24L01_STATUS_OK;
}

module_nrf24l01_status_t module_nrf24l01_set_receive_address(module_nrf24l01_t *me,
                                                             uint8_t pipe_index,
                                                             const uint8_t *address,
                                                             size_t address_size)
{
    if ((me == NULL) || (address == NULL) || (pipe_index > 5U))
    {
        return MODULE_NRF24L01_STATUS_INVALID_ARGUMENT;
    }
    if (!module_device_is_initialized(&me->super))
    {
        return MODULE_NRF24L01_STATUS_NOT_INITIALIZED;
    }
    if (address_size != me->address_size)
    {
        return MODULE_NRF24L01_STATUS_INVALID_ARGUMENT;
    }
    return module_nrf24l01_write_register(
        me, (uint8_t)(MODULE_NRF24L01_REGISTER_RECEIVE_ADDRESS_PIPE_0 + pipe_index), address,
        (pipe_index < 2U) ? address_size : 1U);
}

module_nrf24l01_status_t
module_nrf24l01_set_receive_pipe_enabled(module_nrf24l01_t *me, uint8_t pipe_index, bool is_enabled)
{
    uint8_t enabled_pipe_mask;

    if ((me == NULL) || (pipe_index > 5U))
    {
        return MODULE_NRF24L01_STATUS_INVALID_ARGUMENT;
    }
    if (!module_device_is_initialized(&me->super))
    {
        return MODULE_NRF24L01_STATUS_NOT_INITIALIZED;
    }
    if (module_nrf24l01_read_register(me, MODULE_NRF24L01_REGISTER_ENABLE_RECEIVE_ADDRESS,
                                      &enabled_pipe_mask, 1U) != MODULE_NRF24L01_STATUS_OK)
    {
        return MODULE_NRF24L01_STATUS_TRANSPORT_ERROR;
    }
    if (is_enabled)
    {
        enabled_pipe_mask |= (uint8_t)(1U << pipe_index);
    }
    else
    {
        enabled_pipe_mask &= (uint8_t)(~(uint8_t)(1U << pipe_index));
    }
    if ((module_nrf24l01_write_single_register(me, MODULE_NRF24L01_REGISTER_ENABLE_RECEIVE_ADDRESS,
                                               enabled_pipe_mask) != MODULE_NRF24L01_STATUS_OK) ||
        (module_nrf24l01_write_single_register(
             me, (uint8_t)(MODULE_NRF24L01_REGISTER_RECEIVE_PAYLOAD_WIDTH_PIPE_0 + pipe_index),
             is_enabled ? me->payload_size : 0U) != MODULE_NRF24L01_STATUS_OK))
    {
        return MODULE_NRF24L01_STATUS_TRANSPORT_ERROR;
    }
    return MODULE_NRF24L01_STATUS_OK;
}

module_nrf24l01_status_t module_nrf24l01_set_transmit_address(module_nrf24l01_t *me,
                                                              const uint8_t *address,
                                                              size_t address_size)
{
    if ((me == NULL) || (address == NULL))
    {
        return MODULE_NRF24L01_STATUS_INVALID_ARGUMENT;
    }
    if (!module_device_is_initialized(&me->super))
    {
        return MODULE_NRF24L01_STATUS_NOT_INITIALIZED;
    }
    if (address_size != me->address_size)
    {
        return MODULE_NRF24L01_STATUS_INVALID_ARGUMENT;
    }
    if (module_nrf24l01_write_register(me, MODULE_NRF24L01_REGISTER_TRANSMIT_ADDRESS, address,
                                       address_size) != MODULE_NRF24L01_STATUS_OK)
    {
        return MODULE_NRF24L01_STATUS_TRANSPORT_ERROR;
    }
    return module_nrf24l01_write_register(me, MODULE_NRF24L01_REGISTER_RECEIVE_ADDRESS_PIPE_0,
                                          address, address_size);
}

module_nrf24l01_status_t module_nrf24l01_start_receive(module_nrf24l01_t *me)
{
    module_nrf24l01_status_t status;

    if ((me == NULL) || !module_device_is_initialized(&me->super))
    {
        return MODULE_NRF24L01_STATUS_NOT_INITIALIZED;
    }
    if (!me->is_started)
    {
        return MODULE_NRF24L01_STATUS_NOT_STARTED;
    }
    if (me->transmit_pending)
    {
        return MODULE_NRF24L01_STATUS_BUSY;
    }
    status = module_nrf24l01_set_mode(me, MODULE_NRF24L01_MODE_RECEIVE);
    if (status != MODULE_NRF24L01_STATUS_OK)
    {
        return status;
    }
    return (bsp_gpio_write(me->chip_enable_gpio, true) == BSP_STATUS_OK)
               ? MODULE_NRF24L01_STATUS_OK
               : MODULE_NRF24L01_STATUS_TRANSPORT_ERROR;
}

module_nrf24l01_status_t module_nrf24l01_transmit(module_nrf24l01_t *me, const uint8_t *payload,
                                                  size_t payload_size)
{
    uint8_t transmit_buffer[MODULE_NRF24L01_MAXIMUM_PAYLOAD_SIZE + 1U];
    uint8_t receive_buffer[MODULE_NRF24L01_MAXIMUM_PAYLOAD_SIZE + 1U];
    module_nrf24l01_status_t status;

    if ((me == NULL) || (payload == NULL))
    {
        return MODULE_NRF24L01_STATUS_INVALID_ARGUMENT;
    }
    if (!module_device_is_initialized(&me->super))
    {
        return MODULE_NRF24L01_STATUS_NOT_INITIALIZED;
    }
    if (payload_size != me->payload_size)
    {
        return MODULE_NRF24L01_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_started)
    {
        return MODULE_NRF24L01_STATUS_NOT_STARTED;
    }
    if (me->transmit_pending)
    {
        return MODULE_NRF24L01_STATUS_BUSY;
    }
    status = module_nrf24l01_set_mode(me, MODULE_NRF24L01_MODE_TRANSMIT);
    if (status != MODULE_NRF24L01_STATUS_OK)
    {
        return status;
    }
    (void)module_nrf24l01_write_single_register(me, MODULE_NRF24L01_REGISTER_STATUS,
                                                MODULE_NRF24L01_STATUS_TRANSMIT_DATA_SENT |
                                                    MODULE_NRF24L01_STATUS_MAXIMUM_RETRANSMIT);
    transmit_buffer[0] = MODULE_NRF24L01_COMMAND_WRITE_PAYLOAD;
    memcpy(&transmit_buffer[1], payload, payload_size);
    status = module_nrf24l01_exchange(me, transmit_buffer, receive_buffer, payload_size + 1U);
    if (status != MODULE_NRF24L01_STATUS_OK)
    {
        return status;
    }
    if (bsp_gpio_write(me->chip_enable_gpio, true) != BSP_STATUS_OK)
    {
        return MODULE_NRF24L01_STATUS_TRANSPORT_ERROR;
    }
    me->delay_us(15U, me->delay_user_context);
    if (bsp_gpio_write(me->chip_enable_gpio, false) != BSP_STATUS_OK)
    {
        return MODULE_NRF24L01_STATUS_TRANSPORT_ERROR;
    }
    me->transmit_pending = true;
    return MODULE_NRF24L01_STATUS_OK;
}

module_nrf24l01_status_t module_nrf24l01_poll_transmit(module_nrf24l01_t *me)
{
    uint8_t status_register;

    if ((me == NULL) || !module_device_is_initialized(&me->super))
    {
        return MODULE_NRF24L01_STATUS_NOT_INITIALIZED;
    }
    if (!me->transmit_pending)
    {
        return MODULE_NRF24L01_STATUS_NO_DATA;
    }
    if (module_nrf24l01_command(me, MODULE_NRF24L01_COMMAND_NOP, &status_register) !=
        MODULE_NRF24L01_STATUS_OK)
    {
        return MODULE_NRF24L01_STATUS_TRANSPORT_ERROR;
    }
    if ((status_register & MODULE_NRF24L01_STATUS_MAXIMUM_RETRANSMIT) != 0U)
    {
        me->transmit_pending = false;
        (void)module_nrf24l01_write_single_register(me, MODULE_NRF24L01_REGISTER_STATUS,
                                                    MODULE_NRF24L01_STATUS_MAXIMUM_RETRANSMIT);
        (void)module_nrf24l01_flush_transmit(me);
        return MODULE_NRF24L01_STATUS_MAXIMUM_RETRANSMIT;
    }
    if ((status_register & MODULE_NRF24L01_STATUS_TRANSMIT_DATA_SENT) != 0U)
    {
        me->transmit_pending = false;
        (void)module_nrf24l01_write_single_register(me, MODULE_NRF24L01_REGISTER_STATUS,
                                                    MODULE_NRF24L01_STATUS_TRANSMIT_DATA_SENT);
        return MODULE_NRF24L01_STATUS_OK;
    }
    return MODULE_NRF24L01_STATUS_BUSY;
}

module_nrf24l01_status_t module_nrf24l01_receive(module_nrf24l01_t *me, uint8_t *payload,
                                                 size_t payload_capacity, uint8_t *pipe_index)
{
    uint8_t status_register;
    uint8_t transmit_buffer[MODULE_NRF24L01_MAXIMUM_PAYLOAD_SIZE + 1U];
    uint8_t receive_buffer[MODULE_NRF24L01_MAXIMUM_PAYLOAD_SIZE + 1U];

    if ((me == NULL) || (payload == NULL) || (payload_capacity < me->payload_size))
    {
        return MODULE_NRF24L01_STATUS_INVALID_ARGUMENT;
    }
    if (!module_device_is_initialized(&me->super))
    {
        return MODULE_NRF24L01_STATUS_NOT_INITIALIZED;
    }
    if (!me->is_started)
    {
        return MODULE_NRF24L01_STATUS_NOT_STARTED;
    }
    if (module_nrf24l01_command(me, MODULE_NRF24L01_COMMAND_NOP, &status_register) !=
        MODULE_NRF24L01_STATUS_OK)
    {
        return MODULE_NRF24L01_STATUS_TRANSPORT_ERROR;
    }
    if ((status_register & MODULE_NRF24L01_STATUS_RECEIVE_DATA_READY) == 0U)
    {
        return MODULE_NRF24L01_STATUS_NO_DATA;
    }
    memset(transmit_buffer, MODULE_NRF24L01_COMMAND_NOP, me->payload_size + 1U);
    transmit_buffer[0] = MODULE_NRF24L01_COMMAND_READ_PAYLOAD;
    if (module_nrf24l01_exchange(me, transmit_buffer, receive_buffer, me->payload_size + 1U) !=
        MODULE_NRF24L01_STATUS_OK)
    {
        return MODULE_NRF24L01_STATUS_TRANSPORT_ERROR;
    }
    memcpy(payload, &receive_buffer[1], me->payload_size);
    if (pipe_index != NULL)
    {
        *pipe_index = (uint8_t)((status_register >> 1U) & 0x07U);
    }
    return module_nrf24l01_write_single_register(me, MODULE_NRF24L01_REGISTER_STATUS,
                                                 MODULE_NRF24L01_STATUS_RECEIVE_DATA_READY);
}

module_nrf24l01_status_t module_nrf24l01_get_observe_transmit(module_nrf24l01_t *me,
                                                              uint8_t *lost_packet_count,
                                                              uint8_t *retransmit_count)
{
    uint8_t observe_transmit;

    if ((me == NULL) || (lost_packet_count == NULL) || (retransmit_count == NULL))
    {
        return MODULE_NRF24L01_STATUS_INVALID_ARGUMENT;
    }
    if (!module_device_is_initialized(&me->super))
    {
        return MODULE_NRF24L01_STATUS_NOT_INITIALIZED;
    }
    if (module_nrf24l01_read_register(me, MODULE_NRF24L01_REGISTER_OBSERVE_TRANSMIT,
                                      &observe_transmit, 1U) != MODULE_NRF24L01_STATUS_OK)
    {
        return MODULE_NRF24L01_STATUS_TRANSPORT_ERROR;
    }
    *lost_packet_count = (uint8_t)(observe_transmit >> 4U);
    *retransmit_count = (uint8_t)(observe_transmit & 0x0FU);
    return MODULE_NRF24L01_STATUS_OK;
}

module_nrf24l01_status_t module_nrf24l01_flush_transmit(module_nrf24l01_t *me)
{
    if (me == NULL)
    {
        return MODULE_NRF24L01_STATUS_INVALID_ARGUMENT;
    }
    if (!module_device_is_initialized(&me->super))
    {
        return MODULE_NRF24L01_STATUS_NOT_INITIALIZED;
    }
    return module_nrf24l01_command(me, MODULE_NRF24L01_COMMAND_FLUSH_TRANSMIT, NULL);
}

module_nrf24l01_status_t module_nrf24l01_flush_receive(module_nrf24l01_t *me)
{
    if (me == NULL)
    {
        return MODULE_NRF24L01_STATUS_INVALID_ARGUMENT;
    }
    if (!module_device_is_initialized(&me->super))
    {
        return MODULE_NRF24L01_STATUS_NOT_INITIALIZED;
    }
    return module_nrf24l01_command(me, MODULE_NRF24L01_COMMAND_FLUSH_RECEIVE, NULL);
}
