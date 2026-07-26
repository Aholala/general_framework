#ifndef MODULE_NRF24L01_H
#define MODULE_NRF24L01_H

#include "bsp_gpio.h"
#include "bsp_spi.h"
#include "module_device.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define MODULE_NRF24L01_MAXIMUM_PAYLOAD_SIZE (32U)
#define MODULE_NRF24L01_MAXIMUM_ADDRESS_SIZE (5U)

    typedef enum
    {
        MODULE_NRF24L01_STATUS_OK = 0,
        MODULE_NRF24L01_STATUS_NO_DATA,
        MODULE_NRF24L01_STATUS_BUSY,
        MODULE_NRF24L01_STATUS_MAXIMUM_RETRANSMIT,
        MODULE_NRF24L01_STATUS_INVALID_ARGUMENT,
        MODULE_NRF24L01_STATUS_NOT_INITIALIZED,
        MODULE_NRF24L01_STATUS_NOT_STARTED,
        MODULE_NRF24L01_STATUS_TRANSPORT_ERROR,
        MODULE_NRF24L01_STATUS_DEVICE_NOT_FOUND
    } module_nrf24l01_status_t;

    typedef enum
    {
        MODULE_NRF24L01_DATA_RATE_1_MBPS = 0,
        MODULE_NRF24L01_DATA_RATE_2_MBPS,
        MODULE_NRF24L01_DATA_RATE_250_KBPS
    } module_nrf24l01_data_rate_t;

    typedef enum
    {
        MODULE_NRF24L01_OUTPUT_POWER_NEGATIVE_18_DBM = 0,
        MODULE_NRF24L01_OUTPUT_POWER_NEGATIVE_12_DBM,
        MODULE_NRF24L01_OUTPUT_POWER_NEGATIVE_6_DBM,
        MODULE_NRF24L01_OUTPUT_POWER_0_DBM
    } module_nrf24l01_output_power_t;

    typedef enum
    {
        MODULE_NRF24L01_MODE_STANDBY = 0,
        MODULE_NRF24L01_MODE_RECEIVE,
        MODULE_NRF24L01_MODE_TRANSMIT
    } module_nrf24l01_mode_t;

    typedef void (*module_nrf24l01_delay_us_t)(uint32_t delay_us, void *user_context);

    typedef struct
    {
        bsp_spi_t *spi;
        bsp_gpio_t *chip_enable_gpio;
        bsp_gpio_t *chip_select_gpio;
        uint8_t channel;
        uint8_t address_size;
        uint8_t payload_size;
        uint8_t automatic_retransmit_count;
        uint16_t automatic_retransmit_delay_us;
        module_nrf24l01_data_rate_t data_rate;
        module_nrf24l01_output_power_t output_power;
        bool automatic_acknowledge_enabled;
        uint32_t spi_timeout_ms;
        module_nrf24l01_delay_us_t delay_us;
        void *delay_user_context;
        const char *logical_name;
        uint32_t registration_key;
    } module_nrf24l01_config_t;

    typedef struct
    {
        module_device_t super;
        bsp_spi_t *spi;
        bsp_gpio_t *chip_enable_gpio;
        bsp_gpio_t *chip_select_gpio;
        uint8_t channel;
        uint8_t address_size;
        uint8_t payload_size;
        uint8_t configuration_register;
        uint8_t radio_frequency_setup_register;
        uint8_t automatic_retransmit_setup_register;
        uint32_t spi_timeout_ms;
        module_nrf24l01_delay_us_t delay_us;
        void *delay_user_context;
        module_nrf24l01_mode_t mode;
        bool transmit_pending;
        bool is_started;
    } module_nrf24l01_t;

    module_nrf24l01_status_t module_nrf24l01_init(module_nrf24l01_t *me,
                                                  const module_nrf24l01_config_t *config);
    module_nrf24l01_status_t module_nrf24l01_start(module_nrf24l01_t *me);
    module_nrf24l01_status_t module_nrf24l01_stop(module_nrf24l01_t *me);
    module_nrf24l01_status_t module_nrf24l01_set_receive_address(module_nrf24l01_t *me,
                                                                 uint8_t pipe_index,
                                                                 const uint8_t *address,
                                                                 size_t address_size);
    module_nrf24l01_status_t module_nrf24l01_set_receive_pipe_enabled(module_nrf24l01_t *me,
                                                                      uint8_t pipe_index,
                                                                      bool is_enabled);
    module_nrf24l01_status_t module_nrf24l01_set_transmit_address(module_nrf24l01_t *me,
                                                                  const uint8_t *address,
                                                                  size_t address_size);
    module_nrf24l01_status_t module_nrf24l01_start_receive(module_nrf24l01_t *me);
    module_nrf24l01_status_t module_nrf24l01_transmit(module_nrf24l01_t *me, const uint8_t *payload,
                                                      size_t payload_size);
    module_nrf24l01_status_t module_nrf24l01_poll_transmit(module_nrf24l01_t *me);
    module_nrf24l01_status_t module_nrf24l01_receive(module_nrf24l01_t *me, uint8_t *payload,
                                                     size_t payload_capacity, uint8_t *pipe_index);
    module_nrf24l01_status_t module_nrf24l01_get_observe_transmit(module_nrf24l01_t *me,
                                                                  uint8_t *lost_packet_count,
                                                                  uint8_t *retransmit_count);
    module_nrf24l01_status_t module_nrf24l01_flush_transmit(module_nrf24l01_t *me);
    module_nrf24l01_status_t module_nrf24l01_flush_receive(module_nrf24l01_t *me);

#ifdef __cplusplus
}
#endif

#endif /* MODULE_NRF24L01_H */
