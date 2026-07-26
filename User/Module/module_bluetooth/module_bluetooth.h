#ifndef MODULE_BLUETOOTH_H
#define MODULE_BLUETOOTH_H

#include "bsp_usart.h"
#include "module_device.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef enum
    {
        MODULE_BLUETOOTH_STATUS_OK = 0,
        MODULE_BLUETOOTH_STATUS_INVALID_ARGUMENT,
        MODULE_BLUETOOTH_STATUS_NOT_INITIALIZED,
        MODULE_BLUETOOTH_STATUS_NOT_STARTED,
        MODULE_BLUETOOTH_STATUS_TRANSPORT_ERROR,
        MODULE_BLUETOOTH_STATUS_OFFLINE
    } module_bluetooth_status_t;

    typedef void (*module_bluetooth_receive_callback_t)(const uint8_t *receive_data,
                                                        size_t data_size, void *user_context);

    typedef struct
    {
        bsp_usart_t *usart;
        uint8_t *receive_buffer;
        size_t receive_capacity;
        uint8_t *processing_buffer;
        size_t processing_capacity;
        uint32_t transmit_timeout_ms;
        uint32_t receive_timeout_ms;
        uint32_t offline_timeout_ms;
        bsp_transfer_mode_t receive_mode;
        const char *logical_name;
        uint32_t registration_key;
        module_bluetooth_receive_callback_t receive_callback;
        void *user_context;
    } module_bluetooth_config_t;

    typedef struct
    {
        module_device_t super;
        bsp_usart_t *usart;
        uint8_t *receive_buffer;
        size_t receive_capacity;
        uint8_t *processing_buffer;
        size_t processing_capacity;
        uint32_t transmit_timeout_ms;
        uint32_t receive_timeout_ms;
        uint32_t offline_timeout_ms;
        uint32_t receive_elapsed_time_ms;
        uint32_t receive_overrun_count;
        uint32_t receive_restart_error_count;
        volatile size_t pending_receive_size;
        bsp_transfer_mode_t receive_mode;
        module_bluetooth_receive_callback_t receive_callback;
        void *user_context;
        bool is_online;
        volatile bool is_receive_pending;
        bool is_started;
    } module_bluetooth_t;

    module_bluetooth_status_t module_bluetooth_init(module_bluetooth_t *me,
                                                    const module_bluetooth_config_t *config);
    module_bluetooth_status_t module_bluetooth_start(module_bluetooth_t *me);
    module_bluetooth_status_t module_bluetooth_stop(module_bluetooth_t *me);
    module_bluetooth_status_t module_bluetooth_transmit(module_bluetooth_t *me,
                                                        const uint8_t *transmit_data,
                                                        size_t data_size,
                                                        bsp_transfer_mode_t transfer_mode);
    module_bluetooth_status_t module_bluetooth_send_command(module_bluetooth_t *me,
                                                            const char *command);
    module_bluetooth_status_t module_bluetooth_update(module_bluetooth_t *me,
                                                      uint32_t elapsed_time_ms);
    bool module_bluetooth_is_online(const module_bluetooth_t *me);

#ifdef __cplusplus
}
#endif

#endif /* MODULE_BLUETOOTH_H */
