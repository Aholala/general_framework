#ifndef MODULE_REFEREE_H
#define MODULE_REFEREE_H

#include "bsp_usart.h"
#include "module_device.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define MODULE_REFEREE_START_OF_FRAME (0xA5U)
#define MODULE_REFEREE_HEADER_SIZE (5U)
#define MODULE_REFEREE_COMMAND_ID_SIZE (2U)
#define MODULE_REFEREE_CRC16_SIZE (2U)
#define MODULE_REFEREE_FRAME_OVERHEAD_SIZE                                                         \
    (MODULE_REFEREE_HEADER_SIZE + MODULE_REFEREE_COMMAND_ID_SIZE + MODULE_REFEREE_CRC16_SIZE)
#define MODULE_REFEREE_FRAME_SIZE(payload_size)                                                    \
    ((size_t)(payload_size) + MODULE_REFEREE_FRAME_OVERHEAD_SIZE)

    typedef enum
    {
        MODULE_REFEREE_STATUS_OK = 0,
        MODULE_REFEREE_STATUS_FRAME_HANDLED,
        MODULE_REFEREE_STATUS_BUSY,
        MODULE_REFEREE_STATUS_INVALID_ARGUMENT,
        MODULE_REFEREE_STATUS_NOT_INITIALIZED,
        MODULE_REFEREE_STATUS_NOT_STARTED,
        MODULE_REFEREE_STATUS_BUFFER_TOO_SMALL,
        MODULE_REFEREE_STATUS_INVALID_FRAME,
        MODULE_REFEREE_STATUS_TRANSPORT_ERROR
    } module_referee_status_t;

    typedef void (*module_referee_command_handler_t)(uint16_t command_id, const uint8_t *payload,
                                                     size_t payload_size, uint8_t sequence,
                                                     void *user_context);

    typedef struct
    {
        uint16_t command_id;
        module_referee_command_handler_t handler;
        void *user_context;
    } module_referee_route_t;

    typedef struct
    {
        uint32_t received_frame_count;
        uint32_t handled_frame_count;
        uint32_t unknown_command_count;
        uint32_t crc8_error_count;
        uint32_t crc16_error_count;
        uint32_t oversize_frame_count;
        uint32_t discarded_byte_count;
        uint32_t receive_overrun_count;
        uint32_t receive_restart_error_count;
    } module_referee_statistics_t;

    typedef struct
    {
        bsp_usart_t *usart;
        uint8_t *receive_buffer;
        size_t receive_capacity;
        uint8_t *processing_buffer;
        size_t processing_capacity;
        uint8_t *stream_buffer;
        size_t stream_capacity;
        uint8_t *transmit_buffer;
        size_t transmit_capacity;
        const module_referee_route_t *routes;
        size_t route_count;
        module_referee_command_handler_t default_handler;
        void *default_user_context;
        uint32_t receive_timeout_ms;
        uint32_t transmit_timeout_ms;
        uint32_t offline_timeout_ms;
        bsp_transfer_mode_t receive_mode;
        const char *logical_name;
        uint32_t registration_key;
    } module_referee_config_t;

    typedef struct
    {
        module_device_t super;
        bsp_usart_t *usart;
        uint8_t *receive_buffer;
        size_t receive_capacity;
        uint8_t *processing_buffer;
        size_t processing_capacity;
        uint8_t *stream_buffer;
        size_t stream_capacity;
        size_t stream_size;
        uint8_t *transmit_buffer;
        size_t transmit_capacity;
        const module_referee_route_t *routes;
        size_t route_count;
        module_referee_command_handler_t default_handler;
        void *default_user_context;
        uint32_t receive_timeout_ms;
        uint32_t transmit_timeout_ms;
        uint32_t offline_timeout_ms;
        uint32_t receive_elapsed_time_ms;
        volatile size_t pending_receive_size;
        uint8_t transmit_sequence;
        bsp_transfer_mode_t receive_mode;
        module_referee_statistics_t statistics;
        bool is_online;
        volatile bool is_receive_pending;
        volatile bool is_transmit_busy;
        bool is_started;
    } module_referee_t;

    module_referee_status_t module_referee_init(module_referee_t *me,
                                                const module_referee_config_t *config);
    module_referee_status_t module_referee_start(module_referee_t *me);
    module_referee_status_t module_referee_stop(module_referee_t *me);
    module_referee_status_t module_referee_feed_data(module_referee_t *me,
                                                     const uint8_t *receive_data, size_t data_size);
    module_referee_status_t module_referee_transmit(module_referee_t *me, uint16_t command_id,
                                                    const uint8_t *payload, size_t payload_size,
                                                    bsp_transfer_mode_t transfer_mode);
    module_referee_status_t module_referee_build_frame(uint8_t *frame_buffer, size_t frame_capacity,
                                                       uint8_t sequence, uint16_t command_id,
                                                       const uint8_t *payload, size_t payload_size,
                                                       size_t *frame_size);
    module_referee_status_t module_referee_update(module_referee_t *me, uint32_t elapsed_time_ms);
    bool module_referee_is_online(const module_referee_t *me);
    module_referee_status_t module_referee_get_statistics(const module_referee_t *me,
                                                          module_referee_statistics_t *statistics);
    uint16_t module_referee_read_uint16_le(const uint8_t *data);
    uint32_t module_referee_read_uint32_le(const uint8_t *data);
    float module_referee_read_float_le(const uint8_t *data);

#ifdef __cplusplus
}
#endif

#endif /* MODULE_REFEREE_H */
