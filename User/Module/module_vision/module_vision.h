#ifndef MODULE_VISION_H
#define MODULE_VISION_H

#include "bsp_usb_vcp.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define MODULE_VISION_MAX_PAYLOAD_SIZE (48U)
#define MODULE_VISION_MAX_FRAME_SIZE (58U)

    typedef enum
    {
        MODULE_VISION_STATUS_OK = 0,
        MODULE_VISION_STATUS_INVALID_ARGUMENT,
        MODULE_VISION_STATUS_NOT_INITIALIZED,
        MODULE_VISION_STATUS_TRANSPORT_ERROR,
        MODULE_VISION_STATUS_BUSY,
        MODULE_VISION_STATUS_INVALID_FRAME,
        MODULE_VISION_STATUS_NO_TARGET
    } module_vision_status_t;

    typedef enum
    {
        MODULE_VISION_MESSAGE_IMU = 1,
        MODULE_VISION_MESSAGE_TARGET = 2,
        MODULE_VISION_MESSAGE_HEARTBEAT = 3
    } module_vision_message_t;

    typedef struct
    {
        float quaternion[4];
        float angular_velocity_rad_per_s[3];
        uint32_t timestamp_ms;
    } module_vision_imu_data_t;

    typedef struct
    {
        float target_yaw_rad;
        float target_pitch_rad;
        float target_yaw_velocity_rad_per_s;
        float target_pitch_velocity_rad_per_s;
        float confidence;
        uint32_t timestamp_ms;
        uint32_t update_count;
        bool is_tracking;
        bool is_valid;
    } module_vision_target_t;

    typedef struct
    {
        bsp_usb_vcp_t *usb_vcp;
        uint32_t transmit_timeout_ms;
        uint32_t target_timeout_ms;
    } module_vision_config_t;

    typedef struct
    {
        bsp_usb_vcp_t *usb_vcp;
        uint32_t transmit_timeout_ms;
        uint32_t target_timeout_ms;
        uint32_t target_elapsed_time_ms;
        uint16_t transmit_sequence;
        uint8_t transmit_buffer[MODULE_VISION_MAX_FRAME_SIZE];
        uint8_t stream_buffer[MODULE_VISION_MAX_FRAME_SIZE];
        size_t stream_size;
        module_vision_target_t target;
        bool is_initialized;
    } module_vision_t;

    module_vision_status_t module_vision_init(module_vision_t *me,
                                              const module_vision_config_t *config);
    module_vision_status_t module_vision_send_imu(module_vision_t *me,
                                                  const module_vision_imu_data_t *imu_data);
    module_vision_status_t module_vision_send_heartbeat(module_vision_t *me, uint32_t uptime_ms);
    module_vision_status_t module_vision_feed_data(module_vision_t *me, const uint8_t *receive_data,
                                                   size_t data_size);
    void module_vision_update_time(module_vision_t *me, uint32_t elapsed_time_ms);
    module_vision_status_t module_vision_get_target(const module_vision_t *me,
                                                    module_vision_target_t *target);
    uint16_t module_vision_crc16(const uint8_t *frame_data, size_t data_size);

#ifdef __cplusplus
}
#endif

#endif /* MODULE_VISION_H */
