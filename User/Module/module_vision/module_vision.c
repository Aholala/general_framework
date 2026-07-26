#include "module_vision.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define MODULE_VISION_MAGIC_FIRST (0xA5U)
#define MODULE_VISION_MAGIC_SECOND (0x5AU)
#define MODULE_VISION_PROTOCOL_VERSION (1U)
#define MODULE_VISION_HEADER_SIZE (8U)
#define MODULE_VISION_CRC_SIZE (2U)
#define MODULE_VISION_TARGET_PAYLOAD_SIZE (25U)

static void module_vision_encode_uint16(uint16_t value, uint8_t output[2])
{
    output[0] = (uint8_t)value;
    output[1] = (uint8_t)(value >> 8U);
}

static uint16_t module_vision_decode_uint16(const uint8_t input[2])
{
    return (uint16_t)input[0] | ((uint16_t)input[1] << 8U);
}

static void module_vision_encode_uint32(uint32_t value, uint8_t output[4])
{
    output[0] = (uint8_t)value;
    output[1] = (uint8_t)(value >> 8U);
    output[2] = (uint8_t)(value >> 16U);
    output[3] = (uint8_t)(value >> 24U);
}

static uint32_t module_vision_decode_uint32(const uint8_t input[4])
{
    return (uint32_t)input[0] | ((uint32_t)input[1] << 8U) | ((uint32_t)input[2] << 16U) |
           ((uint32_t)input[3] << 24U);
}

static void module_vision_encode_float(float value, uint8_t output[4])
{
    uint32_t raw_value;
    (void)memcpy(&raw_value, &value, sizeof(raw_value));
    module_vision_encode_uint32(raw_value, output);
}

static float module_vision_decode_float(const uint8_t input[4])
{
    const uint32_t raw_value = module_vision_decode_uint32(input);
    float value;
    (void)memcpy(&value, &raw_value, sizeof(value));
    return value;
}

static bool module_vision_float_array_is_valid(const float *values, size_t value_count)
{
    size_t value_index;

    for (value_index = 0U; value_index < value_count; ++value_index)
    {
        if (!isfinite(values[value_index]))
        {
            return false;
        }
    }
    return true;
}

static module_vision_status_t module_vision_transmit(module_vision_t *me,
                                                     module_vision_message_t message,
                                                     const uint8_t *payload, uint16_t payload_size)
{
    bool is_busy;
    uint16_t crc;
    const size_t frame_size =
        MODULE_VISION_HEADER_SIZE + (size_t)payload_size + MODULE_VISION_CRC_SIZE;

    if ((payload_size > MODULE_VISION_MAX_PAYLOAD_SIZE) ||
        ((payload_size > 0U) && (payload == NULL)))
    {
        return MODULE_VISION_STATUS_INVALID_ARGUMENT;
    }
    if (bsp_usb_vcp_get_busy(me->usb_vcp, &is_busy) != BSP_STATUS_OK)
    {
        return MODULE_VISION_STATUS_TRANSPORT_ERROR;
    }
    if (is_busy)
    {
        return MODULE_VISION_STATUS_BUSY;
    }
    me->transmit_buffer[0] = MODULE_VISION_MAGIC_FIRST;
    me->transmit_buffer[1] = MODULE_VISION_MAGIC_SECOND;
    me->transmit_buffer[2] = MODULE_VISION_PROTOCOL_VERSION;
    me->transmit_buffer[3] = (uint8_t)message;
    module_vision_encode_uint16(payload_size, &me->transmit_buffer[4]);
    module_vision_encode_uint16(me->transmit_sequence++, &me->transmit_buffer[6]);
    if (payload_size > 0U)
    {
        (void)memcpy(&me->transmit_buffer[MODULE_VISION_HEADER_SIZE], payload, payload_size);
    }
    crc = module_vision_crc16(me->transmit_buffer, MODULE_VISION_HEADER_SIZE + payload_size);
    module_vision_encode_uint16(crc,
                                &me->transmit_buffer[MODULE_VISION_HEADER_SIZE + payload_size]);
    return (bsp_usb_vcp_transmit(me->usb_vcp, me->transmit_buffer, frame_size,
                                 me->transmit_timeout_ms) == BSP_STATUS_OK)
               ? MODULE_VISION_STATUS_OK
               : MODULE_VISION_STATUS_TRANSPORT_ERROR;
}

static module_vision_status_t module_vision_parse_frame(module_vision_t *me, const uint8_t *frame,
                                                        size_t frame_size)
{
    const uint16_t payload_size = module_vision_decode_uint16(&frame[4]);
    const uint16_t received_crc =
        module_vision_decode_uint16(&frame[frame_size - MODULE_VISION_CRC_SIZE]);
    const uint16_t calculated_crc = module_vision_crc16(frame, frame_size - MODULE_VISION_CRC_SIZE);
    const uint8_t *const payload = &frame[MODULE_VISION_HEADER_SIZE];
    float values[5];
    size_t value_index;

    if ((frame[2] != MODULE_VISION_PROTOCOL_VERSION) || (received_crc != calculated_crc))
    {
        return MODULE_VISION_STATUS_INVALID_FRAME;
    }
    if (frame[3] != (uint8_t)MODULE_VISION_MESSAGE_TARGET)
    {
        return MODULE_VISION_STATUS_OK;
    }
    if (payload_size != MODULE_VISION_TARGET_PAYLOAD_SIZE)
    {
        return MODULE_VISION_STATUS_INVALID_FRAME;
    }
    for (value_index = 0U; value_index < 5U; ++value_index)
    {
        values[value_index] = module_vision_decode_float(&payload[value_index * 4U]);
    }
    if (!module_vision_float_array_is_valid(values, 5U) || (values[4] < 0.0F) || (values[4] > 1.0F))
    {
        return MODULE_VISION_STATUS_INVALID_FRAME;
    }
    me->target.target_yaw_rad = values[0];
    me->target.target_pitch_rad = values[1];
    me->target.target_yaw_velocity_rad_per_s = values[2];
    me->target.target_pitch_velocity_rad_per_s = values[3];
    me->target.confidence = values[4];
    me->target.timestamp_ms = module_vision_decode_uint32(&payload[20]);
    me->target.is_tracking = payload[24] != 0U;
    me->target.is_valid = me->target.is_tracking;
    ++me->target.update_count;
    me->target_elapsed_time_ms = 0U;
    return MODULE_VISION_STATUS_OK;
}

uint16_t module_vision_crc16(const uint8_t *frame_data, size_t data_size)
{
    uint16_t crc = 0xFFFFU;
    size_t byte_index;
    uint8_t bit_index;

    if ((frame_data == NULL) && (data_size > 0U))
    {
        return 0U;
    }
    for (byte_index = 0U; byte_index < data_size; ++byte_index)
    {
        crc ^= (uint16_t)frame_data[byte_index] << 8U;
        for (bit_index = 0U; bit_index < 8U; ++bit_index)
        {
            crc =
                ((crc & 0x8000U) != 0U) ? (uint16_t)((crc << 1U) ^ 0x1021U) : (uint16_t)(crc << 1U);
        }
    }
    return crc;
}

module_vision_status_t module_vision_init(module_vision_t *me, const module_vision_config_t *config)
{
    bool is_busy;

    if ((me == NULL) || (config == NULL) || (config->usb_vcp == NULL) ||
        !bsp_device_is_initialized(&config->usb_vcp->super))
    {
        return MODULE_VISION_STATUS_INVALID_ARGUMENT;
    }
    if (bsp_usb_vcp_get_busy(config->usb_vcp, &is_busy) != BSP_STATUS_OK)
    {
        return MODULE_VISION_STATUS_INVALID_ARGUMENT;
    }
    (void)memset(me, 0, sizeof(*me));
    me->usb_vcp = config->usb_vcp;
    me->transmit_timeout_ms = config->transmit_timeout_ms;
    me->target_timeout_ms = config->target_timeout_ms;
    me->is_initialized = true;
    return MODULE_VISION_STATUS_OK;
}

module_vision_status_t module_vision_send_imu(module_vision_t *me,
                                              const module_vision_imu_data_t *imu_data)
{
    uint8_t payload[32];
    size_t value_index;

    if ((me == NULL) || (imu_data == NULL) ||
        !module_vision_float_array_is_valid(imu_data->quaternion, 4U) ||
        !module_vision_float_array_is_valid(imu_data->angular_velocity_rad_per_s, 3U))
    {
        return MODULE_VISION_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return MODULE_VISION_STATUS_NOT_INITIALIZED;
    }
    for (value_index = 0U; value_index < 4U; ++value_index)
    {
        module_vision_encode_float(imu_data->quaternion[value_index], &payload[value_index * 4U]);
    }
    for (value_index = 0U; value_index < 3U; ++value_index)
    {
        module_vision_encode_float(imu_data->angular_velocity_rad_per_s[value_index],
                                   &payload[16U + (value_index * 4U)]);
    }
    module_vision_encode_uint32(imu_data->timestamp_ms, &payload[28]);
    return module_vision_transmit(me, MODULE_VISION_MESSAGE_IMU, payload,
                                  (uint16_t)sizeof(payload));
}

module_vision_status_t module_vision_send_heartbeat(module_vision_t *me, uint32_t uptime_ms)
{
    uint8_t payload[4];

    if (me == NULL)
    {
        return MODULE_VISION_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return MODULE_VISION_STATUS_NOT_INITIALIZED;
    }
    module_vision_encode_uint32(uptime_ms, payload);
    return module_vision_transmit(me, MODULE_VISION_MESSAGE_HEARTBEAT, payload,
                                  (uint16_t)sizeof(payload));
}

module_vision_status_t module_vision_feed_data(module_vision_t *me, const uint8_t *receive_data,
                                               size_t data_size)
{
    size_t byte_index;
    module_vision_status_t last_status = MODULE_VISION_STATUS_OK;

    if ((me == NULL) || ((receive_data == NULL) && (data_size > 0U)))
    {
        return MODULE_VISION_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return MODULE_VISION_STATUS_NOT_INITIALIZED;
    }
    for (byte_index = 0U; byte_index < data_size; ++byte_index)
    {
        uint16_t payload_size;
        size_t expected_frame_size;

        if (me->stream_size >= sizeof(me->stream_buffer))
        {
            me->stream_size = 0U;
        }
        me->stream_buffer[me->stream_size++] = receive_data[byte_index];
        while ((me->stream_size > 0U) &&
               ((me->stream_buffer[0] != MODULE_VISION_MAGIC_FIRST) ||
                ((me->stream_size > 1U) && (me->stream_buffer[1] != MODULE_VISION_MAGIC_SECOND))))
        {
            (void)memmove(me->stream_buffer, &me->stream_buffer[1], --me->stream_size);
        }
        if (me->stream_size < MODULE_VISION_HEADER_SIZE)
        {
            continue;
        }
        payload_size = module_vision_decode_uint16(&me->stream_buffer[4]);
        if (payload_size > MODULE_VISION_MAX_PAYLOAD_SIZE)
        {
            (void)memmove(me->stream_buffer, &me->stream_buffer[1], --me->stream_size);
            last_status = MODULE_VISION_STATUS_INVALID_FRAME;
            continue;
        }
        expected_frame_size = MODULE_VISION_HEADER_SIZE + payload_size + MODULE_VISION_CRC_SIZE;
        if (me->stream_size == expected_frame_size)
        {
            last_status = module_vision_parse_frame(me, me->stream_buffer, expected_frame_size);
            me->stream_size = 0U;
        }
    }
    return last_status;
}

void module_vision_update_time(module_vision_t *me, uint32_t elapsed_time_ms)
{
    if ((me == NULL) || !me->is_initialized)
    {
        return;
    }
    if (me->target_elapsed_time_ms > (UINT32_MAX - elapsed_time_ms))
    {
        me->target_elapsed_time_ms = UINT32_MAX;
    }
    else
    {
        me->target_elapsed_time_ms += elapsed_time_ms;
    }
    if ((me->target_timeout_ms > 0U) && (me->target_elapsed_time_ms > me->target_timeout_ms))
    {
        me->target.is_valid = false;
    }
}

module_vision_status_t module_vision_get_target(const module_vision_t *me,
                                                module_vision_target_t *target)
{
    if ((me == NULL) || (target == NULL))
    {
        return MODULE_VISION_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return MODULE_VISION_STATUS_NOT_INITIALIZED;
    }
    if (!me->target.is_valid)
    {
        return MODULE_VISION_STATUS_NO_TARGET;
    }
    *target = me->target;
    return MODULE_VISION_STATUS_OK;
}
