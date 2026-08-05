#include "app_vision.h"

#include "app_exchange.h"
#include "app_types.h"

#include <string.h>

#define APP_VISION_HEADER_FIRST (0xA5U)
#define APP_VISION_HEADER_SECOND (0x5AU)
#define APP_VISION_MODE_INDEX (2U)
#define APP_VISION_PITCH_INDEX (3U)
#define APP_VISION_YAW_INDEX (7U)
#define APP_VISION_CHECKSUM_INDEX (11U)

static app_vision_config_t app_vision_config;
static app_vision_target_t app_vision_target;
static uint8_t app_vision_receive_frame[APP_VISION_FRAME_SIZE];
static uint8_t app_vision_transmit_frame[APP_VISION_FRAME_SIZE];
static uint32_t app_vision_target_elapsed_ms;
static uint32_t app_vision_transmit_elapsed_ms;
static app_vision_mode_t app_vision_mode;
static bool app_vision_initialized;

static uint8_t app_vision_crc8(const uint8_t *data, size_t data_size)
{
    uint8_t crc = 0xFFU;
    size_t index;
    uint8_t bit;
    for (index = 0U; index < data_size; ++index)
    {
        crc ^= data[index];
        for (bit = 0U; bit < 8U; ++bit)
        {
            crc = ((crc & 0x80U) != 0U) ? (uint8_t)((crc << 1U) ^ 0x31U)
                                         : (uint8_t)(crc << 1U);
        }
    }
    return crc;
}

static bool app_vision_frame_is_valid(const uint8_t *frame)
{
    return (frame[0] == APP_VISION_HEADER_FIRST) && (frame[1] == APP_VISION_HEADER_SECOND) &&
           (frame[APP_VISION_MODE_INDEX] <= (uint8_t)APP_VISION_MODE_AUTOMATIC) &&
           (frame[APP_VISION_CHECKSUM_INDEX] ==
            app_vision_crc8(frame, APP_VISION_CHECKSUM_INDEX));
}

bsp_status_t app_vision_init(const app_vision_config_t *config)
{
    if ((config == NULL) || (config->usb_vcp == NULL) || (config->target_timeout_ms == 0U) ||
        (config->transmit_period_ms == 0U))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    app_vision_config = *config;
    app_vision_target = (app_vision_target_t){0};
    app_vision_target_elapsed_ms = config->target_timeout_ms;
    app_vision_transmit_elapsed_ms = 0U;
    app_vision_mode = APP_VISION_MODE_MANUAL;
    app_vision_initialized = true;
    return BSP_STATUS_OK;
}

void app_vision_set_mode(app_vision_mode_t mode)
{
    if (mode <= APP_VISION_MODE_AUTOMATIC)
    {
        app_vision_mode = mode;
    }
}

void app_vision_update(uint32_t elapsed_time_ms)
{
    app_imu_snapshot_t imu;
    bool usb_busy = false;

    if (!app_vision_initialized)
    {
        return;
    }
    if (app_vision_target_elapsed_ms <= UINT32_MAX - elapsed_time_ms)
    {
        app_vision_target_elapsed_ms += elapsed_time_ms;
    }
    if (app_vision_transmit_elapsed_ms <= UINT32_MAX - elapsed_time_ms)
    {
        app_vision_transmit_elapsed_ms += elapsed_time_ms;
    }

    memset(app_vision_receive_frame, 0, sizeof(app_vision_receive_frame));
    if ((bsp_usb_vcp_receive(app_vision_config.usb_vcp, app_vision_receive_frame,
                             sizeof(app_vision_receive_frame)) == BSP_STATUS_OK) &&
        app_vision_frame_is_valid(app_vision_receive_frame))
    {
        memcpy(&app_vision_target.target_pitch_rad,
               &app_vision_receive_frame[APP_VISION_PITCH_INDEX], sizeof(float));
        memcpy(&app_vision_target.target_yaw_rad, &app_vision_receive_frame[APP_VISION_YAW_INDEX],
               sizeof(float));
        app_vision_target.target_valid =
            app_vision_receive_frame[APP_VISION_MODE_INDEX] == APP_VISION_MODE_AUTOMATIC;
        app_vision_target.tracking_ready = app_vision_target.target_valid;
        ++app_vision_target.update_count;
        app_vision_target_elapsed_ms = 0U;
    }
    if (app_vision_target_elapsed_ms > app_vision_config.target_timeout_ms)
    {
        app_vision_target.target_valid = false;
        app_vision_target.tracking_ready = false;
    }
    app_exchange_publish_vision_target(&app_vision_target);

    if ((app_vision_transmit_elapsed_ms < app_vision_config.transmit_period_ms) ||
        (bsp_usb_vcp_get_busy(app_vision_config.usb_vcp, &usb_busy) != BSP_STATUS_OK) || usb_busy)
    {
        return;
    }
    app_exchange_read_imu(&imu);
    memset(app_vision_transmit_frame, 0, sizeof(app_vision_transmit_frame));
    app_vision_transmit_frame[0] = APP_VISION_HEADER_FIRST;
    app_vision_transmit_frame[1] = APP_VISION_HEADER_SECOND;
    app_vision_transmit_frame[APP_VISION_MODE_INDEX] = (uint8_t)app_vision_mode;
    memcpy(&app_vision_transmit_frame[APP_VISION_PITCH_INDEX], &imu.pitch_rad, sizeof(float));
    memcpy(&app_vision_transmit_frame[APP_VISION_YAW_INDEX], &imu.yaw_rad, sizeof(float));
    app_vision_transmit_frame[APP_VISION_CHECKSUM_INDEX] =
        app_vision_crc8(app_vision_transmit_frame, APP_VISION_CHECKSUM_INDEX);
    if (bsp_usb_vcp_transmit(app_vision_config.usb_vcp, app_vision_transmit_frame,
                             sizeof(app_vision_transmit_frame), 2U) == BSP_STATUS_OK)
    {
        app_vision_transmit_elapsed_ms = 0U;
    }
}
