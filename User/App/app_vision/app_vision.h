#ifndef APP_VISION_H
#define APP_VISION_H

#include "bsp_usb_vcp.h"

#include <stdbool.h>
#include <stdint.h>

#define APP_VISION_FRAME_SIZE (12U)

typedef enum
{
    APP_VISION_MODE_MANUAL = 0,
    APP_VISION_MODE_AUTOMATIC = 1
} app_vision_mode_t;

typedef struct
{
    bsp_usb_vcp_t *usb_vcp;
    uint32_t target_timeout_ms;
    uint32_t transmit_period_ms;
} app_vision_config_t;

bool app_vision_init(const app_vision_config_t *config);
void app_vision_set_mode(app_vision_mode_t mode);
void app_vision_update(uint32_t elapsed_time_ms);

#endif
