#ifndef USBD_CONF_H
#define USBD_CONF_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "main.h"
#include "stm32h7xx.h"
#include "stm32h7xx_hal.h"

#define USBD_MAX_NUM_INTERFACES (1U)
#define USBD_MAX_NUM_CONFIGURATION (1U)
#define USBD_MAX_STR_DESC_SIZ (512U)
#define USBD_DEBUG_LEVEL (0U)
#define USBD_LPM_ENABLED (0U)
#define USBD_SELF_POWERED (0U)

#define DEVICE_FS (0U)
#define DEVICE_HS (1U)

#define USBD_malloc (void *)USBD_static_malloc
#define USBD_free USBD_static_free
#define USBD_memset memset
#define USBD_memcpy memcpy
#define USBD_Delay HAL_Delay

#if (USBD_DEBUG_LEVEL > 0U)
#define USBD_UsrLog(...) printf(__VA_ARGS__)
#else
#define USBD_UsrLog(...)
#endif

#if (USBD_DEBUG_LEVEL > 1U)
#define USBD_ErrLog(...) printf(__VA_ARGS__)
#else
#define USBD_ErrLog(...)
#endif

#if (USBD_DEBUG_LEVEL > 2U)
#define USBD_DbgLog(...) printf(__VA_ARGS__)
#else
#define USBD_DbgLog(...)
#endif

void *USBD_static_malloc(uint32_t size);
void USBD_static_free(void *memory);

#ifdef __cplusplus
}
#endif

#endif /* USBD_CONF_H */
