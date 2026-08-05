#ifndef BSP_GPIO_H
#define BSP_GPIO_H

#include "bsp_common.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    bsp_status_t (*init)(void *handle);
    bsp_status_t (*deinit)(void *handle);
    bsp_status_t (*read)(const void *handle, bool *level);
    bsp_status_t (*write)(void *handle, bool level);
    bsp_status_t (*toggle)(void *handle);
} bsp_gpio_driver_ops_t;

typedef struct bsp_gpio
{
    void *device_handle;
    bool is_initialized;
} bsp_gpio_t;

/* Compatibility alias: GPIO is now a lightweight resource handle, not a derived object. */
typedef bsp_gpio_t bsp_gpio_device_t;

typedef struct
{
    void *device_handle;
    const bsp_gpio_driver_ops_t *driver_ops;
} bsp_gpio_config_t;

bsp_status_t bsp_gpio_bind_platform(const bsp_gpio_driver_ops_t *driver_ops);
bsp_status_t bsp_gpio_init(bsp_gpio_t *me, const bsp_gpio_config_t *config);
bsp_status_t bsp_gpio_deinit(bsp_gpio_t *me);
bool bsp_gpio_is_initialized(const bsp_gpio_t *me);
bsp_gpio_t *bsp_gpio_as_base(bsp_gpio_device_t *me);
bsp_status_t bsp_gpio_read(const bsp_gpio_t *me, bool *level);
bsp_status_t bsp_gpio_write(bsp_gpio_t *me, bool level);
bsp_status_t bsp_gpio_toggle(bsp_gpio_t *me);

#ifdef __cplusplus
}
#endif
#endif
