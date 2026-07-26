#ifndef BSP_GPIO_H
#define BSP_GPIO_H

#include "bsp_common.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct bsp_gpio bsp_gpio_t;
    typedef struct bsp_gpio_device bsp_gpio_device_t;

    typedef struct
    {
        bsp_device_ops_t super;
        bsp_status_t (*read)(const bsp_gpio_t *const me, bool *is_high);
        bsp_status_t (*write)(bsp_gpio_t *const me, bool is_high);
        bsp_status_t (*toggle)(bsp_gpio_t *const me);
    } bsp_gpio_ops_t;

    struct bsp_gpio
    {
        bsp_device_t super;
    };

    typedef struct
    {
        bsp_status_t (*init)(void *device_handle);
        bsp_status_t (*deinit)(void *device_handle);
        bsp_status_t (*read)(const void *device_handle, bool *is_high);
        bsp_status_t (*write)(void *device_handle, bool is_high);
        bsp_status_t (*toggle)(void *device_handle);
    } bsp_gpio_driver_ops_t;

    struct bsp_gpio_device
    {
        bsp_gpio_t super;
        const bsp_gpio_driver_ops_t *driver_ops;
    };

    typedef struct
    {
        void *device_handle;
        const bsp_gpio_driver_ops_t *driver_ops;
    } bsp_gpio_config_t;

    bsp_status_t bsp_gpio_init(bsp_gpio_device_t *const me, const bsp_gpio_config_t *const config);
    bsp_gpio_t *bsp_gpio_as_base(bsp_gpio_device_t *const me);
    bsp_status_t bsp_gpio_read(const bsp_gpio_t *const me, bool *is_high);
    bsp_status_t bsp_gpio_write(bsp_gpio_t *const me, bool is_high);
    bsp_status_t bsp_gpio_toggle(bsp_gpio_t *const me);

#ifdef __cplusplus
}
#endif

#endif /* BSP_GPIO_H */
