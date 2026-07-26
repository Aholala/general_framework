#ifndef BSP_SPI_H
#define BSP_SPI_H

#include "bsp_common.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct bsp_spi bsp_spi_t;
    typedef struct bsp_spi_device bsp_spi_device_t;

    typedef struct
    {
        bsp_device_ops_t super;
        bsp_status_t (*transmit)(bsp_spi_t *const, const uint8_t *, size_t, bsp_transfer_mode_t,
                                 uint32_t);
        bsp_status_t (*receive)(bsp_spi_t *const, uint8_t *, size_t, bsp_transfer_mode_t, uint32_t);
        bsp_status_t (*exchange)(bsp_spi_t *const, const uint8_t *, uint8_t *, size_t,
                                 bsp_transfer_mode_t, uint32_t);
        bsp_status_t (*abort)(bsp_spi_t *const);
        bsp_status_t (*get_busy)(const bsp_spi_t *const, bool *);
    } bsp_spi_ops_t;

    struct bsp_spi
    {
        bsp_device_t super;
        bsp_event_callback_t callback;
        void *user_context;
    };

    typedef struct
    {
        bsp_status_t (*init)(void *);
        bsp_status_t (*deinit)(void *);
        bsp_status_t (*transmit)(void *, const uint8_t *, size_t, bsp_transfer_mode_t, uint32_t);
        bsp_status_t (*receive)(void *, uint8_t *, size_t, bsp_transfer_mode_t, uint32_t);
        bsp_status_t (*exchange)(void *, const uint8_t *, uint8_t *, size_t, bsp_transfer_mode_t,
                                 uint32_t);
        bsp_status_t (*abort)(void *);
        bsp_status_t (*get_busy)(const void *, bool *);
    } bsp_spi_driver_ops_t;

    struct bsp_spi_device
    {
        bsp_spi_t super;
        const bsp_spi_driver_ops_t *driver_ops;
    };

    typedef struct
    {
        void *device_handle;
        const bsp_spi_driver_ops_t *driver_ops;
        bsp_event_callback_t callback;
        void *user_context;
    } bsp_spi_config_t;

    bsp_status_t bsp_spi_init(bsp_spi_device_t *const me, const bsp_spi_config_t *const config);
    bsp_spi_t *bsp_spi_as_base(bsp_spi_device_t *const me);
    bsp_status_t bsp_spi_set_callback(bsp_spi_t *const me, bsp_event_callback_t callback,
                                      void *user_context);
    bsp_status_t bsp_spi_transmit(bsp_spi_t *const me, const uint8_t *data, size_t size,
                                  bsp_transfer_mode_t mode, uint32_t timeout_ms);
    bsp_status_t bsp_spi_receive(bsp_spi_t *const me, uint8_t *data, size_t size,
                                 bsp_transfer_mode_t mode, uint32_t timeout_ms);
    bsp_status_t bsp_spi_exchange(bsp_spi_t *const me, const uint8_t *transmit_data,
                                  uint8_t *receive_data, size_t size, bsp_transfer_mode_t mode,
                                  uint32_t timeout_ms);
    bsp_status_t bsp_spi_abort(bsp_spi_t *const me);
    bsp_status_t bsp_spi_get_busy(const bsp_spi_t *const me, bool *is_busy);
    void bsp_spi_notify(bsp_spi_t *const me, bsp_event_t event, bsp_status_t status,
                        size_t transferred_size);

#ifdef __cplusplus
}
#endif

#endif /* BSP_SPI_H */
