#ifndef BSP_I2C_H
#define BSP_I2C_H

#include "bsp_common.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct bsp_i2c bsp_i2c_t;
    typedef struct bsp_i2c_device bsp_i2c_device_t;

    typedef enum
    {
        BSP_I2C_MEMORY_ADDRESS_8_BIT = 1,
        BSP_I2C_MEMORY_ADDRESS_16_BIT = 2
    } bsp_i2c_memory_address_size_t;

    typedef struct
    {
        bsp_device_ops_t super;
        bsp_status_t (*transmit)(bsp_i2c_t *const, uint16_t, const uint8_t *, size_t,
                                 bsp_transfer_mode_t, uint32_t);
        bsp_status_t (*receive)(bsp_i2c_t *const, uint16_t, uint8_t *, size_t, bsp_transfer_mode_t,
                                uint32_t);
        bsp_status_t (*memory_write)(bsp_i2c_t *const, uint16_t, uint16_t,
                                     bsp_i2c_memory_address_size_t, const uint8_t *, size_t,
                                     bsp_transfer_mode_t, uint32_t);
        bsp_status_t (*memory_read)(bsp_i2c_t *const, uint16_t, uint16_t,
                                    bsp_i2c_memory_address_size_t, uint8_t *, size_t,
                                    bsp_transfer_mode_t, uint32_t);
        bsp_status_t (*is_device_ready)(bsp_i2c_t *const, uint16_t, uint32_t, uint32_t);
        bsp_status_t (*abort)(bsp_i2c_t *const, uint16_t);
        bsp_status_t (*get_busy)(const bsp_i2c_t *const, bool *);
    } bsp_i2c_ops_t;

    struct bsp_i2c
    {
        bsp_device_t super;
        bsp_event_callback_t callback;
        void *user_context;
    };

    typedef struct
    {
        bsp_status_t (*init)(void *);
        bsp_status_t (*deinit)(void *);
        bsp_status_t (*transmit)(void *, uint16_t, const uint8_t *, size_t, bsp_transfer_mode_t,
                                 uint32_t);
        bsp_status_t (*receive)(void *, uint16_t, uint8_t *, size_t, bsp_transfer_mode_t, uint32_t);
        bsp_status_t (*memory_write)(void *, uint16_t, uint16_t, bsp_i2c_memory_address_size_t,
                                     const uint8_t *, size_t, bsp_transfer_mode_t, uint32_t);
        bsp_status_t (*memory_read)(void *, uint16_t, uint16_t, bsp_i2c_memory_address_size_t,
                                    uint8_t *, size_t, bsp_transfer_mode_t, uint32_t);
        bsp_status_t (*is_device_ready)(void *, uint16_t, uint32_t, uint32_t);
        bsp_status_t (*abort)(void *, uint16_t);
        bsp_status_t (*get_busy)(const void *, bool *);
    } bsp_i2c_driver_ops_t;

    struct bsp_i2c_device
    {
        bsp_i2c_t super;
        const bsp_i2c_driver_ops_t *driver_ops;
    };

    typedef struct
    {
        void *device_handle;
        const bsp_i2c_driver_ops_t *driver_ops;
        bsp_event_callback_t callback;
        void *user_context;
    } bsp_i2c_config_t;

    bsp_status_t bsp_i2c_init(bsp_i2c_device_t *const me, const bsp_i2c_config_t *const config);
    bsp_i2c_t *bsp_i2c_as_base(bsp_i2c_device_t *const me);
    bsp_status_t bsp_i2c_set_callback(bsp_i2c_t *const me, bsp_event_callback_t callback,
                                      void *user_context);
    bsp_status_t bsp_i2c_transmit(bsp_i2c_t *const me, uint16_t address_7bit, const uint8_t *data,
                                  size_t size, bsp_transfer_mode_t mode, uint32_t timeout_ms);
    bsp_status_t bsp_i2c_receive(bsp_i2c_t *const me, uint16_t address_7bit, uint8_t *data,
                                 size_t size, bsp_transfer_mode_t mode, uint32_t timeout_ms);
    bsp_status_t bsp_i2c_memory_write(bsp_i2c_t *const me, uint16_t address_7bit,
                                      uint16_t memory_address,
                                      bsp_i2c_memory_address_size_t address_size,
                                      const uint8_t *data, size_t size, bsp_transfer_mode_t mode,
                                      uint32_t timeout_ms);
    bsp_status_t bsp_i2c_memory_read(bsp_i2c_t *const me, uint16_t address_7bit,
                                     uint16_t memory_address,
                                     bsp_i2c_memory_address_size_t address_size, uint8_t *data,
                                     size_t size, bsp_transfer_mode_t mode, uint32_t timeout_ms);
    bsp_status_t bsp_i2c_is_device_ready(bsp_i2c_t *const me, uint16_t address_7bit,
                                         uint32_t trial_count, uint32_t timeout_ms);
    bsp_status_t bsp_i2c_abort(bsp_i2c_t *const me, uint16_t address_7bit);
    bsp_status_t bsp_i2c_get_busy(const bsp_i2c_t *const me, bool *is_busy);
    void bsp_i2c_notify(bsp_i2c_t *const me, bsp_event_t event, bsp_status_t status,
                        size_t transferred_size);

#ifdef __cplusplus
}
#endif

#endif /* BSP_I2C_H */
