#ifndef BSP_STORAGE_H
#define BSP_STORAGE_H

#include "bsp_common.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct bsp_storage bsp_storage_t;
    typedef struct bsp_storage_device bsp_storage_device_t;

    typedef struct
    {
        uint64_t capacity_bytes;
        uint32_t read_alignment_bytes;
        uint32_t program_alignment_bytes;
        uint32_t erase_block_bytes;
        bool erase_is_required;
        bool supports_memory_mapping;
    } bsp_storage_geometry_t;

    typedef struct
    {
        bsp_device_ops_t super;
        bsp_status_t (*read)(bsp_storage_t *me, uint64_t address, void *data, size_t size);
        bsp_status_t (*program)(bsp_storage_t *me, uint64_t address, const void *data, size_t size);
        bsp_status_t (*erase)(bsp_storage_t *me, uint64_t address, size_t size);
        bsp_status_t (*sync)(bsp_storage_t *me);
        bsp_status_t (*get_geometry)(const bsp_storage_t *me, bsp_storage_geometry_t *geometry);
    } bsp_storage_ops_t;

    struct bsp_storage
    {
        bsp_device_t super;
    };

    typedef struct
    {
        bsp_status_t (*init)(void *handle);
        bsp_status_t (*deinit)(void *handle);
        bsp_status_t (*read)(void *handle, uint64_t address, void *data, size_t size);
        bsp_status_t (*program)(void *handle, uint64_t address, const void *data, size_t size);
        bsp_status_t (*erase)(void *handle, uint64_t address, size_t size);
        bsp_status_t (*sync)(void *handle);
        bsp_status_t (*get_geometry)(const void *handle, bsp_storage_geometry_t *geometry);
    } bsp_storage_driver_ops_t;

    struct bsp_storage_device
    {
        bsp_storage_t super;
        const bsp_storage_driver_ops_t *driver_ops;
    };

    typedef struct
    {
        void *device_handle;
        const bsp_storage_driver_ops_t *driver_ops;
    } bsp_storage_config_t;

    bsp_status_t bsp_storage_init(bsp_storage_device_t *me, const bsp_storage_config_t *config);
    bsp_storage_t *bsp_storage_as_base(bsp_storage_device_t *me);
    bsp_status_t bsp_storage_read(bsp_storage_t *me, uint64_t address, void *data, size_t size);
    bsp_status_t bsp_storage_program(bsp_storage_t *me, uint64_t address, const void *data,
                                     size_t size);
    bsp_status_t bsp_storage_erase(bsp_storage_t *me, uint64_t address, size_t size);
    bsp_status_t bsp_storage_sync(bsp_storage_t *me);
    bsp_status_t bsp_storage_get_geometry(const bsp_storage_t *me,
                                          bsp_storage_geometry_t *geometry);

#ifdef __cplusplus
}
#endif

#endif
