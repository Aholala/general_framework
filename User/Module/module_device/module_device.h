#ifndef MODULE_DEVICE_H
#define MODULE_DEVICE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define MODULE_CONTAINER_OF(member_pointer, parent_type, member_name)                              \
    ((parent_type *)((uint8_t *)(member_pointer) - offsetof(parent_type, member_name)))
#define MODULE_CONTAINER_OF_CONST(member_pointer, parent_type, member_name)                        \
    ((const parent_type *)((const uint8_t *)(member_pointer) - offsetof(parent_type, member_name)))
#define MODULE_DEVICE_OBJECT_MAGIC (0x4D444556UL)

    typedef struct module_device module_device_t;

    typedef enum
    {
        MODULE_DEVICE_STATUS_OK = 0,
        MODULE_DEVICE_STATUS_INVALID_ARGUMENT,
        MODULE_DEVICE_STATUS_NOT_INITIALIZED,
        MODULE_DEVICE_STATUS_ALREADY_INITIALIZED,
        MODULE_DEVICE_STATUS_UNSUPPORTED,
        MODULE_DEVICE_STATUS_OPERATION_FAILED
    } module_device_status_t;

    typedef struct
    {
        module_device_status_t (*start)(module_device_t *const me);
        module_device_status_t (*stop)(module_device_t *const me);
        module_device_status_t (*update)(module_device_t *const me, uint32_t elapsed_time_ms);
    } module_device_ops_t;

    struct module_device
    {
        const module_device_ops_t *vptr;
        const char *logical_name;
        uint32_t registration_key;
        uint32_t object_magic;
        bool is_initialized;
    };

    module_device_status_t module_device_init_base(module_device_t *const me,
                                                   const module_device_ops_t *const vptr,
                                                   const char *const logical_name,
                                                   uint32_t registration_key);
    module_device_status_t module_device_complete_init(module_device_t *const me);
    void module_device_abort_init(module_device_t *const me);
    module_device_status_t module_device_start(module_device_t *const me);
    module_device_status_t module_device_stop(module_device_t *const me);
    module_device_status_t module_device_update(module_device_t *const me,
                                                uint32_t elapsed_time_ms);
    bool module_device_is_initialized(const module_device_t *const me);
    const char *module_device_get_logical_name(const module_device_t *const me);
    uint32_t module_device_get_registration_key(const module_device_t *const me);

#ifdef __cplusplus
}
#endif

#endif /* MODULE_DEVICE_H */
