#include "module_device.h"

module_device_status_t module_device_init_base(module_device_t *const me,
                                               const module_device_ops_t *const vptr,
                                               const char *const logical_name,
                                               uint32_t registration_key)
{
    if ((me == NULL) || (vptr == NULL) || (logical_name == NULL))
    {
        return MODULE_DEVICE_STATUS_INVALID_ARGUMENT;
    }
    me->vptr = vptr;
    me->logical_name = logical_name;
    me->registration_key = registration_key;
    me->object_magic = MODULE_DEVICE_OBJECT_MAGIC;
    me->is_initialized = false;
    return MODULE_DEVICE_STATUS_OK;
}

module_device_status_t module_device_complete_init(module_device_t *const me)
{
    if ((me == NULL) || (me->object_magic != MODULE_DEVICE_OBJECT_MAGIC) || (me->vptr == NULL) ||
        (me->logical_name == NULL))
    {
        return MODULE_DEVICE_STATUS_INVALID_ARGUMENT;
    }
    if (me->is_initialized)
    {
        return MODULE_DEVICE_STATUS_ALREADY_INITIALIZED;
    }
    me->is_initialized = true;
    return MODULE_DEVICE_STATUS_OK;
}

void module_device_abort_init(module_device_t *const me)
{
    if (me == NULL)
    {
        return;
    }
    me->vptr = NULL;
    me->logical_name = NULL;
    me->registration_key = 0U;
    me->object_magic = 0U;
    me->is_initialized = false;
}

module_device_status_t module_device_start(module_device_t *const me)
{
    if (me == NULL)
    {
        return MODULE_DEVICE_STATUS_INVALID_ARGUMENT;
    }
    if (!module_device_is_initialized(me))
    {
        return MODULE_DEVICE_STATUS_NOT_INITIALIZED;
    }
    if (me->vptr->start == NULL)
    {
        return MODULE_DEVICE_STATUS_UNSUPPORTED;
    }
    return me->vptr->start(me);
}

module_device_status_t module_device_stop(module_device_t *const me)
{
    if (me == NULL)
    {
        return MODULE_DEVICE_STATUS_INVALID_ARGUMENT;
    }
    if (!module_device_is_initialized(me))
    {
        return MODULE_DEVICE_STATUS_NOT_INITIALIZED;
    }
    if (me->vptr->stop == NULL)
    {
        return MODULE_DEVICE_STATUS_UNSUPPORTED;
    }
    return me->vptr->stop(me);
}

module_device_status_t module_device_update(module_device_t *const me, uint32_t elapsed_time_ms)
{
    if (me == NULL)
    {
        return MODULE_DEVICE_STATUS_INVALID_ARGUMENT;
    }
    if (!module_device_is_initialized(me))
    {
        return MODULE_DEVICE_STATUS_NOT_INITIALIZED;
    }
    if (me->vptr->update == NULL)
    {
        return MODULE_DEVICE_STATUS_UNSUPPORTED;
    }
    return me->vptr->update(me, elapsed_time_ms);
}

bool module_device_is_initialized(const module_device_t *const me)
{
    return (me != NULL) && (me->object_magic == MODULE_DEVICE_OBJECT_MAGIC) && (me->vptr != NULL) &&
           (me->logical_name != NULL) && me->is_initialized;
}

const char *module_device_get_logical_name(const module_device_t *const me)
{
    return module_device_is_initialized(me) ? me->logical_name : NULL;
}

uint32_t module_device_get_registration_key(const module_device_t *const me)
{
    return module_device_is_initialized(me) ? me->registration_key : 0U;
}
