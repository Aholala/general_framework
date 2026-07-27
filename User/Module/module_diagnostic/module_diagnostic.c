#include "module_diagnostic.h"

static module_device_status_t module_diagnostic_start_device(module_device_t *device);
static module_device_status_t module_diagnostic_stop_device(module_device_t *device);
static module_device_status_t module_diagnostic_update_device(module_device_t *device,
                                                              uint32_t elapsed_time_ms);

static const module_device_ops_t module_diagnostic_device_ops = {
    .start = module_diagnostic_start_device,
    .stop = module_diagnostic_stop_device,
    .update = module_diagnostic_update_device,
};

static uint32_t module_diagnostic_add_time(uint32_t accumulated, uint32_t elapsed)
{
    return (elapsed > UINT32_MAX - accumulated) ? UINT32_MAX : accumulated + elapsed;
}

module_device_status_t module_diagnostic_init(module_diagnostic_t *me,
                                              const module_diagnostic_config_t *config)
{
    size_t entry_index;
    if ((me == NULL) || (config == NULL) || (config->entries == NULL) ||
        (config->state_storage == NULL) || (config->entry_count == 0U))
    {
        return MODULE_DEVICE_STATUS_INVALID_ARGUMENT;
    }
    if (module_device_init_base(&me->super, &module_diagnostic_device_ops, config->logical_name,
                                config->registration_key) != MODULE_DEVICE_STATUS_OK)
    {
        return MODULE_DEVICE_STATUS_INVALID_ARGUMENT;
    }
    for (entry_index = 0U; entry_index < config->entry_count; ++entry_index)
    {
        size_t previous_index;
        if ((config->entries[entry_index].probe == NULL) ||
            (config->entries[entry_index].severity > MODULE_DIAGNOSTIC_SEVERITY_FATAL))
        {
            module_device_abort_init(&me->super);
            return MODULE_DEVICE_STATUS_INVALID_ARGUMENT;
        }
        for (previous_index = 0U; previous_index < entry_index; ++previous_index)
        {
            if (config->entries[previous_index].diagnostic_id ==
                config->entries[entry_index].diagnostic_id)
            {
                module_device_abort_init(&me->super);
                return MODULE_DEVICE_STATUS_INVALID_ARGUMENT;
            }
        }
        config->state_storage[entry_index] = (module_diagnostic_state_t){0};
    }
    me->entries = config->entries;
    me->states = config->state_storage;
    me->entry_count = config->entry_count;
    me->event_callback = config->event_callback;
    me->event_user_context = config->event_user_context;
    me->highest_active_severity = MODULE_DIAGNOSTIC_SEVERITY_INFO;
    me->active_count = 0U;
    me->is_started = false;
    return module_device_complete_init(&me->super);
}

module_device_status_t module_diagnostic_clear_latched(module_diagnostic_t *me,
                                                       uint16_t diagnostic_id)
{
    size_t entry_index;
    if ((me == NULL) || !module_device_is_initialized(&me->super))
    {
        return MODULE_DEVICE_STATUS_NOT_INITIALIZED;
    }
    for (entry_index = 0U; entry_index < me->entry_count; ++entry_index)
    {
        if (me->entries[entry_index].diagnostic_id == diagnostic_id)
        {
            if (me->states[entry_index].is_active)
            {
                return MODULE_DEVICE_STATUS_OPERATION_FAILED;
            }
            me->states[entry_index].is_latched = false;
            return MODULE_DEVICE_STATUS_OK;
        }
    }
    return MODULE_DEVICE_STATUS_INVALID_ARGUMENT;
}

const module_diagnostic_state_t *module_diagnostic_get_state(const module_diagnostic_t *me,
                                                             uint16_t diagnostic_id)
{
    size_t entry_index;
    if ((me == NULL) || !module_device_is_initialized(&me->super))
    {
        return NULL;
    }
    for (entry_index = 0U; entry_index < me->entry_count; ++entry_index)
    {
        if (me->entries[entry_index].diagnostic_id == diagnostic_id)
        {
            return &me->states[entry_index];
        }
    }
    return NULL;
}

bool module_diagnostic_has_severity(const module_diagnostic_t *me,
                                    module_diagnostic_severity_t minimum_severity)
{
    return (me != NULL) && module_device_is_initialized(&me->super) && (me->active_count > 0U) &&
           (me->highest_active_severity >= minimum_severity);
}

static module_device_status_t module_diagnostic_start_device(module_device_t *device)
{
    module_diagnostic_t *const me = MODULE_CONTAINER_OF(device, module_diagnostic_t, super);
    me->is_started = true;
    return MODULE_DEVICE_STATUS_OK;
}

static module_device_status_t module_diagnostic_stop_device(module_device_t *device)
{
    module_diagnostic_t *const me = MODULE_CONTAINER_OF(device, module_diagnostic_t, super);
    me->is_started = false;
    return MODULE_DEVICE_STATUS_OK;
}

static module_device_status_t module_diagnostic_update_device(module_device_t *device,
                                                              uint32_t elapsed_time_ms)
{
    module_diagnostic_t *const me = MODULE_CONTAINER_OF(device, module_diagnostic_t, super);
    size_t entry_index;
    if (!me->is_started)
    {
        return MODULE_DEVICE_STATUS_OPERATION_FAILED;
    }
    me->active_count = 0U;
    me->highest_active_severity = MODULE_DIAGNOSTIC_SEVERITY_INFO;
    for (entry_index = 0U; entry_index < me->entry_count; ++entry_index)
    {
        const module_diagnostic_entry_t *const entry = &me->entries[entry_index];
        module_diagnostic_state_t *const state = &me->states[entry_index];
        uint32_t detail_code = 0U;
        const bool healthy = entry->probe(entry->user_context, &detail_code);
        const bool previous_active = state->is_active;
        state->detail_code = detail_code;
        if (!healthy)
        {
            state->recovery_elapsed_time_ms = 0U;
            state->fault_elapsed_time_ms =
                module_diagnostic_add_time(state->fault_elapsed_time_ms, elapsed_time_ms);
            if (state->fault_elapsed_time_ms >= entry->confirmation_time_ms)
            {
                state->is_active = true;
                state->is_latched = entry->is_latched || state->is_latched;
                if (!previous_active)
                {
                    ++state->occurrence_count;
                }
            }
        }
        else
        {
            state->fault_elapsed_time_ms = 0U;
            state->recovery_elapsed_time_ms =
                module_diagnostic_add_time(state->recovery_elapsed_time_ms, elapsed_time_ms);
            if (state->recovery_elapsed_time_ms >= entry->recovery_time_ms)
            {
                state->is_active = false;
            }
        }
        if (state->is_active)
        {
            ++me->active_count;
            if (entry->severity > me->highest_active_severity)
            {
                me->highest_active_severity = entry->severity;
            }
        }
        if ((previous_active != state->is_active) && (me->event_callback != NULL))
        {
            me->event_callback(entry, state, state->is_active, me->event_user_context);
        }
    }
    return MODULE_DEVICE_STATUS_OK;
}
