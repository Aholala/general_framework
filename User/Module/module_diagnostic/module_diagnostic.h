/**
 * @file module_diagnostic.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 
 * @version 1.0
 * @date 2026-07-28
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#ifndef MODULE_DIAGNOSTIC_H
#define MODULE_DIAGNOSTIC_H

#include "module_device.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef enum
    {
        MODULE_DIAGNOSTIC_SEVERITY_INFO = 0,
        MODULE_DIAGNOSTIC_SEVERITY_WARNING,
        MODULE_DIAGNOSTIC_SEVERITY_ERROR,
        MODULE_DIAGNOSTIC_SEVERITY_FATAL
    } module_diagnostic_severity_t;

    typedef bool (*module_diagnostic_probe_t)(void *user_context, uint32_t *detail_code);

    typedef struct
    {
        uint16_t diagnostic_id;
        module_diagnostic_severity_t severity;
        module_diagnostic_probe_t probe;
        void *user_context;
        uint32_t confirmation_time_ms;
        uint32_t recovery_time_ms;
        bool is_latched;
    } module_diagnostic_entry_t;

    typedef struct
    {
        uint32_t detail_code;
        uint32_t fault_elapsed_time_ms;
        uint32_t recovery_elapsed_time_ms;
        uint32_t occurrence_count;
        bool is_active;
        bool is_latched;
    } module_diagnostic_state_t;

    typedef void (*module_diagnostic_event_callback_t)(const module_diagnostic_entry_t *entry,
                                                       const module_diagnostic_state_t *state,
                                                       bool became_active, void *user_context);

    typedef struct
    {
        const module_diagnostic_entry_t *entries;
        module_diagnostic_state_t *state_storage;
        size_t entry_count;
        module_diagnostic_event_callback_t event_callback;
        void *event_user_context;
        const char *logical_name;
        uint32_t registration_key;
    } module_diagnostic_config_t;

    typedef struct
    {
        module_device_t super;
        const module_diagnostic_entry_t *entries;
        module_diagnostic_state_t *states;
        size_t entry_count;
        module_diagnostic_event_callback_t event_callback;
        void *event_user_context;
        module_diagnostic_severity_t highest_active_severity;
        uint32_t active_count;
        bool is_started;
    } module_diagnostic_t;

    module_device_status_t module_diagnostic_init(module_diagnostic_t *me,
                                                  const module_diagnostic_config_t *config);
    module_device_status_t module_diagnostic_clear_latched(module_diagnostic_t *me,
                                                           uint16_t diagnostic_id);
    const module_diagnostic_state_t *module_diagnostic_get_state(const module_diagnostic_t *me,
                                                                 uint16_t diagnostic_id);
    bool module_diagnostic_has_severity(const module_diagnostic_t *me,
                                        module_diagnostic_severity_t minimum_severity);

#ifdef __cplusplus
}
#endif

#endif
