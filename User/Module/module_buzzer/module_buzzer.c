#include "module_buzzer.h"

#include <math.h>
#include <stddef.h>

static module_buzzer_status_t module_buzzer_apply_silence(module_buzzer_t *me)
{
    return (bsp_pwm_set_duty_cycle(me->pwm, 0.0F) == BSP_STATUS_OK)
               ? MODULE_BUZZER_STATUS_OK
               : MODULE_BUZZER_STATUS_TRANSPORT_ERROR;
}

static module_buzzer_status_t module_buzzer_apply_note(module_buzzer_t *me,
                                                       const module_buzzer_note_t *note)
{
    if (note->frequency_hz == 0U)
    {
        return module_buzzer_apply_silence(me);
    }
    if ((bsp_pwm_set_frequency(me->pwm, note->frequency_hz) != BSP_STATUS_OK) ||
        (bsp_pwm_set_duty_cycle(me->pwm, me->duty_cycle) != BSP_STATUS_OK))
    {
        return MODULE_BUZZER_STATUS_TRANSPORT_ERROR;
    }
    return MODULE_BUZZER_STATUS_OK;
}

static module_device_status_t module_buzzer_device_start(module_device_t *const device_base)
{
    module_buzzer_t *const me = MODULE_CONTAINER_OF(device_base, module_buzzer_t, super);
    return (module_buzzer_start(me) == MODULE_BUZZER_STATUS_OK)
               ? MODULE_DEVICE_STATUS_OK
               : MODULE_DEVICE_STATUS_OPERATION_FAILED;
}

static module_device_status_t module_buzzer_device_stop(module_device_t *const device_base)
{
    module_buzzer_t *const me = MODULE_CONTAINER_OF(device_base, module_buzzer_t, super);
    return (module_buzzer_stop(me) == MODULE_BUZZER_STATUS_OK)
               ? MODULE_DEVICE_STATUS_OK
               : MODULE_DEVICE_STATUS_OPERATION_FAILED;
}

static module_device_status_t module_buzzer_device_update(module_device_t *const device_base,
                                                          uint32_t elapsed_time_ms)
{
    module_buzzer_t *const me = MODULE_CONTAINER_OF(device_base, module_buzzer_t, super);
    const module_buzzer_status_t status = module_buzzer_update(me, elapsed_time_ms);
    return ((status == MODULE_BUZZER_STATUS_OK) || (status == MODULE_BUZZER_STATUS_FINISHED))
               ? MODULE_DEVICE_STATUS_OK
               : MODULE_DEVICE_STATUS_OPERATION_FAILED;
}

static const module_device_ops_t s_module_buzzer_ops = {
    .start = module_buzzer_device_start,
    .stop = module_buzzer_device_stop,
    .update = module_buzzer_device_update,
};

module_buzzer_status_t module_buzzer_init(module_buzzer_t *me, const module_buzzer_config_t *config)
{
    if ((me == NULL) || (config == NULL) || (config->pwm == NULL) ||
        !bsp_device_is_initialized(&config->pwm->super) || !isfinite(config->duty_cycle) ||
        (config->duty_cycle <= 0.0F) || (config->duty_cycle > 1.0F))
    {
        return MODULE_BUZZER_STATUS_INVALID_ARGUMENT;
    }
    *me = (module_buzzer_t){0};
    me->pwm = config->pwm;
    me->duty_cycle = config->duty_cycle;
    if (module_device_init_base(&me->super, &s_module_buzzer_ops, config->logical_name,
                                config->registration_key) != MODULE_DEVICE_STATUS_OK)
    {
        return MODULE_BUZZER_STATUS_INVALID_ARGUMENT;
    }
    if (module_device_complete_init(&me->super) != MODULE_DEVICE_STATUS_OK)
    {
        module_device_abort_init(&me->super);
        return MODULE_BUZZER_STATUS_INVALID_ARGUMENT;
    }
    return MODULE_BUZZER_STATUS_OK;
}

module_buzzer_status_t module_buzzer_start(module_buzzer_t *me)
{
    if ((me == NULL) || !module_device_is_initialized(&me->super))
    {
        return MODULE_BUZZER_STATUS_NOT_INITIALIZED;
    }
    if (bsp_pwm_start(me->pwm) != BSP_STATUS_OK)
    {
        return MODULE_BUZZER_STATUS_TRANSPORT_ERROR;
    }
    me->is_started = true;
    return module_buzzer_apply_silence(me);
}

module_buzzer_status_t module_buzzer_stop(module_buzzer_t *me)
{
    if ((me == NULL) || !module_device_is_initialized(&me->super))
    {
        return MODULE_BUZZER_STATUS_NOT_INITIALIZED;
    }
    me->is_playing = false;
    me->is_started = false;
    return (bsp_pwm_stop(me->pwm) == BSP_STATUS_OK) ? MODULE_BUZZER_STATUS_OK
                                                    : MODULE_BUZZER_STATUS_TRANSPORT_ERROR;
}

module_buzzer_status_t module_buzzer_play_tone(module_buzzer_t *me, uint32_t frequency_hz)
{
    module_buzzer_note_t note;

    if ((me == NULL) || (frequency_hz == 0U))
    {
        return MODULE_BUZZER_STATUS_INVALID_ARGUMENT;
    }
    if (!module_device_is_initialized(&me->super))
    {
        return MODULE_BUZZER_STATUS_NOT_INITIALIZED;
    }
    if (!me->is_started)
    {
        return MODULE_BUZZER_STATUS_NOT_STARTED;
    }
    note = (module_buzzer_note_t){
        .frequency_hz = frequency_hz,
        .sound_time_ms = 0U,
        .silence_time_ms = 0U,
    };
    me->is_playing = true;
    me->notes = NULL;
    return module_buzzer_apply_note(me, &note);
}

module_buzzer_status_t module_buzzer_play_sequence(module_buzzer_t *me,
                                                   const module_buzzer_note_t *notes,
                                                   size_t note_count, bool is_looping)
{
    size_t note_index;

    if ((me == NULL) || (notes == NULL) || (note_count == 0U))
    {
        return MODULE_BUZZER_STATUS_INVALID_ARGUMENT;
    }
    if (!module_device_is_initialized(&me->super))
    {
        return MODULE_BUZZER_STATUS_NOT_INITIALIZED;
    }
    if (!me->is_started)
    {
        return MODULE_BUZZER_STATUS_NOT_STARTED;
    }
    for (note_index = 0U; note_index < note_count; ++note_index)
    {
        if ((notes[note_index].frequency_hz == 0U) || (notes[note_index].sound_time_ms == 0U))
        {
            return MODULE_BUZZER_STATUS_INVALID_ARGUMENT;
        }
    }
    me->notes = notes;
    me->note_count = note_count;
    me->note_index = 0U;
    me->phase_elapsed_time_ms = 0U;
    me->is_sound_phase = true;
    me->is_looping = is_looping;
    me->is_playing = true;
    return module_buzzer_apply_note(me, &notes[0]);
}

module_buzzer_status_t module_buzzer_silence(module_buzzer_t *me)
{
    if ((me == NULL) || !module_device_is_initialized(&me->super))
    {
        return MODULE_BUZZER_STATUS_NOT_INITIALIZED;
    }
    if (!me->is_started)
    {
        return MODULE_BUZZER_STATUS_NOT_STARTED;
    }
    me->is_playing = false;
    me->notes = NULL;
    return module_buzzer_apply_silence(me);
}

module_buzzer_status_t module_buzzer_update(module_buzzer_t *me, uint32_t elapsed_time_ms)
{
    const module_buzzer_note_t *note;
    uint32_t phase_duration_ms;

    if ((me == NULL) || !module_device_is_initialized(&me->super))
    {
        return MODULE_BUZZER_STATUS_NOT_INITIALIZED;
    }
    if (!me->is_started)
    {
        return MODULE_BUZZER_STATUS_NOT_STARTED;
    }
    if (!me->is_playing || (me->notes == NULL))
    {
        return MODULE_BUZZER_STATUS_FINISHED;
    }
    if (UINT32_MAX - me->phase_elapsed_time_ms < elapsed_time_ms)
    {
        me->phase_elapsed_time_ms = UINT32_MAX;
    }
    else
    {
        me->phase_elapsed_time_ms += elapsed_time_ms;
    }
    note = &me->notes[me->note_index];
    phase_duration_ms = me->is_sound_phase ? note->sound_time_ms : note->silence_time_ms;
    if (me->phase_elapsed_time_ms < phase_duration_ms)
    {
        return MODULE_BUZZER_STATUS_OK;
    }
    me->phase_elapsed_time_ms = 0U;
    if (me->is_sound_phase && (note->silence_time_ms > 0U))
    {
        me->is_sound_phase = false;
        return module_buzzer_apply_silence(me);
    }
    me->is_sound_phase = true;
    ++me->note_index;
    if (me->note_index >= me->note_count)
    {
        if (!me->is_looping)
        {
            me->is_playing = false;
            me->notes = NULL;
            (void)module_buzzer_apply_silence(me);
            return MODULE_BUZZER_STATUS_FINISHED;
        }
        me->note_index = 0U;
    }
    return module_buzzer_apply_note(me, &me->notes[me->note_index]);
}

bool module_buzzer_is_playing(const module_buzzer_t *me)
{
    return (me != NULL) && module_device_is_initialized(&me->super) && me->is_playing;
}
