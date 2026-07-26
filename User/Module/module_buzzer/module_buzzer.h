#ifndef MODULE_BUZZER_H
#define MODULE_BUZZER_H

#include "bsp_pwm.h"
#include "module_device.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef enum
    {
        MODULE_BUZZER_STATUS_OK = 0,
        MODULE_BUZZER_STATUS_FINISHED,
        MODULE_BUZZER_STATUS_INVALID_ARGUMENT,
        MODULE_BUZZER_STATUS_NOT_INITIALIZED,
        MODULE_BUZZER_STATUS_NOT_STARTED,
        MODULE_BUZZER_STATUS_TRANSPORT_ERROR
    } module_buzzer_status_t;

    typedef struct
    {
        uint32_t frequency_hz;
        uint32_t sound_time_ms;
        uint32_t silence_time_ms;
    } module_buzzer_note_t;

    typedef struct
    {
        bsp_pwm_t *pwm;
        const char *logical_name;
        uint32_t registration_key;
        float duty_cycle;
    } module_buzzer_config_t;

    typedef struct
    {
        module_device_t super;
        bsp_pwm_t *pwm;
        const module_buzzer_note_t *notes;
        size_t note_count;
        size_t note_index;
        uint32_t phase_elapsed_time_ms;
        float duty_cycle;
        bool is_sound_phase;
        bool is_looping;
        bool is_playing;
        bool is_started;
    } module_buzzer_t;

    module_buzzer_status_t module_buzzer_init(module_buzzer_t *me,
                                              const module_buzzer_config_t *config);
    module_buzzer_status_t module_buzzer_start(module_buzzer_t *me);
    module_buzzer_status_t module_buzzer_stop(module_buzzer_t *me);
    module_buzzer_status_t module_buzzer_play_tone(module_buzzer_t *me, uint32_t frequency_hz);
    module_buzzer_status_t module_buzzer_play_sequence(module_buzzer_t *me,
                                                       const module_buzzer_note_t *notes,
                                                       size_t note_count, bool is_looping);
    module_buzzer_status_t module_buzzer_silence(module_buzzer_t *me);
    module_buzzer_status_t module_buzzer_update(module_buzzer_t *me, uint32_t elapsed_time_ms);
    bool module_buzzer_is_playing(const module_buzzer_t *me);

#ifdef __cplusplus
}
#endif

#endif /* MODULE_BUZZER_H */
