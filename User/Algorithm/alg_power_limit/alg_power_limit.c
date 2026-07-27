#include "alg_power_limit.h"

#include <math.h>

static float alg_power_limit_clamp(float value, float minimum, float maximum)
{
    return (value < minimum) ? minimum : ((value > maximum) ? maximum : value);
}

alg_power_limit_status_t alg_power_limit_init(alg_power_limit_t *me,
                                              const alg_power_limit_config_t *config)
{
    if ((me == NULL) || (config == NULL) || !isfinite(config->continuous_power_w) ||
        !isfinite(config->peak_power_w) || !isfinite(config->buffer_energy_low_j) ||
        !isfinite(config->buffer_energy_high_j) || (config->continuous_power_w <= 0.0F) ||
        (config->peak_power_w < config->continuous_power_w) ||
        (config->buffer_energy_high_j <= config->buffer_energy_low_j) ||
        (config->minimum_output_scale < 0.0F) || (config->minimum_output_scale > 1.0F) ||
        (config->smoothing_time_constant_s < 0.0F))
    {
        return ALG_POWER_LIMIT_STATUS_INVALID_ARGUMENT;
    }
    me->config = *config;
    me->available_power_w = config->continuous_power_w;
    me->filtered_scale = 1.0F;
    me->is_initialized = true;
    return ALG_POWER_LIMIT_STATUS_OK;
}

alg_power_limit_status_t alg_power_limit_reset(alg_power_limit_t *me)
{
    if (me == NULL)
    {
        return ALG_POWER_LIMIT_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return ALG_POWER_LIMIT_STATUS_NOT_INITIALIZED;
    }
    me->available_power_w = me->config.continuous_power_w;
    me->filtered_scale = 1.0F;
    return ALG_POWER_LIMIT_STATUS_OK;
}

alg_power_limit_status_t alg_power_limit_update(alg_power_limit_t *me, float referee_power_limit_w,
                                                float buffer_energy_j,
                                                const alg_power_limit_channel_input_t *inputs,
                                                alg_power_limit_channel_output_t *outputs,
                                                size_t channel_count, float delta_time_s)
{
    float base_power_w;
    float energy_ratio;
    float requested_weighted_power_w = 0.0F;
    float allocated_power_w = 0.0F;
    float common_scale;
    float smoothing_alpha;
    size_t channel_index;

    if ((me == NULL) || (inputs == NULL) || (outputs == NULL) || (channel_count == 0U) ||
        !isfinite(referee_power_limit_w) || !isfinite(buffer_energy_j) || !isfinite(delta_time_s) ||
        (delta_time_s <= 0.0F))
    {
        return ALG_POWER_LIMIT_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return ALG_POWER_LIMIT_STATUS_NOT_INITIALIZED;
    }

    base_power_w = (referee_power_limit_w > 0.0F)
                       ? fminf(referee_power_limit_w, me->config.continuous_power_w)
                       : me->config.continuous_power_w;
    energy_ratio = alg_power_limit_clamp(
        (buffer_energy_j - me->config.buffer_energy_low_j) /
            (me->config.buffer_energy_high_j - me->config.buffer_energy_low_j),
        0.0F, 1.0F);
    me->available_power_w = base_power_w + energy_ratio * (me->config.peak_power_w - base_power_w);

    for (channel_index = 0U; channel_index < channel_count; ++channel_index)
    {
        if (!isfinite(inputs[channel_index].requested_output) ||
            !isfinite(inputs[channel_index].estimated_power_w) ||
            !isfinite(inputs[channel_index].priority) || (inputs[channel_index].priority < 0.0F))
        {
            return ALG_POWER_LIMIT_STATUS_INVALID_ARGUMENT;
        }
        if (inputs[channel_index].is_enabled)
        {
            requested_weighted_power_w += fmaxf(inputs[channel_index].estimated_power_w, 0.0F) *
                                          fmaxf(inputs[channel_index].priority, 0.001F);
        }
    }
    common_scale = (requested_weighted_power_w > me->available_power_w)
                       ? me->available_power_w / requested_weighted_power_w
                       : 1.0F;
    smoothing_alpha = (me->config.smoothing_time_constant_s <= 0.0F)
                          ? 1.0F
                          : delta_time_s / (me->config.smoothing_time_constant_s + delta_time_s);
    me->filtered_scale += smoothing_alpha * (common_scale - me->filtered_scale);

    for (channel_index = 0U; channel_index < channel_count; ++channel_index)
    {
        const float priority_scale = alg_power_limit_clamp(
            me->filtered_scale * fmaxf(inputs[channel_index].priority, 0.001F),
            me->config.minimum_output_scale, 1.0F);
        outputs[channel_index].scale = inputs[channel_index].is_enabled ? priority_scale : 0.0F;
        outputs[channel_index].limited_output =
            inputs[channel_index].requested_output * outputs[channel_index].scale;
        outputs[channel_index].allocated_power_w =
            fmaxf(inputs[channel_index].estimated_power_w, 0.0F) * outputs[channel_index].scale;
        allocated_power_w += outputs[channel_index].allocated_power_w;
    }
    if (allocated_power_w > me->available_power_w)
    {
        const float safety_scale = me->available_power_w / allocated_power_w;
        for (channel_index = 0U; channel_index < channel_count; ++channel_index)
        {
            outputs[channel_index].scale *= safety_scale;
            outputs[channel_index].limited_output =
                inputs[channel_index].requested_output * outputs[channel_index].scale;
            outputs[channel_index].allocated_power_w *= safety_scale;
        }
    }
    return (common_scale < 0.9999F) ? ALG_POWER_LIMIT_STATUS_LIMITED : ALG_POWER_LIMIT_STATUS_OK;
}

float alg_power_limit_estimate_motor_power(float torque_nm, float velocity_rad_per_s,
                                           float current_a, float winding_resistance_ohm,
                                           float idle_power_w)
{
    if (!isfinite(torque_nm) || !isfinite(velocity_rad_per_s) || !isfinite(current_a) ||
        !isfinite(winding_resistance_ohm) || !isfinite(idle_power_w))
    {
        return 0.0F;
    }
    return fabsf(torque_nm * velocity_rad_per_s) +
           current_a * current_a * fmaxf(winding_resistance_ohm, 0.0F) + fmaxf(idle_power_w, 0.0F);
}
