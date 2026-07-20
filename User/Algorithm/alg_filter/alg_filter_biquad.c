#include "alg_filter.h"

#include <math.h>
#include <stddef.h>

#define ALG_FILTER_PI_F (3.14159265358979323846F)

static bool AlgFilterBiquad_AreFinite(float value_0,
                                      float value_1,
                                      float value_2,
                                      float value_3,
                                      float value_4)
{
    return isfinite(value_0) && isfinite(value_1) && isfinite(value_2) &&
           isfinite(value_3) && isfinite(value_4);
}

AlgFilterStatus_t AlgFilterBiquad_Init(AlgFilterBiquad_t *self,
                                       AlgFilterBiquadType_t type,
                                       float sample_frequency_hz,
                                       float center_frequency_hz,
                                       float quality_factor)
{
    float angular_frequency;
    float cosine;
    float alpha;
    float a0;
    float a1;
    float a2;
    float b0;
    float b1;
    float b2;

    if (self == NULL)
    {
        return ALG_FILTER_STATUS_INVALID_ARGUMENT;
    }

    self->is_initialized = false;
    if (!isfinite(sample_frequency_hz) || !isfinite(center_frequency_hz) ||
        !isfinite(quality_factor) || (sample_frequency_hz <= 0.0F) ||
        (center_frequency_hz <= 0.0F) ||
        (center_frequency_hz >= (0.5F * sample_frequency_hz)) ||
        (quality_factor <= 0.0F) || (type > ALG_FILTER_BIQUAD_NOTCH))
    {
        return ALG_FILTER_STATUS_OUT_OF_RANGE;
    }

    angular_frequency = 2.0F * ALG_FILTER_PI_F * center_frequency_hz /
                        sample_frequency_hz;
    cosine = cosf(angular_frequency);
    alpha = sinf(angular_frequency) / (2.0F * quality_factor);
    a0 = 1.0F + alpha;
    a1 = -2.0F * cosine;
    a2 = 1.0F - alpha;

    switch (type)
    {
        case ALG_FILTER_BIQUAD_LOW_PASS:
            b0 = 0.5F * (1.0F - cosine);
            b1 = 1.0F - cosine;
            b2 = b0;
            break;

        case ALG_FILTER_BIQUAD_HIGH_PASS:
            b0 = 0.5F * (1.0F + cosine);
            b1 = -(1.0F + cosine);
            b2 = b0;
            break;

        case ALG_FILTER_BIQUAD_BAND_PASS:
            b0 = alpha;
            b1 = 0.0F;
            b2 = -alpha;
            break;

        case ALG_FILTER_BIQUAD_NOTCH:
            b0 = 1.0F;
            b1 = -2.0F * cosine;
            b2 = 1.0F;
            break;

        default:
            return ALG_FILTER_STATUS_OUT_OF_RANGE;
    }

    return AlgFilterBiquad_SetCoefficients(self,
                                           b0 / a0,
                                           b1 / a0,
                                           b2 / a0,
                                           a1 / a0,
                                           a2 / a0);
}

AlgFilterStatus_t AlgFilterBiquad_SetCoefficients(AlgFilterBiquad_t *self,
                                                  float b0,
                                                  float b1,
                                                  float b2,
                                                  float a1,
                                                  float a2)
{
    if (self == NULL)
    {
        return ALG_FILTER_STATUS_INVALID_ARGUMENT;
    }
    if (!AlgFilterBiquad_AreFinite(b0, b1, b2, a1, a2))
    {
        return ALG_FILTER_STATUS_OUT_OF_RANGE;
    }

    self->b0 = b0;
    self->b1 = b1;
    self->b2 = b2;
    self->a1 = a1;
    self->a2 = a2;
    self->state_1 = 0.0F;
    self->state_2 = 0.0F;
    self->is_initialized = true;
    return ALG_FILTER_STATUS_OK;
}

AlgFilterStatus_t AlgFilterBiquad_Reset(AlgFilterBiquad_t *self)
{
    if (self == NULL)
    {
        return ALG_FILTER_STATUS_INVALID_ARGUMENT;
    }
    if (!self->is_initialized)
    {
        return ALG_FILTER_STATUS_NOT_INITIALIZED;
    }

    self->state_1 = 0.0F;
    self->state_2 = 0.0F;
    return ALG_FILTER_STATUS_OK;
}

AlgFilterStatus_t AlgFilterBiquad_Update(AlgFilterBiquad_t *self,
                                         float input,
                                         float *output)
{
    float current_output;
    float next_state_1;
    float next_state_2;

    if ((self == NULL) || (output == NULL))
    {
        return ALG_FILTER_STATUS_INVALID_ARGUMENT;
    }
    if (!self->is_initialized)
    {
        return ALG_FILTER_STATUS_NOT_INITIALIZED;
    }
    if (!isfinite(input))
    {
        return ALG_FILTER_STATUS_OUT_OF_RANGE;
    }

    current_output = (self->b0 * input) + self->state_1;
    next_state_1 = (self->b1 * input) - (self->a1 * current_output) + self->state_2;
    next_state_2 = (self->b2 * input) - (self->a2 * current_output);
    if (!AlgFilterBiquad_AreFinite(current_output,
                                   next_state_1,
                                   next_state_2,
                                   self->a1,
                                   self->a2))
    {
        return ALG_FILTER_STATUS_NUMERICAL_ERROR;
    }

    self->state_1 = next_state_1;
    self->state_2 = next_state_2;
    *output = current_output;
    return ALG_FILTER_STATUS_OK;
}
