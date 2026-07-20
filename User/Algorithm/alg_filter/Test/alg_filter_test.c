/* Host-side unit tests for the portable Algorithm-layer filter library. */
#include "alg_filter.h"

#include <math.h>
#include <stdio.h>

#define TEST_TOLERANCE (1.0e-5F)

static int s_failure_count = 0;

#define TEST_EXPECT_TRUE(condition)                                               \
    do                                                                            \
    {                                                                             \
        if (!(condition))                                                         \
        {                                                                         \
            (void)printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #condition);    \
            ++s_failure_count;                                                    \
        }                                                                         \
    } while (0)

#define TEST_EXPECT_NEAR(actual, expected, tolerance)                            \
    TEST_EXPECT_TRUE(fabsf((actual) - (expected)) <= (tolerance))

static void TestBasicFilters(void)
{
    AlgFilterLowPass_t low_pass;
    AlgFilterHighPass_t high_pass;
    AlgFilterExponential_t exponential;
    float output = 0.0F;

    TEST_EXPECT_TRUE(AlgFilterLowPass_Init(&low_pass, 10.0F) == ALG_FILTER_STATUS_OK);
    TEST_EXPECT_TRUE(AlgFilterLowPass_Update(&low_pass, 3.0F, 0.001F, &output) ==
                     ALG_FILTER_STATUS_OK);
    TEST_EXPECT_NEAR(output, 3.0F, TEST_TOLERANCE);
    (void)AlgFilterLowPass_Update(&low_pass, 10.0F, 0.001F, &output);
    TEST_EXPECT_TRUE((output > 3.0F) && (output < 10.0F));
    TEST_EXPECT_TRUE(AlgFilterLowPass_Reset(&low_pass, 5.0F) == ALG_FILTER_STATUS_OK);
    TEST_EXPECT_NEAR(low_pass.output, 5.0F, TEST_TOLERANCE);

    TEST_EXPECT_TRUE(AlgFilterHighPass_Init(&high_pass, 5.0F) == ALG_FILTER_STATUS_OK);
    (void)AlgFilterHighPass_Update(&high_pass, 2.0F, 0.01F, &output);
    TEST_EXPECT_NEAR(output, 0.0F, TEST_TOLERANCE);
    (void)AlgFilterHighPass_Update(&high_pass, 4.0F, 0.01F, &output);
    TEST_EXPECT_TRUE(output > 0.0F);

    TEST_EXPECT_TRUE(AlgFilterExponential_Init(&exponential, 0.25F) ==
                     ALG_FILTER_STATUS_OK);
    (void)AlgFilterExponential_Reset(&exponential, 0.0F);
    (void)AlgFilterExponential_Update(&exponential, 8.0F, &output);
    TEST_EXPECT_NEAR(output, 2.0F, TEST_TOLERANCE);
}

static void TestWindowFilters(void)
{
    AlgFilterMovingAverage_t moving_average;
    AlgFilterMedian_t median;
    float average_buffer[3];
    float median_buffer[5];
    float median_workspace[5];
    float output = 0.0F;

    TEST_EXPECT_TRUE(AlgFilterMovingAverage_Init(&moving_average,
                                                average_buffer,
                                                3U) == ALG_FILTER_STATUS_OK);
    (void)AlgFilterMovingAverage_Update(&moving_average, 1.0F, &output);
    TEST_EXPECT_NEAR(output, 1.0F, TEST_TOLERANCE);
    (void)AlgFilterMovingAverage_Update(&moving_average, 2.0F, &output);
    TEST_EXPECT_NEAR(output, 1.5F, TEST_TOLERANCE);
    (void)AlgFilterMovingAverage_Update(&moving_average, 3.0F, &output);
    (void)AlgFilterMovingAverage_Update(&moving_average, 7.0F, &output);
    TEST_EXPECT_NEAR(output, 4.0F, TEST_TOLERANCE);

    TEST_EXPECT_TRUE(AlgFilterMedian_Init(&median,
                                         median_buffer,
                                         median_workspace,
                                         5U) == ALG_FILTER_STATUS_OK);
    (void)AlgFilterMedian_Update(&median, 1.0F, &output);
    (void)AlgFilterMedian_Update(&median, 100.0F, &output);
    TEST_EXPECT_NEAR(output, 50.5F, TEST_TOLERANCE);
    (void)AlgFilterMedian_Update(&median, 2.0F, &output);
    (void)AlgFilterMedian_Update(&median, 3.0F, &output);
    (void)AlgFilterMedian_Update(&median, 4.0F, &output);
    TEST_EXPECT_NEAR(output, 3.0F, TEST_TOLERANCE);
}

static void TestFir(void)
{
    static const float coefficients[3] = {0.5F, 0.3F, 0.2F};
    AlgFilterFir_t filter;
    float state_buffer[3];
    float output = 0.0F;

    TEST_EXPECT_TRUE(AlgFilterFir_Init(&filter,
                                      coefficients,
                                      state_buffer,
                                      3U) == ALG_FILTER_STATUS_OK);
    (void)AlgFilterFir_Update(&filter, 2.0F, &output);
    TEST_EXPECT_NEAR(output, 1.0F, TEST_TOLERANCE);
    (void)AlgFilterFir_Update(&filter, 4.0F, &output);
    TEST_EXPECT_NEAR(output, 2.6F, TEST_TOLERANCE);
    (void)AlgFilterFir_Update(&filter, 8.0F, &output);
    TEST_EXPECT_NEAR(output, 5.6F, TEST_TOLERANCE);
}

static void TestBiquadType(AlgFilterBiquadType_t type)
{
    AlgFilterBiquad_t filter;
    float output = 0.0F;

    TEST_EXPECT_TRUE(AlgFilterBiquad_Init(&filter,
                                         type,
                                         1000.0F,
                                         20.0F,
                                         0.70710678F) == ALG_FILTER_STATUS_OK);
    TEST_EXPECT_TRUE(AlgFilterBiquad_Update(&filter, 1.0F, &output) ==
                     ALG_FILTER_STATUS_OK);
    TEST_EXPECT_TRUE(isfinite(output));
    TEST_EXPECT_TRUE(AlgFilterBiquad_Reset(&filter) == ALG_FILTER_STATUS_OK);
}

static void TestBiquad(void)
{
    AlgFilterBiquad_t filter;
    float output = 0.0F;
    int iteration;

    TestBiquadType(ALG_FILTER_BIQUAD_LOW_PASS);
    TestBiquadType(ALG_FILTER_BIQUAD_HIGH_PASS);
    TestBiquadType(ALG_FILTER_BIQUAD_BAND_PASS);
    TestBiquadType(ALG_FILTER_BIQUAD_NOTCH);

    (void)AlgFilterBiquad_Init(&filter,
                               ALG_FILTER_BIQUAD_LOW_PASS,
                               1000.0F,
                               20.0F,
                               0.70710678F);
    for (iteration = 0; iteration < 500; ++iteration)
    {
        (void)AlgFilterBiquad_Update(&filter, 1.0F, &output);
    }
    TEST_EXPECT_NEAR(output, 1.0F, 1.0e-3F);

    (void)AlgFilterBiquad_Init(&filter,
                               ALG_FILTER_BIQUAD_HIGH_PASS,
                               1000.0F,
                               20.0F,
                               0.70710678F);
    for (iteration = 0; iteration < 500; ++iteration)
    {
        (void)AlgFilterBiquad_Update(&filter, 1.0F, &output);
    }
    TEST_EXPECT_NEAR(output, 0.0F, 1.0e-3F);
}

static void TestComplementary(void)
{
    AlgFilterComplementary_t filter;
    float output = 0.0F;

    TEST_EXPECT_TRUE(AlgFilterComplementary_Init(&filter, 0.9F, 0.0F) ==
                     ALG_FILTER_STATUS_OK);
    TEST_EXPECT_TRUE(AlgFilterComplementary_Update(&filter,
                                                   0.0F,
                                                   10.0F,
                                                   0.1F,
                                                   &output) == ALG_FILTER_STATUS_OK);
    TEST_EXPECT_NEAR(output, 0.9F, TEST_TOLERANCE);
}

static void TestInvalidArguments(void)
{
    AlgFilterLowPass_t low_pass = {0};
    AlgFilterMovingAverage_t moving_average = {0};
    AlgFilterBiquad_t biquad;
    float output = 0.0F;

    TEST_EXPECT_TRUE(AlgFilterLowPass_Init(NULL, 1.0F) ==
                     ALG_FILTER_STATUS_INVALID_ARGUMENT);
    TEST_EXPECT_TRUE(AlgFilterLowPass_Init(&low_pass, -1.0F) ==
                     ALG_FILTER_STATUS_OUT_OF_RANGE);
    TEST_EXPECT_TRUE(AlgFilterLowPass_Update(&low_pass, 1.0F, 0.1F, &output) ==
                     ALG_FILTER_STATUS_NOT_INITIALIZED);
    TEST_EXPECT_TRUE(AlgFilterMovingAverage_Update(&moving_average, 1.0F, &output) ==
                     ALG_FILTER_STATUS_NOT_INITIALIZED);
    TEST_EXPECT_TRUE(AlgFilterBiquad_Init(&biquad,
                                         ALG_FILTER_BIQUAD_NOTCH,
                                         1000.0F,
                                         500.0F,
                                         1.0F) == ALG_FILTER_STATUS_OUT_OF_RANGE);
}

int main(void)
{
    TestBasicFilters();
    TestWindowFilters();
    TestFir();
    TestBiquad();
    TestComplementary();
    TestInvalidArguments();

    if (s_failure_count == 0)
    {
        (void)printf("All alg_filter tests passed.\n");
        return 0;
    }

    (void)printf("%d alg_filter test(s) failed.\n", s_failure_count);
    return 1;
}
