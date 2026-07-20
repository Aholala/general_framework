#include "alg_pid.h"

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

static AlgPidConfig_t Test_DefaultConfig(void)
{
    const AlgPidConfig_t config = {
        .proportional_gain = 1.0F,
        .integral_gain = 0.0F,
        .derivative_gain = 0.0F,
        .setpoint_weight = 1.0F,
        .derivative_setpoint_weight = 1.0F,
        .velocity_feedforward_gain = 0.0F,
        .acceleration_feedforward_gain = 0.0F,
        .derivative_filter_cutoff_hz = 0.0F,
        .error_deadband = 0.0F,
        .integral_separation_threshold = 0.0F,
        .integral_min = -1000.0F,
        .integral_max = 1000.0F,
        .output_min = -1000.0F,
        .output_max = 1000.0F,
        .back_calculation_gain = 0.0F,
        .anti_windup_mode = ALG_PID_ANTI_WINDUP_NONE,
        .derivative_mode = ALG_PID_DERIVATIVE_ON_ERROR};
    return config;
}

static void TestPositionAndVelocity(void)
{
    AlgPidPosition_t position;
    AlgPidVelocity_t velocity;
    AlgPidConfig_t config = Test_DefaultConfig();
    float output = 0.0F;

    config.proportional_gain = 2.0F;
    TEST_EXPECT_TRUE(AlgPidPosition_Init(&position, &config) == ALG_PID_STATUS_OK);
    TEST_EXPECT_TRUE(AlgPidVelocity_Init(&velocity, &config) == ALG_PID_STATUS_OK);
    TEST_EXPECT_TRUE(AlgPid_Update(&position, 10.0F, 4.0F, 0.01F, &output) ==
                     ALG_PID_STATUS_OK);
    TEST_EXPECT_NEAR(output, 12.0F, TEST_TOLERANCE);
    TEST_EXPECT_TRUE(AlgPid_Update(&velocity, 5.0F, 2.0F, 0.01F, &output) ==
                     ALG_PID_STATUS_OK);
    TEST_EXPECT_NEAR(output, 6.0F, TEST_TOLERANCE);
}

static void TestIntegralAndAntiWindup(void)
{
    AlgPid_t controller;
    AlgPidConfig_t config = Test_DefaultConfig();
    float output = 0.0F;
    int iteration;

    config.proportional_gain = 0.0F;
    config.integral_gain = 2.0F;
    config.integral_min = -0.5F;
    config.integral_max = 0.5F;
    TEST_EXPECT_TRUE(AlgPid_Init(&controller, &config) == ALG_PID_STATUS_OK);
    for (iteration = 0; iteration < 20; ++iteration)
    {
        (void)AlgPid_Update(&controller, 1.0F, 0.0F, 0.1F, &output);
    }
    TEST_EXPECT_NEAR(output, 0.5F, TEST_TOLERANCE);

    config.proportional_gain = 2.0F;
    config.integral_gain = 1.0F;
    config.integral_min = -10.0F;
    config.integral_max = 10.0F;
    config.output_min = -1.0F;
    config.output_max = 1.0F;
    config.anti_windup_mode = ALG_PID_ANTI_WINDUP_CLAMPING;
    TEST_EXPECT_TRUE(AlgPid_Init(&controller, &config) == ALG_PID_STATUS_OK);
    (void)AlgPid_Update(&controller, 10.0F, 0.0F, 0.1F, &output);
    TEST_EXPECT_NEAR(output, 1.0F, TEST_TOLERANCE);
    TEST_EXPECT_NEAR(controller.terms.integral, 0.0F, TEST_TOLERANCE);

    config.anti_windup_mode = ALG_PID_ANTI_WINDUP_BACK_CALCULATION;
    config.back_calculation_gain = 5.0F;
    TEST_EXPECT_TRUE(AlgPid_Init(&controller, &config) == ALG_PID_STATUS_OK);
    (void)AlgPid_Update(&controller, 10.0F, 0.0F, 0.1F, &output);
    TEST_EXPECT_NEAR(output, 1.0F, TEST_TOLERANCE);
    TEST_EXPECT_TRUE(controller.terms.integral < 1.0F);
}

static void TestAdvancedFeatures(void)
{
    AlgPid_t controller;
    AlgPidConfig_t config = Test_DefaultConfig();
    AlgPidInput_t input = {
        .setpoint = 10.0F,
        .measurement = 0.0F,
        .setpoint_rate_per_s = 2.0F,
        .setpoint_acceleration_per_s2 = 3.0F,
        .additional_feedforward = 4.0F,
        .delta_time_s = 0.01F};
    float output = 0.0F;

    config.proportional_gain = 2.0F;
    config.setpoint_weight = 0.0F;
    config.velocity_feedforward_gain = 5.0F;
    config.acceleration_feedforward_gain = 2.0F;
    TEST_EXPECT_TRUE(AlgPid_Init(&controller, &config) == ALG_PID_STATUS_OK);
    TEST_EXPECT_TRUE(AlgPid_UpdateAdvanced(&controller, &input, &output) ==
                     ALG_PID_STATUS_OK);
    TEST_EXPECT_NEAR(controller.terms.proportional, 0.0F, TEST_TOLERANCE);
    TEST_EXPECT_NEAR(controller.terms.feedforward, 20.0F, TEST_TOLERANCE);
    TEST_EXPECT_NEAR(output, 20.0F, TEST_TOLERANCE);

    config = Test_DefaultConfig();
    config.derivative_gain = 1.0F;
    config.derivative_mode = ALG_PID_DERIVATIVE_ON_MEASUREMENT;
    TEST_EXPECT_TRUE(AlgPid_Init(&controller, &config) == ALG_PID_STATUS_OK);
    (void)AlgPid_Update(&controller, 0.0F, 0.0F, 0.01F, &output);
    (void)AlgPid_Update(&controller, 10.0F, 0.0F, 0.01F, &output);
    TEST_EXPECT_NEAR(controller.terms.derivative, 0.0F, TEST_TOLERANCE);

    config = Test_DefaultConfig();
    config.integral_gain = 1.0F;
    config.integral_separation_threshold = 1.0F;
    config.error_deadband = 0.1F;
    TEST_EXPECT_TRUE(AlgPid_Init(&controller, &config) == ALG_PID_STATUS_OK);
    (void)AlgPid_Update(&controller, 10.0F, 0.0F, 0.1F, &output);
    TEST_EXPECT_NEAR(controller.terms.integral, 0.0F, TEST_TOLERANCE);
    (void)AlgPid_Update(&controller, 0.05F, 0.0F, 0.1F, &output);
    TEST_EXPECT_NEAR(output, 0.0F, TEST_TOLERANCE);

    TEST_EXPECT_TRUE(AlgPid_TrackOutput(&controller, 1.0F, 0.0F, 2.0F, 7.0F) ==
                     ALG_PID_STATUS_OK);
    TEST_EXPECT_NEAR(controller.terms.output, 7.0F, TEST_TOLERANCE);
}

static void TestIncremental(void)
{
    const AlgPidIncrementalConfig_t config = {
        .proportional_gain = 1.0F,
        .integral_gain = 1.0F,
        .derivative_gain = 0.0F,
        .derivative_filter_cutoff_hz = 0.0F,
        .error_deadband = 0.0F,
        .delta_output_min = -10.0F,
        .delta_output_max = 10.0F,
        .output_min = -100.0F,
        .output_max = 100.0F};
    AlgPidIncremental_t controller;
    float output = 0.0F;

    TEST_EXPECT_TRUE(AlgPidIncremental_Init(&controller, &config) ==
                     ALG_PID_STATUS_OK);
    TEST_EXPECT_TRUE(AlgPidIncremental_Update(&controller,
                                             2.0F,
                                             0.0F,
                                             0.0F,
                                             0.5F,
                                             &output) == ALG_PID_STATUS_OK);
    TEST_EXPECT_NEAR(output, 3.0F, TEST_TOLERANCE);
    TEST_EXPECT_TRUE(AlgPidIncremental_Update(&controller,
                                             2.0F,
                                             0.0F,
                                             0.0F,
                                             0.5F,
                                             &output) == ALG_PID_STATUS_OK);
    TEST_EXPECT_NEAR(output, 4.0F, TEST_TOLERANCE);
}

static void TestGainSchedule(void)
{
    static const AlgPidGainPoint_t points[2] = {
        {.operating_point = 0.0F,
         .proportional_gain = 1.0F,
         .integral_gain = 0.0F,
         .derivative_gain = 0.0F},
        {.operating_point = 10.0F,
         .proportional_gain = 3.0F,
         .integral_gain = 0.0F,
         .derivative_gain = 0.0F}};
    AlgPidConfig_t config = Test_DefaultConfig();
    const AlgPidInput_t input = {
        .setpoint = 2.0F,
        .measurement = 0.0F,
        .setpoint_rate_per_s = 0.0F,
        .setpoint_acceleration_per_s2 = 0.0F,
        .additional_feedforward = 0.0F,
        .delta_time_s = 0.01F};
    AlgPidGainSchedule_t controller;
    float output = 0.0F;

    TEST_EXPECT_TRUE(AlgPidGainSchedule_Init(&controller, &config, points, 2U) ==
                     ALG_PID_STATUS_OK);
    TEST_EXPECT_TRUE(AlgPidGainSchedule_Update(&controller, 5.0F, &input, &output) ==
                     ALG_PID_STATUS_OK);
    TEST_EXPECT_NEAR(controller.controller.config.proportional_gain,
                     2.0F,
                     TEST_TOLERANCE);
    TEST_EXPECT_NEAR(output, 4.0F, TEST_TOLERANCE);
}

static void TestFuzzy(void)
{
    static const float proportional_adjustments[9] = {
        0.0F, 0.0F, 0.0F,
        1.0F, 1.0F, 1.0F,
        2.0F, 2.0F, 2.0F};
    static const float zero_adjustments[9] = {0.0F};
    AlgPidFuzzyConfig_t config;
    const AlgPidInput_t input = {
        .setpoint = 5.0F,
        .measurement = 0.0F,
        .setpoint_rate_per_s = 0.0F,
        .setpoint_acceleration_per_s2 = 0.0F,
        .additional_feedforward = 0.0F,
        .delta_time_s = 0.1F};
    AlgPidFuzzy_t controller;
    float output = 0.0F;

    config.base_config = Test_DefaultConfig();
    config.proportional_adjustment_table = proportional_adjustments;
    config.integral_adjustment_table = zero_adjustments;
    config.derivative_adjustment_table = zero_adjustments;
    config.axis_point_count = 3U;
    config.error_normalization = 10.0F;
    config.error_rate_normalization = 100.0F;

    TEST_EXPECT_TRUE(AlgPidFuzzy_Init(&controller, &config) == ALG_PID_STATUS_OK);
    TEST_EXPECT_TRUE(AlgPidFuzzy_Update(&controller, &input, &output) ==
                     ALG_PID_STATUS_OK);
    TEST_EXPECT_NEAR(controller.controller.config.proportional_gain,
                     2.5F,
                     TEST_TOLERANCE);
    TEST_EXPECT_NEAR(output, 12.5F, TEST_TOLERANCE);
}

static void TestCascade(void)
{
    AlgPidCascadeConfig_t config;
    AlgPidCascadeInput_t input = {
        .position_setpoint = 10.0F,
        .position_measurement = 0.0F,
        .velocity_measurement = 0.0F,
        .velocity_feedforward = 0.0F,
        .actuator_feedforward = 0.0F,
        .delta_time_s = 0.001F};
    AlgPidCascade_t controller;
    float output = 0.0F;

    config.position_config = Test_DefaultConfig();
    config.velocity_config = Test_DefaultConfig();
    config.position_config.proportional_gain = 1.0F;
    config.velocity_config.proportional_gain = 2.0F;
    config.position_loop_divider = 2U;
    config.velocity_setpoint_min = -20.0F;
    config.velocity_setpoint_max = 20.0F;

    TEST_EXPECT_TRUE(AlgPidCascade_Init(&controller, &config) == ALG_PID_STATUS_OK);
    TEST_EXPECT_TRUE(AlgPidCascade_Update(&controller, &input, &output) ==
                     ALG_PID_STATUS_OK);
    TEST_EXPECT_NEAR(AlgPidCascade_GetVelocitySetpoint(&controller),
                     10.0F,
                     TEST_TOLERANCE);
    TEST_EXPECT_NEAR(output, 20.0F, TEST_TOLERANCE);

    input.position_measurement = 5.0F;
    (void)AlgPidCascade_Update(&controller, &input, &output);
    TEST_EXPECT_NEAR(AlgPidCascade_GetVelocitySetpoint(&controller),
                     10.0F,
                     TEST_TOLERANCE);
    (void)AlgPidCascade_Update(&controller, &input, &output);
    TEST_EXPECT_NEAR(AlgPidCascade_GetVelocitySetpoint(&controller),
                     5.0F,
                     TEST_TOLERANCE);
}

static void TestErrors(void)
{
    AlgPid_t controller = {0};
    AlgPidConfig_t config = Test_DefaultConfig();
    float output = 0.0F;

    TEST_EXPECT_TRUE(AlgPid_Init(NULL, &config) == ALG_PID_STATUS_INVALID_ARGUMENT);
    config.output_min = 1.0F;
    config.output_max = 1.0F;
    TEST_EXPECT_TRUE(AlgPid_Init(&controller, &config) == ALG_PID_STATUS_OUT_OF_RANGE);
    TEST_EXPECT_TRUE(AlgPid_Update(&controller, 1.0F, 0.0F, 0.1F, &output) ==
                     ALG_PID_STATUS_NOT_INITIALIZED);
}

int main(void)
{
    TestPositionAndVelocity();
    TestIntegralAndAntiWindup();
    TestAdvancedFeatures();
    TestIncremental();
    TestGainSchedule();
    TestFuzzy();
    TestCascade();
    TestErrors();

    if (s_failure_count == 0)
    {
        (void)printf("All alg_pid tests passed.\n");
        return 0;
    }

    (void)printf("%d alg_pid test(s) failed.\n", s_failure_count);
    return 1;
}
