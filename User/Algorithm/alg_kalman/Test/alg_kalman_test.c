#include "alg_kalman.h"

#include <math.h>
#include <stdio.h>

#define TEST_TOLERANCE (1.0e-4F)

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

static void TestScalar(void)
{
    AlgKalmanScalar_t filter;
    float output = 0.0F;
    int iteration;

    TEST_EXPECT_TRUE(AlgKalmanScalar_Init(&filter, 0.01F, 1.0F, 0.0F, 1.0F) ==
                     ALG_KALMAN_STATUS_OK);
    for (iteration = 0; iteration < 100; ++iteration)
    {
        TEST_EXPECT_TRUE(AlgKalmanScalar_Update(&filter, 10.0F, &output) ==
                         ALG_KALMAN_STATUS_OK);
    }
    TEST_EXPECT_NEAR(output, 10.0F, 1.0e-2F);
    TEST_EXPECT_TRUE((filter.gain > 0.0F) && (filter.gain < 1.0F));

    TEST_EXPECT_TRUE(AlgKalmanScalar_Reset(&filter, 1.0F, 2.0F) ==
                     ALG_KALMAN_STATUS_OK);
    TEST_EXPECT_TRUE(AlgKalmanScalar_Predict(&filter, 3.0F) ==
                     ALG_KALMAN_STATUS_OK);
    TEST_EXPECT_NEAR(filter.estimate, 4.0F, TEST_TOLERANCE);
}

static void TestLinearConstantAcceleration(void)
{
    enum
    {
        STATE_DIMENSION = 2,
        MEASUREMENT_DIMENSION = 1,
        CONTROL_DIMENSION = 1
    };
    float state[STATE_DIMENSION] = {0.0F, 0.0F};
    float covariance[STATE_DIMENSION * STATE_DIMENSION] = {
        1.0F, 0.0F,
        0.0F, 1.0F};
    const float transition[STATE_DIMENSION * STATE_DIMENSION] = {
        1.0F, 1.0F,
        0.0F, 1.0F};
    const float control_model[STATE_DIMENSION * CONTROL_DIMENSION] = {0.5F, 1.0F};
    const float process_noise[STATE_DIMENSION * STATE_DIMENSION] = {
        0.001F, 0.0F,
        0.0F, 0.001F};
    const float measurement_model[MEASUREMENT_DIMENSION * STATE_DIMENSION] = {
        1.0F, 0.0F};
    const float measurement_noise[MEASUREMENT_DIMENSION * MEASUREMENT_DIMENSION] = {
        0.05F};
    float workspace[ALG_KALMAN_WORKSPACE_SIZE(STATE_DIMENSION,
                                              MEASUREMENT_DIMENSION)];
    AlgKalmanLinearConfig_t config = {
        .state_dimension = STATE_DIMENSION,
        .measurement_dimension = MEASUREMENT_DIMENSION,
        .control_dimension = CONTROL_DIMENSION,
        .state = state,
        .covariance = covariance,
        .transition_matrix = transition,
        .control_matrix = control_model,
        .process_noise = process_noise,
        .measurement_matrix = measurement_model,
        .measurement_noise = measurement_noise,
        .workspace = workspace,
        .workspace_size = sizeof(workspace) / sizeof(workspace[0])};
    AlgKalmanLinear_t filter;
    const float control_input[CONTROL_DIMENSION] = {1.0F};
    float measurement[MEASUREMENT_DIMENSION];
    float true_position = 0.0F;
    float true_velocity = 0.0F;
    int iteration;

    TEST_EXPECT_TRUE(AlgKalmanLinear_Init(&filter, &config) == ALG_KALMAN_STATUS_OK);
    for (iteration = 0; iteration < 20; ++iteration)
    {
        true_position += true_velocity + 0.5F;
        true_velocity += 1.0F;
        measurement[0] = true_position + (((iteration % 2) == 0) ? 0.1F : -0.1F);
        TEST_EXPECT_TRUE(AlgKalmanLinear_Predict(&filter, control_input) ==
                         ALG_KALMAN_STATUS_OK);
        TEST_EXPECT_TRUE(AlgKalmanLinear_Correct(&filter, measurement) ==
                         ALG_KALMAN_STATUS_OK);
    }

    TEST_EXPECT_NEAR(state[0], true_position, 0.2F);
    TEST_EXPECT_NEAR(state[1], true_velocity, 0.2F);
    TEST_EXPECT_NEAR(covariance[1], covariance[2], TEST_TOLERANCE);
    TEST_EXPECT_TRUE(AlgKalmanLinear_GetState(&filter) == state);
}

static void TestLinearMultipleMeasurements(void)
{
    enum
    {
        DIMENSION = 2
    };
    float state[DIMENSION] = {0.0F, 0.0F};
    float covariance[DIMENSION * DIMENSION] = {1.0F, 0.0F, 0.0F, 1.0F};
    const float identity[DIMENSION * DIMENSION] = {1.0F, 0.0F, 0.0F, 1.0F};
    const float process_noise[DIMENSION * DIMENSION] = {0.01F, 0.0F, 0.0F, 0.01F};
    const float measurement_noise[DIMENSION * DIMENSION] = {0.1F, 0.0F, 0.0F, 0.2F};
    const float measurement[DIMENSION] = {1.0F, -2.0F};
    float workspace[ALG_KALMAN_WORKSPACE_SIZE(DIMENSION, DIMENSION)];
    AlgKalmanLinearConfig_t config = {
        .state_dimension = DIMENSION,
        .measurement_dimension = DIMENSION,
        .control_dimension = 0U,
        .state = state,
        .covariance = covariance,
        .transition_matrix = identity,
        .control_matrix = NULL,
        .process_noise = process_noise,
        .measurement_matrix = identity,
        .measurement_noise = measurement_noise,
        .workspace = workspace,
        .workspace_size = sizeof(workspace) / sizeof(workspace[0])};
    AlgKalmanLinear_t filter;

    TEST_EXPECT_TRUE(AlgKalmanLinear_Init(&filter, &config) == ALG_KALMAN_STATUS_OK);
    TEST_EXPECT_TRUE(AlgKalmanLinear_Predict(&filter, NULL) == ALG_KALMAN_STATUS_OK);
    TEST_EXPECT_TRUE(AlgKalmanLinear_Correct(&filter, measurement) ==
                     ALG_KALMAN_STATUS_OK);
    TEST_EXPECT_TRUE((state[0] > 0.0F) && (state[0] < 1.0F));
    TEST_EXPECT_TRUE((state[1] < 0.0F) && (state[1] > -2.0F));
}

static AlgKalmanStatus_t TestEkf_State(const float *state,
                                       size_t state_dimension,
                                       const float *control_input,
                                       size_t control_dimension,
                                       float delta_time_s,
                                       float *predicted_state,
                                       void *user_context)
{
    (void)control_input;
    (void)control_dimension;
    (void)delta_time_s;
    (void)user_context;
    if (state_dimension != 1U)
    {
        return ALG_KALMAN_STATUS_MODEL_ERROR;
    }
    predicted_state[0] = state[0];
    return ALG_KALMAN_STATUS_OK;
}

static AlgKalmanStatus_t TestEkf_StateJacobian(const float *state,
                                               size_t state_dimension,
                                               const float *control_input,
                                               size_t control_dimension,
                                               float delta_time_s,
                                               float *state_jacobian,
                                               void *user_context)
{
    (void)state;
    (void)control_input;
    (void)control_dimension;
    (void)delta_time_s;
    (void)user_context;
    if (state_dimension != 1U)
    {
        return ALG_KALMAN_STATUS_MODEL_ERROR;
    }
    state_jacobian[0] = 1.0F;
    return ALG_KALMAN_STATUS_OK;
}

static AlgKalmanStatus_t TestEkf_Measurement(const float *state,
                                             size_t state_dimension,
                                             size_t measurement_dimension,
                                             float *predicted_measurement,
                                             void *user_context)
{
    (void)user_context;
    if ((state_dimension != 1U) || (measurement_dimension != 1U))
    {
        return ALG_KALMAN_STATUS_MODEL_ERROR;
    }
    predicted_measurement[0] = state[0] * state[0];
    return ALG_KALMAN_STATUS_OK;
}

static AlgKalmanStatus_t TestEkf_MeasurementJacobian(
    const float *state,
    size_t state_dimension,
    size_t measurement_dimension,
    float *measurement_jacobian,
    void *user_context)
{
    (void)user_context;
    if ((state_dimension != 1U) || (measurement_dimension != 1U))
    {
        return ALG_KALMAN_STATUS_MODEL_ERROR;
    }
    measurement_jacobian[0] = 2.0F * state[0];
    return ALG_KALMAN_STATUS_OK;
}

static void TestExtended(void)
{
    float state[1] = {2.0F};
    float covariance[1] = {1.0F};
    const float process_noise[1] = {0.001F};
    const float measurement_noise[1] = {0.05F};
    const float measurement[1] = {9.0F};
    float workspace[ALG_KALMAN_WORKSPACE_SIZE(1U, 1U)];
    AlgKalmanExtendedConfig_t config = {
        .state_dimension = 1U,
        .measurement_dimension = 1U,
        .control_dimension = 0U,
        .state = state,
        .covariance = covariance,
        .process_noise = process_noise,
        .measurement_noise = measurement_noise,
        .workspace = workspace,
        .workspace_size = sizeof(workspace) / sizeof(workspace[0]),
        .state_function = TestEkf_State,
        .state_jacobian_function = TestEkf_StateJacobian,
        .measurement_function = TestEkf_Measurement,
        .measurement_jacobian_function = TestEkf_MeasurementJacobian,
        .user_context = NULL};
    AlgKalmanExtended_t filter;
    int iteration;

    TEST_EXPECT_TRUE(AlgKalmanExtended_Init(&filter, &config) == ALG_KALMAN_STATUS_OK);
    for (iteration = 0; iteration < 20; ++iteration)
    {
        TEST_EXPECT_TRUE(AlgKalmanExtended_Predict(&filter, NULL, 0.01F) ==
                         ALG_KALMAN_STATUS_OK);
        TEST_EXPECT_TRUE(AlgKalmanExtended_Correct(&filter, measurement) ==
                         ALG_KALMAN_STATUS_OK);
    }
    TEST_EXPECT_NEAR(state[0], 3.0F, 1.0e-2F);
    TEST_EXPECT_TRUE(AlgKalmanExtended_GetCovariance(&filter) == covariance);
}

static void TestErrors(void)
{
    AlgKalmanScalar_t scalar = {0};
    float state[1] = {0.0F};
    float covariance[1] = {0.0F};
    const float identity[1] = {1.0F};
    const float zero_noise[1] = {0.0F};
    const float measurement[1] = {1.0F};
    float workspace[ALG_KALMAN_WORKSPACE_SIZE(1U, 1U)];
    AlgKalmanLinearConfig_t config = {
        .state_dimension = 1U,
        .measurement_dimension = 1U,
        .control_dimension = 0U,
        .state = state,
        .covariance = covariance,
        .transition_matrix = identity,
        .control_matrix = NULL,
        .process_noise = zero_noise,
        .measurement_matrix = identity,
        .measurement_noise = zero_noise,
        .workspace = workspace,
        .workspace_size = (sizeof(workspace) / sizeof(workspace[0])) - 1U};
    AlgKalmanLinear_t linear;
    float output = 0.0F;

    TEST_EXPECT_TRUE(AlgKalmanScalar_Init(NULL, 0.1F, 1.0F, 0.0F, 1.0F) ==
                     ALG_KALMAN_STATUS_INVALID_ARGUMENT);
    TEST_EXPECT_TRUE(AlgKalmanScalar_Init(&scalar, -0.1F, 1.0F, 0.0F, 1.0F) ==
                     ALG_KALMAN_STATUS_OUT_OF_RANGE);
    TEST_EXPECT_TRUE(AlgKalmanScalar_Update(&scalar, 1.0F, &output) ==
                     ALG_KALMAN_STATUS_NOT_INITIALIZED);

    TEST_EXPECT_TRUE(AlgKalmanLinear_Init(&linear, &config) ==
                     ALG_KALMAN_STATUS_INSUFFICIENT_WORKSPACE);
    config.workspace_size = sizeof(workspace) / sizeof(workspace[0]);
    TEST_EXPECT_TRUE(AlgKalmanLinear_Init(&linear, &config) == ALG_KALMAN_STATUS_OK);
    TEST_EXPECT_TRUE(AlgKalmanLinear_Correct(&linear, measurement) ==
                     ALG_KALMAN_STATUS_SINGULAR_MATRIX);
}

int main(void)
{
    TestScalar();
    TestLinearConstantAcceleration();
    TestLinearMultipleMeasurements();
    TestExtended();
    TestErrors();

    if (s_failure_count == 0)
    {
        (void)printf("All alg_kalman tests passed.\n");
        return 0;
    }

    (void)printf("%d alg_kalman test(s) failed.\n", s_failure_count);
    return 1;
}
