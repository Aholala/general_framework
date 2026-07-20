#include "alg_lqr.h"

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

static void TestInfiniteHorizonScalar(void)
{
    const float state_matrix[1] = {1.0F};
    const float control_matrix[1] = {1.0F};
    const float state_weight[1] = {1.0F};
    const float control_weight[1] = {1.0F};
    float workspace[ALG_LQR_RICCATI_WORKSPACE_SIZE(1U, 1U)];
    const AlgLqrDareConfig_t config = {
        .state_dimension = 1U,
        .control_dimension = 1U,
        .state_matrix = state_matrix,
        .control_matrix = control_matrix,
        .state_weight = state_weight,
        .control_weight = control_weight,
        .cross_weight = NULL,
        .tolerance = 1.0e-6F,
        .maximum_iterations = 100U,
        .workspace = workspace,
        .workspace_size = sizeof(workspace) / sizeof(workspace[0])};
    float riccati_solution[1];
    float gain[1];
    size_t completed_iterations = 0U;

    TEST_EXPECT_TRUE(AlgLqrDare_Solve(&config,
                                     riccati_solution,
                                     gain,
                                     &completed_iterations) == ALG_LQR_STATUS_OK);
    TEST_EXPECT_NEAR(riccati_solution[0], 1.61803399F, 1.0e-4F);
    TEST_EXPECT_NEAR(gain[0], 0.61803399F, 1.0e-4F);
    TEST_EXPECT_TRUE(completed_iterations > 0U);
}

static void TestDoubleIntegratorClosedLoop(void)
{
    const float state_matrix[4] = {
        1.0F, 0.1F,
        0.0F, 1.0F};
    const float control_matrix[2] = {0.005F, 0.1F};
    const float state_weight[4] = {
        10.0F, 0.0F,
        0.0F, 1.0F};
    const float control_weight[1] = {0.1F};
    float workspace[ALG_LQR_RICCATI_WORKSPACE_SIZE(2U, 1U)];
    const AlgLqrDareConfig_t solve_config = {
        .state_dimension = 2U,
        .control_dimension = 1U,
        .state_matrix = state_matrix,
        .control_matrix = control_matrix,
        .state_weight = state_weight,
        .control_weight = control_weight,
        .cross_weight = NULL,
        .tolerance = 1.0e-5F,
        .maximum_iterations = 1000U,
        .workspace = workspace,
        .workspace_size = sizeof(workspace) / sizeof(workspace[0])};
    float riccati[4];
    float gain[2];
    const AlgLqrControllerConfig_t controller_config = {
        .state_dimension = 2U,
        .control_dimension = 1U,
        .gain_matrix = gain,
        .control_min = NULL,
        .control_max = NULL};
    AlgLqrController_t controller;
    float state[2] = {1.0F, 0.0F};
    float next_state[2];
    float control[1];
    int iteration;

    TEST_EXPECT_TRUE(AlgLqrDare_Solve(&solve_config, riccati, gain, NULL) ==
                     ALG_LQR_STATUS_OK);
    TEST_EXPECT_TRUE(AlgLqrController_Init(&controller, &controller_config) ==
                     ALG_LQR_STATUS_OK);
    for (iteration = 0; iteration < 200; ++iteration)
    {
        (void)AlgLqrController_Update(&controller,
                                      state,
                                      NULL,
                                      NULL,
                                      NULL,
                                      control);
        next_state[0] = state[0] + (0.1F * state[1]) +
                        (0.005F * control[0]);
        next_state[1] = state[1] + (0.1F * control[0]);
        state[0] = next_state[0];
        state[1] = next_state[1];
    }
    TEST_EXPECT_NEAR(state[0], 0.0F, 1.0e-3F);
    TEST_EXPECT_NEAR(state[1], 0.0F, 1.0e-3F);
}

static void TestController(void)
{
    const float gain[4] = {
        2.0F, 1.0F,
        -1.0F, 3.0F};
    const float control_min[2] = {-5.0F, -5.0F};
    const float control_max[2] = {5.0F, 5.0F};
    const AlgLqrControllerConfig_t config = {
        .state_dimension = 2U,
        .control_dimension = 2U,
        .gain_matrix = gain,
        .control_min = control_min,
        .control_max = control_max};
    AlgLqrController_t controller;
    const float state[2] = {3.0F, 1.0F};
    const float reference[2] = {1.0F, 0.0F};
    const float equilibrium[2] = {1.0F, -1.0F};
    const float feedforward[2] = {0.5F, 0.5F};
    float output[2];

    TEST_EXPECT_TRUE(AlgLqrController_Init(&controller, &config) ==
                     ALG_LQR_STATUS_OK);
    TEST_EXPECT_TRUE(AlgLqrController_Update(&controller,
                                            state,
                                            reference,
                                            equilibrium,
                                            feedforward,
                                            output) == ALG_LQR_STATUS_OK);
    TEST_EXPECT_NEAR(output[0], -3.5F, TEST_TOLERANCE);
    TEST_EXPECT_NEAR(output[1], -1.5F, TEST_TOLERANCE);

    TEST_EXPECT_TRUE(AlgLqrController_Update(&controller,
                                            (const float[2]){100.0F, 100.0F},
                                            NULL,
                                            NULL,
                                            NULL,
                                            output) == ALG_LQR_STATUS_OK);
    TEST_EXPECT_NEAR(output[0], -5.0F, TEST_TOLERANCE);
}

static void TestFiniteHorizon(void)
{
    const float state_matrix[1] = {1.0F};
    const float control_matrix[1] = {1.0F};
    const float state_weight[1] = {1.0F};
    const float control_weight[1] = {1.0F};
    const float terminal_weight[1] = {1.0F};
    float workspace[ALG_LQR_FINITE_WORKSPACE_SIZE(1U, 1U)];
    const AlgLqrFiniteConfig_t config = {
        .state_dimension = 1U,
        .control_dimension = 1U,
        .horizon_length = 2U,
        .state_matrix = state_matrix,
        .control_matrix = control_matrix,
        .state_weight = state_weight,
        .control_weight = control_weight,
        .cross_weight = NULL,
        .terminal_state_weight = terminal_weight,
        .workspace = workspace,
        .workspace_size = sizeof(workspace) / sizeof(workspace[0])};
    float gain_sequence[2];
    float initial_riccati[1];

    TEST_EXPECT_TRUE(AlgLqrFinite_Solve(&config,
                                       gain_sequence,
                                       initial_riccati) == ALG_LQR_STATUS_OK);
    TEST_EXPECT_NEAR(gain_sequence[0], 0.6F, TEST_TOLERANCE);
    TEST_EXPECT_NEAR(gain_sequence[1], 0.5F, TEST_TOLERANCE);
    TEST_EXPECT_NEAR(initial_riccati[0], 1.6F, TEST_TOLERANCE);
}

static void TestCrossWeight(void)
{
    const float one[1] = {1.0F};
    const float cross_weight[1] = {0.2F};
    float workspace[ALG_LQR_FINITE_WORKSPACE_SIZE(1U, 1U)];
    const AlgLqrFiniteConfig_t config = {
        .state_dimension = 1U,
        .control_dimension = 1U,
        .horizon_length = 1U,
        .state_matrix = one,
        .control_matrix = one,
        .state_weight = one,
        .control_weight = one,
        .cross_weight = cross_weight,
        .terminal_state_weight = one,
        .workspace = workspace,
        .workspace_size = sizeof(workspace) / sizeof(workspace[0])};
    float gain[1];
    float riccati[1];

    TEST_EXPECT_TRUE(AlgLqrFinite_Solve(&config, gain, riccati) ==
                     ALG_LQR_STATUS_OK);
    TEST_EXPECT_NEAR(gain[0], 0.6F, TEST_TOLERANCE);
    TEST_EXPECT_NEAR(riccati[0], 1.28F, TEST_TOLERANCE);
}

static void TestTustinDiscretization(void)
{
    const float continuous_state[1] = {-2.0F};
    const float continuous_control[1] = {1.0F};
    float discrete_state[1];
    float discrete_control[1];
    float workspace[ALG_LQR_DISCRETIZE_WORKSPACE_SIZE(1U, 1U)];

    TEST_EXPECT_TRUE(AlgLqrDiscretize_Tustin(
                         continuous_state,
                         continuous_control,
                         1U,
                         1U,
                         0.1F,
                         discrete_state,
                         discrete_control,
                         workspace,
                         sizeof(workspace) / sizeof(workspace[0])) ==
                     ALG_LQR_STATUS_OK);
    TEST_EXPECT_NEAR(discrete_state[0], 0.81818182F, TEST_TOLERANCE);
    TEST_EXPECT_NEAR(discrete_control[0], 0.09090909F, TEST_TOLERANCE);
}

static void TestLqiAugmentation(void)
{
    const float state_matrix[1] = {0.9F};
    const float control_matrix[1] = {0.1F};
    const float output_matrix[1] = {1.0F};
    float augmented_state[4];
    float augmented_control[2];

    TEST_EXPECT_TRUE(AlgLqrLqi_BuildAugmentedModel(state_matrix,
                                                   control_matrix,
                                                   output_matrix,
                                                   1U,
                                                   1U,
                                                   1U,
                                                   0.1F,
                                                   augmented_state,
                                                   augmented_control) ==
                     ALG_LQR_STATUS_OK);
    TEST_EXPECT_NEAR(augmented_state[0], 0.9F, TEST_TOLERANCE);
    TEST_EXPECT_NEAR(augmented_state[1], 0.0F, TEST_TOLERANCE);
    TEST_EXPECT_NEAR(augmented_state[2], -0.1F, TEST_TOLERANCE);
    TEST_EXPECT_NEAR(augmented_state[3], 1.0F, TEST_TOLERANCE);
    TEST_EXPECT_NEAR(augmented_control[0], 0.1F, TEST_TOLERANCE);
    TEST_EXPECT_NEAR(augmented_control[1], 0.0F, TEST_TOLERANCE);
}

static void TestErrors(void)
{
    const float state_matrix[1] = {1.0F};
    const float zero_control[1] = {0.0F};
    const float state_weight[1] = {1.0F};
    const float zero_control_weight[1] = {0.0F};
    float workspace[ALG_LQR_RICCATI_WORKSPACE_SIZE(1U, 1U)];
    AlgLqrDareConfig_t config = {
        .state_dimension = 1U,
        .control_dimension = 1U,
        .state_matrix = state_matrix,
        .control_matrix = zero_control,
        .state_weight = state_weight,
        .control_weight = zero_control_weight,
        .cross_weight = NULL,
        .tolerance = 1.0e-6F,
        .maximum_iterations = 10U,
        .workspace = workspace,
        .workspace_size = sizeof(workspace) / sizeof(workspace[0])};
    float riccati[1];
    float gain[1];

    TEST_EXPECT_TRUE(AlgLqrDare_Solve(&config, riccati, gain, NULL) ==
                     ALG_LQR_STATUS_SINGULAR_MATRIX);
    config.control_matrix = (const float[1]){1.0F};
    config.control_weight = (const float[1]){1.0F};
    config.maximum_iterations = 1U;
    config.tolerance = 1.0e-12F;
    TEST_EXPECT_TRUE(AlgLqrDare_Solve(&config, riccati, gain, NULL) ==
                     ALG_LQR_STATUS_NOT_CONVERGED);
}

int main(void)
{
    TestInfiniteHorizonScalar();
    TestDoubleIntegratorClosedLoop();
    TestController();
    TestFiniteHorizon();
    TestCrossWeight();
    TestTustinDiscretization();
    TestLqiAugmentation();
    TestErrors();

    if (s_failure_count == 0)
    {
        (void)printf("All alg_lqr tests passed.\n");
        return 0;
    }

    (void)printf("%d alg_lqr test(s) failed.\n", s_failure_count);
    return 1;
}
