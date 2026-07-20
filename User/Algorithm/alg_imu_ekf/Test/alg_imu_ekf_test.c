#include "alg_imu_ekf.h"

#include <math.h>
#include <stdio.h>

#define TEST_PI_F      (3.14159265358979323846F)
#define TEST_GRAVITY   (9.80665F)
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

static void Test_InitFilter(AlgImuEkf_t *filter)
{
    AlgImuEkfConfig_t config;

    (void)AlgImuEkfConfig_Init(&config);
    TEST_EXPECT_TRUE(AlgImuEkf_Init(filter, &config) == ALG_IMU_EKF_STATUS_OK);
}

static void TestStationary(void)
{
    AlgImuEkf_t filter;
    const float gyroscope[3] = {0.0F, 0.0F, 0.0F};
    const float accelerometer[3] = {0.0F, 0.0F, TEST_GRAVITY};
    AlgImuEkfQuaternion_t quaternion;
    AlgImuEkfEuler_t euler;
    float gravity_body[3];
    float linear_acceleration_world[3];
    bool accelerometer_used = false;
    int iteration;

    Test_InitFilter(&filter);

    for (iteration = 0; iteration < 100; ++iteration)
    {
        TEST_EXPECT_TRUE(AlgImuEkf_Update(&filter,
                                         gyroscope,
                                         accelerometer,
                                         0.01F,
                                         &accelerometer_used) ==
                         ALG_IMU_EKF_STATUS_OK);
        TEST_EXPECT_TRUE(accelerometer_used);
    }
    (void)AlgImuEkf_GetQuaternion(&filter, &quaternion);
    (void)AlgImuEkf_GetEuler(&filter, &euler);
    TEST_EXPECT_NEAR(quaternion.w, 1.0F, TEST_TOLERANCE);
    TEST_EXPECT_NEAR(quaternion.x, 0.0F, TEST_TOLERANCE);
    TEST_EXPECT_NEAR(euler.roll_rad, 0.0F, TEST_TOLERANCE);
    TEST_EXPECT_NEAR(euler.pitch_rad, 0.0F, TEST_TOLERANCE);
    (void)AlgImuEkf_GetGravityBody(&filter, gravity_body);
    (void)AlgImuEkf_GetLinearAccelerationWorld(&filter,
                                                accelerometer,
                                                linear_acceleration_world);
    TEST_EXPECT_NEAR(gravity_body[2], TEST_GRAVITY, TEST_TOLERANCE);
    TEST_EXPECT_NEAR(linear_acceleration_world[0], 0.0F, TEST_TOLERANCE);
    TEST_EXPECT_NEAR(linear_acceleration_world[1], 0.0F, TEST_TOLERANCE);
    TEST_EXPECT_NEAR(linear_acceleration_world[2], 0.0F, TEST_TOLERANCE);
}

static void TestResetFromAccelerometer(void)
{
    AlgImuEkf_t filter;
    const float roll_rad = TEST_PI_F / 6.0F;
    const float accelerometer[3] = {
        0.0F,
        TEST_GRAVITY * 0.5F,
        TEST_GRAVITY * 0.8660254038F};
    AlgImuEkfEuler_t euler;

    Test_InitFilter(&filter);

    TEST_EXPECT_TRUE(AlgImuEkf_ResetFromAccelerometer(&filter, accelerometer) ==
                     ALG_IMU_EKF_STATUS_OK);
    (void)AlgImuEkf_GetEuler(&filter, &euler);
    TEST_EXPECT_NEAR(euler.roll_rad, roll_rad, 1.0e-4F);
    TEST_EXPECT_NEAR(euler.pitch_rad, 0.0F, TEST_TOLERANCE);
    TEST_EXPECT_NEAR(euler.yaw_rad, 0.0F, TEST_TOLERANCE);
}

static void TestYawIntegration(void)
{
    AlgImuEkf_t filter;
    const float gyroscope[3] = {0.0F, 0.0F, TEST_PI_F / 2.0F};
    const float accelerometer[3] = {0.0F, 0.0F, TEST_GRAVITY};
    AlgImuEkfEuler_t euler;
    int iteration;

    Test_InitFilter(&filter);

    for (iteration = 0; iteration < 100; ++iteration)
    {
        (void)AlgImuEkf_Update(&filter,
                               gyroscope,
                               accelerometer,
                               0.01F,
                               NULL);
    }
    (void)AlgImuEkf_GetEuler(&filter, &euler);
    TEST_EXPECT_NEAR(euler.yaw_rad, TEST_PI_F / 2.0F, 2.0e-3F);
    TEST_EXPECT_NEAR(euler.roll_rad, 0.0F, 2.0e-3F);
    TEST_EXPECT_NEAR(euler.pitch_rad, 0.0F, 2.0e-3F);
}

static void TestTiltCorrection(void)
{
    AlgImuEkf_t filter;
    const AlgImuEkfQuaternion_t tilted = {
        .w = 0.984807753F,
        .x = 0.173648178F,
        .y = 0.0F,
        .z = 0.0F};
    const float zero_bias[3] = {0.0F, 0.0F, 0.0F};
    const float gyroscope[3] = {0.0F, 0.0F, 0.0F};
    const float accelerometer[3] = {0.0F, 0.0F, TEST_GRAVITY};
    AlgImuEkfEuler_t euler;
    int iteration;

    Test_InitFilter(&filter);

    (void)AlgImuEkf_Reset(&filter, &tilted, zero_bias);
    for (iteration = 0; iteration < 500; ++iteration)
    {
        (void)AlgImuEkf_Update(&filter,
                               gyroscope,
                               accelerometer,
                               0.01F,
                               NULL);
    }
    (void)AlgImuEkf_GetEuler(&filter, &euler);
    TEST_EXPECT_NEAR(euler.roll_rad, 0.0F, 5.0e-3F);
}

static void TestBiasEstimation(void)
{
    AlgImuEkf_t filter;
    const float gyroscope[3] = {0.05F, 0.0F, 0.0F};
    const float accelerometer[3] = {0.0F, 0.0F, TEST_GRAVITY};
    float bias[3];
    float corrected[3];
    int iteration;

    Test_InitFilter(&filter);

    for (iteration = 0; iteration < 2000; ++iteration)
    {
        (void)AlgImuEkf_Update(&filter,
                               gyroscope,
                               accelerometer,
                               0.005F,
                               NULL);
    }
    (void)AlgImuEkf_GetGyroBias(&filter, bias);
    (void)AlgImuEkf_GetCorrectedGyroscope(&filter, gyroscope, corrected);
    TEST_EXPECT_NEAR(bias[0], 0.05F, 5.0e-3F);
    TEST_EXPECT_NEAR(corrected[0], 0.0F, 5.0e-3F);
}

static void TestAccelerometerRejection(void)
{
    AlgImuEkf_t filter;
    const float gyroscope[3] = {0.0F, 0.0F, 0.0F};
    const float accelerometer[3] = {0.0F, 0.0F, 2.0F * TEST_GRAVITY};
    bool accelerometer_used = true;

    Test_InitFilter(&filter);

    TEST_EXPECT_TRUE(AlgImuEkf_CorrectAccelerometer(&filter, accelerometer) ==
                     ALG_IMU_EKF_STATUS_ACCELEROMETER_REJECTED);
    TEST_EXPECT_TRUE(AlgImuEkf_Update(&filter,
                                     gyroscope,
                                     accelerometer,
                                     0.01F,
                                     &accelerometer_used) ==
                     ALG_IMU_EKF_STATUS_OK);
    TEST_EXPECT_TRUE(!accelerometer_used);
}

static void TestErrors(void)
{
    AlgImuEkfConfig_t config;
    AlgImuEkf_t filter = {0};
    AlgImuEkfEuler_t euler;

    TEST_EXPECT_TRUE(AlgImuEkfConfig_Init(NULL) ==
                     ALG_IMU_EKF_STATUS_INVALID_ARGUMENT);
    (void)AlgImuEkfConfig_Init(&config);
    config.gravity_m_s2 = 0.0F;
    TEST_EXPECT_TRUE(AlgImuEkf_Init(&filter, &config) ==
                     ALG_IMU_EKF_STATUS_OUT_OF_RANGE);
    TEST_EXPECT_TRUE(AlgImuEkf_GetEuler(&filter, &euler) ==
                     ALG_IMU_EKF_STATUS_NOT_INITIALIZED);
}

int main(void)
{
    TestStationary();
    TestResetFromAccelerometer();
    TestYawIntegration();
    TestTiltCorrection();
    TestBiasEstimation();
    TestAccelerometerRejection();
    TestErrors();

    if (s_failure_count == 0)
    {
        (void)printf("All alg_imu_ekf tests passed.\n");
        return 0;
    }

    (void)printf("%d alg_imu_ekf test(s) failed.\n", s_failure_count);
    return 1;
}
