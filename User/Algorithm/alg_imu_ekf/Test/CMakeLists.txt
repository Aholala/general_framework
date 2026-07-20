cmake_minimum_required(VERSION 3.22)
project(alg_imu_ekf_tests LANGUAGES C)

enable_testing()
add_subdirectory(../../alg_filter alg_filter_library)
add_subdirectory(../../alg_kalman alg_kalman_library)
add_subdirectory(.. alg_imu_ekf_library)

add_executable(alg_imu_ekf_tests alg_imu_ekf_test.c)
target_link_libraries(alg_imu_ekf_tests PRIVATE alg_imu_ekf)

if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(alg_imu_ekf_tests PRIVATE
        -Wall
        -Wextra
        -Wshadow
        -Wconversion
        -Wdouble-promotion
        -Wundef
        -Werror
    )
endif()

add_test(NAME alg_imu_ekf_tests COMMAND alg_imu_ekf_tests)
