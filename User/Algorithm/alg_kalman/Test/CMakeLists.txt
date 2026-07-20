cmake_minimum_required(VERSION 3.22)
project(alg_kalman_tests LANGUAGES C)

enable_testing()
add_subdirectory(.. alg_kalman_library)

add_executable(alg_kalman_tests alg_kalman_test.c)
target_link_libraries(alg_kalman_tests PRIVATE alg_kalman)

if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(alg_kalman_tests PRIVATE
        -Wall
        -Wextra
        -Wshadow
        -Wconversion
        -Wdouble-promotion
        -Wundef
        -Werror
    )
endif()

add_test(NAME alg_kalman_tests COMMAND alg_kalman_tests)
