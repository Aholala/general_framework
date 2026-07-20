cmake_minimum_required(VERSION 3.22)
project(alg_lqr_tests LANGUAGES C)

enable_testing()
add_subdirectory(.. alg_lqr_library)

add_executable(alg_lqr_tests alg_lqr_test.c)
target_link_libraries(alg_lqr_tests PRIVATE alg_lqr)

if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(alg_lqr_tests PRIVATE
        -Wall
        -Wextra
        -Wshadow
        -Wconversion
        -Wdouble-promotion
        -Wundef
        -Werror
    )
endif()

add_test(NAME alg_lqr_tests COMMAND alg_lqr_tests)
