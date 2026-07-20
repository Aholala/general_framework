cmake_minimum_required(VERSION 3.22)
project(alg_pid_tests LANGUAGES C)

enable_testing()
add_subdirectory(.. alg_pid_library)

add_executable(alg_pid_tests alg_pid_test.c)
target_link_libraries(alg_pid_tests PRIVATE alg_pid)

if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(alg_pid_tests PRIVATE
        -Wall
        -Wextra
        -Wshadow
        -Wconversion
        -Wdouble-promotion
        -Wundef
        -Werror
    )
endif()

add_test(NAME alg_pid_tests COMMAND alg_pid_tests)
