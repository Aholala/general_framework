add_library(alg_kalman STATIC
    alg_kalman_matrix.c
    alg_kalman_scalar.c
    alg_kalman_linear.c
    alg_kalman_extended.c
)

target_include_directories(alg_kalman
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
)

target_compile_features(alg_kalman PUBLIC c_std_11)

if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(alg_kalman PRIVATE
        -Wall
        -Wextra
        -Wshadow
        -Wconversion
        -Wdouble-promotion
        -Wundef
        -Werror=return-type
    )
endif()

target_link_libraries(alg_kalman PUBLIC m)
