add_library(alg_lqr STATIC
    alg_lqr_matrix.c
    alg_lqr_riccati.c
    alg_lqr_controller.c
    alg_lqr_model.c
)

target_include_directories(alg_lqr
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
)

target_compile_features(alg_lqr PUBLIC c_std_11)

if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(alg_lqr PRIVATE
        -Wall
        -Wextra
        -Wshadow
        -Wconversion
        -Wdouble-promotion
        -Wundef
        -Werror=return-type
    )
endif()

target_link_libraries(alg_lqr PUBLIC m)
