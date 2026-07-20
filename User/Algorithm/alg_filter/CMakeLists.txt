add_library(alg_filter STATIC
    alg_filter_basic.c
    alg_filter_window.c
    alg_filter_fir.c
    alg_filter_biquad.c
    alg_filter_complementary.c
)

target_include_directories(alg_filter
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
)

target_compile_features(alg_filter PUBLIC c_std_11)

if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(alg_filter PRIVATE
        -Wall
        -Wextra
        -Wshadow
        -Wconversion
        -Wdouble-promotion
        -Wundef
        -Werror=return-type
    )
endif()

target_link_libraries(alg_filter PUBLIC m)
