add_library(alg_pid STATIC
    alg_pid_core.c
    alg_pid_incremental.c
    alg_pid_schedule.c
    alg_pid_fuzzy.c
    alg_pid_cascade.c
)

target_include_directories(alg_pid
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
)

target_compile_features(alg_pid PUBLIC c_std_11)

if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(alg_pid PRIVATE
        -Wall
        -Wextra
        -Wshadow
        -Wconversion
        -Wdouble-promotion
        -Wundef
        -Werror=return-type
    )
endif()

target_link_libraries(alg_pid PUBLIC m)
