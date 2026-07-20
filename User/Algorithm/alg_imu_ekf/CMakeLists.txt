add_library(alg_imu_ekf STATIC
    alg_imu_ekf_core.c
    alg_imu_ekf_model.c
    alg_imu_ekf_update.c
    alg_imu_ekf_output.c
)

target_include_directories(alg_imu_ekf
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
)

target_compile_features(alg_imu_ekf PUBLIC c_std_11)

if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(alg_imu_ekf PRIVATE
        -Wall
        -Wextra
        -Wshadow
        -Wconversion
        -Wdouble-promotion
        -Wundef
        -Werror=return-type
    )
endif()

target_link_libraries(alg_imu_ekf PUBLIC alg_filter alg_kalman m)
