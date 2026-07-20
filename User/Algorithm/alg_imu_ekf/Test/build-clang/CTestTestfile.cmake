# CMake generated Testfile for 
# Source directory: D:/ACE/general_framework/User/Algorithm/alg_imu_ekf/Test
# Build directory: D:/ACE/general_framework/User/Algorithm/alg_imu_ekf/Test/build-clang
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test([=[alg_imu_ekf_tests]=] "D:/ACE/general_framework/User/Algorithm/alg_imu_ekf/Test/build-clang/alg_imu_ekf_tests.exe")
set_tests_properties([=[alg_imu_ekf_tests]=] PROPERTIES  _BACKTRACE_TRIPLES "D:/ACE/general_framework/User/Algorithm/alg_imu_ekf/Test/CMakeLists.txt;23;add_test;D:/ACE/general_framework/User/Algorithm/alg_imu_ekf/Test/CMakeLists.txt;0;")
subdirs("alg_kalman_library")
subdirs("alg_imu_ekf_library")
