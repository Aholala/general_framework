# CMake generated Testfile for 
# Source directory: D:/ACE/general_framework/User/Algorithm/alg_kalman/Test
# Build directory: D:/ACE/general_framework/User/Algorithm/alg_kalman/Test/build-clang
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test([=[alg_kalman_tests]=] "D:/ACE/general_framework/User/Algorithm/alg_kalman/Test/build-clang/alg_kalman_tests.exe")
set_tests_properties([=[alg_kalman_tests]=] PROPERTIES  _BACKTRACE_TRIPLES "D:/ACE/general_framework/User/Algorithm/alg_kalman/Test/CMakeLists.txt;22;add_test;D:/ACE/general_framework/User/Algorithm/alg_kalman/Test/CMakeLists.txt;0;")
subdirs("alg_kalman_library")
