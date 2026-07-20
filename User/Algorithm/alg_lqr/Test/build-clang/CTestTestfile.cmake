# CMake generated Testfile for 
# Source directory: D:/ACE/general_framework/User/Algorithm/alg_lqr/Test
# Build directory: D:/ACE/general_framework/User/Algorithm/alg_lqr/Test/build-clang
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test([=[alg_lqr_tests]=] "D:/ACE/general_framework/User/Algorithm/alg_lqr/Test/build-clang/alg_lqr_tests.exe")
set_tests_properties([=[alg_lqr_tests]=] PROPERTIES  _BACKTRACE_TRIPLES "D:/ACE/general_framework/User/Algorithm/alg_lqr/Test/CMakeLists.txt;22;add_test;D:/ACE/general_framework/User/Algorithm/alg_lqr/Test/CMakeLists.txt;0;")
subdirs("alg_lqr_library")
