# CMake generated Testfile for 
# Source directory: D:/ACE/general_framework/User/Algorithm/alg_filter/Test
# Build directory: D:/ACE/general_framework/User/Algorithm/alg_filter/Test/build-clang
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test([=[alg_filter_tests]=] "D:/ACE/general_framework/User/Algorithm/alg_filter/Test/build-clang/alg_filter_tests.exe")
set_tests_properties([=[alg_filter_tests]=] PROPERTIES  _BACKTRACE_TRIPLES "D:/ACE/general_framework/User/Algorithm/alg_filter/Test/CMakeLists.txt;22;add_test;D:/ACE/general_framework/User/Algorithm/alg_filter/Test/CMakeLists.txt;0;")
subdirs("alg_filter_library")
