cmake_minimum_required(VERSION 2.6)

set (CMAKE_CXX_STANDARD 11)

set(DEF_BIN_PREFIX bin)
set(DEF_LIB_PREFIX lib)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)

add_executable(aatc_test AATC_test.cpp)
target_link_libraries(aatc_test casadi)

add_executable(FBL main.cpp)
add_library(aatc AATC.cpp)
target_link_libraries(aatc casadi)
target_link_libraries(FBL aatc casadi)