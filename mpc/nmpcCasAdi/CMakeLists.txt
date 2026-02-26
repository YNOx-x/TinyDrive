cmake_minimum_required(VERSION 3.10)

project(diffmpc)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})


add_executable(virtualSand ./main.cpp )
target_link_libraries(virtualSand casadi) 
set_target_properties(virtualSand PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")


