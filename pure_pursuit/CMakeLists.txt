cmake_minimum_required(VERSION 3.22.1)
project(VIRTUALMAIN)

set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(CMAKE_BUILD_TYPE Debug)

set(MYSRC main.cpp)

add_executable(virtualSand ${MYSRC})

