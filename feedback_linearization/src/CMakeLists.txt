cmake_minimum_required(VERSION 3.22.1)
project(VIRTUALMAIN)

set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
# set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -o -Wall")
# add_compile_options(-Wall -std=c++11)

set(CMAKE_BUILD_TYPE Debug)
# set(CMAKE_BUILD_TYPE Release)

# include_directories(${CMAKE_SOURCE_DIR}/Inc)
# include_directories(./Inc)
# include_directories(./Inc/MyDef ./Inc)

set(MYSRC main.cpp)

add_executable(VirtualSand ${MYSRC})

