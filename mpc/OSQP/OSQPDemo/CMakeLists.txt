cmake_minimum_required(VERSION 3.22.1)
project(OSQP_Demo)

set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
# set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -o -Wall")
# add_compile_options(-Wall -std=c++11)

# find_package(osqp REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(Eigen3 REQUIRED)

set(CMAKE_BUILD_TYPE Debug)
# set(CMAKE_BUILD_TYPE Release)

# include_directories(${CMAKE_SOURCE_DIR}/Inc)
# include_directories(./Inc)
# include_directories(./Inc/MyDef ./Inc)

set(MYSRC main.cpp)

add_executable(osqp_demo ${MYSRC})
target_link_libraries(osqp_demo
    # osqp::osqp
    OsqpEigen::OsqpEigen
    Eigen3::Eigen
)