# # Methode 1
# g++ -o demo main.cpp -lipopt -I/usr/local/include/coin-or/



# # Methode 2
cmake_minimum_required(VERSION 3.22.1)
project(IPOPT_Demo)

set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
# set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -o -Wall")
# add_compile_options(-Wall -std=c++11)

find_package(Eigen3 REQUIRED)


set(MYSRC main.cpp controller.cpp common.cpp)
add_executable(ipopt_demo ${MYSRC})

target_include_directories(ipopt_demo PRIVATE
    /usr/local/include/coin-or/
)

set(CMAKE_BUILD_TYPE Debug)
# set(CMAKE_BUILD_TYPE Release)

target_link_libraries(ipopt_demo PRIVATE
    ipopt
    # coinmumps
    # coinhsl
    # dl
    Eigen3::Eigen
)

# # Methode 3
# # Using PkgConfig to find IPOPT and link it to the project.
# cmake_minimum_required(VERSION 3.22.1)
# project(IPOPT_Demo)

# set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
# # set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -o -Wall")
# # add_compile_options(-Wall -std=c++11)

# find_package(PkgConfig REQUIRED)
# pkg_search_module(IPOPT REQUIRED ipopt)

# set(MYSRC main.cpp)
# add_executable(ipopt_demo ${MYSRC})

# target_include_directories(ipopt_demo PRIVATE
#     /usr/local/include/coin-or/
# )

# set(CMAKE_BUILD_TYPE Debug)
# # set(CMAKE_BUILD_TYPE Release)


# target_include_directories(ipopt_demo PRIVATE
#     ${IPOPT_INCLUDE_DIRS}
# )

# target_link_directories(ipopt_demo PRIVATE
#     ${IPOPT_LIBRARY_DIRS}
# )

# target_link_libraries(ipopt_demo
#     ${IPOPT_LIBRARIES}
# )

