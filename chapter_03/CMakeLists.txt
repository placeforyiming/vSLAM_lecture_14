cmake_minimum_required(VERSION 3.2)

set(CMAKE_BUILD_TYPE "Debug" )

project(Geometry)

include_directories("/usr/include/eigen3")

add_executable(useEigen useEigen/eigenMatrix.cpp)

add_executable(useGeometry useGeometry/useGeometry.cpp)

find_package(Pangolin)
include_directories(${Pangolin_INCLUDE_DIRS} )
add_executable(plotTraj plotTraj/plotTraj.cpp)
target_link_libraries( plotTraj ${Pangolin_LIBRARIES} )
