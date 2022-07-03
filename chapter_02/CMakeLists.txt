cmake_minimum_required(VERSION 3.2)

set(CMAKE_BUILD_TYPE "Debug" )

project(HelloSLAM)

#add_library(HelloSlam src/helloSLAM.cpp)

add_library(HelloSlam SHARED src/helloSLAM.cpp)

add_executable(useHello main.cpp)

target_link_libraries(useHello HelloSlam)
