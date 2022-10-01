cmake_minimum_required(VERSION 3.2)

set(CMAKE_BUILD_TYPE "Debug" )

project(Sophus)

find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})
include_directories("/usr/include/eigen3")

add_executable(useSophus useSophus/useSophus.cpp)
add_executable(plotTrajError plotTrajError/plotTrajError.cpp)

target_link_libraries(useSophus Sophus::Sophus)
target_link_libraries(plotTrajError Sophus::Sophus)

find_package(Pangolin)
include_directories(${Pangolin_INCLUDE_DIRS} )
target_link_libraries(plotTrajError ${Pangolin_LIBRARIES} )
