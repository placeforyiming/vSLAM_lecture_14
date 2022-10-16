cmake_minimum_required(VERSION 3.2)
set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(OpenCV REQUIRED)
find_package(g2o REQUIRED)
find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})
include_directories("/usr/include/eigen3")
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${G2O_INCLUDE_DIRS})

add_executable(orbOpenCV orb_opencv/orbOpenCV.cpp)
target_link_libraries(orbOpenCV ${OpenCV_LIBS})


add_executable(pose_estimation_2d2d pose_estimation/pose_estimation_2d2d.cpp)
target_link_libraries(pose_estimation_2d2d ${OpenCV_LIBS})

add_executable(pose_estimation_3d2d_g2o pose_estimation/pose_estimation_3d2d_g2o.cpp)
target_link_libraries(pose_estimation_3d2d_g2o
    Sophus::Sophus
    ${OpenCV_LIBS}
    g2o_core g2o_stuff)
