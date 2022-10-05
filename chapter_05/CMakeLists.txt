set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(OpenCV REQUIRED)
find_package(Pangolin)
find_package(Sophus REQUIRED)
find_package(FMT REQUIRED)

include_directories("/usr/include/eigen3")
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${Pangolin_INCLUDE_DIRS} )
include_directories(${Sophus_INCLUDE_DIRS})

add_executable(imageBasics imageBasics/imageBasics.cpp)
add_executable(imageUndistort imageBasics/imageUndistort.cpp)
add_executable(stereoVision stereoVision/stereoVision.cpp)
add_executable(jointMap rgbd/jointMap.cpp)


target_link_libraries(imageBasics ${OpenCV_LIBS})
target_link_libraries(imageUndistort ${OpenCV_LIBS})
target_link_libraries(stereoVision ${OpenCV_LIBS})
target_link_libraries(stereoVision ${Pangolin_LIBRARIES})
target_link_libraries(jointMap ${OpenCV_LIBS})
target_link_libraries(jointMap ${Pangolin_LIBRARIES})
target_link_libraries(jointMap fmt::fmt)
