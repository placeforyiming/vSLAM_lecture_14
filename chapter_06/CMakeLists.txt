cmake_minimum_required(VERSION 3.2)
set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
find_package(g2o REQUIRED)

include_directories("/usr/include/eigen3")
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${CERES_INCLUDE_DIRS})
include_directories(${G2O_INCLUDE_DIRS})

add_executable(curveFittingManual curveFitting/gaussNewton.cpp)
target_link_libraries(curveFittingManual ${OpenCV_LIBS})

add_executable(curveFittingCeres curveFittingCeres/ceresCurveFitting.cpp)
target_link_libraries(curveFittingCeres ${OpenCV_LIBS} ${CERES_LIBRARIES})

add_executable(curveFittingG2O curveFittingG2O/g2oCurveFitting.cpp)
target_link_libraries(curveFittingG2O 
    ${OpenCV_LIBS} 
    g2o_core g2o_stuff
)

