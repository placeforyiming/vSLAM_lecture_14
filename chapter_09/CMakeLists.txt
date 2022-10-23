cmake_minimum_required(VERSION 3.2)
set(CMAKE_CXX_FLAGS "-std=c++11")

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(OpenCV REQUIRED)
find_package(g2o REQUIRED)
find_package(Sophus REQUIRED)
find_package(Ceres REQUIRED) 
find_package(CSparse REQUIRED)


include_directories(${Sophus_INCLUDE_DIRS})
include_directories("/usr/include/eigen3")
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${G2O_INCLUDE_DIRS})
include_directories(${CERES_INCLUDE_DIRS})
include_directories(${CSPARSE_INCLUDE_DIR})

add_executable(BACeres src/BACeres.cpp src/common.cpp)
target_link_libraries(BACeres ${OpenCV_LIBS} ${CERES_LIBRARIES})

add_executable(BAG2O src/BAG2O.cpp src/common.cpp) 
target_link_libraries(BAG2O 
    Sophus::Sophus
    ${OpenCV_LIBS} 
    g2o_core g2o_stuff g2o_csparse_extension
    cxsparse
)
