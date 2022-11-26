add_library(myslam STATIC
        data_structure/frame.cc
        data_structure/map_point.cc
        data_structure/map.cc
        data_structure/camera.cc
        utils/config.cc
        data_structure/feature.cc
        frontend/frontend.cc
        backend/backend.cc
        viewer/viewer.cc
        visual_odometry.cc
        data_structure/dataset.cc)

target_link_libraries(myslam
        ${THIRD_PARTY_LIBS})


target_link_libraries(myslam fmt::fmt)
