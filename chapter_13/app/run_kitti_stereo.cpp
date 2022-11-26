
#include <gflags/gflags.h>
#include <myslam/visual_odometry.hh>

DEFINE_string(config_file, "/workspace/vSLAM_lecture_14/chapter_13/config/default.yaml", "config file path");

int main(int argc, char **argv) {
    google::ParseCommandLineFlags(&argc, &argv, true);

    myslam::VisualOdometry::Ptr vo(
        new myslam::VisualOdometry(FLAGS_config_file));
    assert(vo->Init() == true);
    vo->Run();

    return 0;
}
