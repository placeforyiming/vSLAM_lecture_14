
#include <myslam/data_structure/feature.hh>
#include <myslam/data_structure/frame.hh>
#include <myslam/data_structure/map_point.hh>
namespace myslam {
Feature::Feature(std::shared_ptr<Frame> frame, const cv::KeyPoint &kp)
        : frame_(frame), position_(kp) {}
}