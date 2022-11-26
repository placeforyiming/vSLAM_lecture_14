#pragma once

#include <myslam/common_include.hh>

namespace myslam {
    
class Feature;

class Frame {
 public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Frame> Ptr;

    unsigned long id_ = 0;           
    unsigned long keyframe_id_ = 0;  
    bool is_keyframe_ = false;       
    double time_stamp_;              
    SE3 pose_;                       
    std::mutex pose_mutex_;          
    cv::Mat left_img_, right_img_;   

    // extracted features in left image
    std::vector<std::shared_ptr<Feature>> features_left_;
    // corresponding features in right image, set to nullptr if no corresponding
    std::vector<std::shared_ptr<Feature>> features_right_;

 public:  // data members
    Frame() {}

    Frame(long id, double time_stamp, const SE3 &pose, const cv::Mat &left,
          const cv::Mat &right);

    // set and get pose, thread safe
    SE3 Pose() {
        std::unique_lock<std::mutex> lck(pose_mutex_);
        return pose_;
    }

    void SetPose(const SE3 &pose) {
        std::unique_lock<std::mutex> lck(pose_mutex_);
        pose_ = pose;
    }

    /// set key frame and assign key frame id
    void SetKeyFrame();

    /// create frame and assign frame id
    static std::shared_ptr<Frame> CreateFrame();
};
}
