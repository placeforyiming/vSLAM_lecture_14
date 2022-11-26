#pragma once

#include <myslam/common_include.hh>

namespace myslam{

class Camera;

class Frame;

class Dataset {
 public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Dataset> Ptr;
    Dataset(const std::string& dataset_path);

    /// return if initialize successeful
    bool Init();

    /// create and return the next frame containing the stereo images
    std::shared_ptr<Frame> NextFrame();

    /// get camera by id
    std::shared_ptr<Camera> GetCamera(int camera_id) const {
        return cameras_.at(camera_id);
    }

 private:
    std::string dataset_path_;
    int current_image_index_ = 0;

    std::vector<Camera::Ptr> cameras_;
};
}