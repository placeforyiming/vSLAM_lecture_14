#pragma once

#include <myslam/common_include.hh>

namespace myslam {

class Frame;

class MapPoint;

/**
 * @brief map 
 * using InsertKeyframe and InsertMapPoint to insert new frame and new map point on frontend stage
 * maintain map structure and determine outlier on backend stage
 */
class Map {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Map> Ptr;
    typedef std::unordered_map<unsigned long, std::shared_ptr<MapPoint>> LandmarksType;
    typedef std::unordered_map<unsigned long, std::shared_ptr<Frame>> KeyframesType;

    Map() {}

    /// 增加一个关键帧
    void InsertKeyFrame(std::shared_ptr<Frame> frame);
    /// 增加一个地图顶点
    void InsertMapPoint(std::shared_ptr<MapPoint> map_point);

    /// 获取所有地图点
    LandmarksType GetAllMapPoints() {
        std::unique_lock<std::mutex> lck(data_mutex_);
        return landmarks_;
    }
    /// 获取所有关键帧
    KeyframesType GetAllKeyFrames() {
        std::unique_lock<std::mutex> lck(data_mutex_);
        return keyframes_;
    }

    /// 获取激活地图点
    LandmarksType GetActiveMapPoints() {
        std::unique_lock<std::mutex> lck(data_mutex_);
        return active_landmarks_;
    }

    /// 获取激活关键帧
    KeyframesType GetActiveKeyFrames() {
        std::unique_lock<std::mutex> lck(data_mutex_);
        return active_keyframes_;
    }

    /// 清理map中观测数量为零的点
    void CleanMap();

   private:
    // 将旧的关键帧置为不活跃状态
    void RemoveOldKeyframe();

    std::mutex data_mutex_;
    LandmarksType landmarks_;         // all landmarks
    LandmarksType active_landmarks_;  // active landmarks
    KeyframesType keyframes_;         // all key-frames
    KeyframesType active_keyframes_;  // all key-frames

    std::shared_ptr<Frame> current_frame_ = nullptr;

    // settings
    int num_active_keyframes_ = 7;  // 激活的关键帧数量
};
}  // namespace myslam