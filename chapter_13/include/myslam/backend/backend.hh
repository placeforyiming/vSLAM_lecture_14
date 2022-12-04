#pragma once
#include <myslam/common_include.hh>
#include <myslam/data_structure/frame.hh>
#include <myslam/data_structure/map.hh>

namespace myslam {

class Map;

class Frame;

class Camera;
/**
 * 后端
 * 有单独优化线程，在Map更新时启动优化
 * Map更新由前端触发
 */ 
class Backend {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Backend> Ptr;

    /// 构造函数中启动优化线程并挂起
    Backend();

    // 设置左右目的相机，用于获得内外参
    void SetCameras(std::shared_ptr<Camera> left, std::shared_ptr<Camera> right) {
        cam_left_ = left;
        cam_right_ = right;
    }

    /// 设置地图
    void SetMap(std::shared_ptr<Map> map) { map_ = map; }

    /// 触发地图更新，启动优化
    void UpdateMap();

    /// 关闭后端线程
    void Stop();

   private:
    /// 后端线程
    void BackendLoop();

    /// 对给定关键帧和路标点进行优化
    void Optimize(Map::KeyframesType& keyframes, Map::LandmarksType& landmarks);

    std::shared_ptr<Map> map_;
    std::thread backend_thread_;
    std::mutex data_mutex_;

    std::condition_variable map_update_;
    std::atomic<bool> backend_running_;

    std::shared_ptr<Camera> cam_left_ = nullptr, cam_right_ = nullptr;
};

}  // namespace myslam
