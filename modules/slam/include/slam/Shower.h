#ifndef SLAM_SHOWER_H
#define SLAM_SHOWER_H

#include <memory>
#include <mutex>

#include "slam/Map.h"
#include "slam/Sensors.h"
namespace slam {
class Shower {
  public:
    Shower(std::shared_ptr<GlobalVoxelMap> map_ptr, std::shared_ptr<Sensors> sensors_ptr);
    void GetGlobalMap(std::vector<Eigen::Vector3d> &pts, std::vector<Eigen::Vector3d> &colors);
    void GetTrajectory(std::vector<Eigen::Vector3f> &traj);

    void GetFrameMap(int idx, Eigen::Vector3d color, std::vector<Eigen::Vector3d> &pts, std::vector<Eigen::Vector3d> &colors);
  private:
    std::mutex mtx_;
    std::shared_ptr<GlobalVoxelMap> map_ptr_;
    std::shared_ptr<Sensors> sensors_ptr_;
}; // class Shower
} // namespace slam
#endif // SLAM_SHOWER_H