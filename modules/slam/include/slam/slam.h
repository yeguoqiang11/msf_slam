// slam interface.
// Copyright 2021 Yeguoqiang Co.Ltd. All rights reserved.

#ifndef MODULES_SLAM_INCLUDE_SLAM_SLAM_H_
#define MODULES_SLAM_INCLUDE_SLAM_SLAM_H_

#include <map>
#include <memory>
#include "Eigen/Dense"
#include "slam/graph.h"
#include "slam/map_data.h"
#include "slam/sensor_data.h"
#include "sophus/se3.hpp"

namespace dmslam {

using SE3d  = Sophus::SE3<double>;
using Cov3d = Eigen::Matrix<double, 6, 6>;

struct PoseWithCovariance {
  SE3d                        pose;
  Eigen::Matrix<double, 6, 6> cov;
};

class BoundingVolume {};

class Result {
 public:
  using Ptr = std::shared_ptr<Result>;
  int64_t     timestamp() const { return timestamp_; }
  virtual int status() const = 0;
  int         trajectory() const { return trajectory_; }
  int         node() const { return node_; }
  SE3d        node_pose() const { return node_pose_; }
  SE3d        odom2map() const { return odom2map_; }
  Cov3d       covariance() const { return cov_; }

 protected:
  int64_t timestamp_;
  int     trajectory_;
  int     node_;
  SE3d    node_pose_;
  SE3d    odom2map_;
  Cov3d   cov_ = Cov3d::Identity();
};

class Mapping {
 public:
  virtual Result::Ptr Process(const SensorData::ConstPtr& data,
                              const PoseWithCovariance&   pos_cov) = 0;
  virtual int         StartNewTrajectory()                       = 0;
  virtual void        EnterMappingMode()                         = 0;

  /**
   * 进入重定位模式。
   * @param trajectory_id 指定重定位到哪个trajectory.
   *                      若为-1, 则在全部trajectory上重定位.
   * @param BoundingVolume 先验范围, 如果范围特别小，直接认为重定位成功
   *                       例如机器在充电座上重启
   */
  virtual void EnterLocalizationMode(int trajectory_id, const BoundingVolume* bv) = 0;

  /**
   * 是否在正常运行时通过与其他trajectory闭环合并trajectory
   */
  virtual void EnableAutoMerge() { auto_merge_ = true; }
  virtual void DisableAutoMerge() { auto_merge_ = false; }

  // 删除地图的操作需要调用SLAM提供的接口
  // 因为可能会改变SLAM当前状态
  // 其他的查询操作则可以通过const接口进行
  virtual void           DeleteTrajectory(int trajectory_id)                  = 0;
  virtual void           LoadMap(int trajectory_id, const BoundingVolume* bv) = 0;
  virtual const MapData* MapData() const                                      = 0;

  virtual bool                    Localization() const         = 0;
  virtual std::map<int, NodeInfo> GetCurrentTrajectory() const = 0;

 protected:
  bool auto_merge_ = true;
};

}  // namespace dmslam
#endif  // MODULES_SLAM_INCLUDE_SLAM_SLAM_H_
