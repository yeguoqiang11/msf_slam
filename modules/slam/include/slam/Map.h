/**
 * @file Map.h
 * @author Guoqiang Ye (yegq@dreame.tech)
 * @brief 
 * @version 0.1
 * @date 2022-01-18
 * 
 * @copyright Copyright (c) 2022 Dreame Technology Co.Ltd. All rights reserved
 * 
 */
#ifndef SRC_MAP_H
#define SRC_MAP_H
#include <ctime>
#include <cstdlib>

#include <eigen3/Eigen/Core>

#include "slam/ContourMap.h"
#include "slam/Imu.h"
#include "slam/ImuWheel.h"
#include "slam/VoxelMap.h"
#include "slam/WheelModel.h"
#include "utils/DMmath.h"
#include "utils/json.hpp"

namespace slam {
class GroundInfo {
  public:
    GroundInfo(){ center_.setZero(); normal_vector_.setZero(); N_ = 0; plane_matched_thres_ = 0.23; }
    GroundInfo(const Eigen::Vector3d &pt, const Eigen::Vector3d &nv, int N): center_(pt), normal_vector_(nv), N_(N) { plane_matched_thres_ = 0.23; }
    GroundInfo(std::shared_ptr<GroundInfo> &ground_ptr);
    GroundInfo(std::shared_ptr<GroundInfo> &ground_ptr, const Eigen::Matrix3d &Rbw, const Eigen::Vector3d &tbw);
    void Transform(const Eigen::Matrix3d &Rbw, const Eigen::Vector3d &tbw);

    void SetInfo(const Eigen::Vector3d &pt, const Eigen::Vector3d &nv, int N);
    bool GroundUpdate(const Eigen::Vector3d &center, const Eigen::Vector3d &nv, int N);
    bool GroundUpdate(const GroundInfo &gi, const Eigen::Matrix3d &Rbw, const Eigen::Vector3d &tbw);
    bool GroundUpdate(const std::shared_ptr<GroundInfo> &gi, const Eigen::Matrix3d &Rbw, const Eigen::Vector3d &tbw);

    Eigen::Vector3d Center() { return center_; }
    Eigen::Vector3d NormalVector() { return normal_vector_; }
    int PointNum() { return N_; }
  private:
    Eigen::Vector3d center_, normal_vector_;
    int N_;
    double plane_matched_thres_;
};

class PointCloudFrame {
  public:
    PointCloudFrame(double voxelgrid_size = 0.1);
    void AddPoint(const Eigen::Vector3d &pt) {voxelgrid_.AddPoint(pt); }
    void AddPointCloud(const std::vector<Eigen::Vector3d> &pointcloud) { voxelgrid_.AddPointCloud(pointcloud); }
    void Reset() { voxelgrid_.Initialize(); }
    void SetVoxelGridSize(const double &voxelgrid_size) { voxelgrid_.SetGridSize(voxelgrid_size); }
    VoxelGrid &VoxelGridObject() { return voxelgrid_; }
    void SetGround(std::shared_ptr<GroundInfo> ground) { ground_ptr_ = std::move(ground); }
    std::shared_ptr<GroundInfo> GroundPtr() { return ground_ptr_; }
  private:
    VoxelGrid voxelgrid_;
    std::shared_ptr<GroundInfo> ground_ptr_;
};

class Frame {
  public:
    Frame(double voxelgrid_size = 0.1, std::shared_ptr<Imu> imu_ptr = nullptr,
          std::shared_ptr<EulerWMPreintegration> wheel_ptr = nullptr);
    PointCloudFrame &PointCloudFrameRef() { return pointcloud_frame_; }
    void GetPose(Eigen::Matrix3d &Rbw, Eigen::Vector3d &tbw) { Rbw = Rbw_; tbw = tbw_; }
    void GetPose(utils::Transform3d &Tbw) { Tbw = Tbw_; }
    void SetTimeStamp(double timestamp) { timestamp_ = timestamp; }
    double TimeStamp() const { return timestamp_; }
    void SetImuSharedPtr(std::shared_ptr<Imu> imu_ptr);
    void SetWheelSharedPtr(std::shared_ptr<EulerWMPreintegration> wheel_ptr) { wheel_ptr_ = std::move(wheel_ptr); }
    void SetGyroWheelSharedPtr(std::shared_ptr<GyroWheelModelPreintegration> obj_ptr) { gyro_wheel_ptr_ = std::move(obj_ptr); }
    std::shared_ptr<Imu> ImuSharedPtr() { return imu_ptr_; }
    std::shared_ptr<EulerWMPreintegration> WheelSharedPtr() { return wheel_ptr_; }
    std::shared_ptr<GyroWheelModelPreintegration> GyroWheelSharedPtr() { return gyro_wheel_ptr_; }
    void SetPose(const Eigen::Matrix3d &Rbw, const Eigen::Vector3d &tbw) { Rbw_ = Rbw; tbw_ = tbw; }
    void SetPose(const utils::Transform3d &Tbw) { Tbw_ = Tbw; }
    // const Descriptor &GlobalDescriptor() const { return des_; }
    void SetBga(const Eigen::Vector3d &ba, const Eigen::Vector3d &bg) { ba_ = ba; bg_ = bg; }
    void SetBg(const Eigen::Vector3d &bg) { bg_ = bg; }
    void SetVelocity(const Eigen::Vector3d &Vw) { Vw_ = Vw; }
    Eigen::Vector3d Ba() { return ba_; }
    Eigen::Vector3d Bg() { return bg_; }
    Eigen::Vector3d Vw() { return Vw_; }
    void SetKeyFrame(const bool &flag) { is_keyframe_ = flag; }
    bool IsKeyFrame() { return is_keyframe_; }
    void ImuRePreintegration();
    void WheelRePreintegration();
    void GyroWheelRePreintegration();
    void SetFrameId(uint32_t frame_id) { frame_id_ = frame_id; }
    uint32_t FrameId() { return frame_id_; }
    const Eigen::Matrix3d &Rbw() { return Rbw_; }
    const Eigen::Vector3d &tbw() { return tbw_; }
    utils::Transform3d &Tbw() { return Tbw_; }
    const utils::Transform3d &Tbw() const { return Tbw_; }

    static uint32_t frame_count_;
  private:
    double timestamp_;
    PointCloudFrame pointcloud_frame_;
    utils::Transform3d Tbw_;
    Eigen::Matrix3d Rbw_;
    Eigen::Vector3d tbw_, Vw_;
    std::shared_ptr<Imu> imu_ptr_;
    std::shared_ptr<EulerWMPreintegration> wheel_ptr_;
    std::shared_ptr<GyroWheelModelPreintegration> gyro_wheel_ptr_;
    Eigen::Vector3d ba_, bg_;
    uint32_t frame_id_;
    // global descriptor
    // Descriptor des_;

    bool is_keyframe_;  
};

class GlobalVoxelMap {
  public:
    GlobalVoxelMap(double voxel_size = 0.3, double weight = 2.): map_(voxel_size), depth_based_weight_(weight) {}
    GlobalVoxelMap(const nlohmann::json &config, double weight = 2.): map_(config["global_voxel_size"]), depth_based_weight_(weight) {}
    void InsertKeyFrame(std::shared_ptr<Frame> frame_ptr);
    void FusingFrame(std::shared_ptr<Frame> frame_ptr);
    void LastKeyFrameRePreintegration();
    VoxelTreeMap &MapRef() { return map_; }
    bool Empty() { return map_.Empty(); }
    void GetGlobalMapPoints(std::vector<Eigen::Vector3d> &points, std::vector<Eigen::Vector3d> &colors);
    size_t FrameSize() { return keyframe_set_.size(); }
    bool FrameEmpty() { return keyframe_set_.empty(); }
    std::vector<std::shared_ptr<Frame>> &KeyFrames() { return keyframe_set_; }
    void ReMapFromKeyFrames();

  private:
    std::vector<std::shared_ptr<Frame>> keyframe_set_;
    VoxelTreeMap map_;
    ContourMapStack contour_maps_;
    double depth_based_weight_;
};

class GroundExtraction {
  public:
    GroundExtraction(const Eigen::Vector3d &ground_dir, double noise = 0.1);
    bool Extract(std::shared_ptr<Frame> frame_ptr);
    double MatchedPlaneCost(const Eigen::Matrix<double, 6, 1> &plane0, const Eigen::Matrix<double, 6, 1> &plane1);
    GroundInfo Ground() { return ground_; }
  private:
    Eigen::Vector3d ground_dir_;
    Eigen::Matrix3d ground_cov_, ground_cov_inv_;
    GroundInfo ground_;
    double weight_, dir_thres_;
    double plane_inlier_thres_;
};

class GroundPlanes {
  public:
    GroundPlanes();
    double MatchedPlaneCost(GroundInfo &gd0, GroundInfo &gd1);
    std::shared_ptr<GroundInfo> FindGroundPlane(std::shared_ptr<GroundInfo> &ground);
    void GroundUpdate(std::shared_ptr<Frame> frame_ptr);
    size_t Size() { return grounds_.size(); }
    void OutlierGroundPlaneRemovement();

    // set gravity in robot navigation coordinate
    void SetGravityInImuFrame(const Eigen::Vector3d &Gw) { Gw_ptr_ = std::make_shared<Eigen::Vector3d>(Gw); }

    // gravity in robot navigation coordinate
    std::shared_ptr<Eigen::Vector3d> GravitySharedPtr() { return Gw_ptr_; }
  private:
    std::vector<std::shared_ptr<GroundInfo>> grounds_;
    double matched_thres_;
    double matched_angle_thres, matched_dist_thres_;
    int outlier_plane_thres_;
    std::shared_ptr<Eigen::Vector3d> Gw_ptr_; // gravity in robot navigation coordinate
};
} // namespace slam
#endif // SRC_MAP_H
