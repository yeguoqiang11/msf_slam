/**
 * @file DepthMapping.h
 * @author Guoqiang Ye (yegq@Yeguoqiang.tech)
 * @brief 
 * @version 0.1
 * @date 2022-01-18
 * 
 * @copyright Copyright (c) 2022 Yeguoqiang Technology Co.Ltd. All rights reserved
 * 
 */
#ifndef MODULES_SLAM_H
#define MODULES_SLAM_H
#include <iostream>
#include <memory>

#include "utils/DMmath.h"
#include "utils/logs.h"
#include "slam/Map.h"
#include "slam/PoseGraph.h"
#include "slam/VoxelMap.h"

namespace slam {
class VoxelMapping {
  public:
    VoxelMapping(std::shared_ptr<GlobalVoxelMap> map_ptr, std::shared_ptr<Sensors> sensors_ptr);
    void InsertKeyFrame(std::shared_ptr<Frame> frame_ptr);
    void FusingFrame(std::shared_ptr<Frame> frame_ptr);
    void KeyFrameOptimization(std::shared_ptr<Frame> frame_ptr);
    void SurfaceBasedOptimization(std::shared_ptr<Frame> frame_ptr);
    int DistributionCost(std::shared_ptr<Frame> frame_ptr, Eigen::Matrix<double, 6, 6> &H, Eigen::Matrix<double, 6, 1> &b, double &cost,
                          const Eigen::Matrix3d &Rbw, const Eigen::Vector3d &tbw);
    int SurfaceCost(std::shared_ptr<Frame> frame_ptr, Eigen::Matrix<double, 9, 9> &H, Eigen::Matrix<double, 9, 1> &b, double &cost,
                    const Eigen::Matrix3d &Rbw, const Eigen::Vector3d &tbw, double info_mat);
    void GroundConstraint(std::shared_ptr<Frame> frame_ptr, Eigen::Matrix<double, 9, 9> &H, Eigen::Matrix<double, 9, 1> &b, double &cost,
                          const Eigen::Matrix3d &Rbw, const Eigen::Vector3d &tbw, Eigen::Matrix<double, 4, 4> &info_mat);
    void GyroCost(std::shared_ptr<Frame> frame_ptr, Eigen::Matrix<double, 9, 9> &H, Eigen::Matrix<double, 9, 1> &b, double &cost,
                  const Eigen::Matrix3d &Rbw, const Eigen::Matrix3d &last_Rbw, const Eigen::Vector3d &bg,
                  Eigen::Matrix<double, 6, 6> &info_mat);
    void WheelCost(std::shared_ptr<Frame> frame_ptr, Eigen::Matrix<double, 9, 9> &H, Eigen::Matrix<double, 9, 1> &b, double &cost,
                   const Eigen::Matrix3d &Rbw, const Eigen::Vector3d &tbw, const Eigen::Matrix3d &last_Rbw,
                   const Eigen::Vector3d &last_tbw, Eigen::Matrix<double, 3, 3> &info_mat);

    void GetMatchedPoints(std::vector<Eigen::Vector3d> &pts, std::vector<Eigen::Vector3d> &colors);
    void GetLoopMatchedPoints(std::vector<Eigen::Vector3d> &pts, std::vector<Eigen::Vector3d> &colors);
    void LoopOptimization(std::shared_ptr<Frame> frame_ptr);
    int LoopDetection(std::shared_ptr<Frame> frame_ptr);
    bool LoopTransformationCal(std::shared_ptr<Frame> frame_ptr, int idx, Eigen::Matrix3d &Rbjbi, Eigen::Vector3d &tbjbi);
    bool LoopPoseGraph(std::shared_ptr<Frame> frame_ptr, int idx, const Eigen::Matrix3d &Rbjbi, const Eigen::Vector3d &tbjbi);
    void LoopPoseCost(const std::vector<Eigen::Matrix3d> &Rs, const std::vector<Eigen::Vector3d> &ts, const std::vector<Eigen::Matrix3d> &dRs,
                      const std::vector<Eigen::Vector3d> &dts, const Eigen::Matrix3d &Rbjbi, const Eigen::Vector3d &tbjbi, Eigen::MatrixXd &H,
                      Eigen::VectorXd &b, double &cost);
    int FramesCost(std::shared_ptr<Frame> frame_ptri, std::shared_ptr<Frame> frame_ptrj, const Eigen::Matrix3d &Rbjbi, const Eigen::Vector3d &tbjbi,
                   Eigen::Matrix<double, 6, 6> &H, Eigen::Matrix<double, 6, 1> &b, double &cost, double info_mat);
    void FrameGroundCost(std::shared_ptr<Frame> frame_ptri, std::shared_ptr<Frame> frame_ptrj, const Eigen::Matrix3d &Rbjbi, const Eigen::Vector3d &tbjbi,
                         Eigen::Matrix<double, 6, 6> &H, Eigen::Matrix<double, 6, 1> &b, double &cost, double info_mat);
    void SetGlobalGround(std::shared_ptr<GroundPlanes> ground_ptr);
    void PoseGraphOptimization(std::shared_ptr<Frame> frame_ptr);
    void StateUpdate(std::shared_ptr<Frame> frame_ptr);
    void GetFramesPtis(std::vector<Eigen::Vector3d> &pts, std::vector<Eigen::Vector3d> &colors);
    void GetFramesPtjs(std::vector<Eigen::Vector3d> &pts, std::vector<Eigen::Vector3d> &colors);
  private:
    std::shared_ptr<GlobalVoxelMap> map_ptr_;
    double damping_value_;
    double huber_thres_, GM_rho_;
    int pcd_num_thres_;
    double surface_ratio_thres_;
    std::shared_ptr<GroundPlanes> grounds_ptr_;
    std::shared_ptr<Sensors> sensor_ptr_;
    std::shared_ptr<PoseGraph> posegraph_ptr_;

    std::vector<Eigen::Vector3d> matched_pts_, origin_pts_;
    std::vector<Eigen::Vector3d> loop_matched_pts_, loop_origin_pts_;
    int last_loop_idx_;
};
} // namespace slam
#endif // MODULES_SLAM_H