/**
 * @file KLOdometry.h
 * @author Guoqiang Ye (yegq@dreame.tech)
 * @brief 
 * @version 0.1
 * @date 2022-01-18
 * 
 * @copyright Copyright (c) 2022 Dreame Technology Co.Ltd. All rights reserved
 * 
 */
#ifndef MODULES_SLAM_KLODOMETRY
#define MODULES_SLAM_KLODOMETRY
#include <memory>

#include <eigen3/Eigen/Core>
#include <opencv2/opencv.hpp>

#include "slam/Imu.h"
#include "slam/Map.h"
#include "slam/Sensors.h"
#include "slam/VoxelMap.h"
#include "utils/json.hpp"
#include "utils/DMmath.h"
#include "utils/logs.h"

namespace slam {
class KLDivergenceOdometry {
  public:
    struct Parameter {
        int width, height;
        double fx, fy, cx, cy;
        double fx_inv, fy_inv;
        double huber_thres_;
        double GM_kernel_rho_;
    };

    KLDivergenceOdometry(const nlohmann::json &config, std::shared_ptr<Sensors> sensor_ptr);

    void Reset();

    void LoadFrame(std::shared_ptr<Frame> frame_ptr, const double &timestamp);

    void LocalMapUpdate(std::shared_ptr<Frame> frame_ptr);

    void AddFrame2LocalMap(std::shared_ptr<Frame> frame_ptr, const utils::Transform3d &Tbw);

    void SurfaceBasedOptimization(std::shared_ptr<Frame> frame_ptr);

    void GroundConstraint(std::shared_ptr<Frame> frame_ptr, Eigen::Matrix<double, 9, 9> &H, Eigen::Matrix<double, 9, 1> &b, double &cost,
                          const utils::Transform3d &Tbl, const utils::Transform3d &Tlw, Eigen::Matrix<double, 4, 4> &info_mat);

    int DistributionsCost(std::shared_ptr<Frame> frame_ptr, Eigen::Matrix<double, 6, 6> &H, Eigen::Matrix<double, 6, 1> &b, double &cost,
                          const Eigen::Matrix3d &Rbl, const Eigen::Vector3d &tbl);

    int SurfaceCost(std::shared_ptr<Frame> frame_ptr, Eigen::Matrix<double, 9, 9> &H, Eigen::Matrix<double, 9, 1> &b, double &cost,
                    const utils::Transform3d &Tbl, double info_mat);
    void GyroCost(std::shared_ptr<Frame> frame_ptr, Eigen::Matrix<double, 9, 9> &H, Eigen::Matrix<double, 9, 1> &b, double &cost,
                  const Eigen::Matrix3d &Rbl, const Eigen::Matrix3d &last_Rbl, const Eigen::Vector3d &bg, Eigen::Matrix<double, 6, 6> &info_mat);

    void WheelCost(std::shared_ptr<Frame> frame_ptr, Eigen::Matrix<double, 9, 9> &H, Eigen::Matrix<double, 9, 1> &b, double &cost,
                   const utils::Transform3d &Tbl, const utils::Transform3d &last_Tbl, Eigen::Matrix<double, 3, 3> &info_mat);

    void GyroWheelCost(std::shared_ptr<Frame> frame_ptr, Eigen::Matrix<double, 9, 9> &H, Eigen::Matrix<double, 9, 1> &b, double &cost,
                       const utils::Transform3d &Tbl, const utils::Transform3d &last_Tbl, Eigen::Matrix<double, 3, 3> &info_mat);
    
    int SurfaceShapeCost(std::shared_ptr<Frame> frame_ptr, Eigen::Matrix<double, 6, 6> &H, Eigen::Matrix<double, 6, 1> &b, double &cost,
                         const Eigen::Matrix3d &Rbl, const Eigen::Vector3d &tbl);

    void MotionConstraintCost(const Eigen::Matrix3d &Rbl, const Eigen::Vector3d &dtl, Eigen::Matrix<double, 6, 6> &H,
                              Eigen::Matrix<double, 6, 1> &b, double &cost, double w = 1.);

    void GetCurrentFramePoints(std::vector<Eigen::Vector3d> &pts, std::vector<Eigen::Vector3d> &colors);

    void GetLocalMapPoints(std::vector<Eigen::Vector3d> &pts, std::vector<Eigen::Vector3d> &colors);

    void GetMatchedPoints(std::vector<Eigen::Vector3d> &points, std::vector<Eigen::Vector3d> &colors);

    void GetMatchedDirection(std::vector<Eigen::Vector3d> &points, std::vector<Eigen::Vector3d> &dir);

    void GetGroundPoints(std::vector<Eigen::Vector3d> &points, std::vector<Eigen::Vector3d> &colors);

    bool IsKeyFrame() { return is_keyframe_; }

    void RectifyLocalMapPose(const utils::Transform3d &Tbw);

    void SetGroundPtr(std::shared_ptr<GroundPlanes> ground_ptr) { grounds_ptr_ = std::move(ground_ptr); }
  private:
    double timestamp_;
    double voxel_size_;
    Parameter param_;
    std::shared_ptr<LocalMap> localmap_ptr_;
    std::shared_ptr<Sensors> sensor_ptr_;
    Eigen::Vector3d Vw_;
    utils::Transform3d Tbw_, last_Tbw_, last_keyTbw_;
    double min_depth_, max_depth_;
    double damping_val_;
    std::vector<std::shared_ptr<Frame>> frames_;
    bool is_keyframe_;
    double outlier_thres_;
    int voxelpoint_num_thres_;
    double surface_ratio_thres_;
    double plane_shape_weight_;

    // show pt;
    std::vector<Eigen::Vector3d> origin_pts_, matched_pts_, origin_pts1_, matched_pts1_;
    std::vector<Eigen::Vector3d> matched_dir_;
    std::vector<Eigen::Vector3d> current_pts_;
    Eigen::Matrix<double, 6, 1> origin_ground_, matched_ground_, origin_ground1_, matched_ground1_;

    // imu object
    std::shared_ptr<GroundPlanes> grounds_ptr_;
    bool is_initialized_;
};
} // namespace slam
#endif // MODULES_SLAM_KLODOMETRY