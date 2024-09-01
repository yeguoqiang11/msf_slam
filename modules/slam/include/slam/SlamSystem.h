/**
 * @file SlamSystem.h
 * @author Guoqiang Ye (yegq@Yeguoqiang.tech)
 * @brief 
 * @version 0.1
 * @date 2022-01-18
 * 
 * @copyright Copyright (c) 2022 Yeguoqiang Technology Co.Ltd. All rights reserved
 * 
 */
#ifndef SRC_SLAMSYSTEM_H
#define SRC_SLAMSYSTEM_H
#include <fstream>
#include <iostream>
#include <string>

#include <opencv2/opencv.hpp>
#include "utils/json.hpp"

#include "slam/Attitude.h"
#include "slam/DepthMapping.h"
#include "slam/Imu.h"
#include "slam/ImuWheel.h"
#include "slam/KLOdometry.h"
#include "slam/Map.h"
#include "slam/PCLoopClosure.h"
#include "slam/Sensors.h"
#include "slam/Shower.h"
#include "slam/WheelModel.h"

namespace slam {
struct DirectObject {
  DirectObject() { Rbw.setIdentity(); tbw.setZero(); }
  cv::Mat last_img;
  Eigen::Matrix3d Rbw;
  Eigen::Vector3d tbw;
  Eigen::Matrix3d K, K_inv;
  double timestamp;
};

// slam body coordinate system: right hand coordinate:
// x-right axis, y-down axis, z-forward axis from view of robot tail
class SlamSystem {
  public:
    SlamSystem(const std::string &config, bool is_feature_based = true);

    struct SensorsData {
      void Initialization() { images.clear(); imus.clear(); wheels.clear(); points.clear(); }
      std::vector<cv::Mat> images; // depth images
      std::vector<Eigen::Matrix<double, 1, 7>> imus; // [timestamp, acc, gyro]
      std::vector<Eigen::Matrix<double, 1, 3>> wheels; // [left rotation speed, right rotation speed]
      std::vector<std::vector<Eigen::Vector3d>> points;
      double timestamp;
    };

    void LoadData(SensorsData &data);

    void NdtBasedMethod(SensorsData &data);

    void DepthImageBasedMethod(const std::vector<cv::Mat> &images, std::vector<Eigen::Matrix<double, 1, 7>> imus,
                               const std::vector<double> &timestamps);

    int DepthCostfunction(const cv::Mat &image0, const cv::Mat &image1, const Eigen::Matrix3d R10, const Eigen::Vector3d &t10,
                                   Eigen::Matrix<double, 6, 6> &H, Eigen::Matrix<double, 6, 1> &b, double &cost);
    
    void AddPointCloud2Frame(const std::vector<cv::Mat> &images, double timestamp, std::shared_ptr<Frame> frame_ptr);

    void AddPointCloud2Frame(const std::vector<std::vector<Eigen::Vector3d>> &points, std::shared_ptr<Frame> frame_ptr);

    std::shared_ptr<Frame> CreateFrame(SensorsData &data);

    std::shared_ptr<GyroWheelModelPreintegration> GyroWheelObject(SensorsData &data);

    bool IsKeyFrame();

    // for show
    void GetDepthFeaturePoints(std::vector<Eigen::Vector3d> &points, std::vector<Eigen::Vector3d> &colors);

    void GetLocalFeaturesPoints(std::vector<Eigen::Vector3d> &points, std::vector<Eigen::Vector3d> &colors);

    void GetMatchedFeaturePoints(std::vector<Eigen::Vector3d> &points, std::vector<Eigen::Vector3d> &colors);

    // void GetCurrentPose(Eigen::Matrix3d &Rbw, Eigen::Vector3d &tbw);

    void GetGlobalMapoints(std::vector<Eigen::Vector3d> &points, std::vector<Eigen::Vector3d> &colors);

    void GetLoopPoints(std::vector<Eigen::Vector3d> &pts, std::vector<Eigen::Vector3d> &colors);

    void GetBackendMatchedPoints(std::vector<Eigen::Vector3d> &points, std::vector<Eigen::Vector3d> &colors);

    void GetMatchedDirection(std::vector<Eigen::Vector3d> &points, std::vector<Eigen::Vector3d> &dirs);

    void GetGroundPoints(std::vector<Eigen::Vector3d> &points, std::vector<Eigen::Vector3d> &colors);

    void GetFramesPtis(std::vector<Eigen::Vector3d> &pts, std::vector<Eigen::Vector3d> &colors);
  
    void GetFramesPtjs(std::vector<Eigen::Vector3d> &pts, std::vector<Eigen::Vector3d> &colors);


    void StateUpdate(std::shared_ptr<Frame> frame_ptr);

    std::shared_ptr<Shower> ShowerPtr() { return shower_ptr_; }

    const utils::Transform3d &CurrentPose() { return Tbw_; }
  private:
    std::shared_ptr<GlobalVoxelMap> voxelmap_ptr_;
    std::shared_ptr<KLDivergenceOdometry> KL_odom_ptr_;
    std::shared_ptr<VoxelMapping> voxelmapping_ptr_;
    std::shared_ptr<Sensors> sensors_ptr_;
    std::shared_ptr<VoxelMapLoopClosure> pcloopclosure_ptr_;
    std::shared_ptr<MEKFAttitude> attitude_ptr_;
    std::shared_ptr<Shower> shower_ptr_;
    Eigen::Vector3d Vw_;
    utils::Transform3d Tbw_;
    Eigen::Vector3d ba_, bg_;
    double voxelgrid_size_;
    double last_timestamp_, timestamp_;
    bool is_feature_based_;

    // descriptors
    std::vector<cv::Mat> images_;

    // current pts for show
    std::vector<Eigen::Vector3d> current_points_;

    // direct method object
    DirectObject direct_;

    // ground extractor
    std::shared_ptr<GroundExtraction> ground_ptr_;
    std::shared_ptr<GroundPlanes> groundplanes_ptr_;
    
    // keyframe flag
    bool is_keyframe_ = false;
};

} // namespace slam
#endif // SRC_SLAMSYSTEM_H
