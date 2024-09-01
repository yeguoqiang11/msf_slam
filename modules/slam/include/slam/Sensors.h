/**
 * @file Sensors.h
 * @author Guoqiang Ye (yegq@Yeguoqiang.tech)
 * @brief 
 * @version 0.1
 * @date 2022-01-18
 * 
 * @copyright Copyright (c) 2022 Yeguoqiang Technology Co.Ltd. All rights reserved
 * 
 */
#ifndef MODULES_SLAM_SENSOR_H
#define MODULES_SLAM_SENSOR_H
#include <iostream>
#include <vector>

#include <eigen3/Eigen/Core>
#include <opencv2/opencv.hpp>

#include "utils/DMmath.h"
#include "utils/json.hpp"
#include "utils/logs.h"

// coordinate system description
// body coordinate: right hand coordinate
// x-axis: right; y-axis: down; z-axis: forward
namespace slam {
class PinHoleCam {
  public:
  /**
   * @brief Construct a new Pin Hole Cam object
   * 
   * @param intrinsic [fx, fy, cx, cy]
   * @param distort [k1, k2, p1, p2, k3]
   */
    PinHoleCam(const Eigen::Vector4d &intrinsic, int width, int height, Eigen::Matrix<double, 5, 1> distort =
        Eigen::Matrix<double, 5, 1>::Zero());
    cv::Mat Undistort(cv::Mat image);
    void UndistortPoint2fs(const std::vector<cv::Point2f> &points, std::vector<cv::Point2f> &undistorted);

    /**
     * @brief intrinsic of zooming image with scale
     * 
     * @param intrinsic origin intrinsic
     * @param out result intrinsic after zooming image with scale of s
     * @param s zooming scale at image
     */
    static void IntrinsicZooming(const Eigen::Vector4d &intrinsic, Eigen::Vector4d &out, double s);
    static void DistortCoeffZooming(const Eigen::Matrix<double, 5, 1> &dist_coef, Eigen::Matrix<double, 5, 1> &out, double s);
    cv::Mat CamIntrinsicMatrix() { return K_; }
    cv::Mat CamDistortCoef() { return D_; }
  private:
    Eigen::Matrix<double, 5, 1> distort_; // [k1, k2, p1, p2, k3, k4,...]
    Eigen::Vector4d intrinsic_; // [fx, fy, cx, cy]
    cv::Mat K_, D_, mapx_, mapy_;
    double width_, height_;
};

struct TofCamera {
    // point cloud: left hand coordinate: x: right; y: down; z: forward
    inline Eigen::Vector3d Depth2BodyPt(int r, int c, double depth) const {
        Eigen::Vector3d out;
        out(0) = (c - cx) * depth * fx_inv;
        out(1) = (r - cy) * depth * fy_inv;
        out(2) = depth;
        return Rcb * out + tcb;
    }
    static void IntrinsicZooming(const Eigen::Vector4d &intrinsic, Eigen::Vector4d &out, double s);
    static void DistortCoeffZooming(const Eigen::Matrix<double, 5, 1> &dist_coef, Eigen::Matrix<double, 5, 1> &out, double s);
    bool IsDepthValid(const double &d) { return (d < max_depth && d > min_depth); }
    bool IsInside(const int &r, const int &c) { return (r > 2 && r < height - 2 && c > 2 && c < width - 2); }
    Eigen::Matrix3d Rcb;
    Eigen::Vector3d tcb;
    utils::Transform3d Tcb;
    double fx, fy, cx, cy;
    double fx_inv, fy_inv;
    double width, height;
    double max_depth, min_depth;
    Eigen::Vector3d noise;
};
struct ImuSensor {
    Eigen::Matrix3d Rib;
    Eigen::Vector3d tib;
    utils::Transform3d Tib;
    Eigen::Vector3d ba, bg, na, ng, nbg, nba;
};
struct DiffWheelSensor {
    double distance;
    double radius;
    Eigen::Matrix3d Rob;
    Eigen::Vector3d tob;
    utils::Transform3d Tob;
    double noise_left, noise_right;
};

struct Sensors {
    Sensors(const nlohmann::json &config);
    Sensors() {}
    void SensorsConfigPrint();
    std::vector<TofCamera> tofcams;
    ImuSensor imu;
    DiffWheelSensor diff_wheel;
};
} // namespace slam
#endif // MODULES_SLAM_SENSOR_H