/**
 * @file WheelModel.h
 * @author Guoqiang Ye (yegq@Yeguoqiang.tech)
 * @brief differential drive model of two wheel car with imu
 * @version 0.1
 * @date 2022-01-17
 * 
 * @copyright Copyright (c) 2022 Yeguoqiang Technology Co.Ltd. All rights reserved
 * 
 */

#ifndef MODELS_IMUWHEELMODEL_H
#define MODELS_IMUWHEELMODEL_H
#include <iostream>
#include <memory>
#include <vector>

#include <eigen3/Eigen/Core>

#include <utils/DMmath.h>

namespace slam {
// coordinate system
// standpoint: robot tail => x-forward, y-left, z-up
class GyroWheelModelPreintegration {
  public:
    struct WheelData {
        WheelData(const std::vector<Eigen::Vector2d> &gs, const Eigen::Vector2d &wh);
        std::vector<Eigen::Vector2d> gyros;
        Eigen::Vector2d wheel;
    };
    // noise = [nl, nr, ng]
    GyroWheelModelPreintegration(const double &rl, const double &rr, const double &bg, const Eigen::Vector3d &noise);
    // gyros = set of [t, gz], wheels = [t, w_left, w_right]
    void InsertDatas(const std::vector<Eigen::RowVector2d> &gyros, const std::vector<Eigen::RowVector3d> &wheels);
    void InsertData(const std::vector<Eigen::Vector2d> &gyros, const Eigen::Vector2d &wheel);
    void Initialization();
    void RePreintegration();
    void PoseCalFromLastPose(const Eigen::Matrix3d &last_Rbw, const Eigen::Vector3d &last_tbw,
                             const Eigen::Matrix3d &Rob, const Eigen::Vector3d &tob, Eigen::Matrix3d &Rbw, Eigen::Vector3d &tbw);
    void PoseCalFromLastPose(const utils::Transform3d &last_Tbw, const utils::Transform3d &Tob,
                             utils::Transform3d &Tbw);
    void FusingGyrWheelWithoutPreintegration(std::shared_ptr<GyroWheelModelPreintegration> &obj_ptr);
    bool Evaluate(const double *Pj, const double *Pi, double *residual, double **jacobians) const;

    Eigen::Vector3d Alpha() { return alphaij_; }
    Eigen::Matrix3d Cov() { return cov_; }
    double Dt() { return dt_; }
  private:
    double rl_, rr_, bg_, thetaij_, dt_;
    Eigen::Matrix3d noise_;
    Eigen::Vector3d alphaij_;
    Eigen::Matrix2d Rji_;
    Eigen::Matrix3d cov_;
    std::vector<WheelData> wheel_data_;
};

struct GyroWheel3DFactor {
    bool Evaluate(const utils::Transform3d &Tbiw, const utils::Transform3d &Tbjw, const utils::Transform3d &Tob,
                  std::shared_ptr<GyroWheelModelPreintegration> gyrowheel_ptr);
    Eigen::Vector3d residual;
    Eigen::Matrix<double, 3, 6> Jj;
};

struct GyroWheel3Dfactorij {
    bool Evaluate(const Eigen::Matrix3d &Rbiw, const Eigen::Vector3d &tbiw, const Eigen::Matrix3d &Rbjw, const Eigen::Vector3d &tbjw,
                  const Eigen::Matrix3d &Rob, Eigen::Vector3d &tob, std::shared_ptr<GyroWheelModelPreintegration> gyrowheel_ptr);
    bool Evaluate(const utils::Transform3d &Tbiw, const utils::Transform3d &Tbjw, const utils::Transform3d &Tob,
                  std::shared_ptr<GyroWheelModelPreintegration> gyrowheel_ptr);
    Eigen::Vector3d residual;
    Eigen::Matrix<double, 3, 6> Ji, Jj;
};
} // namespace slam
#endif // MODELS_IMUWHEELMODEL_H