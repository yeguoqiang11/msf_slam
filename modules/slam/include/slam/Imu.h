/**
 * @file Imu.h
 * @author Guoqiang Ye (yegq@dreame.tech)
 * @brief 
 * @version 0.1
 * @date 2022-01-18
 * 
 * @copyright Copyright (c) 2022 Dreame Technology Co.Ltd. All rights reserved
 * 
 */
#ifndef MODULES_SLAM_IMU_H
#define MODULES_SLAM_IMU_H
#include <iostream>

#include <eigen3/Eigen/Core>

#include "utils/DMmath.h"
#include "slam/Sensors.h"

namespace slam {
// imu preintegration
class Imu {
  public:    
    Imu(std::shared_ptr<Sensors> sensor_ptr);

    Imu(const double &ba, const double &bg, const double &na, const double &ng, const double &nbg, const double &nba);

    Imu(const Eigen::Vector3d &ba, const Eigen::Vector3d &bg);

    void Initialize();

    static void GlobalParametersInitialize(std::shared_ptr<Sensors> sensor_ptr);

    void PreIntegration(const std::vector<Eigen::Matrix<double, 1, 7>> &datas);
    
    void RePreintegration();
    
    void InsertImuWithoutPreintegration(std::shared_ptr<Imu> imu_ptr);

    void StateUpdate(const Eigen::Vector3d &acc, const Eigen::Vector3d &gyr, const double &dt);

    void ImuStatePrediction(const Eigen::Matrix3d &Riw, const Eigen::Vector3d &Vi, const Eigen::Vector3d &Pi, const Eigen::Vector3d &Gravity,
                         Eigen::Matrix3d &Rjw, Eigen::Vector3d &Vj, Eigen::Vector3d &Pj);

    void ImuStatePrediction(const utils::Transform3d &Tiw, const Eigen::Vector3d &Vi, const Eigen::Vector3d &Gravity,
                             utils::Transform3d &Tjw, Eigen::Vector3d &Vj);

    void Reset(const Eigen::Vector3d &ba, const Eigen::Vector3d &bg);
    
    void SetBg(const Eigen::Vector3d &bg) { bg_ = bg; }
    void SetBa(const Eigen::Vector3d &ba) { ba_ = ba; }

    static Eigen::Matrix3d Rib_; // rotation from imu frame to body frame
    static Eigen::Vector3d tib_;
    static Eigen::Vector3d na_, ng_, nba_, nbg_;
    static Eigen::Matrix<double, 6, 6> nga_, nbga_;

    // const reference to measurement and jacobian
    const Eigen::Matrix3d &JRg() const { return dRji_dbg_; }
    const Eigen::Matrix3d &JRa() const { return dRji_dba_; }
    const Eigen::Matrix3d &Jpg() const { return dpij_dbg_; }
    const Eigen::Matrix3d &Jpa() const { return dpij_dba_; }
    const Eigen::Matrix3d &Jvg() const { return dvij_dbg_; }
    const Eigen::Matrix3d &Jva() const { return dvij_dba_; }
    const Eigen::Matrix3d &Rji() const { return Rji_; }
    const Eigen::Vector3d &Pij() const { return pij_; }
    const Eigen::Vector3d &Vij() const { return vij_; }
    const Eigen::Vector3d &Ba() const { return ba_; }
    const Eigen::Vector3d &Bg() const { return bg_; }
    const double &Dt() const { return dt_; }
    const Eigen::Matrix<double, 15, 15> &Cov() const { return cov_; }

    struct ImuData {
      ImuData(const Eigen::Vector3d &gyr0, const Eigen::Vector3d &acc0, const double &dt0)
      : gyr(gyr0), acc(acc0), dt(dt0) {}
      Eigen::Vector3d gyr, acc;
      double dt;
    };
    const std::vector<ImuData> &ImuDatas() { return imu_datas_; }
  private:
    double t1_, t0_;
    Eigen::Vector3d ba_, bg_;
    std::vector<ImuData> imu_datas_;
    Eigen::Matrix3d Rji_;
    Eigen::Vector3d vij_, pij_;
    Eigen::Matrix3d dRji_dbg_, dRji_dba_, dvij_dbg_, dvij_dba_, dpij_dbg_, dpij_dba_;
    Eigen::Matrix<double, 15, 15> cov_;
    Eigen::Matrix<double, 9, 9> A_;
    Eigen::Matrix<double, 9, 6> B_;
    double dt_;
};

// imu factor
struct ImuFactor {
    Eigen::Matrix<double, 15, 1> residual; // state = [dtheta, dv, dp, dbg, dba]
    Eigen::Matrix<double, 15, 15> Ji, Jj; // state = [dtheta, dv, dp, dbg, dba], Jacobian = dresidual / dstate
    void Evaluate(const Eigen::Matrix3d &Riw, const Eigen::Matrix3d &Rjw, const Eigen::Vector3d &Pi, const Eigen::Vector3d &Pj,
                  const Eigen::Vector3d &Vi, const Eigen::Vector3d &Vj, const Eigen::Vector3d &ba, const Eigen::Vector3d &bg,
                  const Eigen::Vector3d &baj, const Eigen::Vector3d &bgj, const Eigen::Vector3d &Gravity, const Imu &imu);
};

// imu rotation factor
struct GyroFactor {
    Eigen::Matrix<double, 6, 1> residual;
    Eigen::Matrix<double, 6, 6> Ji, Jj;
    void Evaluate(const Eigen::Matrix3d &Rbiw, const Eigen::Matrix3d &Rbjw, const Eigen::Vector3d &bgi, const Eigen::Vector3d &bgj,
                  const Eigen::Matrix3d &Rib, const Imu &imu);
    void Evaluate(const Eigen::Matrix3d &Rbiw, const Eigen::Matrix3d &Rbjw, const Eigen::Vector3d &bgi, const Eigen::Vector3d &bgj,
                  const Eigen::Matrix3d &Rib, std::shared_ptr<Imu> imu);
};
} // namespace slam
#endif // MODULES_SLAM_IMU_H