/**
 * @file WheelModel.h
 * @author Guoqiang Ye (yegq@dreame.tech)
 * @brief differential drive model of two wheel car
 * @version 0.1
 * @date 2022-01-17
 * 
 * @copyright Copyright (c) 2022 Dreame Technology Co.Ltd. All rights reserved
 * 
 */

#ifndef MODELS_WHEELMODEL_H
#define MODELS_WHEELMODEL_H
#include <iostream>
#include <memory>
#include <vector>

#include <eigen3/Eigen/Core>

#include <utils/DMmath.h>

namespace slam {
// differential wheel model
// [dx_dt; dy_dt; dtheta_dt] = [cos_theta; sin_theta; 0] * v + [0; 0; 1] * w
// v = 0.5 * (vl + vr), w = (vl  - vr) / l;
// accuracy rank: ArcIntegration, RungeKuttaintegration, EulerIntegration
class WheelModelBase {
  public:
    WheelModelBase(const double &rl, const double &rr, const double &l): r_l_(rl), r_r_(rr), l_inv_(1./l) {}
    virtual Eigen::Vector3d VelCalculation(const Eigen::Vector2d &wheel_vel) const = 0;
    virtual Eigen::Vector3d MovementCalculation(const Eigen::Vector2d &wheel_vel, double dt) const = 0;
    virtual Eigen::Vector3d EulerIntegrationFromLastPose(const Eigen::Vector3d &Pi, const Eigen::Vector3d &v, double dt) const = 0;
    virtual Eigen::Vector3d ArcIntegrationFromLastPose(const Eigen::Vector3d &Pi, const Eigen::Vector3d &v, double dt) const = 0;
    virtual Eigen::Vector3d ArcWheelFromLastPose(const Eigen::Vector3d &Pi, const Eigen::Vector2d &Wheel_vel, double dt) const = 0;
    virtual Eigen::Vector3d RungeKuttaIntegrationFromLastPose(const Eigen::Vector3d &Pi, const Eigen::Vector3d &v, double dt) const = 0;
  protected:
    double r_l_, r_r_, l_inv_;
};

class DiffWheelModel: public WheelModelBase {
  public:
    DiffWheelModel(const double &rl, const double &rr, const double &l): WheelModelBase(rl, rr, l) {}

    void SetIntrinsic(const double &rl, const double &rr, const double &l) { r_l_ = rl; r_r_ = rr; l_inv_ = 1. / l; }

    /**
     * @brief odometry velocity from two wheel velocity
     * 
     * @param wheel_vel two wheel velocity = [w_left, w_right] at body coordinate
     * @return Eigen::Vector3d odometry velocity = [vx, vy, v_theta], vx: forward velocity; vy: lateral velocity at body coordinate
     */
    Eigen::Vector3d VelCalculation(const Eigen::Vector2d &wheel_vel) const;

    Eigen::Vector3d MovementCalculation(const Eigen::Vector2d &wheel_vel, double dt) const;

    Eigen::Vector3d EulerIntegrationFromLastPose(const Eigen::Vector3d &Pi, const Eigen::Vector3d &v, double dt) const;

    Eigen::Vector3d EulerWheelFromLastPose(const Eigen::Vector3d &Pi, const Eigen::Vector2d &wheel_vel, double dt) const;

    Eigen::Vector3d ArcWheelFromLastPose(const Eigen::Vector3d &Pi, const Eigen::Vector2d &Wheel_vel, double dt) const;

    Eigen::Vector3d ArcIntegrationFromLastPose(const Eigen::Vector3d &Pi, const Eigen::Vector3d &v, double dt) const;

    Eigen::Vector3d RungeKuttaIntegrationFromLastPose(const Eigen::Vector3d &Pi, const Eigen::Vector3d &v, double dt) const;

    Eigen::Vector3d RungeKuttawheelFromLastPose(const Eigen::Vector3d &Pi, const Eigen::Vector2d &wheel_data, double dt) const;
};

class EulerDiffWheelModel {
  public:
    EulerDiffWheelModel(double radius, double distance);

    /**
     * @brief wheel odmetry calculation
     * @param wheel [left wheel speed, right wheel speed]
     * @param dt time difference
     */
    void LoadWheeldata(const Eigen::Vector2d &wheel, double dt);

    /**
     * @brief wheel odometry calculation
     * @param wheels vector [[left wheel speed, right wheel speed, timestamp]]
     */
    void LoadWheelDatas(std::vector<Eigen::Matrix<double, 1, 3>> &wheels);

    /**
     * @brief get pose
     * @return pose [x, y, theta] x-right, y-forward
     */
    Eigen::Vector3d Pose() { return pose_; }

    /**
     * @brief get 3D pose
     * @param Rbw output 3D rotation x-right, y-forward, z-up
     * @param tbw output 3D translation
     */
    void GetPose(Eigen::Matrix3d &Rbw, Eigen::Vector3d &tbw);

    double Dt() { return dt_; }
  private:
    Eigen::Vector3d pose_; // pose = [x, y, theta], x-right, y-forward
    double d_inv_, r_; // d_inv-distance inverse between two wheel, r-wheel radius
    double dt_;
};

class ArcWheelModelPreintegration: public DiffWheelModel {
  public:
    ArcWheelModelPreintegration(const double &rl, const double &rr, const double &l, double nl = 0.01, double nr = 0.01);
    void Initialization();
    void InsertWheelVel(const Eigen::Vector2d &wheel_vel, double dt);
    void Evaluate(const Eigen::Vector3d &Pj, const Eigen::Vector3d &Pi, Eigen::Vector3d &residual, Eigen::Ref<Eigen::Matrix<double, 3, 6>> jacobian) const;
    void EvaluateWithIntrinsics(const Eigen::Vector3d &Pj, const Eigen::Vector3d &Pi, Eigen::Vector3d &residual,
                                Eigen::Ref<Eigen::Matrix<double, 3, 9>> jacobian) const;
    /**
     * @brief evaluate residual and jacobian from preintegration
     * 
     * @param Pj pose of j th time, j is after i
     * @param Pi pose of i th time
     * @param residual preintegration residual
     * @param jacobian jacobian of residual = [pose i, pose j, intrinsic], rowMajor of eigen matrix
     */
    void Evaluate(const double *Pj, const double *Pi, double *residual, double **jacobian) const;

  private:
    double n_l_, n_r_;
    double dthetaij_, dt_;
    Eigen::Vector3d alphaij_;
    Eigen::Matrix2d Rji_;
    Eigen::Matrix2d noise_;
    Eigen::Matrix3d cov_;
    Eigen::Matrix3d dalpha_drl_drr_dl_;
    Eigen::Matrix<double, 1, 3> dtheta_dp_;
};

class EulerWMPreintegration: public DiffWheelModel {
  public:
    EulerWMPreintegration(const double &rl, const double &rr, const double &l, double nl = 0.01, double nr = 0.01,
                                          bool calib_intrinsic_flag = false);
    void Initialization();
    void LoadWheelDatas(std::vector<Eigen::Matrix<double, 1, 3>> &wheels);
    void LoadWheelData(const Eigen::Vector2d &wheel, const double &dt);
    void InsertWheelVel(const Eigen::Vector2d &wheel_vel, const double &dt);
    void RePreintegration();
    void FusingWMwithoutPreintegration(std::shared_ptr<EulerWMPreintegration> wh_ptr);
    bool Evaluate(const double *Pj, const double *Pi, double *residual, double **jacobians) const;
    Eigen::Matrix2d Rji() { return Rji_; }
    double MoveLength() { return alphaij_.topRows(2).squaredNorm(); }
    double Dt() { return dt_; }
    Eigen::Matrix3d Cov() { return cov_; }
    Eigen::Vector3d Alpha() { return alphaij_; }
    void PoseCalFromLastPose(const Eigen::Matrix3d &last_Rbw, const Eigen::Vector3d &last_tbw,
        const Eigen::Matrix3d &Rob, const Eigen::Vector3d &tob, Eigen::Matrix3d &Rbw, Eigen::Vector3d &tbw);
    void PoseCalFromLastPose(const utils::Transform3d &last_Tbw, const utils::Transform3d &Tob,
                             utils::Transform3d &Tbw);
  protected:
    double n_l_, n_r_;
    double dt_, dthetaij_;
    Eigen::Vector3d alphaij_;
    Eigen::Matrix2d noise_, Rji_;
    Eigen::Matrix3d cov_;
    Eigen::Matrix3d dalpha_drl_drr_dl_;
    Eigen::Matrix<double, 1, 3> dtheta_dp_;
    bool calib_intrinsic_flag_;
    std::vector<Eigen::Vector3d> wheel_data_;
};

class WheelModel2DCalibrationFactor: public EulerWMPreintegration {
  public:
    WheelModel2DCalibrationFactor(const double &rl, const double &rr, const double &l, const Eigen::Vector3d &Twl, double nl = 0.01, double nr = 0.01);
    void LoadWheelData(const Eigen::Vector2d &wheel_vel, const double &dt);
    void RePreintegration(const double &rl, const double &rr, const double &l);
    double Dt() { return dt_; }
    void SetExtrinsic(const Eigen::Vector3d &Twl);
    void SetIntrinsic(const double &rl, const double &rr, const double &l_inv);
    /**
     * @brief factor evaluation
     * 
     * @param Pj input current pose j
     * @param Pi input last pose i
     * @param residual output residual
     * @param jacobians output jacobian = [JPj, JPi, Jintrinsic, Jextrinsic]
     * @return true 
     * @return false 
     */
    bool CalibEvaluate(const double *Pj, const double *Pi, double *residual, double **jacobians);
    struct WheelData {
        WheelData(const Eigen::Vector2d &wheel_data, const double &delta_t): wheel_vel(wheel_data), dt(delta_t) {}
        Eigen::Vector2d wheel_vel;
        double dt;
    };
  private:
    Eigen::Matrix2d Rwl_; // rotation matrix from LDS to differential wheel
    Eigen::Vector2d twl_; // translation from LDS to differential wheel
    Eigen::Vector3d Twl_; // transformation from LDS to differential wheel = [tx, ty, theta_lw]
    std::vector<WheelData> wheels_;
};

struct DiffWheelModel3DFactor {
  Eigen::Matrix<double, 3, 6> Jj;
  Eigen::Matrix<double, 3, 1> residual;
  bool Evalute(const Eigen::Matrix3d &Rbiw, const Eigen::Vector3d &tbiw, const Eigen::Matrix3d &Rbjw, const Eigen::Vector3d &tbjw,
               const Eigen::Matrix3d &Rob, Eigen::Vector3d &tob, std::shared_ptr<EulerWMPreintegration> &wheel_odom);
};

} // namespace Calibration
#endif // MODELS_WHEELMODEL_H