#include "slam/ImuWheel.h"

namespace slam {
GyroWheelModelPreintegration::WheelData::WheelData(const std::vector<Eigen::Vector2d> &gs, const Eigen::Vector2d &wh)
  : wheel(wh) {
    gyros.insert(gyros.end(), gs.begin(), gs.end());
}

GyroWheelModelPreintegration::GyroWheelModelPreintegration(const double &rl, const double &rr, const double &bg, const Eigen::Vector3d &noise)
  : rl_(rl),
    rr_(rr),
    bg_(bg) {
    noise_ = noise.asDiagonal();
    noise_ = noise_ * noise_;
    Initialization();
}

void GyroWheelModelPreintegration::InsertDatas(const std::vector<Eigen::RowVector2d> &gyros, const std::vector<Eigen::RowVector3d> &wheels) {
    int g_idx = 0;
    std::vector<Eigen::Vector2d> gs;
    for (int i = 1; i < wheels.size(); i++) {
        double tw0 = wheels[i - 1](0);
        double tw1 = wheels[i](0);
        gs.clear();
        for (; g_idx < gyros.size(); g_idx++) {
            if (gyros[g_idx](0) <= tw0 + 0.00001 && gyros[g_idx + 1](0) > tw0) {
                if (gyros[g_idx + 1](0) - tw0 > tw0 - gyros[g_idx](0)) {
                    gs.emplace_back(tw0, gyros[g_idx](1));
                } else {
                    gs.emplace_back(tw0, gyros[g_idx + 1](1));
                }
                continue;
            }

            if (gyros[g_idx](0) > tw0 && gyros[g_idx](0) < tw1) {
                gs.push_back(gyros[g_idx]);
            }

            if (gyros[g_idx](0) >= tw1) {
                if (gyros[g_idx](0) - tw1 > tw1 - gyros[g_idx - 1](0)) {
                    gs.emplace_back(tw1, gyros[g_idx - 1](1));
                } else {
                    gs.emplace_back(tw1, gyros[g_idx](1));
                }
                g_idx--;
                break;
            }
        }
        Eigen::Vector2d wheel;
        wheel = 0.5 * (wheels[i].rightCols(2) + wheels[i - 1].rightCols(2));
        InsertData(gs, wheel);
    }
}

void GyroWheelModelPreintegration::Initialization() {
    cov_.setZero();
    thetaij_ = 0.;
    Rji_.setIdentity();
    dt_ = 0.;
    alphaij_.setZero();
    wheel_data_.clear();
}


void GyroWheelModelPreintegration::InsertData(const std::vector<Eigen::Vector2d> &gyros, const Eigen::Vector2d &wheel) {
    // gyro integration
    double dtheta = 0;
    double dt = 0;
    for (size_t i = 1; i < gyros.size(); i++) {
        double dt_tmp = gyros[i](0) - gyros[i - 1](0);
        dtheta += 0.5 * (gyros[i](1) + gyros[i - 1](1)) * dt_tmp;
        dt += dt_tmp;
    }

    // preintegration
    double vl = wheel(0) * rl_;
    double vr = wheel(1) * rr_;
    double ds = 0.5 * (vl + vr) * dt;
    double cos_dtheta = cos(dtheta);
    double sin_dtheta = sin(dtheta);
    double cos_half_dtheta = cos(0.5 * dtheta);
    double sin_half_dtheta = sin(0.5 * dtheta);

    Eigen::Matrix2d dR;
    dR << cos_dtheta, -sin_dtheta, sin_dtheta, cos_dtheta;
    Eigen::Vector2d drj(cos_half_dtheta * ds, sin_half_dtheta * ds);
    alphaij_.topRows(2) += Rji_ * drj;
    alphaij_(2) += dtheta;

    // noise propagation
    Eigen::Matrix<double, 2, 3> dsdtheta_dwldwrdgyr;
    dsdtheta_dwldwrdgyr.topRows(1) << 0.5 * rl_ * dt, 0.5 * rr_ * dt, 0.;
    dsdtheta_dwldwrdgyr.bottomRows(1) << 0., 0., dt;
    Eigen::Matrix<double, 3, 2> dalpha_dsdtheta;
    dalpha_dsdtheta.topLeftCorner(2, 1) = Rji_ * Eigen::Vector2d(cos_half_dtheta, sin_half_dtheta);
    dalpha_dsdtheta.topRightCorner(2, 1) = Rji_ * Eigen::Vector2d(-sin_half_dtheta * 0.5 * ds, cos_half_dtheta * 0.5 * ds);
    dalpha_dsdtheta.bottomRows(1) << 0., 1.;
    Eigen::Matrix<double, 3, 3> dalpha_dnoise = dalpha_dsdtheta * dsdtheta_dwldwrdgyr;
    Eigen::Matrix3d dalpha_dlast_alpha; dalpha_dlast_alpha.setIdentity();
    Eigen::Matrix2d dRji_dtheta;
    dRji_dtheta << Rji_(0, 1), -Rji_(0, 0), Rji_(0, 0), Rji_(0, 1);
    dalpha_dlast_alpha.topRightCorner(2, 1) = Rji_ * drj;
    cov_ = dalpha_dlast_alpha * cov_ * dalpha_dlast_alpha.transpose() + dalpha_dnoise * noise_ * dalpha_dnoise.transpose() / dt;

    // state propagation
    Rji_ = Rji_ * dR;
    thetaij_ += dtheta;
    dt_ += dt;

    // save data
    wheel_data_.emplace_back(gyros, wheel);
}

void GyroWheelModelPreintegration::RePreintegration() {
    std::vector<WheelData> wheel_data;
    wheel_data.swap(wheel_data_);
    Initialization();
    for (size_t i = 0; i < wheel_data.size(); i++) {
        InsertData(wheel_data[i].gyros, wheel_data[i].wheel);
    }
}

void GyroWheelModelPreintegration::PoseCalFromLastPose(const Eigen::Matrix3d &last_Rbw, const Eigen::Vector3d &last_tbw,
                                const Eigen::Matrix3d &Rob, const Eigen::Vector3d &tob, Eigen::Matrix3d &Rbw, Eigen::Vector3d &tbw) {
    Eigen::Vector3d tojoi;
    tojoi << alphaij_.topRows(2), 0.;
    Eigen::Matrix3d Rojoi = utils::DMmath::RotationVector2Matrix(Eigen::Vector3d(0., 0., alphaij_(2)));

    Rbw = last_Rbw * Rob * Rojoi * Rob.transpose();

    tbw = tojoi - Rojoi * Rob.transpose() * tob + Rob.transpose() * tob;
    tbw = last_Rbw * Rob * tbw + last_tbw;
}

void GyroWheelModelPreintegration::PoseCalFromLastPose(const utils::Transform3d &last_Tbw, const utils::Transform3d &Tob,
                                                       utils::Transform3d &Tbw) {
    utils::Transform3d Tojoi;
    Tojoi.t() << alphaij_.topRows(2), 0.;
    Tojoi.R() = utils::DMmath::RotationVector2Matrix(Eigen::Vector3d(0., 0., alphaij_(2)));

    Tbw = last_Tbw * Tob * Tojoi * Tob.Inverse();
}

void GyroWheelModelPreintegration::FusingGyrWheelWithoutPreintegration(std::shared_ptr<GyroWheelModelPreintegration> &obj_ptr) {
    if (obj_ptr != nullptr) {
        this->wheel_data_.insert(this->wheel_data_.end(), obj_ptr->wheel_data_.begin(), obj_ptr->wheel_data_.end());
    }
}


bool GyroWheelModelPreintegration::Evaluate(const double *Pj, const double *Pi, double *residual, double **jacobians) const {
    Eigen::Map<const Eigen::Vector3d> pose_pj(Pj);
    Eigen::Map<const Eigen::Vector3d> pose_Pi(Pi);
    Eigen::Map<Eigen::Vector3d> res(residual);
    double cos_theta = cos(pose_Pi(2));
    double sin_theta = sin(pose_Pi(2));
    Eigen::Matrix2d Riw;
    Riw << cos_theta, -sin_theta, sin_theta, cos_theta;
    res.topRows(2) = Riw.transpose() * (pose_pj.topRows(2) - pose_Pi.topRows(2));
    res(2) = pose_pj(2) - pose_Pi(2);
    res -= alphaij_;
    if (jacobians != nullptr) {
        // jacobian of pi
        if (jacobians[0] != nullptr) {
            Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> jacobi(jacobians[0]);
            jacobi.setIdentity();
            Eigen::Matrix2d R;
            R << -sin_theta, cos_theta, -cos_theta, -sin_theta;
            jacobi.topLeftCorner(2, 2) = -Riw.transpose();
            jacobi.block<2, 1>(0, 2) = R * (pose_pj - pose_Pi).topRows(2);
            jacobi(2, 2) = -1.;
        }

        // jacobian of pj
        if (jacobians[1] != nullptr) {
            Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> jacobj(jacobians[1]);
            jacobj.setIdentity();
            jacobj.block<2, 2>(0, 0) = Riw.transpose();
            jacobj(2, 2) = 1.;
        }
    }
    return true;
}

bool GyroWheel3DFactor::Evaluate(const utils::Transform3d &Tbiw, const utils::Transform3d &Tbjw, const utils::Transform3d &Tob,
                                 std::shared_ptr<GyroWheelModelPreintegration> gyrowheel_ptr) {
    utils::Transform3d Tojoi = Tob.Inverse() * Tbiw.Inverse() * Tbjw * Tob;

    Eigen::Vector3d rvji = utils::DMmath::LogSO3(Tojoi.R());

    Eigen::Vector3d Pj, Pi;
    Pj << Tojoi.t().topRows(2), rvji(2);
    Pi.setZero();
    Eigen::Matrix<double, 3, 3, Eigen::RowMajor> Jpj;
    double *jacobian[3];
    jacobian[0] = nullptr;
    jacobian[1] = Jpj.data();
    jacobian[2] = nullptr;
    gyrowheel_ptr->Evaluate(Pj.data(), Pi.data(), residual.data(), jacobian);

    Jj.setZero();
    Jj.col(2) = Jpj.col(2);
    Jj.middleCols(3, 2) = Jpj.leftCols(2);

    Jj.leftCols(3) = Jj.leftCols(3) * Tob.R().transpose() - Jj.rightCols(3) * Tojoi.R() * Tob.R().transpose() * utils::DMmath::SkewMatrix(Tob.t());
    Jj.rightCols(3) = Jj.rightCols(3) * Tob.R().transpose() * Tbiw.R().transpose();
    return true;
}

bool GyroWheel3Dfactorij::Evaluate(const utils::Transform3d &Tbiw, const utils::Transform3d &Tbjw, const utils::Transform3d &Tob,
                                   std::shared_ptr<GyroWheelModelPreintegration> gyrowheel_ptr) {
    utils::Transform3d Tojoi = Tob.Inverse() * Tbiw.Inverse() * Tbjw * Tob;
    Eigen::Vector3d rvji = utils::DMmath::LogSO3(Tojoi.R());

        Eigen::Vector3d Pj, Pi;
    Pj << Tojoi.t().topRows(2), rvji(2);
    Pi.setZero();
    Eigen::Matrix<double, 3, 3, Eigen::RowMajor> Jpj;
    double *jacobian[3];
    jacobian[0] = nullptr;
    jacobian[1] = Jpj.data();
    jacobian[2] = nullptr;
    gyrowheel_ptr->Evaluate(Pj.data(), Pi.data(), residual.data(), jacobian);

    Eigen::Matrix3d dr_drojoi, dr_dtojoi;
    dr_drojoi.setZero();
    dr_drojoi.col(2) = Jpj.col(2);
    dr_dtojoi.setZero();
    dr_dtojoi.leftCols(2) = Jpj.leftCols(2);

    Jj.leftCols(3) = dr_drojoi * Tob.R().transpose() - dr_dtojoi * Tojoi.R() * Tob.R().transpose() * utils::DMmath::SkewMatrix(Tob.t());
    Jj.rightCols(3) = dr_dtojoi * Tob.R().transpose() * Tbiw.R().transpose();

    // Rojoi * exp(drv1) = Rob^t * exp(-drv0) * Rbiw^t * Rbjw * Rob
    // = exp(-Rob^t * drv0) * Rojoi
    // => exp(drv1) = Rojoi^t * exp(-Rob^t * drv0) * Rojoi
    // = exp(-Rojoi^t * Rob^t * drv0)
    // tojoi + dt1 = Rob^t * exp(-drv0) * Rbiw^t * Rbjw * tob + Rob^t * (exp(-drv0) * Rbiw^t * （tbjw - tbiw) - tob)
    // tojoi + dt1 = expt(-Rob^t * drv0) * Rob^t * Rbiw^t * Rbjw * tob + exp(-Rob^t * drv0) * Rob^t * Rbiw^t * (tbjw - tbiw) - Rob^t - tob
    // tojoi + dt1 = -(Rob^t * drv0)^ * Rob^t * Rbiw^t * (Rbjw * tob + tbjw - tbiw) + tojoi
    // dt1 = [Rob^t * Rbiw^t * (Rbjw * tob + tbjw - tbiw)]^ * Rob^t * drv0 
    Ji.leftCols(3) = -dr_drojoi * Tojoi.R().transpose() * Tob.R().transpose();
    Ji.leftCols(3) += dr_dtojoi * utils::DMmath::SkewMatrix(Tob.R().transpose() * Tbiw.R().transpose() *
                      (Tbjw.R() * Tob.t() + Tbjw.t() - Tbiw.t())) * Tob.R().transpose();

    // tojoi + dt1 = Rob^t * Rbiw^t * Rbjw * tob + Rob^t * (Rbiw^t * （tbjw - tbiw - dti) - tob)
    // tojoi + dt1 = tojoi - Rob^t * Rbiw^t * tob
    // dt1 = -Rob^t * Rbiw^t * tob
    Ji.rightCols(3) = -dr_dtojoi * Tob.R().transpose() * Tbiw.R().transpose();
    return true;
}

bool GyroWheel3Dfactorij::Evaluate(const Eigen::Matrix3d &Rbiw, const Eigen::Vector3d &tbiw, const Eigen::Matrix3d &Rbjw, const Eigen::Vector3d &tbjw,
                  const Eigen::Matrix3d &Rob, Eigen::Vector3d &tob, std::shared_ptr<GyroWheelModelPreintegration> gyrowheel_ptr) {
    Eigen::Matrix3d Rojoi;
    Eigen::Vector3d tojoi;
    utils::DMmath::TransformMultiply(Rbiw.transpose(), -Rbiw.transpose() * tbiw, Rbjw, tbjw, Rojoi, tojoi);
    utils::DMmath::TransformMultiply(Rob.transpose(), -Rob.transpose() * tob, Rojoi, tojoi, Rojoi, tojoi);
    utils::DMmath::TransformMultiply(Rojoi, tojoi, Rob, tob, Rojoi, tojoi);

    Eigen::Vector3d rvji = utils::DMmath::LogSO3(Rojoi);

    Eigen::Vector3d Pj, Pi;
    Pj << tojoi.topRows(2), rvji(2);
    Pi.setZero();
    Eigen::Matrix<double, 3, 3, Eigen::RowMajor> Jpj;
    double *jacobian[3];
    jacobian[0] = nullptr;
    jacobian[1] = Jpj.data();
    jacobian[2] = nullptr;
    gyrowheel_ptr->Evaluate(Pj.data(), Pi.data(), residual.data(), jacobian);

    Eigen::Matrix3d dr_drojoi, dr_dtojoi;
    dr_drojoi.setZero();
    dr_drojoi.col(2) = Jpj.col(2);
    dr_dtojoi.setZero();
    dr_dtojoi.leftCols(2) = Jpj.leftCols(2);

    Jj.leftCols(3) = dr_drojoi * Rob.transpose() - dr_dtojoi * Rojoi * Rob.transpose() * utils::DMmath::SkewMatrix(tob);
    Jj.rightCols(3) = dr_dtojoi * Rob.transpose() * Rbiw.transpose();

    // Rojoi * exp(drv1) = Rob^t * exp(-drv0) * Rbiw^t * Rbjw * Rob
    // = exp(-Rob^t * drv0) * Rojoi
    // => exp(drv1) = Rojoi^t * exp(-Rob^t * drv0) * Rojoi
    // = exp(-Rojoi^t * Rob^t * drv0)
    // tojoi + dt1 = Rob^t * exp(-drv0) * Rbiw^t * Rbjw * tob + Rob^t * (exp(-drv0) * Rbiw^t * （tbjw - tbiw) - tob)
    // tojoi + dt1 = expt(-Rob^t * drv0) * Rob^t * Rbiw^t * Rbjw * tob + exp(-Rob^t * drv0) * Rob^t * Rbiw^t * (tbjw - tbiw) - Rob^t - tob
    // tojoi + dt1 = -(Rob^t * drv0)^ * Rob^t * Rbiw^t * (Rbjw * tob + tbjw - tbiw) + tojoi
    // dt1 = [Rob^t * Rbiw^t * (Rbjw * tob + tbjw - tbiw)]^ * Rob^t * drv0 
    Ji.leftCols(3) = -dr_drojoi * Rojoi.transpose() * Rob.transpose();
    Ji.leftCols(3) += dr_dtojoi * utils::DMmath::SkewMatrix(Rob.transpose() * Rbiw.transpose() * (Rbjw * tob + tbjw - tbiw)) * Rob.transpose();

    // tojoi + dt1 = Rob^t * Rbiw^t * Rbjw * tob + Rob^t * (Rbiw^t * （tbjw - tbiw - dti) - tob)
    // tojoi + dt1 = tojoi - Rob^t * Rbiw^t * tob
    // dt1 = -Rob^t * Rbiw^t * tob
    Ji.rightCols(3) = -dr_dtojoi * Rob.transpose() * Rbiw.transpose();
    return true;
}

} // namespace slam