#include "slam/Imu.h"

namespace slam {
// define static variouty
Eigen::Matrix3d Imu::Rib_ = Eigen::Matrix3d::Identity();
Eigen::Vector3d Imu::tib_ = Eigen::Vector3d::Zero();
Eigen::Vector3d Imu::nba_ = Eigen::Vector3d::Zero();
Eigen::Vector3d Imu::nbg_ = Eigen::Vector3d::Zero();
Eigen::Vector3d Imu::na_ = Eigen::Vector3d::Zero();
Eigen::Vector3d Imu::ng_ = Eigen::Vector3d::Zero();
Eigen::Matrix<double, 6, 6> Imu::nbga_ = Eigen::Matrix<double, 6, 6>::Identity();
Eigen::Matrix<double, 6, 6> Imu::nga_ = Eigen::Matrix<double, 6, 6>::Identity();

Imu::Imu(std::shared_ptr<Sensors> sensor_ptr) {
    ba_ = sensor_ptr->imu.ba;
    bg_ = sensor_ptr->imu.bg;
    na_ = sensor_ptr->imu.na;
    ng_ = sensor_ptr->imu.ng;
    nba_ = sensor_ptr->imu.nba;
    nbg_ = sensor_ptr->imu.nbg;

    nga_.topLeftCorner(3, 3) = ng_.asDiagonal();
    nga_.bottomRightCorner(3, 3) = na_.asDiagonal();
    nbga_.topLeftCorner(3, 3) = nbg_.asDiagonal();
    nbga_.bottomRightCorner(3, 3) = nba_.asDiagonal();

    Rib_ = sensor_ptr->imu.Rib;
    tib_ = sensor_ptr->imu.tib;

    Initialize();
}

Imu::Imu(const double &ba, const double &bg, const double &na, const double &ng, const double &nbg, const double &nba) {
    Eigen::Vector3d one; one.fill(1.);
    ba_ = one * ba;
    bg_ = one * bg;
    na_ = one * na;
    ng_ = one * ng;
    nga_.setIdentity();
    nga_.topLeftCorner(3, 3) *= ng;
    nga_.bottomRightCorner(3, 3) *= na;
    nbga_.setIdentity();
    nbga_.topLeftCorner(3, 3) *= nbg;
    nbga_.bottomRightCorner(3, 3) *= nba;

    Initialize();
}

Imu::Imu(const Eigen::Vector3d &ba, const Eigen::Vector3d &bg) {
    ba_ = ba;
    bg_ = bg;

    Initialize();
}

void Imu::Initialize() {
    Rji_.setIdentity();
    vij_.setZero();
    pij_.setZero();
    dRji_dba_.setZero();
    dRji_dbg_.setZero();
    dvij_dba_.setZero();
    dvij_dbg_.setZero();
    dpij_dba_.setZero();
    dpij_dbg_.setZero();
    cov_.setZero();
    dt_ = 0;

    A_.setIdentity();
    B_.setZero();

    imu_datas_.clear();
}

void Imu::Reset(const Eigen::Vector3d &ba, const Eigen::Vector3d &bg) {
    Initialize();

    ba_ = ba;
    bg_ = bg;
}

void Imu::GlobalParametersInitialize(std::shared_ptr<Sensors> sensor_ptr) {
    Rib_ = sensor_ptr->imu.Rib;
    tib_ = sensor_ptr->imu.tib;
    na_ = sensor_ptr->imu.na;
    nba_ = sensor_ptr->imu.nba;
    ng_ = sensor_ptr->imu.ng;
    nbg_ = sensor_ptr->imu.nbg;

    nga_.topLeftCorner(3, 3) = ng_.asDiagonal();
    nga_.bottomRightCorner(3, 3) = na_.asDiagonal();
    nbga_.topLeftCorner(3, 3) = nbg_.asDiagonal();
    nbga_.bottomRightCorner(3, 3) = nba_.asDiagonal();
    nbga_ = nbga_ * nbga_;
    nga_ = nga_ * nga_;
}

void Imu::StateUpdate(const Eigen::Vector3d &acc, const Eigen::Vector3d &gyr, const double &dt) {
    if (dt < 1.0e-05) return;
    Eigen::Matrix3d Jr = utils::DMmath::RightJacobian(gyr * dt);
    Eigen::Matrix3d skew_acc = utils::DMmath::SkewMatrix(acc);
    Eigen::Matrix3d dR = utils::DMmath::RotationVector2Matrix(gyr * dt);

    // covariance update
    A_.topLeftCorner(3, 3) = dR.transpose(); // dR
    A_.block<3, 3>(3, 0) = -Rji_ * skew_acc * dt; // dv
    A_.block<3, 3>(6, 0) = -0.5 * Rji_ * skew_acc * dt * dt; // dp
    A_.block<3, 3>(6, 3) = Eigen::Matrix3d::Identity() * dt;
    B_.topLeftCorner(3, 3) = Jr * dt;
    B_.block<3, 3>(3, 3) = Rji_ * dt;
    B_.block<3, 3>(6, 3) = 0.5 * Rji_ * dt * dt;
    // xj = A * xi + B * n;
    // cov_xj = A * cov_xi * A^t + B * N * B^t
    cov_.topLeftCorner(9, 9) = A_ * cov_.topLeftCorner(9, 9) * A_.transpose() + B_ * nga_ * B_.transpose() / dt;
    
    // bj = bi + nbga
    // cov_bj = cov_bi + Nbga
    cov_.bottomRightCorner(6, 6) += nbga_ * dt;

    // jacobian over bias
    dvij_dba_ += -Rji_ * dt; // bai = ba + dba
    dvij_dbg_ += -Rji_ * skew_acc * dRji_dbg_ * dt; // bgi = bg + dbg
    dpij_dba_ += dvij_dba_ * dt - 0.5 * Rji_ * dt * dt;
    dpij_dbg_ += dvij_dbg_ * dt - 0.5 * Rji_ * skew_acc * dRji_dbg_ * dt * dt;
    dRji_dbg_ = dR.transpose() * dRji_dbg_ - Jr * dt;

    // observation update
    pij_ += vij_ * dt + 0.5 * Rji_ * acc * dt * dt;
    vij_ += Rji_ * acc * dt;
    Rji_ = Rji_ * dR;
    dt_ += dt;

    imu_datas_.emplace_back(gyr, acc, dt);
}

// data = [timestamp, acc, gyro]
// [dtheta; dv; dp] = A * [dtheta; dv; dp] + B * [ng; na]
// [dtheta; dv; dp; dbg; dba] = Gaussian(u, cov)
void Imu::PreIntegration(const std::vector<Eigen::Matrix<double, 1, 7>> &datas) {
    double dt;
    Eigen::Vector3d acc, gyr;
    for (size_t i = 1; i < datas.size(); i++) {
        dt = datas[i](0) - datas[i - 1](0);
        Eigen::Matrix<double, 7, 1> mean = datas[i].transpose() + datas[i - 1].transpose();
        mean *= 0.5;
        acc = mean.segment(1, 3) - ba_;
        gyr = mean.segment(4, 3) - bg_;
        StateUpdate(acc, gyr, dt);
    }
}

void Imu::RePreintegration() {
    std::vector<ImuData> imus;
    imus.swap(imu_datas_);
    Initialize();
    for (size_t i = 0; i < imus.size(); i++) {
        StateUpdate(imus[i].acc, imus[i].gyr, imus[i].dt);
    }
}

void Imu::InsertImuWithoutPreintegration(std::shared_ptr<Imu> imu_ptr) {
    if (imu_ptr == nullptr) return;
    imu_datas_.insert(imu_datas_.end(), imu_ptr->imu_datas_.begin(), imu_ptr->imu_datas_.end());
}

void Imu::ImuStatePrediction(const Eigen::Matrix3d &Riw, const Eigen::Vector3d &Vi, const Eigen::Vector3d &Pi, const Eigen::Vector3d &Gravity,
                          Eigen::Matrix3d &Rjw, Eigen::Vector3d &Vj, Eigen::Vector3d &Pj) {
    Rjw = Riw * Rji_;
    Vj = Vi + Gravity * dt_ + Riw * vij_;
    Pj = Pi + Vi * dt_ + 0.5 * Gravity * dt_ * dt_ + Riw * pij_;
}

void Imu::ImuStatePrediction(const utils::Transform3d &Tiw, const Eigen::Vector3d &Vi, const Eigen::Vector3d &Gravity,
                             utils::Transform3d &Tjw, Eigen::Vector3d &Vj) {
    Tjw.R() = Tiw.R() * Rji_;
    Vj = Vi + Gravity * dt_ + Tiw.R() * vij_;
    Tjw.t() = Tiw.t() + Vi * dt_ + 0.5 * Gravity * dt_ * dt_ + Tiw.R() * pij_;
}

void ImuFactor::Evaluate(const Eigen::Matrix3d &Riw, const Eigen::Matrix3d &Rjw, const Eigen::Vector3d &Pi, const Eigen::Vector3d &Pj,
                  const Eigen::Vector3d &Vi, const Eigen::Vector3d &Vj, const Eigen::Vector3d &bai, const Eigen::Vector3d &bgi,
                  const Eigen::Vector3d &baj, const Eigen::Vector3d &bgj, const Eigen::Vector3d &Gravity, const Imu &imu) {
    Eigen::Vector3d dbg = bgi - imu.Bg();
    Eigen::Vector3d dba = bai - imu.Ba();
    const double dt = imu.Dt();

    // build residual
    Eigen::Matrix3d dR = (imu.Rji() * utils::DMmath::RotationVector2Matrix(imu.JRg() * dbg)).transpose() * Riw.transpose() * Rjw;
    residual.topRows(3) = utils::DMmath::LogSO3(dR);   
    residual.segment(3, 3) = Riw.transpose() * (Vj - Vi + Gravity * dt) - (imu.Vij() + imu.Jvg() * dbg + imu.Jva() * dba);
    residual.segment(6, 3) = Riw.transpose() * (Pj - Pi - Vi * dt + 0.5 * Gravity * dt * dt) - (imu.Pij() + imu.Jpg() * dbg + imu.Jpa() * dba);
    residual.segment(9, 3) = bgj - bgi;
    residual.segment(12, 3) = baj - bai;

    Eigen::Matrix3d I3; I3.setIdentity();
    // build jacobian
    Ji.setZero();
    Jj.setZero();

    Eigen::Matrix3d Jr_inv = utils::DMmath::RightJacobianInverse(residual.topRows(3));
    Eigen::Matrix3d Jr = utils::DMmath::RightJacobian(residual.topRows(3));
    // Jacobian of dRji
    // r + dr = log((Rji)^t * (Riw * exp(drvi))^t * Rjw) = log ((Rji)^t * exp(-drvi) * Riw^t * Rjw)
    // = log(Rji^t * Riw^t* Rjw * exp(-Rjw^t * Riw * drvi)) = log(exp(r + Jr_inv(log(Rji^t * Riw^t * Rjw)) * (-Rjw^t * Riw * drvi)))
    // => r + dr = r - Jr_inv(r) * (Rjw^t * Riw * drvi)
    Ji.topLeftCorner(3, 3) = -Jr_inv * Rjw.transpose() * Riw;
    // r + dr = log((Rji * exp(JRg * dbg + JRg * dbgi))^t * Riw^t * Rjw) = log(exp(-Jr(JRg * dbg) * dbgi) * (Rji * exp(JRg * dbg)^t)^t * Riw^t * Rjw)
    // = log(exp(-Jr(JRg * dbg) * JRg * dbgi) * exp(r)) = log(exp(r) * exp(-exp(r)^t * Jr(Jrg * dbg) * JRg * dbgi))
    // = r - Jr_inv(r) * exp(r)^t * Jr(JRg * dbg) * JRg * dbgi
    Ji.block<3, 3>(0, 9) =  -Jr_inv * dR.transpose() * utils::DMmath::RightJacobian(imu.JRg() * dbg) * imu.JRg();
    // r + dr = log((Rji)^t * Riw^t * Rjw * exp(drvj)) = log(exp(r + Jr_inv(r) * drvj)) = r + Jr_inv(r) * drvj
    Jj.topLeftCorner(3, 3) = Jr_inv;

    // jacobian of dvij
    // r + dr = (I - drvi^) * Riw^t * (Vj + dvj - vi - dvi - Gravity * dt) - (Vij + Jvg * (dbg + dbgi) + Jva * (dba + dbai))
    // = r - drvi^ * Riw^t * (Vj - Vi - G * dt) + Riw^t * dvj - Riw^t * dvi - Jvg * dbgi - Jva * dbai
    // = r + (Riw^t * (Vj - Vi - G * dt))^ * drvi + Riw^t * dvj - Riw^t * dvi - Jvg * dbgi - Jva * dbai 
    Ji.block<3, 3>(3, 0) = utils::DMmath::SkewMatrix(Riw.transpose() * (Vj - Vi + Gravity * dt));
    Ji.block<3, 3>(3, 3) = -Riw.transpose();
    Ji.block<3, 3>(3, 9) = -imu.Jvg();
    Ji.block<3, 3>(3, 12) = -imu.Jva();

    Jj.block<3, 3>(3, 3) = Riw.transpose();

    // jacobian of dpij
    // r + dr = (I - drvi^) * Riw^t * (Pj + dpj - Pi - dpi - Vi * dt - dvi * dt - 0.5 * G * dt) - (Pij + Jpg * (dbg + dbgi) + Jpa * (dba + dbai))
    // = r + Skew(Riw^t * (Pj - Pi - Vi * dt - 0.5 * G * dt * dt)) * drvi + Riw^t * dpj - Riw^t * dpi - Riw^t * dvi * dt - Jpg * dbgi - Jpa * dbai
    Ji.block<3, 3>(6, 0) = utils::DMmath::SkewMatrix(Riw.transpose() * (Pj - Pi - Vi * dt + 0.5 * Gravity * dt * dt));
    Ji.block<3, 3>(6, 3) = -Riw.transpose() * dt;
    Ji.block<3, 3>(6, 6) = -Riw.transpose();
    Ji.block<3, 3>(6, 9) = -imu.Jpg();
    Ji.block<3, 3>(6, 12) = -imu.Jpa();

    Jj.block<3, 3>(6, 6) = Riw.transpose();

    // jacobian of dbgi bgj
    Ji.block<3, 3>(9, 9) = -I3;
    Jj.block<3, 3>(9, 9) = I3;

    // jacobian of dbai baj
    Ji.block<3, 3>(12, 12) = -I3;
    Jj.block<3, 3>(12, 12) = I3;        
}

void GyroFactor::Evaluate(const Eigen::Matrix3d &Rbiw, const Eigen::Matrix3d &Rbjw, const Eigen::Vector3d &bgi, const Eigen::Vector3d &bgj,
                                 const Eigen::Matrix3d &Rib, const Imu &imu) {
    Eigen::Vector3d dbg = bgi - imu.Bg();
    Eigen::Matrix3d Riw = Rbiw * Rib;
    Eigen::Matrix3d Rjw = Rbjw * Rib;

    Eigen::Matrix3d dR = imu.Rji() * utils::DMmath::RotationVector2Matrix(imu.JRg() * dbg) * Rjw.transpose() * Riw;

    residual << utils::DMmath::LogSO3(dR), bgj - bgi;

    Ji.setZero();
    Jj.setZero();

    Eigen::Matrix3d Jr_inv = utils::DMmath::RightJacobianInverse(residual.topRows(3));
    // jacob of dR
    // e + de = log(Rji * exp(JRg * dbg) * Rjw^t * (Rbiw * exp(drvi) * Rib)) = log(exp(r) * Rib^t * exp(drvi) * Rib)
    // e + de = log(exp(r) * exp(Rib^t * drvi)) = log(exp(r + Jr_inv * Rib^t * drvi)) = r + Jr_inv * Rib^t * drvi
    Ji.topLeftCorner(3, 3) = Jr_inv * Rib.transpose();
    // e + de = log(Rji * exp(JRg * (bgi - bg + dbgi)) * Rjw^t * Riw)
    // = log(Rji * exp(JRg * dbg) * exp(Jr(JRg * dbg) * JRg * dbgi) * Rjw^t * Riw)
    // = log(Rji * exp(JRg * dbg) * Rjw^t * Riw * exp(Riw^t * Rjw * Jr(JRg * dbg) * JRg * dbgi))
    // = log(exp(r) * exp(Riw^t * Rjw * Jr(JRg * dbg) * JRg * dbgi))
    // = r + Jr_inv * Riw^t * Rjw * Jr(JRg * dbg) * JRg * dbgi
    Ji.topRightCorner(3, 3) = Jr_inv * Riw.transpose() * Rjw * utils::DMmath::RightJacobian(imu.JRg() * dbg) * imu.JRg();
    Ji.bottomRightCorner(3, 3) = -Eigen::Matrix3d::Identity();

    // e + de = log(Rji * exp(JRg * dbg) * (Rjw * exp(drvj) * Rib)^t * Riw)
    // = log(Rji * exp(JRg * dbg) * exp(-Rib^t * drvj) * Rjw^t * Riw
    // = log(exp(r) * exp(-Riw^t * Rjw * Rib^t * drvj)) = log(exp(r + Jr_inv * (-Riw^t * Rjw * Rib^t * drvj)))
    // = r - Jr_inv * Riw^t * Rjw * Rib^t * drvj
    Jj.topLeftCorner(3, 3) = -Jr_inv * Riw.transpose() * Rjw * Rib.transpose();
    Jj.bottomRightCorner(3, 3).setIdentity();
}

void GyroFactor::Evaluate(const Eigen::Matrix3d &Rbiw, const Eigen::Matrix3d &Rbjw, const Eigen::Vector3d &bgi, const Eigen::Vector3d &bgj,
                  const Eigen::Matrix3d &Rib, std::shared_ptr<Imu> imu) {
    Eigen::Vector3d dbg = bgi - imu->Bg();
    Eigen::Matrix3d Riw = Rbiw * Rib;
    Eigen::Matrix3d Rjw = Rbjw * Rib;

    Eigen::Matrix3d dR = imu->Rji() * utils::DMmath::RotationVector2Matrix(imu->JRg() * dbg) * Rjw.transpose() * Riw;

    residual << utils::DMmath::LogSO3(dR), bgj - bgi;

    Ji.setZero();
    Jj.setZero();

    Eigen::Matrix3d Jr_inv = utils::DMmath::RightJacobianInverse(residual.topRows(3));
    // jacob of dR
    // e + de = log(Rji * exp(JRg * dbg) * Rjw^t * (Rbiw * exp(drvi) * Rib)) = log(exp(r) * Rib^t * exp(drvi) * Rib)
    // e + de = log(exp(r) * exp(Rib^t * drvi)) = log(exp(r + Jr_inv * Rib^t * drvi)) = r + Jr_inv * Rib^t * drvi
    Ji.topLeftCorner(3, 3) = Jr_inv * Rib.transpose();
    // e + de = log(Rji * exp(JRg * (bgi - bg + dbgi)) * Rjw^t * Riw)
    // = log(Rji * exp(JRg * dbg) * exp(Jr(JRg * dbg) * JRg * dbgi) * Rjw^t * Riw)
    // = log(Rji * exp(JRg * dbg) * Rjw^t * Riw * exp(Riw^t * Rjw * Jr(JRg * dbg) * JRg * dbgi))
    // = log(exp(r) * exp(Riw^t * Rjw * Jr(JRg * dbg) * JRg * dbgi))
    // = r + Jr_inv * Riw^t * Rjw * Jr(JRg * dbg) * JRg * dbgi
    Ji.topRightCorner(3, 3) = Jr_inv * Riw.transpose() * Rjw * utils::DMmath::RightJacobian(imu->JRg() * dbg) * imu->JRg();
    Ji.bottomRightCorner(3, 3) = -Eigen::Matrix3d::Identity();

    // e + de = log(Rji * exp(JRg * dbg) * (Rjw * exp(drvj) * Rib)^t * Riw)
    // = log(Rji * exp(JRg * dbg) * exp(-Rib^t * drvj) * Rjw^t * Riw
    // = log(exp(r) * exp(-Riw^t * Rjw * Rib^t * drvj)) = log(exp(r + Jr_inv * (-Riw^t * Rjw * Rib^t * drvj)))
    // = r - Jr_inv * Riw^t * Rjw * Rib^t * drvj
    Jj.topLeftCorner(3, 3) = -Jr_inv * Riw.transpose() * Rjw * Rib.transpose();
    Jj.bottomRightCorner(3, 3).setIdentity();
}
} // namespace slam