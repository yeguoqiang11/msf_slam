#include "slam/WheelModel.h"

namespace slam {
Eigen::Vector3d DiffWheelModel::VelCalculation(const Eigen::Vector2d &wheel_vel) const {
    // v_l = w_l * r_l, v_r = w_r * r_r
    // r * w = v_l, (r + l) * w = r * w + l * w = v_l + l * w = v_r => w = (v_r - v_l) / l
    // vx = w * (r + 0.5 * l) = 0.5 * (v_l + v_r)

    double vl = wheel_vel(0) * r_l_;
    double vr = wheel_vel(1) * r_r_;
    Eigen::Vector3d out;
    out(0) = (vl + vr) * 0.5;
    out(1) = 0;
    out(2) = (vr - vl) * l_inv_;
    return out;
}

Eigen::Vector3d DiffWheelModel::MovementCalculation(const Eigen::Vector2d &wheel_vel, double dt) const {
    double vl = wheel_vel(0) * r_l_;
    double vr = wheel_vel(1) * r_r_;
    double s = 0.5 * (vl + vr) * dt;
    double theta = (vr - vl) * l_inv_ * dt;

    Eigen::Vector3d out;
    out << s, 0., theta;
    return out;
}

Eigen::Vector3d DiffWheelModel::EulerIntegrationFromLastPose(const Eigen::Vector3d &Pi, const Eigen::Vector3d &v, double dt) const {
    Eigen::Vector3d Pj;
    double dtheta = v(2) * dt;
    double s = v(0) * dt;

    Pj << Pi(0) + s * cos(Pi(2) + 0.5 * dtheta), Pi(1) + s * sin(Pi(2) + 0.5 * dtheta), Pi(2) + dtheta;
    return Pj;
}

Eigen::Vector3d DiffWheelModel::EulerWheelFromLastPose(const Eigen::Vector3d &Pi, const Eigen::Vector2d &wheel_vel, double dt) const {
    Eigen::Vector3d v = VelCalculation(wheel_vel);
    return EulerIntegrationFromLastPose(Pi, v, dt);
}

Eigen::Vector3d DiffWheelModel::ArcWheelFromLastPose(const Eigen::Vector3d &Pi, const Eigen::Vector2d &Wheel_vel, double dt) const {
    Eigen::Vector3d v = VelCalculation(Wheel_vel);
    return ArcIntegrationFromLastPose(Pi, v, dt);
}

Eigen::Vector3d DiffWheelModel::ArcIntegrationFromLastPose(const Eigen::Vector3d &Pi, const Eigen::Vector3d &v, double dt) const {
    Eigen::Vector3d Pj;
    double dtheta = v(2) * dt;
    double s = v(0) * dt;
    if (fabs(dtheta) <= std::numeric_limits<double>::epsilon() * 1000) {
        Pj << Pi(0) + s * cos(Pi(2)), Pi(1) + s * sin(Pi(2)), Pi(2) + dtheta;
    } else {
        double dtheta_inv = 1. / dtheta;
        Pj(0) = Pi(0) + s * dtheta_inv * (sin(Pi(2) + dtheta) - sin(Pi(2)));
        Pj(1) = Pi(1) + s * dtheta_inv * (-cos(Pi(2) + dtheta) + cos(Pi(2)));
        Pj(2) = Pi(2) + dtheta;
    }
    return Pj;
}

// differential wheel model
// [dx_dt; dy_dt; dtheta_dt] = [cos_theta; sin_theta; 0] * v + [0; 0; 1] * w
// v = 0.5 * (vl + vr), w = (vl  - vr) / l;
Eigen::Vector3d DiffWheelModel::RungeKuttaIntegrationFromLastPose(const Eigen::Vector3d &Pi, const Eigen::Vector3d &v, double dt) const {
    Eigen::Vector3d dv0, dv1, dv2;
    dv0 << cos(Pi(2)) * v(0), sin(Pi(2)) * v(0), v(2);
    double dtheta = v(2) * dt;
    double dtheta_half = 0.5 * v(2) * dt;

    dv1 << cos(Pi(2) + dtheta_half) * v(0), sin(Pi(2) + dtheta_half) * v(0), v(2);

    dv2 << cos(Pi(2) + dtheta) * v(0), sin(Pi(2) + dtheta) * v(0), v(2);
    Eigen::Vector3d Pj;
    Pj = Pi + dt * (dv0 + 4. * dv1 + dv2) / 6.;
    return Pj;
}

Eigen::Vector3d DiffWheelModel::RungeKuttawheelFromLastPose(const Eigen::Vector3d &Pi, const Eigen::Vector2d &wheel_data, double dt) const {
    Eigen::Vector3d v = VelCalculation(wheel_data);
    return RungeKuttaIntegrationFromLastPose(Pi, v, dt);
}

EulerDiffWheelModel::EulerDiffWheelModel(double radius, double distance)
: r_(radius), d_inv_(1./ distance), pose_(0., 0., 0.) {
    dt_ = 0.;
}

void EulerDiffWheelModel::LoadWheeldata(const Eigen::Vector2d &wheel, double dt) {
    double vl = wheel(0);// * r_;
    double vr = wheel(1);// * r_;
    double s = 0.5 * (vl + vr) * dt;
    double dtheta = (vr - vl) * d_inv_ * dt;

    pose_ << pose_(0) + s * cos(pose_(2) + 0.5 * dtheta), pose_(1) + s * sin(pose_(2) + 0.5 * dtheta), pose_(2) + dtheta;
    dt_ += dt;
}

void EulerDiffWheelModel::LoadWheelDatas(std::vector<Eigen::Matrix<double, 1, 3>> &wheels) {
    double dt;
    Eigen::Vector2d wheel;
    for (size_t i = 1; i < wheels.size(); i++) {
        dt = wheels[i](0) - wheels[i - 1](0);
        wheel = 0.5 * (wheels[i].rightCols(2) + wheels[i - 1].rightCols(2)).transpose();
        LoadWheeldata(wheel, dt);
    }
}

void EulerDiffWheelModel::GetPose(Eigen::Matrix3d &Rbw, Eigen::Vector3d &tbw) {
   Rbw = utils::DMmath::RotationVector2Matrix(Eigen::Vector3d(0., 0., pose_(2)));
   tbw << pose_(0), pose_(1), 0.;
}

ArcWheelModelPreintegration::ArcWheelModelPreintegration(const double &rl, const double &rr, const double &l, double nl, double nr)
: DiffWheelModel(rl, rr, l),
  n_l_(nl),
  n_r_(nr) { 
    Initialization();
}

void ArcWheelModelPreintegration::Initialization() {
    dthetaij_ = 0.; 
    dt_ = 0;
    alphaij_.fill(0.);
    Rji_.setIdentity();
    cov_.fill(0.);
    noise_ << n_l_ * n_l_, 0., 0., n_r_ * n_r_;
    dtheta_dp_.setZero();
    dalpha_drl_drr_dl_.setZero();
}

// preintegration
// dxj = dxi + v * dt * (sini * cos_dtheta + cosi * sin_dtheta - sini) / dtheta = dxi + (-sini * (1 - cos_dtheta) + cosi * sin_dtheta) * v * dt / dtheta
// dyj = dyi + v * dt * (sini * sin_dtheta - cosi * cos_dtheta + cosi) / dtheta = dyi + (sini * sin_dtheta + cosi * (1 - cos_dtheta)) * v * dt / dtheta
// [dxj; dyj] = [dxi; dyi] + [cosi, -sini; sini, cosi] * [sin_dtheta / dtheta; (1 - cos_dtheta) / dtheta] * v * dt
// Pj(0:1) - Pi(0:1) = Riw * alpha_ij(0:1); Pk(0:1) - Pj(0:1) = Rjw * alpha_jk(0:1);
// Pk(0:1) - Pi(0:1) = Riw * alpha_ij(0:1) + Rjw * alpha_jk(0:1) = Riw * (alpha_ij(0:1) + Rji * alpha_jk(0:1))
// so, Pk(0:1) - Pi(0:1) = Riw * sum(j = i...k-1)(Rji * alpha_(j-1, j)(0:1)) => Riw_inv * (Pk(0:1) - Pi(0:1)) = sum(j = i...k-1)(Rji * alpha_(j, j+1)(0:1))
// alpha_ik = sum(j = i...k-1)(Rji * alpha_(j, j+1));
// alpha_(j-1, j) = [[sin_dtheta / dtheta; (1 - cos_dtheta) / dtheta] * v * dt, dtheta];

// noise propagation
// v = 0.5 * ((wl + nl) * rl + (wr + nr) * rr) = c + 0.5 * rl * nl + 0.5 * rr * nr; dv/dnl = 0.5 * rl, dv/dnr = 0.5 * rr;
// w = ((wr + nr) * rr - (wl + nl) * rl) * l_inv = c + l_inv * rr * nr - l_inv * rl * nl; dw/dnl = -l_inv * rl, dw/dnr=l_inv * rr;
// dalpha0_dtheta = (-sin_dtheta / (dtheta)^2 + cos_dtheta / dtheta) * v * dt = (cos_dtheta - sin_dtheta/dtheta) * v * dt/dtheta
// dalpha0_dv = sin_dtheta * dt / dtheta;
// dalpha1_dtheta = (-(1 - cos_dtheta) / (dtheta)^2 + sin_dtheta / dtheta) * v * dt;
// dalpha1_dv = (1 - cos_dtheta) * dt / dtheta;
// dalpha2_dtheta = 1; dalpha2_dv = 0;
// dalpha/dn = dalpha/d[v;theta] * d[v; theta]/d[nl; nr];
// lim(cos_dtheta * dtheta - sin_dtheta)/(dtheta)^2 = lim(-sin_dtheta * dtheta + cos_dtheta - cos_dtheta)/(2. * dtheta)
// = lim(-sin_dtheta * / 2) = 0.;
// alpha(0:1) = sum(j = i...k)(Rji * alpha(j, j+1)(0:1)) = sum(j = i...k-1)(Rji * alpha(j, j+1)(0:1)) + Rki * alpha(k, k+1)(0:1)
// = alpha_last(0:1) + Rki * alpha(k, k+1)(0:1)
// dalpha01_dalpha_last = I + [021, 021, dRki * alpha(k, k+1)]
void ArcWheelModelPreintegration::InsertWheelVel(const Eigen::Vector2d &wheel_vel, double dt) {
    Eigen::Vector3d v = VelCalculation(wheel_vel);

    double dtheta = v(2) * dt;
    double ds = v(0) * dt;
    double dtheta_inv = 1. / dtheta;

    double sin_dtheta = sin(dtheta);
    double cos_dtheta = cos(dtheta);

    Eigen::Vector2d dalpha;
    Eigen::Matrix<double, 3, 2> ddalpha_dvdtheta; ddalpha_dvdtheta.fill(0.);
    if (fabs(dtheta) <= std::numeric_limits<double>::epsilon() * 1000) {
        dalpha << ds, 0.;
        ddalpha_dvdtheta << dt, 0., 0., 0.5 * ds, 0, 1;
    } else {
        dalpha << sin_dtheta * dtheta_inv * ds, (1. - cos_dtheta) * dtheta_inv * ds;
        ddalpha_dvdtheta << sin_dtheta * dt * dtheta_inv, (cos_dtheta  - sin_dtheta * dtheta_inv) * ds * dtheta_inv,
                           (1. - cos_dtheta) * dt * dtheta_inv, (-(1. - cos_dtheta) * dtheta_inv + sin_dtheta) * ds * dtheta_inv,
                           0, 1;
    }
    // wheel model measurement
    alphaij_.topRows(2) += Rji_ * dalpha;
    alphaij_(2) += dtheta;

    // noise propagation
    Eigen::Matrix2d dvdtheta_dn;
    dvdtheta_dn << 0.5 * r_l_, 0.5 * r_r_,
                   -l_inv_ * r_l_ * dt, l_inv_ * r_r_ * dt;
    Eigen::Matrix<double, 3, 2> dalpha_dn = ddalpha_dvdtheta * dvdtheta_dn;
    dalpha_dn.topLeftCorner(2, 2) = Rji_ * dalpha_dn.topLeftCorner(2, 2);

    Eigen::Matrix3d dalpha_dalpha_last; dalpha_dalpha_last.setIdentity();
    Eigen::Matrix2d dRji;
    double sin_theta = sin(dthetaij_);
    double cos_theta = cos(dthetaij_);
    dRji << -sin_theta, -cos_theta, cos_theta, -sin_theta;
    dalpha_dalpha_last.topRightCorner(2, 1) += dRji * dalpha;

    cov_ = dalpha_dalpha_last * cov_ * dalpha_dalpha_last.transpose() + dalpha_dn * noise_ * dalpha_dn.transpose();

    // jacobian of r_l_, r_r_, l_;
    // v = (wl * rl + wr * rr) * 0.5 => dv_drl_drr_dl = [0.5 * wl, 0.5 * wr, 0]
    // theta = (wr * rr - wl * rl) * dt / l => dtheta_drl_drr_dl = [-wl * l_inv * dt, wr * l_inv * dt, -(wr * rr - wl * rl) * dt * l_inv^2]
    // alphaik(0:1) = sum(j = i...k-1)(Rji * alpha_(j, j+1)(0:1)) = sum(j = i...k-1)(exp(sum(h=i...k-1)(dthetah)) * alpha_(j, j+1)(0:1))
    // dalphaik_dp = sum(j = i...k-1)(dRji_dp * alpha(j, j+1)(0:1) + Rji * dalpha(j, j+1)_dp)
    // = sum(j=i...k-1)(dexp(sum(h=i...k-1){thetah})_dsum(h=i..k-1){thetah} * alpha(j, j+1)(0:1) * dsum(h=i...k-1){thetah}_dp
    // + Rji * dalpha(j, j+1)(0:1)_dp; dp = [drl, drr, dl]
    // = dalpha(i,k-1)_dp + dexp(sum(h=i...k-1){thetah})_dsum(h=i...k-1){thetah} * alpha(j, j+1)(0:1) * dsum(h=i...k-1){thetah}_dp
    // + Rji * dalpha(j, j+1)_dp
    // dalpha(j, j+1)(0:1)_dp = dalpha(j, j+1)(0:1)_dvdtheta * dvdtheta_dp;
    // dalphaik_dp = sum(j=i...k){thetaj} = dalpha(i,k-1)_dp + dthetak_dp;
    Eigen::Matrix<double, 2, 3> dvdtheta_dp;
    dvdtheta_dp.row(0) << 0.5 * wheel_vel(0), 0.5 * wheel_vel(1), 0.;
    dvdtheta_dp.row(1) << -wheel_vel(0) * dt * l_inv_, wheel_vel(1) * dt * l_inv_, -(wheel_vel(1) * r_r_ - wheel_vel(0) * r_l_) *
                          dt * l_inv_ * l_inv_;
    dalpha_drl_drr_dl_.topLeftCorner(2, 3) += dRji * dalpha * dtheta_dp_ + Rji_ * ddalpha_dvdtheta.topLeftCorner(2, 2) * dvdtheta_dp;
    dalpha_drl_drr_dl_.row(2) += dvdtheta_dp.row(1);
    dtheta_dp_ += dvdtheta_dp.row(1);
    
    // rotation propagation
    Eigen::Matrix2d dR;
    dR << cos_dtheta, -sin_dtheta, sin_dtheta, cos_dtheta;
    Rji_ = Rji_ * dR;
    dthetaij_ += dtheta;
    dt_ += dt;
}

// residual and jacobian
// r(0:1) = Riw_inv * (Pk - Pi) - alpha_ik; r(2) = Pk(2) - Pi(2);
// dr(0:1)/dPk = Riw^t; dr(0:1)/dPi = -Riw^t; Pk - Pi = [dx, dy]; dr(2)/dPk(2) = 1; dr(2)/dPi(2) = -1;
// dr(0:1)/dthetai = d[cosi, sini; -sini, cosi] * [dx, dy]/dtheta = d[cosi * dx + sini * dy; -sini * dx + cosi * dy]/dtheta
// = [-sini * dx + cosi * dy; -cosi * dx - sini * dy] = [-sini, cosi; -cosi, -sini] * [dx; dy]
void ArcWheelModelPreintegration::Evaluate(const Eigen::Vector3d &Pj, const Eigen::Vector3d &Pi, Eigen::Vector3d &residual, 
                                           Eigen::Ref<Eigen::Matrix<double, 3, 6>> jacobian) const {
    double sin_theta = sin(Pi(2));
    double cos_theta = cos(Pi(2));

    Eigen::Matrix2d Riw;
    Riw << cos_theta, -sin_theta, sin_theta, cos_theta;

    // residual
    residual.topRows(2) = Riw.transpose() * (Pj - Pi).topRows(2);
    residual(2) = Pj(2) - Pi(2);

    residual -= alphaij_;

    jacobian.setIdentity();
    // r's jacobian of Pi
    Eigen::Matrix2d R;
    R << -sin_theta, cos_theta, -cos_theta, -sin_theta;
    jacobian.topLeftCorner(2, 2) = -Riw.transpose();
    jacobian.block<2, 1>(0, 2) = R * (Pj - Pi).topRows(2);
    jacobian(2, 2) = -1;

    // r's jacobian of Pj
    jacobian.block<2, 2>(0, 3) = Riw.transpose();
    jacobian(2, 5) = 1;
}

void ArcWheelModelPreintegration::EvaluateWithIntrinsics(const Eigen::Vector3d &Pj, const Eigen::Vector3d &Pi, Eigen::Vector3d &residual,
                                                         Eigen::Ref<Eigen::Matrix<double, 3, 9>> jacobian) const {
    Evaluate(Pj, Pi, residual, jacobian.topLeftCorner(3, 6));
    jacobian.topRightCorner(3, 3) = -dalpha_drl_drr_dl_;
}

void ArcWheelModelPreintegration::Evaluate(const double *Pj, const double *Pi, double *residual, double **jacobian) const {
    Eigen::Map<const Eigen::Vector3d> pose_Pj(Pj);
    Eigen::Map<const Eigen::Vector3d> pose_Pi(Pi);
    Eigen::Map<Eigen::Vector3d> res(residual);

    double sin_theta = sin(pose_Pi(2));
    double cos_theta = cos(pose_Pi(2));

    Eigen::Matrix2d Riw;
    Riw << cos_theta, -sin_theta, sin_theta, cos_theta;

    // residual
    res.topRows(2) = Riw.transpose() * (pose_Pj - pose_Pi).topRows(2);
    res(2) = pose_Pj(2) - pose_Pi(2);

    res -= alphaij_;

    if (jacobian != nullptr) {
        // jacobian of i's pose
        if (jacobian[0] != nullptr) {
            Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> jacobi(jacobian[0]);
            jacobi.setIdentity();
            // r's jacobian of Pi
            Eigen::Matrix2d R;
            R << -sin_theta, cos_theta, -cos_theta, -sin_theta;
            jacobi.topLeftCorner(2, 2) = -Riw.transpose();
            jacobi.block<2, 1>(0, 2) = R * (pose_Pj - pose_Pi).topRows(2);
            jacobi(2, 2) = -1;
        }

        // jacobian of j's pose
        if (jacobian[1] != nullptr) {
            Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> jacobj(jacobian[1]);
            jacobj.setIdentity();
            jacobj.block<2, 2>(0, 0) = Riw.transpose();
            jacobj(2, 2) = 1;
        }

        // jacobian of wheel odom intrinsic
        if (jacobian[2] != nullptr) {
            Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> jacob_intrinsic(jacobian[2]);
            jacob_intrinsic = -dalpha_drl_drr_dl_;
        }
    }
}

EulerWMPreintegration::EulerWMPreintegration(const double &rl, const double &rr, const double &l,
    double nl, double nr, bool calib_intrinsic_flag): DiffWheelModel(rl, rr, l), n_l_(nl), n_r_(nr),
    calib_intrinsic_flag_(calib_intrinsic_flag) {
    Initialization();
}

void EulerWMPreintegration::Initialization() {
    dt_ = 0;
    dthetaij_ = 0;
    alphaij_.setZero();
    noise_ << n_l_, 0., 0., n_r_;

    cov_.setZero();
    dalpha_drl_drr_dl_.setZero();
    Rji_.setIdentity();

    wheel_data_.clear();
}

void EulerWMPreintegration::LoadWheelDatas(std::vector<Eigen::Matrix<double, 1, 3>> &wheels) {
    double dt;
    Eigen::Vector2d wheel;
    for (int i = 1; i < wheels.size(); i++) {
        dt = wheels[i](0) - wheels[i - 1](0);
        wheel = 0.5 * (wheels[i].rightCols(2) + wheels[i - 1].rightCols(2)).transpose();
        LoadWheelData(wheel, dt);
    }
}

void EulerWMPreintegration::FusingWMwithoutPreintegration(std::shared_ptr<EulerWMPreintegration> wh_ptr) {
    if (wh_ptr == nullptr) return;
    wheel_data_.insert(wheel_data_.end(), wh_ptr->wheel_data_.begin(), wh_ptr->wheel_data_.end());
}

void EulerWMPreintegration::LoadWheelData(const Eigen::Vector2d &wheel, const double &dt) {
    if (dt < 1.e-05) return;
    double vl = wheel(0) * r_l_;
    double vr = wheel(1) * r_r_;
    double ds = 0.5 * (vl + vr) * dt;
    double dtheta = (vr - vl) * l_inv_ * dt;

    // preintegration measurement
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
    Eigen::Matrix2d ddsdtheta_ddwldwr;
    ddsdtheta_ddwldwr << 0.5 * r_l_ * dt, 0.5 * r_r_ * dt, -r_l_ * l_inv_ * dt, r_r_ * l_inv_ * dt;
    Eigen::Matrix<double, 3, 2> dalpha_ddsdtheta;
    Eigen::Vector2d ddrj_ddtheta(-0.5 * drj(1), 0.5 * drj(0));
    dalpha_ddsdtheta.topLeftCorner(2, 1) = Rji_ * Eigen::Vector2d(cos_half_dtheta, sin_half_dtheta);
    dalpha_ddsdtheta(2, 0) = 0.;
    dalpha_ddsdtheta.block<2, 1>(0, 1) = Rji_ * ddrj_ddtheta;
    dalpha_ddsdtheta(2, 1) = 1.;
    Eigen::Matrix<double, 3, 2> dalpha_dnoise = dalpha_ddsdtheta * ddsdtheta_ddwldwr;
    // alphaij = last_alphaij + [Rji * dr(j+1); dtheta(j+1)]
    // dalphaij_dlast_alphij = I + [d(Rji * dr(j+1))/dthetaij)] = I + [031, 031, [dRji_dthetaij * dr(j+1); 0]]
    Eigen::Matrix2d dRji_dtheta;
    dRji_dtheta << Rji_(0, 1), -Rji_(0, 0), Rji_(0, 0), Rji_(0, 1);
    Eigen::Matrix3d dalpha_dlast_alpha; dalpha_dlast_alpha.setIdentity();
    dalpha_dlast_alpha.topRightCorner(2, 1) += dRji_dtheta * drj;

    // y = A * x + B * u
    // E(y * y^t) - E(y) * E(y^t) = E((A * x + B * u) * (A * x + B * u)^t) = E(A * x * x^t * A^t) + E(B * x * x^t * B^t) - A * E(x) * E(x^t) * A^t
    // - B * E(u) * E(u^t) * B^t = A * cov(x) * A^t + B * cov(u) * B^t
    cov_ = dalpha_dlast_alpha * cov_ * dalpha_dlast_alpha.transpose() + dalpha_dnoise * noise_ * dalpha_dnoise.transpose() / dt;

    // rotation propagation
    Rji_ = Rji_ * dR;
    dthetaij_ += dtheta;
    dt_ += dt;

    Eigen::Vector3d wh;
    wh.topRows(2) = wheel;
    wh(2) = dt;
    wheel_data_.push_back(wh);
}

// txj = txi + Vc*dt*cos(i + 0.5*dtheta) = txi + ds * (cosi*cos(0.5*dtheta)-sini*sin(0.5*dtheta))
// = txi + [cosi, -sini]*[cos(0.5*dtheta); sin(0.5*dtheta)]*ds
// tyj = tyi + Vc*dt*sin(i + 0.5*dtheta) = tyi + ds * (sini*cos(0.5*dtheta) + cosi*sin(0.5*dtheta))
// = tyi + [sini, cosi]*[cos(0.5*dtheta); sin(0.5*dtheta)]*ds
//=> [txj; tyj] = [txi;tyi]+Riw*[cos(0.5*dtheta);sin(0.5*dtheta)]*ds
// thetaj = thetai + dtheta
// [txj;tyj;thetaj]=[txi;tyi;thetai]+[Riw*drj;dthetaj];drj=[cos(0.5*dthetaj);sin(0.5*dthetaj)]*dsj;
// [txk;tyk;thetak]=[txj;tyj;thetaj]+[Rjw*drk;dthetak];drk=[cos(0.5*dthetak);sin(0.5*dthetak)]*dsk
// so [txk;tyk;thetak]=[txi;tyi;thetai]+[Riw*drj+Riw*Rji*drk;dthetaj+dthetak]
// = [Riw*[drj+Rji*drk];dthetaj+dthetak]
// Generally [Riw_inv*[txk-txi;tyk-tyi];thetak-thetai]=[sum(j=i...k-1)(R(j+1)i*drj);sum(j=i+1...k)(dthetaj)]
// alphaij = [sum(j=i...k-1)(Rji*dr(j+1);sum(j=i+1...k)(dthetaj)]
// [Rji_inv*[txj-txi;tyj-tyi];thetaj-thetai]=[dr;dtheta]
void EulerWMPreintegration::InsertWheelVel(const Eigen::Vector2d &wheel_vel, const double &dt) {
    Eigen::Vector3d v = VelCalculation(wheel_vel);

    double dtheta = v(2) * dt;
    double ds = v(0) * dt;

    // preintegration measurement
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
    // ds = Vc * dt = 0.5 * (wl*rl + wr * rr) * dt; dds/dwl = 0.5 * rl * dt; dds/dwr = 0.5 * rr * dt;
    // dtheta = w * dt = (wr*rr - wl*rl)*l_inv*dt; ddtheta/dwl = -rl*l_inv*dt; ddtheta/dwr = rr*l_inv*dt;
    // alphaij=sum(j=i...k-1)[Rji*dr(j+1);dtheta(j+1)]
    // dalphaij/ds = sum(j=i...k-1)[d(Rji*dr(j+1))/ds; 0] = sum(j=i...k-1)[Rji*dr(j+1)/ds(j+1); 0]
    // dalphaij/ddtheta = sum(j=i...k-1)[d(Rji*dr(j+1))/ddtheta; 1] = [Rji*dr(j+1)/ddtheta; 1]
    // dRji/dtheta = [-sini, -cosi; cosi, -sini]; dr(j+1)/ds(j+1) = [cos(0.5*dtheta); sin(0.5*dtheta)];
    // dr(j+1)/ddtheta = [-0.5*sin(0.5*dtheta)*ds; 0.5*cos(0.5*dtheta)*ds]
    Eigen::Matrix2d ddsdtheta_ddwldwr;
    ddsdtheta_ddwldwr << 0.5 * r_l_ * dt, 0.5 * r_r_ * dt, -r_l_ * l_inv_ * dt, r_r_ * l_inv_ * dt;
    Eigen::Matrix<double, 3, 2> dalpha_ddsdtheta;
    Eigen::Vector2d ddrj_ddtheta(-0.5 * drj(1), 0.5 * drj(0));
    dalpha_ddsdtheta.topLeftCorner(2, 1) = Rji_ * Eigen::Vector2d(cos_half_dtheta, sin_half_dtheta);
    dalpha_ddsdtheta(2, 0) = 0.;
    dalpha_ddsdtheta.block<2, 1>(0, 1) = Rji_ * ddrj_ddtheta;
    dalpha_ddsdtheta(2, 1) = 1.;
    Eigen::Matrix<double, 3, 2> dalpha_dnoise = dalpha_ddsdtheta * ddsdtheta_ddwldwr;
    // alphaij = last_alphaij + [Rji * dr(j+1); dtheta(j+1)]
    // dalphaij_dlast_alphij = I + [d(Rji * dr(j+1))/dthetaij)] = I + [031, 031, [dRji_dthetaij * dr(j+1); 0]]
    Eigen::Matrix2d dRji_dtheta;
    dRji_dtheta << Rji_(0, 1), -Rji_(0, 0), Rji_(0, 0), Rji_(0, 1);
    Eigen::Matrix3d dalpha_dlast_alpha; dalpha_dlast_alpha.setIdentity();
    dalpha_dlast_alpha.topRightCorner(2, 1) += dRji_dtheta * drj;
    cov_ = dalpha_dlast_alpha * cov_ * dalpha_dlast_alpha.transpose() + dalpha_dnoise * noise_ * dalpha_dnoise.transpose();
    
    if (calib_intrinsic_flag_) {
        // jacobian of r_l, r_r, l_inv_;
        // dds/drl = 0.5 * wl * dt; dds/drr = 0.5 * wr * dt; dds/dl_inv = 0;
        // ddtheta/drl = -wl * l_inv * dt; ddtheta/drr = wr * l_inv * dt; ddtheta/dl_inv = (wr*rr-wl*rl)*dt;
        Eigen::Matrix<double, 2, 3> ddsdtheta_ddrldrrdl_inv;
        ddsdtheta_ddrldrrdl_inv.row(0) << 0.5 * wheel_vel(0) * dt, 0.5 * wheel_vel(1) * dt, 0;
        ddsdtheta_ddrldrrdl_inv.row(1) << -wheel_vel(0) * l_inv_ * dt, wheel_vel(1) * l_inv_ * dt,
                                    (wheel_vel(1) * r_r_ - wheel_vel(0) * r_l_) * dt;
        dalpha_drl_drr_dl_ = dalpha_dlast_alpha * dalpha_drl_drr_dl_ + dalpha_ddsdtheta * ddsdtheta_ddrldrrdl_inv;
    }

    // rotation propagation
    Rji_ = Rji_ * dR;
    dthetaij_ += dtheta;
    dt_ += dt;

    Eigen::Vector3d wh;
    wh.topRows(2) = wheel_vel;
    wh(2) = dt;
    wheel_data_.push_back(wh);
}

void EulerWMPreintegration::RePreintegration() {
    std::vector<Eigen::Vector3d> wh_data;

    wh_data.swap(wheel_data_);
    Initialization();
    for (size_t i = 0; i < wh_data.size(); i++) {
        LoadWheelData(wh_data[i].topRows(2), wh_data[i](2));
    }
}

void EulerWMPreintegration::PoseCalFromLastPose(const Eigen::Matrix3d &last_Rbw, const Eigen::Vector3d &last_tbw,
    const Eigen::Matrix3d &Rob, const Eigen::Vector3d &tob, Eigen::Matrix3d &Rbw, Eigen::Vector3d &tbw) {
    // Rojoi = Rob^t * Rbiw^t * Rbjw * Rob
    // tojoi = Rob^t * Rbiw^t * Rbjw * tob + Rob^t * (Rbiw^t * （tbjw - tbiw) - tob)

    // Eigen::Vector3d rvji = utils::DMmath::LogSO3(Rojoi);
    // double rp_an = rvji.topRows(2).norm();

    // Eigen::Vector3d Pj, Pi;
    // Pj << tojoi.topRows(2), rvji(2);
    // Pi.setZero();
    // tj - ti = alpha(0,1) => tojoi(0,1) = alpha(0, 1) + ti
    // rvj(2) - rvi(2) = alpha(2) => Rojoi = R(alpha(2))
    Eigen::Vector3d tojoi;
    tojoi << alphaij_.topRows(2), 0.;
    Eigen::Matrix3d Rojoi = utils::DMmath::RotationVector2Matrix(Eigen::Vector3d(0.0, 0.0, alphaij_(2)));
    
    // Rbjw = Rob * Rbiw * Rojoi * Rob^t
    Rbw = last_Rbw * Rob * Rojoi * Rob.transpose();

    // Rob^t * Rbiw^t * (tbjw - tbiw) = tojoi - Rojoi * Rob^t * tob + Rob^t * tob
    tbw = tojoi - Rojoi * Rob.transpose() * tob + Rob.transpose() * tob;
    tbw = last_Rbw * Rob * tbw + last_tbw;
}

void EulerWMPreintegration::PoseCalFromLastPose(const utils::Transform3d &last_Tbw, const utils::Transform3d &Tob,
                                                utils::Transform3d &Tbw) {
    // Rojoi = Rob^t * Rbiw^t * Rbjw * Rob
    // tojoi = Rob^t * Rbiw^t * Rbjw * tob + Rob^t * (Rbiw^t * （tbjw - tbiw) - tob)

    // Eigen::Vector3d rvji = utils::DMmath::LogSO3(Rojoi);
    // double rp_an = rvji.topRows(2).norm();

    // Eigen::Vector3d Pj, Pi;
    // Pj << tojoi.topRows(2), rvji(2);
    // Pi.setZero();
    // tj - ti = alpha(0,1) => tojoi(0,1) = alpha(0, 1) + ti
    // rvj(2) - rvi(2) = alpha(2) => Rojoi = R(alpha(2))
    utils::Transform3d Tojoi;
    Tojoi.t() << alphaij_.topRows(2), 0.;
    Tojoi.R() = utils::DMmath::RotationVector2Matrix(Eigen::Vector3d(0.0, 0.0, alphaij_(2)));    

    Tbw = last_Tbw * Tob * Tojoi * Tob.Inverse();
}

bool EulerWMPreintegration::Evaluate(const double *Pj, const double *Pi, double *residual, double **jacobians) const {
    Eigen::Map<const Eigen::Vector3d> pose_Pj(Pj);
    Eigen::Map<const Eigen::Vector3d> pose_Pi(Pi);
    Eigen::Map<Eigen::Vector3d> res(residual);

    double cos_theta = cos(pose_Pi(2));
    double sin_theta = sin(pose_Pi(2));
    Eigen::Matrix2d Riw;
    Riw << cos_theta, -sin_theta, sin_theta, cos_theta;
    res.topRows(2) = Riw.transpose() * (pose_Pj.topRows(2) - pose_Pi.topRows(2));
    res(2) = pose_Pj(2) - pose_Pi(2);

    res = res - alphaij_;
    
    if (jacobians != nullptr) {
        // jacobian of Pi
        if (jacobians[0] != nullptr) {
            Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> jacobi(jacobians[0]);
            jacobi.setIdentity();
            // r's jacobian of Pi
            Eigen::Matrix2d R;
            R << -sin_theta, cos_theta, -cos_theta, -sin_theta;
            jacobi.topLeftCorner(2, 2) = -Riw.transpose();
            jacobi.block<2, 1>(0, 2) = R * (pose_Pj - pose_Pi).topRows(2);
            jacobi(2, 2) = -1;
        }

        // jacobian of Pj
        if (jacobians[1] != nullptr) {
            Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> jacobj(jacobians[1]);
            jacobj.setIdentity();
            jacobj.block<2, 2>(0, 0) = Riw.transpose();
            jacobj(2, 2) = 1;
        }

        // jacobian of [rl, rr, l_inv]
        if (jacobians [2] != nullptr) {
            Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> jacob_intrinsic(jacobians[2]);
            jacob_intrinsic = -dalpha_drl_drr_dl_;
        }
    }
    return true;
}

WheelModel2DCalibrationFactor::WheelModel2DCalibrationFactor(const double &rl, const double &rr, const double &l,
    const Eigen::Vector3d &Twl, double nl, double nr)
: EulerWMPreintegration(rl, rr, l, nl, nr) {
    twl_ = Twl.topRows(2);
    double sinx = sin(Twl(2));
    double cosx = cos(Twl(2));
    Rwl_ << cosx, -sinx, sinx, cosx;
    Twl_ = Twl;
}

void WheelModel2DCalibrationFactor::LoadWheelData(const Eigen::Vector2d &wheel_vel, const double &dt) {
    InsertWheelVel(wheel_vel, dt);
    wheels_.emplace_back(wheel_vel, dt);
}

void WheelModel2DCalibrationFactor::RePreintegration(const double &rl, const double &rr, const double &l) {
    Initialization();
    r_r_ = rr;
    r_l_ = rl;
    l_inv_ = 1. / l;
    for (size_t i = 0; i < wheels_.size(); i++) {
        InsertWheelVel(wheels_[i].wheel_vel, wheels_[i].dt);
    }
}

void WheelModel2DCalibrationFactor::SetExtrinsic(const Eigen::Vector3d &Twl) {
    twl_ = Twl.topRows(2);
    double sinx = sin(Twl(2));
    double cosx = cos(Twl(2));
    Rwl_ << cosx, -sinx, sinx, cosx;
    Twl_ = Twl;
}

void WheelModel2DCalibrationFactor::SetIntrinsic(const double &rl, const double &rr, const double &l_inv) {
    r_l_ = rl;
    r_r_ = rr;
    l_inv_ = l_inv;
}

// evaluate input: [twjl0, theta_wjl0] [twil0, theta_wil0]
// calibevaluate input: Pj = [tljl0, theta_ljl0], Pi = [tlil0, theta_ljl0]
// [twjl0, theta_wjl0] = [tljl0, theta_ljl0] * [twl, theta_wl]
// [Rljl0, tljl0; 0, 1] * [Rwl, twl; 0, 1] = [Rljl0 * Rwl, Rljl0 * twl + tljl0; 0, 1]
// = [Rljl0 * twl + tljl0; theta_ljl0 + theta_wl]
bool WheelModel2DCalibrationFactor::CalibEvaluate(const double *Pj, const double *Pi, double *residual, double **jacobians) {
    Eigen::Map<const Eigen::Vector3d> pose_Pj(Pj);
    Eigen::Map<const Eigen::Vector3d> pose_Pi(Pi);

    // transform to wheel motion
    double sin_j = sin(pose_Pj(2)), cos_j = cos(pose_Pj(2));
    double sin_i = sin(pose_Pi(2)), cos_i = cos(pose_Pi(2));
    Eigen::Matrix2d Riw, Rjw;
    Riw << cos_i, -sin_i, sin_i, cos_i;
    Rjw << cos_j, -sin_j, sin_j, cos_j;
    Eigen::Vector3d Pj_w;
    Pj_w << Rjw * twl_ + pose_Pj.topRows(2), pose_Pj(2) + Twl_(2);
    Eigen::Vector3d Pi_w;
    Pi_w << Riw * twl_ + pose_Pi.topRows(2), pose_Pi(2) + Twl_(2);

    // evaluation of residual
    Eigen::Matrix<double, 3, 3, Eigen::RowMajor> dr_dPjw, dr_dPiw, dr_dintrinsic;
    double *jacobs[3];
    jacobs[0] = dr_dPiw.data();
    jacobs[1] = dr_dPjw.data();
    jacobs[2] = dr_dintrinsic.data();
    bool flag = Evaluate(Pj_w.data(), Pi_w.data(), residual, jacobs);

    if (jacobians != nullptr) {
        // jacobian of Pi
        if (jacobians[0] != nullptr) {
            Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> jacob_Pi(jacobians[0]);
            Eigen::Matrix3d dPiw_dPi;
            dPiw_dPi.setZero();
            dPiw_dPi.topLeftCorner(2, 2).setIdentity();
            dPiw_dPi.rightCols(1) << -sin_i * twl_(0) - cos_i * twl_(1), cos_i * twl_(0) - sin_i * twl_(1), 1.;
            jacob_Pi = dr_dPiw * dPiw_dPi;
        }

        // jacobian of Pj
        if (jacobians[1] != nullptr) {
            Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> jacob_Pj(jacobians[1]);
            Eigen::Matrix3d dPjw_dPj;
            dPjw_dPj.setZero();
            dPjw_dPj.topLeftCorner(2, 2).setIdentity();
            dPjw_dPj.rightCols(1) << -sin_j * twl_(0) - cos_j * twl_(1), cos_j * twl_(0) - sin_j * twl_(1), 1.;
            jacob_Pj = dr_dPjw * dPjw_dPj;
        }

        // jacobian of r_l_, r_r_, l_inv
        if (jacobians[2] != nullptr) {
            Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> jacob_intrinsic(jacobians[2]);
            jacob_intrinsic = dr_dintrinsic;
        }

        // jacobian of extrinsic
        if (jacobians[3] != nullptr) {
            // dPjw01_dtwl = Rljl0, dPjw2_dtheta_wl = 1.;
            // dPiw01_dtwl = Rlil0, dPiw2_dtheta_wl = 1.;
            Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> jacob_extrinsic(jacobians[3]);
            Eigen::Matrix3d dPjw_dTwl, dPiw_dTwl;
            dPjw_dTwl.setZero();
            dPjw_dTwl.topLeftCorner(2, 2) = Rjw;
            dPjw_dTwl(2, 2) = 1.;

            dPiw_dTwl.setZero();
            dPiw_dTwl.topLeftCorner(2, 2) = Riw;
            dPiw_dTwl(2, 2) = 1.;
            jacob_extrinsic = dr_dPjw * dPjw_dTwl + dr_dPiw * dPiw_dTwl;
        }
    }
    return flag;
}

bool DiffWheelModel3DFactor::Evalute(const Eigen::Matrix3d &Rbiw, const Eigen::Vector3d &tbiw, const Eigen::Matrix3d &Rbjw, const Eigen::Vector3d &tbjw,
                                     const Eigen::Matrix3d &Rob, Eigen::Vector3d &tob, std::shared_ptr<EulerWMPreintegration> &wheel_odom) {
    // [Rbiw^t, -Rbiw^t * tbiw] * [Rbjw, tbjw]
    // = [Rbiw^t * Rbjw, Rbiw^t * tbjw - Rbiw^t * tbiw]
    // [Rob^t, -Rob^t * tob] * [Rbiw^t * Rbjw, Rbiw^t * (tbjw - tbiw)]
    // = [Rob^t * Rbiw^t * Rbjw, Rob^t * (Rbiw^t * (tbjw - tbiw) - tob)]
    // * [Rob, tob] = [Rob^t * Rbiw^t * Rbjw * Rob, Rob^t * Rbiw^t * Rbjw * tob + Rob^t * (Rbiw^t * (tbjw - tbiw) - tob)]
    // Rojoi * exp(drv1) = Rob^t * Rbiw^t * Rbjw * exp(drv0) * Rob = Rojoi * exp(Rob^t * drvo)
    // drv1 = Rob^t * drvo
    // tojoi + dt1 = Rob^t * Rbiw^t * Rbjw * tob + Rob^t * (Rbiw^t * （tbjw + dt0 - tbiw) - tob) = tojoi + Rob^t * Rbiw^t * dt0
    // dt1 = Rob^t * Rbiw^t * dt0
    Eigen::Matrix3d Rojoi;
    Eigen::Vector3d tojoi;
    utils::DMmath::TransformMultiply(Rbiw.transpose(), -Rbiw.transpose() * tbiw, Rbjw, tbjw, Rojoi, tojoi);
    utils::DMmath::TransformMultiply(Rob.transpose(), -Rob.transpose() * tob, Rojoi, tojoi, Rojoi, tojoi);
    utils::DMmath::TransformMultiply(Rojoi, tojoi, Rob, tob, Rojoi, tojoi);

    Eigen::Vector3d rvji = utils::DMmath::LogSO3(Rojoi);
    double rp_an = rvji.topRows(2).norm();

    Eigen::Vector3d Pj, Pi;
    Pj << tojoi.topRows(2), rvji(2);
    Pi.setZero();
    Eigen::Matrix<double, 3, 3, Eigen::RowMajor> Jpj;
    double *jacobian[3];
    jacobian[0] = nullptr;
    jacobian[1] = Jpj.data();
    jacobian[2] = nullptr;
    wheel_odom->Evaluate(Pj.data(), Pi.data(), residual.data(), jacobian);

    // Jj = [dr / drojoi, dr / dtojoi]
    Jj.setZero();
    Jj.col(2) = Jpj.col(2);
    Jj.middleCols(3, 2) = Jpj.leftCols(2);

    // Jj = [dr / drojoi, dr / dtojoi] * [drojoi / drbjw + dtojoi / drbjw, dtojoi / dtbjw]
    // tojoi + dt1 = Rob^t * Rbiw^t * Rbjw * tob + Rob^t * (Rbiw^t * （tbjw + dt0 - tbiw) - tob)
    // dt1 = Rob^t * Rbiw^t * dt0
    // tojoi + dt1 = Rob^t * Rbiw^t * Rbjw * exp(drv) * tob + dtojoi
    // tojoi + dt1 = Rob^t * Rbiw^t * Rbjw * (I + drv^) * tob + dtojoi = tojoi + Rob^t * Rbiw^t * Rbjw * drv^ * tob
    // dt1 = -Rob^t * Rbiw^t * Rbjw * tob^ * drv
    Jj.leftCols(3) = Jj.leftCols(3) * Rob.transpose() - Jj.rightCols(3) * Rojoi * Rob.transpose() * utils::DMmath::SkewMatrix(tob);
    Jj.rightCols(3) = Jj.rightCols(3) * Rob.transpose() * Rbiw.transpose();
    return true;
}

} // namespace Calibration