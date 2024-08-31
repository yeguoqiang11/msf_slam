#include "slam/Attitude.h" 

namespace slam {

MEKFAttitude::MEKFAttitude(std::shared_ptr<Sensors> sensor_ptr, Eigen::Vector3d gravity)
    : AttitudeBase(sensor_ptr, gravity),
      Gw_ptr_(nullptr) {
    bg_.setZero();
    ba_.setZero();
    Rin_.setIdentity();

    init_num_ = 0;
    init_gyro_cov_.fill(0.);
    init_acc_cov_.fill(0.);

    init_gyro_mean_.fill(0.);
    init_acc_mean_.fill(0.);

    is_initialized_ = false;
    last_imu_.fill(-1.);

    Q = Eigen::Vector3d(0.001, 0.001, 0.001).asDiagonal();
    R = Eigen::Vector3d(100., 100., 100.).asDiagonal();
    P = Eigen::Vector3d(1., 1., 1.).asDiagonal();
}

Eigen::Vector2d MEKFAttitude::LoadImus(std::vector<Eigen::Matrix<double, 1, 7>> imus) {
    Eigen::Vector2d atti(0., 0.);
    // transform imu from imu coordinate to robot coordinate
    for (size_t i = 0; i < imus.size(); i++) {
        Eigen::Matrix<double, 1, 7> imu = imus[i];
        // imu.middleCols(1, 3) = imu.middleCols(1, 3) * Rib_.transpose();
        // imu.rightCols(3) = imu.rightCols(3) * Rib_.transpose();
        atti = LoadImu(imu);
    }
    return atti;
}

Eigen::Vector2d MEKFAttitude::LoadImu(Eigen::Matrix<double, 1, 7> imu) {
    Eigen::Vector2d atti(0., 0.);
    if (!is_initialized_) {
        if(Initialization(imu)) {
            Eigen::Vector3d euler = utils::DMmath::RotationMatrix2EulerAngleZXY(Rin_);
            atti = euler.topRows(2);
            if (Gw_ptr_ == nullptr) {
                Gw_ptr_ = std::make_shared<Eigen::Vector3d>(-Rib_ * init_acc_mean_);
            }
        }
    } else {
        if (Gw_ptr_ == nullptr) {
            if (IsStatic(imu)) {
                Gw_ptr_ = std::make_shared<Eigen::Vector3d>(-Rib_ * init_acc_mean_);
            }
        }

        InsertImu(imu);
        Eigen::Vector3d euler = utils::DMmath::RotationMatrix2EulerAngle(Rin_);
        atti = euler.topRows(2);
    }
    last_imu_ = imu;
    return atti;
}

bool MEKFAttitude::IsStatic(Eigen::Matrix<double, 1, 7> imu) {
    double s = 1.0 / (init_num_ + 1.0);
    // acc mean and covariance
    Eigen::Vector3d tmp_acc = imu.block<1, 3>(0, 1).transpose() - init_acc_mean_;
    init_acc_mean_ += tmp_acc * s;
    init_acc_cov_ *= init_num_;
    init_acc_cov_ += tmp_acc * tmp_acc.transpose() * init_num_ * s;
    init_acc_cov_ *= s;

    // gyro mean and covariance
    Eigen::Vector3d tmp_gyro = imu.block<1, 3>(0, 4).transpose() - init_gyro_mean_;
    init_gyro_mean_ += tmp_gyro * s;
    init_gyro_cov_ *= init_num_;
    init_gyro_cov_ += tmp_gyro * tmp_gyro.transpose() * init_num_ * s;
    init_gyro_cov_ *= s;

    init_num_++;
    init_num_ = init_num_ > 10? 10 : init_num_;

    double acc_cov_norm = init_acc_cov_.trace();
    double gyro_cov_norm = init_gyro_cov_.trace();

    std::cout << "acc gyro cov norm: " << acc_cov_norm << ";" << gyro_cov_norm << std::endl;
    if (acc_cov_norm < 0.05 && gyro_cov_norm < 3 * 3.14 / 180. && init_num_ >= 10) {
        return true;
    } else {
        return false;
    }
}

bool MEKFAttitude::Initialization(Eigen::Matrix<double, 1, 7> imu) {
    if (IsStatic(imu)) {
        bg_ = init_gyro_mean_;
        is_initialized_ = true;
        Rin_ = EstimateRotationFromAcc(init_acc_mean_);
        std::cout << "attitude init succeed!!!" << std::endl;
        std::cout << "acc mean: " << init_acc_mean_.transpose() << std::endl;
        std::cout << "init attitude matrix Rbw: \n" << Rin_.transpose() << std::endl;
        std::cout << "Gravity: " << (Rin_ * imu.block<1, 3>(0, 1).transpose()).transpose() << std::endl;
        return true;
    } else {
        return false;
    }
}

// Gw = Rbw * Gb
// a = Gb X Gw
// ||a|| = ||Gw|| * ||Gb|| * sin(theta)
// b = ||Gw|| * ||Gb|| * cos(theta)
// tan(theta) = ||a|| / ||b||
Eigen::Matrix3d MEKFAttitude::EstimateRotationFromAcc(Eigen::Vector3d acc) {
    Eigen::Vector3d a = acc.cross(Gravity_);
    double b = acc.dot(Gravity_);
    double theta = atan2(a.norm(), b);
    if (fabs(theta - M_PI) <= std::numeric_limits<double>::epsilon() * 1000) {
        a << 1.0, 0, 0;
    }
    a /= a.norm();
    a *= theta;
    Eigen::Matrix3d Rin = utils::DMmath::RotationVector2Matrix(a);
    return Rin;
}

void MEKFAttitude::InsertImu(Eigen::Matrix<double, 1, 7> imu) {
    double dt = imu(0) - last_imu_(0);
    Eigen::Vector3d gyro = 0.5 * (imu.block<1, 3>(0, 4) + last_imu_.block<1, 3>(0, 4)).transpose();
    gyro -= bg_;
    // Eigen::Vector3d acc = 0.5 * (imu.block<1, 3>(0, 1) + last_imu_.block<1, 3>(0, 1));
    Eigen::Vector3d acc = imu.block<1, 3>(0, 1).transpose();
    acc -= ba_;

    // rotation matrix prediction
    Eigen::Matrix3d dR = utils::DMmath::RotationVector2Matrix(gyro * dt);
    Rin_ = Rin_ * dR;

    // covariance update of prediction
    // exp(dtheta0) * R1 = exp(dtheta1) * R0 * exp(g * dt + w * dt)
    // (I + dtheta0^) * R1 = (I + dtheta1^) * R0 * exp(Jl * w * dt) * exp(g * dt); Jl = Jl(g * dt)
    // (I + dtheta0^) * R0 * exp(g * dt) = (I + dtheta1^) * R0 * exp(Jl * w * dt) * exp(g * dt)
    // (I + dtheta0^) = (I + dtheta1^) * R0 * exp(Jl * w * dt) * R0^t
    // (I + dtheta0^) = (I + dtheta1^) * exp(R0 * Jl * w * dt)
    // (I + dtheta0^) = (I + dtheta1^) * (I + (R0 * Jl * w * dt)^)
    // dtheta0^ = dtheta1^ + (R0 * Jl * w * dt)^
    // dtheta0 = dtheta1 + R0 * Jl * dt * w
    Eigen::Matrix3d Jl = utils::DMmath::LeftJacobian(gyro * dt);
    Jl.setIdentity(); // to keep unobservability of yaw axis
    P = P + Rin_ * Jl * Q * Jl.transpose() * Rin_.transpose();

    // error state updation
    // Gb = Rbw^t * Gw + v
    // Gb = (exp(dtheta) * Rbw_)^t * Gw + v
    // Gb = Rbw_^t * (I - dtheta) * Gw + v
    // Gb = Rbw_^t * Gw + Rbw^t * Gw^ * dtheta + v 
    // Gb = Rbw^t * Gw^ * dtheta + c + v; c = Rbw^t * Gw
    // K = P * H^t * (H * P * H^t + R)_inv = [(H * P * H^t + R)_inv * H^t * P]^t
    Eigen::Matrix3d H = Rin_.transpose() * utils::DMmath::SkewMatrix(Gravity_);
    Eigen::Matrix3d K = ((H * P * H.transpose() + R).lu().solve(H * P)).transpose();
    Eigen::Vector3d dtheta = K * (acc - Rin_.transpose() * Gravity_);

    // rotation update
    // LOG(INFO) << "acc recity attitude dtheta: " << dtheta.transpose();
    Rin_ = utils::DMmath::RotationVector2Matrix(dtheta) * Rin_;

    // covariance update
    P = P - K * H * P;
}

Eigen::Vector3d MEKFAttitude::EulerAngle() const {
    return utils::DMmath::RotationMatrix2EulerAngle(Rin_);
}
} // namespace slam