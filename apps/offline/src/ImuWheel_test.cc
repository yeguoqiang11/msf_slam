#include <iostream>
#include <memory>

#include "slam/ImuWheel.h"
#include "slam/WheelModel.h"
#include "utils/DMmath.h"

using namespace slam;
int main(int argc, char** argv) {
    Eigen::Vector3d noise;
    noise << 0.0001, 0.0001, 0.000;
    double rl = 0.035;
    double rr = 0.035;
    double bg = 0.0;
    std::shared_ptr<GyroWheelModelPreintegration> wheel_ptr
      = std::make_shared<GyroWheelModelPreintegration>(rl, rr, bg, noise);
    std::shared_ptr<GyroWheelModelPreintegration> dwheel_ptr
      = std::make_shared<GyroWheelModelPreintegration>(rl, rr, bg, noise);

    Eigen::Matrix3d Rob = utils::DMmath::RotationVector2Matrix(Eigen::Vector3d(0.5 * M_PI, 0., 0.));
    Eigen::Vector3d tob(0.0012, -0.0035, 0.023);
    
    double t = 0.;
    int w_n = 3;
    int g_n = 5;
    double dt = 0.005;
    double omega_wheell = 0.1 / rl, omega_wheelr = 0.14 / rr;
    double omega_gyr = 0.12;
    Eigen::Vector3d pose(0., 0., 0.);
    double angle = 0.0;
    Eigen::Vector3d last_pose(0., 0., 0.);
    for (int n = 0; n < 10; n++) {
        std::vector<Eigen::RowVector3d> wheels;
        std::vector<Eigen::RowVector2d> gs;
        dwheel_ptr->Initialization();
        for (int i = 0; i < w_n; i++) {
            Eigen::Vector3d wheel;
            if (i == 0) {
                wheel << t, omega_wheell, omega_wheelr;
            } else {
                wheel << t + 0.002, omega_wheell, omega_wheelr;
            }
            wheels.push_back(wheel);

            double dtheta = 0.;
            for (int j = 0; j < g_n; j++) {
                Eigen::Vector2d gyro;
                gyro << t, omega_gyr;
                gs.push_back(gyro);
                dtheta += dt * omega_gyr;
                t += dt;
            }
            // std::cout << "theta: " << dtheta << std::endl;

            if (i == w_n - 1) {
                wheels.emplace_back(t, omega_wheell, omega_wheelr);
                gs.emplace_back(t, omega_gyr);
            }
        }

        wheel_ptr->InsertDatas(gs, wheels);
        wheel_ptr->RePreintegration();
        dwheel_ptr->InsertDatas(gs, wheels);
        std::cout << "alpha: " << wheel_ptr->Alpha().transpose() << std::endl;
        // std::cout << "cov: \n" << wheel_ptr->Cov() << std::endl;
        std::cout << "dt: " << wheel_ptr->Dt() << std::endl;
        // pose calculate;
        Eigen::Matrix3d last_Rbw = utils::DMmath::RotationVector2Matrix(Eigen::Vector3d(0., 0., last_pose(2)));
        Eigen::Vector3d last_tbw(last_pose(0), last_pose(1), 0.);

        Eigen::Matrix3d R;
        Eigen::Vector3d t;
        utils::DMmath::TransformMultiply(last_Rbw, last_tbw, Rob.transpose(), -Rob.transpose() * tob, R, t);
        utils::DMmath::TransformMultiply(Rob, tob, R, t, last_Rbw, last_tbw);

        Eigen::Matrix3d Rbw;
        Eigen::Vector3d tbw;
        dwheel_ptr->PoseCalFromLastPose(last_Rbw, last_tbw, Rob, tob, Rbw, tbw);

        Eigen::Matrix3d Rbwj = Rbw;
        Eigen::Vector3d tbwj = tbw;
        utils::DMmath::TransformMultiply(Rbw, tbw, Rob, tob, R, t);
        utils::DMmath::TransformMultiply(Rob.transpose(), -Rob.transpose() * tob, R, t, Rbw, tbw);

        Eigen::Vector3d rv = utils::DMmath::LogSO3(Rbw).transpose();
        std::cout << "tbw: " << tbw.topRows(2).transpose() << "," << rv(2) << std::endl;

        slam::GyroWheel3DFactor factor;
        // Rbwj = Rbwj * utils::DMmath::RotationVector2Matrix(Eigen::Vector3d(0.001, 0.000, 0.000));
        tbwj(2) += 0.001;
        // last_tbw(2) += 0.001;
        // factor.Evaluate(last_Rbw, last_tbw, Rbwj, tbwj, Rob, tob, dwheel_ptr);
        std::cout << "residual: " << factor.residual.transpose() << std::endl;
        std::cout << "jacob: \n" << factor.Jj << std::endl;

        // pose update
        for (size_t k = 1; k < gs.size(); k++) {
            angle += (gs[k](0) - gs[k - 1](0)) * gs[k](1);
        }
        double x = sin(angle);
        double y = cos(angle);
        std::cout << "gt: " << x << "," << -(y - 1.) << "," << angle << std::endl;
        last_pose << x, -(y - 1.), angle;
        Eigen::Vector3d alpha = wheel_ptr->Alpha();
        last_pose = alpha;

        std::cout << "--------------------" << std::endl;
        if (n == 1)
            exit(0);
    }
}