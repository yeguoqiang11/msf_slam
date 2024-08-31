#include <iostream>
#include <memory>

#include "gtsam/navigation/ImuFactor.h"
#include "gtsam/navigation/CombinedImuFactor.h"

#include "slam/Attitude.h"
#include "slam/Imu.h"
#include "slam/Sensors.h"
#include "utils/logs.h"
#include "utils/DMmath.h"

int main(int argc, char **argv) {
    std::shared_ptr<slam::Sensors> sensors_ptr = std::make_shared<slam::Sensors>();
    std::shared_ptr<gtsam::PreintegratedImuMeasurements::Params> imu_ptr;

    sensors_ptr->imu.Rib.setIdentity();
    sensors_ptr->imu.tib.setZero();
    sensors_ptr->imu.na = Eigen::Vector3d(0.0001, 0.001, 0.001);
    sensors_ptr->imu.ng = Eigen::Vector3d(0.001, 0.001, 0.001);
    sensors_ptr->imu.nba = Eigen::Vector3d(0.0023, 0.0032, 0.0021);
    sensors_ptr->imu.nbg = Eigen::Vector3d(0.0003, 0.0003, 0.0003);
    sensors_ptr->imu.ba = Eigen::Vector3d(0, 0, 0);
    sensors_ptr->imu.bg = Eigen::Vector3d(0, 0, 0);
    gtsam::imuBias::ConstantBias prior_imubias(sensors_ptr->imu.ba, sensors_ptr->imu.bg);
    auto p = gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedD(0.0);
    p->accelerometerCovariance = sensors_ptr->imu.na.asDiagonal();
    p->gyroscopeCovariance = sensors_ptr->imu.ng.asDiagonal();
    p->biasAccCovariance = sensors_ptr->imu.nba.asDiagonal();
    p->biasOmegaCovariance = sensors_ptr->imu.nbg.asDiagonal();
    p->integrationCovariance = Eigen::Matrix3d::Zero();
    std::shared_ptr<gtsam::PreintegrationType> preintegrated =
        std::make_shared<gtsam::PreintegratedImuMeasurements>(p, prior_imubias);


    slam::Imu imu(sensors_ptr);
    slam::MEKFAttitude atti(sensors_ptr, Eigen::Vector3d(0., 0., -9.8));

    Eigen::Matrix3d Rji; Rji.setIdentity();
    Eigen::Vector3d t; t.setZero();

    std::vector<Eigen::Vector3d> gyros, accs;
    gyros.push_back(Eigen::Vector3d(90., 0, 0) * DEG2RAD);
    gyros.push_back(Eigen::Vector3d(0, 80, 0) * DEG2RAD);
    gyros.push_back(Eigen::Vector3d(0, 0, 100) * DEG2RAD);

    accs.push_back(Eigen::Vector3d(0.05, 0, 0));
    accs.push_back(Eigen::Vector3d(0, 0.08, 0.));
    accs.push_back(Eigen::Vector3d(0, 0, -0.03));
    
    Eigen::Matrix3d Riw; Riw.setIdentity();
    Eigen::Vector3d Pw, Vw; Pw.setZero(); Vw.setZero();
    Eigen::Vector3d ba = sensors_ptr->imu.ba;
    Eigen::Vector3d bg = sensors_ptr->imu.bg;
    Eigen::Vector3d Gravity(0, 0, 9.8);

    Eigen::Matrix3d Riw0 = utils::DMmath::RotationVector2Matrix(Eigen::Vector3d(0.2, 0.1, -0.3));
    Eigen::Vector3d Vw0 = Eigen::Vector3d(0.1, 2., -0.3);
    Eigen::Vector3d Pw0 = Eigen::Vector3d(0.2, 0.5, -0.3);
    Riw = Riw0;
    Vw = Vw0;
    Pw = Pw0;
    Eigen::Vector3d G = Gravity;
    double dt = 0.01;
    double timestamp = 0.;
    for (int i = 0; i < 3; i++) {
        Eigen::Vector3d gt = gyros[i] + bg;
        Eigen::Vector3d a = accs[i] + ba;

        for (int j = 0; j < 100; j++) {
            // std::cout << "---------------" << std::endl;
            // if ( i == 0 && j < 21) {
            //     gt.setZero();
            //     Riw = atti.Rbw();
            //     Riw0 = atti.Rbw();
            //     std::cout << "Riw: " << Riw << std::endl;
            // } else {
                // gt = gyros[i] + bg;
            // }

            Eigen::Vector3d at = a + G;
            imu.StateUpdate(at, gt, dt);
            preintegrated->integrateMeasurement(at, gt, dt);
            Eigen::Matrix3d Rjw;
            Eigen::Vector3d Pwj, Vwj;
            imu.ImuStatePrediction(Riw0, Vw0, Pw0, -Gravity, Rjw, Vwj, Pwj);
            // if (j == 2) {
            //     std::cout << "cov: \n" << imu.Cov() << std::endl;
            //     exit(0);
            // }
            
            G = Rjw.transpose() * Riw0 * Gravity;
            at = a + G;

            Eigen::Matrix3d dRji = utils::DMmath::RotationVector2Matrix(gt * dt);
            Pw = Pw + Vw * dt + 0.5 * (Riw * at - Gravity) * dt * dt;
            Vw = Vw + (Riw * at - Gravity) * dt;
            Riw = Riw * dRji;

            Eigen::Matrix<double, 1, 7> imu_data;
            imu_data << timestamp, at.transpose(), gt.transpose();
            // std::cout << "imu: " << imu_data << std::endl;
            // Eigen::Vector2d attitude = atti.LoadImu(imu_data);
            // std::cout << "Mekf Rbw: \n" << atti.Rbw() << std::endl;
            // std::cout << "gt Rbw: \n" << Riw << std::endl;
            // std::cout << "gt G: " << (Riw * at).transpose() << std::endl;
            // std::cout << j << "th mekf G: " << (atti.Rbw() * at).transpose() << std::endl;
            timestamp += dt;
            // if (attitude.norm() > 0) exit(0);

            // if (j == 21) exit(0);
        }
        std::cout << "gt Rji: " << preintegrated->deltaRij() << std::endl;
        std::cout << "Rji: " << imu.Rji() << std::endl;
        std::cout << "gt Pji: " << preintegrated->deltaPij().transpose() << std::endl;
        std::cout << "Pji: " << imu.Pij().transpose() << std::endl;
        std::cout << "gt vji: " << preintegrated->deltaVij().transpose() << std::endl;
        std::cout << "Vji: " << imu.Vij().transpose() << std::endl;
        std::cout << "cov: \n" << imu.Cov() << std::endl;
        // preintegrated->computeErrorAndJacobians()
        preintegrated->print();
        std::cout << "dt: " << timestamp << std::endl;
        // if (i < 2) continue;
        // slam::ImuFactor factor;
        // Eigen::Matrix3d dR = utils::DMmath::RotationVector2Matrix(Eigen::Vector3d(0.001, 0., 0.));
        // // Riw0 = Riw0 * dR;
        // // Pw += Eigen::Vector3d(0.00, 0.001, 0.00);
        // Eigen::Vector3d ba0 = Eigen::Vector3d(0.000, 0.000, 0.);
        // Eigen::Vector3d ba = Eigen::Vector3d(0.000, 0.000, 0.);
        // Eigen::Vector3d bg0 = Eigen::Vector3d(0.000, 0.0, 0.000);
        // Eigen::Vector3d bg = Eigen::Vector3d(0.000, 0., 0.);
        // factor.Evaluate(Riw0, Riw, Pw0, Pw, Vw0, Vw, ba0, bg0, ba, bg, Gravity, imu);
        // std::cout << "residual: \n" << factor.residual.transpose() << std::endl;
        // std::cout << "jacobi: \n" << factor.Ji << std::endl;
        // std::cout << "jacobj: \n" << factor.Jj << std::endl;
        // std::cout << "cov: \n" << imu.Cov() << std::endl;
        exit(0);
    }

}