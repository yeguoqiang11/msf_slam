#include <boost/program_options.hpp>

#include <eigen3/Eigen/Core>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Key.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include "utils/DMmath.h"
#include "utils/logs.h"
#include "slam/Map.h"
#include "slam/VoxelMap.h"

using gtsam::symbol_shorthand::X; // pose3d (t, R)

class OdomFactor: public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3> {
  public:
    OdomFactor(const Eigen::Matrix3d &Rij, const Eigen::Vector3d &tij,
                        const gtsam::SharedNoiseModel& noise_model, gtsam::Key keyi, gtsam::Key keyj)
      : Rij_(Rij), tij_(tij), NoiseModelFactor2(noise_model, keyi, keyj) {}
    gtsam::Vector evaluateError(const gtsam::Pose3 &posei, const gtsam::Pose3 &posej, boost::optional<gtsam::Matrix&> Hi = boost::none,
                                boost::optional<gtsam::Matrix&> Hj = boost::none) const {
        Eigen::Matrix3d Rij_hat = posej.rotation().matrix().transpose() * posei.rotation().matrix();
        Eigen::Vector3d tij_hat = posej.rotation().matrix().transpose() * (posei.translation() - posej.translation());

        Eigen::Matrix3d dR = Rij_hat * Rij_.transpose();
        Eigen::Vector3d drv = utils::DMmath::LogSO3(dR);
        Eigen::Vector3d dt = tij_hat - tij_;

        Eigen::Matrix<double, 6, 1> r;
        r << drv, dt;
        // [R, t; 0, 1] * [dR, dt; 0, 1] = [R * dR, R * dt + t]
        // [R * dR, R * dt + t]^-1 = [dR^t * R^t,  -dR^t * R^t * (R * dt + t)]
        // = [dR^t * R^t, -R^t * R * dt - dR^t * R^t * t]
        // = [dR^t * R^t, -dt - R^t * t]
        // dtj = -dt - R^t * (ti + dti)
        // dtj = -dt - R^t * ti - R^t * dti => -R^t * ti - dt - R^t * dti
        // tx = R * dt + ti
        // tx = ti + dti => dti = R * dt

        if (Hi) {
            Eigen::Matrix<double, 6, 6> dr_dTi; dr_dTi.setZero();
            dr_dTi.topLeftCorner(3, 3) = Rij_;
            dr_dTi.bottomRightCorner(3, 3) = posej.rotation().matrix().transpose() * posei.rotation().matrix();
            (*Hi) = dr_dTi;
        }
        if (Hj) {
            Eigen::Matrix<double, 6, 6> dr_dTj; dr_dTj.setZero();
            dr_dTj.topLeftCorner(3, 3) = -dR.transpose();
            dr_dTj.bottomLeftCorner(3, 3) = utils::DMmath::SkewMatrix(tij_hat);
            dr_dTj.bottomRightCorner(3, 3) = -Eigen::Matrix3d::Identity();
            (*Hj) = dr_dTj;
        }
         
        Eigen::Matrix<double, 6, 6> H = (*Hj).transpose() * (*Hj);
        Eigen::Matrix<double, 6, 1> b = (*Hj).transpose() * r;
        Eigen::Matrix<double, 6, 1> dx = (H + Eigen::Matrix<double, 6, 6>::Identity() * 1.0e-06).ldlt().solve(-b);

        return r;
    }
  private:
    Eigen::Matrix3d Rij_;
    Eigen::Vector3d tij_;
};

int main(int argc, char** argv) {
    Eigen::Matrix3d Ri, Rj;
    Ri = utils::DMmath::RotationVector2Matrix(Eigen::Vector3d(1., 2., -1.));
    Rj = utils::DMmath::RotationVector2Matrix(Eigen::Vector3d(-1, 0.2, 1.));
    Eigen::Vector3d ti, tj;
    ti << 1., 2., 3.;
    tj << -1., 1., 2.5;
    std::cout << "gt Ri: \n" << Ri << std::endl;
    std::cout << "gt ti: " << ti.transpose() << std::endl;
    std::cout << "gt Rj: \n" << Rj << std::endl;
    std::cout << "gt tj: " << tj.transpose() << std::endl;
    Eigen::Matrix3d Rij = Rj.transpose() * Ri;
    Eigen::Vector3d tij = Rj.transpose() * (ti - tj);
    gtsam::noiseModel::Diagonal::shared_ptr noise;
    noise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector6::Ones() * 0.0001);

    gtsam::Pose3 posei(gtsam::Rot3(Ri), ti);
    gtsam::Pose3 posej(gtsam::Rot3(Rj), tj);

    gtsam::NonlinearFactorGraph graph;
    gtsam::ISAM2Params isam2_params;
    isam2_params.relinearizeThreshold = 0.01;
    isam2_params.relinearizeSkip = 1;
    gtsam::ISAM2 isam(isam2_params);
    gtsam::Values initial_values;

    initial_values.insert(X(0), posei);
    graph.addPrior(X(0), posei, noise);
    isam.update(graph, initial_values);

    initial_values.clear();
    graph.resize(0);

    Eigen::Matrix3d Rj_hat = Rj;// * utils::DMmath::RotationVector2Matrix(Eigen::Vector3d(0.0, 0.0, 0.07));
    Eigen::Vector3d tj_hat = tj + Eigen::Vector3d(0.1, 0.0, 0.0);
    gtsam::Pose3 posej_hat(gtsam::Rot3(Rj_hat), tj_hat);
    initial_values.insert(X(1), posej_hat);
    std::cout << "****************" << std::endl;
    std::cout << "init Rj: \n" << Rj_hat << std::endl;
    std::cout << "init tj: " << tj_hat.transpose() << std::endl;

    OdomFactor factor(Rij, tij, noise, X(0), X(1));
    graph.add(factor);
    std::cout << "---------------------pose graph----------------" << std::endl;
    gtsam::ISAM2Result result = isam.update(graph, initial_values);
    result.print("result:\n");
    gtsam::Values current_estimate = isam.calculateEstimate();
    std::cout << "current estimate size: " << current_estimate.size() << std::endl;
    current_estimate.print("current estimate: \n");
}