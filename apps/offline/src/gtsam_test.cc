#include <iostream>
#include <cstdlib>
#include <ctime>
#include <boost/optional.hpp>
#include <unistd.h>
#include <stdio.h>

#include <eigen3/Eigen/Core>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Key.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/geometry/SimpleCamera.h>
#include <gtsam/slam/SmartProjectionPoseFactor.h>
#include <gtsam/nonlinear/NonlinearISAM.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include "utils/DMmath.h"

// EKF
// xj = A * xi; xj_ = A * xi^ + v; D(v) = Q;
// yj = H * xj + w; D(w) = R;
// e(xj_) = xj - xj_ = A * xi - (A * xi^ + v) = A * (xi - xi^) - v
// Pj_ = D(ej) = E(ej * ej^t) - E(ej)^2 = E((A(xi - xi^) + v) * (A(xi - xi^) + v)^t) = E(A(xi - xi^) * (xi - xi^)^t * A  - A(xi - xi^) * v^t + v * v^t)
// = A * P(e(xi^)) * A^t + Q = A * Pi^ * A^t + Q;
// xj^ = xj_ + K * (yj - H * xj_)
// e(xj^) = xj - xj^ = xj - xj_ + K * (H * xj + w - H * xj_) = (I - K * H) * (xj - xj_) + K * w = F * e(xj_) + K * w, F = I - K * H
// Pj^ = P(e(xj^)) = D(e(xj^)) = E(e(xj^) * e(xj^)^t) = E((F * e(xj_) + K * w) * (F * e(xj_) + K * w)^t)
// = F * E(e(xj_) * e(xj_)^t) * F^t + K * E(w * w^t) * K^t = F * Pj_ * F^t + K * R * K^t;
// Pj^ = F * Pj_ * F^t + K * R * K^t;
// Pj^ = Pj_ - K * H * Pj_ - Pj_ * H^t * K^t + K * (H * Pj_ * H^t + R) * K^t
// dPj^_dK = -2 * Pj_ * H^t + 2 * K * (H * Pj_ * H^t + R) = 0
// K * (H * Pj_ * H^t + R) = Pj_ * H^t => K = Pj_ * H^t * (H * Pj_ * H^t + R)^(-1)
// MAP
// P(x/z) = P(z/x) * P(x) / P(z) == P(z/x) * P(x)
// P(xj/zk) = P(zk/xj) * P(xj/xi) * P(xi);
// max{xj}(P(xj/zk)) = max(P(zk/xj) * P(xj/xi) * P(xi)) = min(-log(P(zk/xj) * P(xj/xi) * P(xi)))
// zk = H * xj + w => w = zk - H * xj; w ~ N(0, sigma) =>
// P(zk/xj) = N(zk - H * xj, sigma) = (1/(sigma * sqrt(2 * pi))) * exp(-(zk - H * xj) * Pz * (zk - H * xj)^t); Pz = sigma_inv;
// xj = A * xi + v; v = xj - A * xi; v ~ N(0, sj);
// P(xj/xi) = N(xj - A * xi, sj) = (1 / (sj * sqrt(2 * pi))) * exp(-(xj - A * xi) * Pj * (xj - A * xi)^t); Pj = sj_inv;
// P(xi) = N(0, Pi_inv) = (1 / (Pi_inv * sqrt(2 * pi))) * exp(-(xi - xi^) * Pi * (xi - xi^)^t);
// max{xj}(P(xj/zk)) = min{xj}((zk-H*xj) * Pz * (zk-H*xj)^t + (xj-A*xi)*Pj*(xj-A*xi)^t + (xi-xi^)*Pi*(xi-xi^);
// Z = [xi^; 0; zk] => B = [1, 0; -A, 1; 0, H], x = [xi, xj], P = [Pi, 0, 0; 0, Qj, 0; 0, 0, Rz] => Z = B * x;
// max{Xj}P(xj/zk) = min{xj}(Z - B * x)^t * P * (Z - B * x)
// dr_dxj = -B * P * (Z - B * x) = 0
// B * P * B * x = B * P * Z;
// P(xj/zk) = k * exp(-0.5 * (xj-xj^) * H^t * Pz * H * (xj - xj^)), Pj = (H^t * Pz * H)^(-1);

// Bayes inference
// x^ = A * u + w => P(x/u) = N(x^, A * Pu * A^t)
// y = C * x + v => P(y/x) = N(C * X^, C * Px * C^t + R)
// P(x,y/u) = P(x, y/u) = N(x^, A * Pu * A^t) * N(C * X^, C * Px * C^t + R) = N([x^; C * x^], [Px, Px * C^t; C * Px, C * Px * C^t + R])
// let S = [Sxx, Sxy; Syx, Syy] = [Px, Px * C^t; C * Px, C * Px * C^t + R]
// ([x;y] - [x^; C * x^])^t * S_inv * ([x;y] - [x^; C*x^]) = e^t * [1, 0; -Syy_inv * Syx, 1] * [...] * [...] * e
// = (x - x^ - Sxy * Syy_inv * (y - C*x^)) * (Sxx - Sxy * Syy_inv * Syx)_inv * (...) + (y - C*x^)^t*Syy_inv * (y - C*x^)
// P(x/y, u) = (x - x^ - Sxy * Syy_inv * (y-C*x^))^t * (Sxx - Sxy * Syy_inv * Syx)_inv * (x - x^ - Sxy * Syy_inv * Syx * (y - C*X^))
// so x = x^ + Sxy * Syy_inv * (y-C*x^), Px = Sxx - Sxy * Syy_inv * Syx

// P(x,y/v) = P(x/y,v) * P(y) => P(x/y,v) = P(x,y/v)/P(y) == Bayes inference
// P(x/y,v) == P(y/x) * P(x/v)/p(y) == maximum a posterior

// IEKF
// xj = f(xi) + v => P(xj) = N(f(xi^), Sxx)
// yj = h(xj) + w => yj = h(xjo) + G * (xj_ - xjo) + w
// P(xj,yj/v) = N([xj_; h(xjo) + G * (xj_ - xjo)], [Sxx, Sxy; Syx, G * Sxx * G^t + R])

// min((cosx - a)^2 + (sinx - b)^2), sinx^2 + cosx^2 = 1;


using namespace gtsam;
using symbol_shorthand::X;
using symbol_shorthand::V;
using symbol_shorthand::B;
using symbol_shorthand::P;

typedef SmartProjectionPoseFactor<Cal3_S2> SmartFactor;

Eigen::Vector3d RandNoise(double std) {
    std::srand((int)std::time(0));
    double x0 = (std::rand() % 1000) * std * 0.001;
    double x1 = (std::rand() % 1000) * std * 0.001;
    double x2 = (std::rand() % 1000) * std * 0.001;

    return Eigen::Vector3d(x0, x1, x2);
}

gtsam::Point2 RandNoise2D(double std) {
    std::srand((int)std::time(0));
    double x0 = (std::rand() % 1000) * std * 0.001;
    double x1 = (std::rand() % 1000) * std * 0.001;
    return gtsam::Point2(x0, x1);
}

Matrix H1_opt(2, 6);
Matrix H2_opt(2, 3);

Matrix Hessian1(6, 6);
Matrix Hessian2(3, 3);

Vector b1(6);
Vector b2(3);
class ProjectionError2D: public NoiseModelFactor2<gtsam::Pose3, gtsam::Point3> {
  public:
    ProjectionError2D(const Point2& measured, const SharedNoiseModel& model,
        Key poseKey, Key pointKey, const boost::shared_ptr<Cal3_S2>& K)
        : K_(K), measurement_(measured), NoiseModelFactor2(model, poseKey, pointKey) {
    }

    Vector evaluateError(const Pose3& pose, const Point3& point,
        boost::optional<Matrix&> H1 = boost::none, boost::optional<Matrix&> H2 = boost::none) const {
        PinholeCamera<Cal3_S2> camera(pose, *K_);
        gtsam::Vector err = camera.project(point, H1, H2, boost::none) - measurement_;
        // std::cout << "err: " << err.transpose() << std::endl;
        if (H1 != boost::none) {
            H1_opt = *H1;
            Matrix h1 = H1_opt.transpose() * H1_opt;
            Hessian1 += h1;
            double err_norm = err.norm();
            double max_coef = h1.maxCoeff();
            b1 += H1_opt.transpose() * err;
        }
        if (H2 != boost::none) {
            H2_opt = *H2;
            Hessian2 += H2_opt.transpose() * H2_opt;
            b2 += H2_opt * err;
        }
        // std::cout << "-------------" << std::endl;
        return err;
    }
  protected:
    gtsam::Point2 measurement_;
    boost::shared_ptr<Cal3_S2> K_;
};

int main(int argc, char** argv) {
    gtsam::ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.01;
    parameters.relinearizeSkip = 1;
    parameters.cacheLinearizedFactors = false;
    parameters.enableDetailedResults = false;
    parameters.print();
    gtsam::ISAM2 isam2(parameters);

    int relinearizeInterval = 3;
    NonlinearISAM isam(relinearizeInterval);

    gtsam::NonlinearFactorGraph graph;

    gtsam::Vector6 odom_noise_diag;
    odom_noise_diag.setOnes();
    odom_noise_diag *= 1.0e-03;
    gtsam::noiseModel::Diagonal::shared_ptr odom_noise;
    odom_noise = gtsam::noiseModel::Diagonal::Variances(odom_noise_diag);
    Eigen::Matrix3d I3; I3.setIdentity();
    gtsam::Rot3 gI3(I3);

    // generate prior pose
    int count = 1;
    Eigen::Matrix3d R0 = utils::DMmath::RotationVector2Matrix(Eigen::Vector3d(0.0, 0.0, 0.0));
    gtsam::Rot3 r0(R0);
    gtsam::Point3 t0; t0 << 0, 0, 0;
    gtsam::Pose3 prior_pose0(r0, t0);
    gtsam::Values initial_values;
    initial_values.insert(X(count), prior_pose0);
    std::cout << "prior: " << prior_pose0.matrix() << std::endl;
    double n0 = 0.05;
    gtsam::noiseModel::Diagonal::shared_ptr prior_pose_noise =
        gtsam::noiseModel::Diagonal::Sigmas((Vector(6)<<n0, n0, n0, n0, n0, n0).finished());
    graph.add(gtsam::PriorFactor<Pose3>(X(count), prior_pose0, prior_pose_noise));

    utils::Timer timecount;
    timecount.Start();

    Eigen::Matrix3d Rbw = R0;
    Eigen::Vector3d tbw = t0;

    Eigen::Matrix3d Rwb = Rbw.transpose();
    Eigen::Vector3d twb;

    for (size_t i = 2; i < 1000009; i++) {
        count++;
        H1_opt.fill(0);
        H2_opt.fill(0);
        Hessian1.fill(0);
        Hessian2.fill(0);
        b1.fill(0);
        b2.fill(0);
        std::cout << "------------------" << i << "th--------------------" << std::endl;
        Eigen::Vector3d dt;
        dt << 0, 0, 0.3 * 2 - 0.1;
        Eigen::Matrix3d dR = utils::DMmath::RotationVector2Matrix(Eigen::Vector3d(-0.06, 0.01, 0.05));
        gtsam::Rot3 gdR(dR);
        Eigen::Vector3d dtbw = Rbw * dt;

        gtsam::Point3 tmp_t(tbw);
        tmp_t << tbw(0), tbw(1), tbw(2) + 0.4;
        Rbw = Rbw * dR;
        gtsam::Rot3 gR1(Rbw);
        gtsam::Pose3 tmp_pose(gR1, tmp_t);
        initial_values.insert(X(count), tmp_pose);

        tbw += dtbw;
        twb = -Rwb * tbw;

        // loop closure
        if (i == 15) {
            Eigen::Matrix3d dRbx = R0.transpose() * Rbw * utils::DMmath::RotationVector2Matrix(Eigen::Vector3d(0.00, 0.005, 0.01));
            gtsam::Rot3 gdRbx(dRbx);
            Eigen::Vector3d dtx = R0.transpose() * (tbw - t0);
            gtsam::Pose3 loop(gdRbx, dtx);
            graph.add(BetweenFactor<Pose3>(X(1), X(count), loop, odom_noise));
            std::cout << "loop closure!!!!!" << std::endl;
        }

        std::cout << "real tbw: " << tbw.transpose() << std::endl;
        std::cout << "init tbw: " << tmp_t.transpose() << std::endl;
        std::cout << "real Rbw: " << Rbw << std::endl;

        gtsam::Pose3 odom(gdR, dt);
        graph.add(BetweenFactor<Pose3>(X(count - 1), X(count), odom, odom_noise));

        timecount.Start(); 

        // isam2
        gtsam::ISAM2Result result = isam2.update(graph, initial_values);
        // result.print("final result:\n");
        // gtsam::Values current_estimate = isam2.calculateEstimate();
        // std::cout << "current estimate size: " << current_estimate.size() << std::endl;
        // current_estimate.print("current estimate: \n");

        // next optimization pass
        timecount.End(std::to_string(i) + "th optimization update");
        getchar();
        graph.resize(0);
        initial_values.clear();
        std::cout << "-----------" << i << "th end-----------------" << std::endl;
    }
}