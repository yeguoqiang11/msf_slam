#include "slam/PoseGraph.h"
namespace slam {
FramesTransformSolver::FramesTransformSolver() {
    voxel_cost_huber_thres_ = 0.1 * 0.1;
    outlier_thres = 0.2;
}

bool FramesTransformSolver::FramesTransformCal(std::shared_ptr<Frame> framei_ptr, std::shared_ptr<Frame> framej_ptr, utils::Transform3d &dTji) {
    Eigen::Matrix<double, 6, 6> H; H.setZero();
    Eigen::Matrix<double, 6, 1> b; b.setZero();
    double cost = 0.;

    double voxel_info_mat = 10.;
    double ground_info_mat = 1000.;
    
    std::cout << "*******loop frame Tji cal begin****************" << std::endl;
    std::cout << "init dTji: " << dTji << std::endl;

    // new cost calculation
    frames_ptis_.clear();
    frames_ptjs_.clear();
    int inlier_count = FramesVoxelCost(framei_ptr, framej_ptr, dTji, H, b, cost, voxel_info_mat);
    if (inlier_count > 5) {
        cost /= inlier_count;
        H /= inlier_count;
        b /= inlier_count;
    }
    FramesGroundConstraintCost(framei_ptr, framej_ptr, dTji, H, b, cost, ground_info_mat);
    std::cout << "init cost: " << cost << ",inlier count: " << inlier_count << std::endl;
    Eigen::Matrix<double, 6, 6> I6; I6.setIdentity();
    double lambda = 1.e-03;
    for (int iteration = 0; iteration < 6; iteration++) {
        Eigen::Matrix<double, 6, 1> dx = (H + lambda * I6).ldlt().solve(-b);
        utils::Transform3d dTji_hat = dTji * dx;
        Eigen::JacobiSVD<Eigen::Matrix3d> t_svd(H.bottomRightCorner(3, 3), Eigen::ComputeFullU | Eigen::ComputeFullV);
        std::cout << "t eig: " << t_svd.singularValues().transpose() << std::endl;
        Eigen::Vector3d t_sig = t_svd.singularValues();
        if (fabs(t_sig(2) / t_sig(1)) < 0.1 || inlier_count < 50) return false;
        // std::cout << "t vt: \n" << t_svd.matrixV().transpose() << std::endl;
        if (framej_ptr->FrameId() == 0 || framej_ptr->FrameId() == 0 ||
            framej_ptr->FrameId() == 0) {
            return true;
        }
        // new cost calculation
        frames_ptis_.clear();
        frames_ptjs_.clear();
        Eigen::Matrix<double, 6, 6> H_hat; H_hat.setZero();
        Eigen::Matrix<double, 6, 1> b_hat; b_hat.setZero();
        double cost0 = 0.;
        inlier_count = FramesVoxelCost(framei_ptr, framej_ptr, dTji_hat, H_hat, b_hat, cost0, voxel_info_mat);
        if (inlier_count > 5) {
            cost0 /= inlier_count;
            H_hat /= inlier_count;
            b_hat /= inlier_count;
        }
        FramesGroundConstraintCost(framei_ptr, framej_ptr, dTji_hat, H_hat, b_hat, cost0, ground_info_mat);

        if (cost0 < cost) {
            cost = cost0;
            H = H_hat;
            b = b_hat;

            dTji = dTji_hat;
            lambda = lambda < 1.0e-05? lambda : lambda * 0.1;
        } else {
            lambda = lambda > 1000? lambda : lambda * 10;
        }
        // if (dx.norm() < 0.01) {
        //     std::cout << "dx norm: " << dx.norm() << std::endl;
        //     break;
        // }
        std::cout << "min cost and current cost: " << cost << "," << cost0 << ", inlier count: " << inlier_count << std::endl;
    }
    std::cout << "final: " << dTji << std::endl;
    std::cout << "Tbjw: " << framei_ptr->Tbw() * dTji << std::endl;
    std::cout << "*******loop frame Tji cal end****************" << std::endl;
    return true;
}

void FramesTransformSolver::FramesGroundConstraintCost(std::shared_ptr<Frame> framei_ptr, std::shared_ptr<Frame> framej_ptr, const utils::Transform3d &dTji,
                                    Eigen::Matrix<double, 6, 6> &H, Eigen::Matrix<double, 6, 1> &b, double &cost, double info_mat) {
    if (framei_ptr->PointCloudFrameRef().GroundPtr() == nullptr ||
        framej_ptr->PointCloudFrameRef().GroundPtr() == nullptr) return;
    Eigen::Vector3d ci = framei_ptr->PointCloudFrameRef().GroundPtr()->Center();
    Eigen::Vector3d nvi = framei_ptr->PointCloudFrameRef().GroundPtr()->NormalVector();
    Eigen::Vector3d cj = framej_ptr->PointCloudFrameRef().GroundPtr()->Center();
    Eigen::Vector3d nvj = framej_ptr->PointCloudFrameRef().GroundPtr()->NormalVector();

    Eigen::Vector3d ci_hat = dTji.transform(cj);
    double r = nvi.dot(ci_hat - ci);

    // r = nvi^t * (Rji * cj + tji - ci)
    // r + dr = nvi^t * (Rji * exp(drv) * cj + tji + dt - ci)
    // r + dr = nvi^t * (Rji * (I + drv^) * cj + tji + dt - ci)
    // r + dr = nvi^t * (Rji * cj + tji - ci + Rji * drv^ * cj + dt)
    // r + dr = r + nvi^t * (Rji * drv^ * cj + dt)
    // dr = nvi^t * Rji * drv^ * cj + nvi^t * dt
    // = -nvi^t * Rji * cj^ * drv + nvi^t * dt
    Eigen::Matrix<double, 1, 6> J;
    J.topLeftCorner(1, 3) = -nvi.transpose() * dTji.R() * utils::DMmath::SkewMatrix(cj);
    J.topRightCorner(1, 3) = nvi.transpose();

    H += J.transpose() * info_mat * J;
    b += J.transpose() * info_mat * r;
    cost += r * info_mat * r;
}

std::vector<Eigen::Vector3d> frames_ptis_;
std::vector<Eigen::Vector3d> frames_ptjs_;
int FramesTransformSolver::FramesVoxelCost(std::shared_ptr<Frame> framei_ptr, std::shared_ptr<Frame> framej_ptr, const utils::Transform3d &dTji,
                                     Eigen::Matrix<double, 6, 6> &H, Eigen::Matrix<double, 6, 1> &b, double &cost, double info_mat) {
    int inlier_count = 0;
    VoxelGrid &gridi = framei_ptr->PointCloudFrameRef().VoxelGridObject();
    VoxelGrid &gridj = framej_ptr->PointCloudFrameRef().VoxelGridObject();
    // for show
    utils::Transform3d Tbiw = framei_ptr->Tbw();

    // add voxel cost
    utils::Transform3d dTij = dTji.Inverse();
    VoxelGrid::MapType::iterator iter = gridj.Begin();
    Eigen::Vector3d pti, pti_hat, ptj_hat, eigj, nvj, nvi_hat;
    for (; iter != gridj.End(); iter++) {
        if (!iter->second.GetEigenInfo(eigj, nvj)) {
            iter->second.EigenCal();
            if (!iter->second.GetEigenInfo(eigj, nvj)) {
                continue;
            }
        }

        // std::cout << "ptj: " << iter->second.center.transpose() << std::endl;
        pti_hat = dTji.transform(iter->second.center);
        nvi_hat = dTji.R() * nvj;
        // std::cout << "pti hat: " << pti_hat.transpose() << std::endl;
        // std::cout << "nvi hat: " << nvi_hat.transpose() << std::endl;
        VoxelGrid::VoxelType *result = gridi.FindNeighbors(pti_hat, nvi_hat, 3);
        if (result == nullptr) continue;
        // std::cout << "result: " << result->center.transpose() << std::endl;
        // std::cout << "nvj: " << nvj.transpose() << std::endl;
        // std::cout << "*******************" << std::endl;

        ptj_hat = dTij.transform(result->center);
        Eigen::Vector3d err = iter->second.center - ptj_hat;
        double residual = nvj.dot(err);
        if (fabs(residual) > outlier_thres) continue;
        double cost0 = residual * residual;
        double w = 1.;
        if (cost0 > voxel_cost_huber_thres_) {
            w = voxel_cost_huber_thres_ / cost0;
        }

        // jacobian
        // ptj_hat = dRji^t * pti - dRji^t * dtji = dRji^t * (pti - dtji)
        // err = ptj - ptj_hat
        // r = nvj^t * err = nvj^t * (ptj - dRji^t * (pti - dtji))
        // r + dr = nvj^t * (ptj - exp(-drv) * dRji^t * (pti - dtji - dt))
        // r + dr = nvj^t * (ptj - (I - drv^) * dRji^t * (pti - dtji - dt))
        // = nvj^t * (ptj - dRji^t * (pti - dtji - dt) + drv^ * dRji^t * (pti - dtji - dt))
        // = nvj^t * (ptj - dRji^t * (pti - dtji) + dRji^t * dt - (dRji^t * (pti - dtji))^ * drv)
        // = r + nvj^t * dRji^t * dt - nvj^t * (dRji^t * (pti - dtji))^ * drv
        // dr = nvj^t * dRji^t * dt - nvj^t * (dRji^t * (pti - dtji))^ * drv
        // dr = nvj^t * dRij * dt - nvj^t * (dRij * (pti - dtji))^
        Eigen::Matrix<double, 1, 6> J;
        J.topLeftCorner(1, 3) = -nvj.transpose() * (utils::DMmath::SkewMatrix(dTij.R() *
                                (result->center - dTji.t())));
        J.topRightCorner(1, 3) = nvj.transpose() * dTij.R();

        H += w * J.transpose() * info_mat * J;
        b += w * J.transpose() * info_mat * residual;
        cost += residual * info_mat * residual;
        inlier_count++;

        frames_ptis_.push_back(Tbiw.transform(result->center));
        frames_ptjs_.push_back(Tbiw.transform(pti_hat)); 
    }
    return inlier_count;
}

PoseGraph::PoseGraph(std::shared_ptr<GlobalVoxelMap> map_ptr, std::shared_ptr<Sensors> sensors_ptr)
  : map_ptr_(std::move(map_ptr)),
    sensors_ptr_(std::move(sensors_ptr)),
    imu_param_ptr_(nullptr),
    is_initialized_(false) {
    graph_ = new gtsam::NonlinearFactorGraph();
    gtsam::ISAM2Params isam2_params;
    isam2_params.relinearizeThreshold = 0.01;
    isam2_params.relinearizeSkip = 1;
    isam2_ = new gtsam::ISAM2(isam2_params);

    frames_solver_ptr_ = std::make_shared<FramesTransformSolver>();
}

void PoseGraph::LoadFrame(std::shared_ptr<Frame> frame_ptr) {
    std::cout << "-----------------" << frame_ptr->FrameId() << "th pose graph------------------" << std::endl;
    if (frame_ptr == nullptr) return;

    // initial values
    utils::Transform3d Tiw = frame_ptr->Tbw() * sensors_ptr_->imu.Tib;
    uint32_t frame_id = frame_ptr->FrameId();
    timestamp_ = frame_ptr->TimeStamp();

    // set initial values and first frame prior constraint
    AddInitialValueAndFirstPriorConstraint(frame_id, Tiw, frame_ptr->Vw(), frame_ptr->Ba(), frame_ptr->Bg());

    // tof odometry constraint
    AddOdomConstraint(Tiw, frame_id);

    // gyro wheel constraint
    // AddGyroWheelConstraint(frame_id);

    // imu constraint
    // AddImuConstraint(frame_id, frame_ptr->Ba(), frame_ptr->Bg());

    // loop constraint
    AddLoopConstraint(frame_ptr, frame_id);

    // graph optimization
    gtsam::ISAM2Result result = Optimization(frame_ptr);

    // state update
    StateUpdate(frame_ptr, result);
    std::cout << "------------------" << frame_ptr->FrameId() << "th end pose graph---------------------" << std::endl;
}

void PoseGraph::AddInitialValueAndFirstPriorConstraint(uint32_t frame_id, const utils::Transform3d &Tiw, const Eigen::Vector3d &Vw,
                                                       const Eigen::Vector3d &ba, const Eigen::Vector3d &bg) {
    gtsam::Pose3 prior_pose(gtsam::Rot3(Tiw.R()), Tiw.t());
    gtsam::Vector3 prior_vel = Vw;
    gtsam::imuBias::ConstantBias prior_bias(ba, bg);
    initial_values_.insert(X(frame_id), prior_pose);
    initial_values_.insert(V(frame_id), prior_vel);
    initial_values_.insert(B(frame_id), prior_bias);

    if (map_ptr_->FrameEmpty()) {
        Eigen::Matrix<double, 6, 1> noise_vec = Eigen::Matrix<double, 6, 1>::Ones() * 100;
        auto pose_noise_model = gtsam::noiseModel::Diagonal::Sigmas((noise_vec));
        auto vel_noise_model = gtsam::noiseModel::Isotropic::Sigma(3, 10.);
        auto bias_noise_model = gtsam::noiseModel::Isotropic::Sigma(6, 10.);
        graph_->addPrior(X(frame_id), prior_pose, pose_noise_model);
        // graph_->addPrior(V(frame_id), prior_vel, vel_noise_model);
        // graph_->addPrior(B(frame_id), prior_bias, bias_noise_model);
    }
}

void PoseGraph::AddOdomConstraint(const utils::Transform3d &Tiw, uint32_t frame_id) {
    if (!map_ptr_->FrameEmpty()) {
        const std::shared_ptr<Frame> frame0_ptr = map_ptr_->KeyFrames().back();
        uint32_t frame0_id = frame0_ptr->FrameId();

        //tof odometry constraint
        utils::Transform3d Tiw0 = frame0_ptr->Tbw() * sensors_ptr_->imu.Tib;
        utils::Transform3d dT = Tiw0.Inverse() * Tiw;

        gtsam::Pose3 odom(gtsam::Rot3(dT.R()), dT.t());
        auto odom_noise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector6::Ones() * 0.0001);
        graph_->add(gtsam::BetweenFactor<gtsam::Pose3>(X(frame0_id), X(frame_id), odom, odom_noise));
    }
}

void PoseGraph::AddGyroWheelConstraint(uint32_t frame1_id) {
    if (!map_ptr_->FrameEmpty()) {        
        std::shared_ptr<Frame> frame0_ptr = map_ptr_->KeyFrames().back();
        uint32_t frame0_id = frame0_ptr->FrameId();
        std::shared_ptr<GyroWheelModelPreintegration> wh_ptr = frame0_ptr->GyroWheelSharedPtr();
        if (wh_ptr != nullptr) {
            Eigen::Matrix3d wh_cov = wh_ptr->Cov();
            std::cout << "wh cov: " << wh_cov << std::endl;
            gtsam::noiseModel::Gaussian::shared_ptr gyrwheel_noise;
            gyrwheel_noise = gtsam::noiseModel::Gaussian::Covariance(wh_cov);
            GyroDiffWheelFactor wheel_factor(wh_ptr, sensors_ptr_, gyrwheel_noise, X(frame0_id), X(frame1_id));
            graph_->add(wheel_factor);
        }
    }
}

void PoseGraph::AddImuConstraint(uint32_t frame1_id, const Eigen::Vector3d &ba, const Eigen::Vector3d &bg) {
    if (imu_param_ptr_ == nullptr) {
        ImuParamInitialization();
    }
    if (!map_ptr_->FrameEmpty() && imu_param_ptr_ != nullptr) {
        std::shared_ptr<Frame> frame0_ptr = map_ptr_->KeyFrames().back();
        uint32_t frame0_id = frame0_ptr->FrameId();
        std::shared_ptr<Imu> imu_ptr = frame0_ptr->ImuSharedPtr();

        std::cout << "preinte imu ba: " << ba.transpose() << std::endl;
        std::cout << "preinte imu bg: " << bg.transpose() << std::endl;
        if (imu_ptr != nullptr) {
            gtsam::imuBias::ConstantBias prior_imubias(ba, bg);
            std::shared_ptr<gtsam::PreintegratedCombinedMeasurements> preintegration_ptr =
                std::make_shared<gtsam::PreintegratedCombinedMeasurements>(imu_param_ptr_, prior_imubias);
            const std::vector<Imu::ImuData> &imu_data = imu_ptr->ImuDatas();
            for (size_t i = 0; i < imu_data.size(); i++) {
                Eigen::Vector3d acc = imu_data[i].acc;
                Eigen::Vector3d gyr = imu_data[i].gyr;
                preintegration_ptr->integrateMeasurement(acc, gyr, imu_data[i].dt);
                // std::cout << "imu: " << (Riw * imu_data[i].acc).transpose() << "," << imu_data[i].gyr.transpose() << "," << imu_data[i].dt << std::endl;
            }
            gtsam::CombinedImuFactor imu_factor(X(frame0_id), V(frame0_id), X(frame1_id), V(frame1_id),
                    B(frame0_id), B(frame1_id), *preintegration_ptr);
            graph_->add(imu_factor);

            double det = imu_ptr->Dt() - (timestamp_ - frame0_ptr->TimeStamp());
            if (fabs(det) > 0.0001) {
                std::cout << "dt error in imu preintegration of pose graph!!!" << std::endl;
                exit(0);
            }
        }
    }
}

gtsam::ISAM2Result PoseGraph::Optimization(std::shared_ptr<Frame> frame_ptr) {
    // initial_values_.print("initial value: \n");
    utils::Timer tick;
    gtsam::ISAM2Result result = isam2_->update(*graph_, initial_values_);
    tick.End("first upate");
    result = isam2_->update();
    tick.End("second update");
    result = isam2_->update();
    tick.End("third update");
    result = isam2_->update();
    tick.End("forth update");
    result = isam2_->update();
    tick.End("fifth update");
    result = isam2_->update();
    tick.End("sixth update");

    // get update values
    uint32_t frame_id = frame_ptr->FrameId();
    const gtsam::Value &estimated = isam2_->calculateEstimate(X(frame_id));
    // const gtsam::Value &estimated_v = isam2_->calculateEstimate(V(frame_id));
    // const gtsam::Value &estimated_b = isam2_->calculateEstimate(B(frame_id));
    gtsam::Pose3 pose = estimated.cast<gtsam::Pose3>();
    // gtsam::Vector3 vw = estimated_v.cast<gtsam::Vector3>();
    // gtsam::imuBias::ConstantBias bias = estimated_b.cast<gtsam::imuBias::ConstantBias>();

    // std::cout << "estimated Riw: \n" << pose.rotation().matrix() << std::endl;
    // std::cout << "estimated tiw: \n" << pose.translation().transpose() << std::endl;
    // std::cout << "estimate Vw: " << vw.transpose() << std::endl;
    // std::cout << "estimate ba: " << bias.accelerometer().transpose() << std::endl;
    // std::cout << "estimate bg: " << bias.gyroscope().transpose() << std::endl;
    // check imu constraint  
    // if (frame_id >= 1 && graph_->size() >= 2) {
    //     // std::cout << "graph size: " << graph_->size() << std::endl;
    //     auto it = graph_->begin();
    //     auto object0 = it->get();
    //     it++;
    //     auto object1 = it->get();

    //     // object->print("imu graph: \n");
    //     // object->printKeys("imu graph keys: \n");
    //     gtsam::Values vars;
    //     const gtsam::Value &estimated0 = isam2_->calculateEstimate(X(frame_id - 1));
    //     const gtsam::Value &v0 = isam2_->calculateEstimate(V(frame_id - 1));
    //     const gtsam::Value &b = isam2_->calculateEstimate(B(frame_id - 1));
    //     vars.insert(X(frame_id - 1), estimated0.cast<gtsam::Pose3>());
    //     vars.insert(X(frame_id), pose);
    //     std::cout << "odom error: " << object0->error(vars) << std::endl;
    //     std::cout << "wheel error: " << object1->error(vars) << std::endl;

        // if (graph_->size() >= 4) {
        //     it++;
        //     auto object = it->get();
        //     vars.insert(V(frame_id - 1), v0.cast<gtsam::Vector3>());
        //     vars.insert(V(frame_id), vw);
        //     vars.insert(B(frame_id - 1), b.cast<gtsam::imuBias::ConstantBias>());
        //     vars.insert(B(frame_id), bias);

        //     std::cout << "imu error: " << object->error(vars) << std::endl;
        // }

        // gtsam::Pose3 pose0 = estimated0.cast<gtsam::Pose3>();
        // std::cout << "Riw0: " << pose0.rotation().matrix() << std::endl;
        // std::cout << "tiw0: " << pose0.translation().transpose() << std::endl;
        // std::cout << "vw0: " << (v0.cast<gtsam::Vector3>()).transpose() << std::endl;
        // gtsam::imuBias::ConstantBias bias0 = b.cast<gtsam::imuBias::ConstantBias>();
        // std::cout << "ba0: " << bias0.accelerometer().transpose() << std::endl;
        // std::cout << "bg0: " << bias0.gyroscope().transpose() << std::endl;
    // }

    // set values
    utils::Transform3d Tiw(pose.rotation().matrix(), pose.translation());
    utils::Transform3d Tbw = Tiw * sensors_ptr_->imu.Tib.Inverse();
    frame_ptr->SetPose(Tbw);
    // frame_ptr->SetBga(bias.accelerometer(), bias.gyroscope());
    // frame_ptr->SetVelocity(vw);

    // std::cout << "pose graph estimated Tbw: \n" << Tbw << std::endl;

    // current_estimate.print("current estimate: \n");
    // graph_->print("factor graph");
    graph_->resize(0);
    initial_values_.clear();
    return result;
}

void PoseGraph::StateUpdate(std::shared_ptr<Frame> frame_ptr, gtsam::ISAM2Result &result) {
    gtsam::Values isamCurrentEstimate = isam2_->calculateBestEstimate();
    utils::Transform3d Tbi = sensors_ptr_->imu.Tib.Inverse();

    // set current state
    auto idx =  frame_ptr->FrameId();
    gtsam::Pose3 pose = isamCurrentEstimate.at<gtsam::Pose3>(X(idx));
    gtsam::Vector3 vw = isamCurrentEstimate.at<gtsam::Vector3>(V(idx));
    gtsam::imuBias::ConstantBias bias = isamCurrentEstimate.at<gtsam::imuBias::ConstantBias>(B(idx));
    utils::Transform3d Tiw(pose.rotation().matrix(), pose.translation());
    utils::Transform3d Tbw = Tiw * Tbi;
    frame_ptr->SetPose(Tbw);
    // frame_ptr->SetBga(bias.accelerometer(), bias.gyroscope());
    // frame_ptr->SetVelocity(vw);

    if (loop_options_.last_loop_idx == frame_ptr->FrameId()) {
        // set state for past frames
        std::vector<std::shared_ptr<Frame>> &keyframes = map_ptr_->KeyFrames();
        for (int i = 0; i < static_cast<int>(frame_ptr->FrameId()); i++) {
            gtsam::Pose3 pose = isamCurrentEstimate.at<gtsam::Pose3>(X(i));
            gtsam::Vector3 vw = isamCurrentEstimate.at<gtsam::Vector3>(V(i));
            gtsam::imuBias::ConstantBias bias = isamCurrentEstimate.at<gtsam::imuBias::ConstantBias>(B(i));

            // set value
            utils::Transform3d Tiw(pose.rotation().matrix(), pose.translation());
            utils::Transform3d Tbw = Tiw * Tbi;
            keyframes[i]->SetPose(Tbw);
            // keyframes[i]->SetBga(bias.accelerometer(), bias.gyroscope());
            // keyframes[i]->SetVelocity(vw);
            std::cout << i << "th updated Tbw: " << Tbw << std::endl;
        }
        map_ptr_->ReMapFromKeyFrames();
    }
    std::cout << idx << "th updated Tbw: " << Tbw << std::endl;
}

void PoseGraph::AddLoopConstraint(std::shared_ptr<Frame> framej_ptr, uint32_t framej_id) {
    std::vector<std::pair<int, utils::Transform3d>> loop_frame = LoopClosure(framej_ptr);
    const utils::Transform3d &Tib = sensors_ptr_->imu.Tib;
    auto odom_noise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector6::Ones() * 0.00001);
    for (int i = 0; i < loop_frame.size() && i < 0; i++) {
        uint32_t framei_id = loop_frame[i].first;
        utils::Transform3d dTji = Tib.Inverse() * loop_frame[i].second * Tib;
        
        std::cout << "loop dTji: " << dTji << std::endl;
        gtsam::Pose3 odom(gtsam::Rot3(dTji.R()), dTji.t());
        graph_->add(gtsam::BetweenFactor<gtsam::Pose3>(X(framei_id), X(framej_id), odom, odom_noise));
    }

    if (framej_id == 3) {
        uint32_t framei_id = map_ptr_->KeyFrames()[2]->FrameId();
        utils::Transform3d Tbiw = map_ptr_->KeyFrames()[2]->Tbw();

        // j-1th loop
        // uint32_t framek_id = map_ptr_->KeyFrames()[framej_id - 1]->FrameId();
        // utils::Transform3d Tbkw = map_ptr_->KeyFrames()[framek_id]->Tbw();
        // Tbkw.t(0) += 0.05;
        // utils::Transform3d dTki = Tib.Inverse() * Tbiw.Inverse() * Tbkw * Tib;
        // gtsam::Pose3 odom_ki(gtsam::Rot3(dTki.R()), dTki.t());
        // graph_->add(gtsam::BetweenFactor<gtsam::Pose3>(X(framei_id), X(framek_id), odom_ki, odom_noise));
        // std::cout << "loop: (" << framei_id << "," << framek_id << ")" << std::endl;


        // jth loop
        utils::Transform3d Tbjw = framej_ptr->Tbw();
        Eigen::Matrix<double, 6, 1> dv; dv.setZero();
        dv << 0., 0.2, 0., 0., 0., 0.;
        utils::Transform3d dT; dT.SetIdentity();
        dT *= dv;
        Tbjw *= dT;
        // Tbjw.t(0) += 0.05;
        utils::Transform3d dTji = Tib.Inverse() * Tbiw.Inverse() * Tbjw * Tib;
        gtsam::Pose3 odom(gtsam::Rot3(dTji.R()), dTji.t());
        graph_->add(gtsam::BetweenFactor<gtsam::Pose3>(X(framei_id), X(framej_id), odom, odom_noise));

        loop_options_.last_loop_idx = framej_ptr->FrameId();
    }
}

// return [[idx_i, Tbjbi]], Tbjbi = frame_ptr's Tbw => keyframes[idx_i]'s Tbw
std::vector<std::pair<int, utils::Transform3d>> PoseGraph::LoopClosure(std::shared_ptr<Frame> frame_ptr) {
    std::vector<std::pair<int, utils::Transform3d>> results;
    std::vector<int> loop_candids = LoopCandidateDetection(frame_ptr);
    std::cout << "loop candids size: " << loop_candids.size() << std::endl;
    const auto &Tbwj = frame_ptr->Tbw();
    // frame_ptr->SetKeyFrame(false);
    utils::Transform3d dTji;
    const utils::Transform3d &Tcb = sensors_ptr_->tofcams[0].Tcb;
    for (int i = 0; i < loop_candids.size() && i < 1; i++) {
        const auto &Tbwi = map_ptr_->KeyFrames()[loop_candids[i]]->Tbw();
        std::cout << "loop: (" << loop_candids[i] << "," << frame_ptr->FrameId() << ")" << std::endl;
        dTji = Tbwi.Inverse() * Tbwj;
        Eigen::Vector3d drv = utils::DMmath::LogSO3(dTji.R());
        // std::cout << "dT: " << drv.norm() * RAD2DEG << ";" << dTji.t().norm() << std::endl;
        if (LoopTransformCal(loop_candids[i], frame_ptr, dTji)) {
            results.emplace_back(loop_candids[i], dTji);
            // frame_ptr->SetKeyFrame(true);
            loop_options_.last_loop_idx = frame_ptr->FrameId();
        }
    }
    return results;
}

bool PoseGraph::LoopTransformCal(int idx, std::shared_ptr<Frame> framej_ptr, utils::Transform3d &Tji) {
    std::shared_ptr<Frame> framei_ptr = map_ptr_->KeyFrames()[idx];
    return frames_solver_ptr_->FramesTransformCal(framei_ptr, framej_ptr, Tji);
}

// loop candidates detection
std::vector<int> PoseGraph::LoopCandidateDetection(std::shared_ptr<Frame> frame_ptr) {
    std::vector<int> candid_idxs;
    std::vector<std::shared_ptr<Frame>> &key_frames = map_ptr_->KeyFrames();
    if (key_frames.size() - loop_options_.last_loop_idx < loop_options_.min_frame_dis_from_lastloop) return candid_idxs;

    utils::Transform3d Tbw, Tbw0, dT;
    frame_ptr->GetPose(Tbw);

    bool loop_flag = false;
    double angle_thres = loop_options_.angle_thres * loop_options_.angle_thres;
    double dtxy_thres = loop_options_.dtxy_thres * loop_options_.dtxy_thres;
    Eigen::Vector3d drv;
    for (size_t i = loop_options_.min_frame_dis_from_lastloop; i < key_frames.size() && i < key_frames.size(); i++) {
        size_t idx = key_frames.size() - 1 - i;
        key_frames[idx]->GetPose(Tbw0);
        dT = Tbw0.Inverse() * Tbw;
        drv = utils::DMmath::LogSO3(dT.R());

        if (drv.squaredNorm() < angle_thres && fabs(dT.t(2)) < loop_options_.dtz_thres &&
            dT.t().topRows(2).squaredNorm() < dtxy_thres) {
            candid_idxs.push_back(idx);

            std::cout << "dt: " << dT.t().transpose() << std::endl;
            i += loop_options_.skip_num;
        }
    }
    return candid_idxs;
}

void PoseGraph::SetGroundPtr(std::shared_ptr<GroundPlanes> grounds_ptr) {
    grounds_ptr_ = grounds_ptr;
    ImuParamInitialization();
}

void PoseGraph::ImuParamInitialization() {
    if (sensors_ptr_ == nullptr || grounds_ptr_ == nullptr
        || grounds_ptr_->GravitySharedPtr() == nullptr) return;

    Eigen::Vector3d Gw = (*grounds_ptr_->GravitySharedPtr());
    imu_param_ptr_ = boost::make_shared<gtsam::PreintegrationCombinedParams>(Gw);

    Eigen::Matrix3d tmp;
    tmp = sensors_ptr_->imu.na.asDiagonal();
    tmp = tmp * tmp;
    imu_param_ptr_->accelerometerCovariance = tmp;

    tmp = sensors_ptr_->imu.ng.asDiagonal();
    tmp = tmp * tmp;
    imu_param_ptr_->gyroscopeCovariance = tmp;

    imu_param_ptr_->integrationCovariance = gtsam::I_3x3 * 1.0e-08;

    tmp = sensors_ptr_->imu.nba.asDiagonal();
    tmp = tmp * tmp;
    imu_param_ptr_->biasAccCovariance = tmp;
    
    tmp = sensors_ptr_->imu.nbg.asDiagonal();
    tmp = tmp * tmp;
    imu_param_ptr_->biasOmegaCovariance = tmp;

    imu_param_ptr_->biasAccOmegaInt = gtsam::I_6x6 * 1.0e-05;
}


GyroDiffWheelFactor::GyroDiffWheelFactor(std::shared_ptr<GyroWheelModelPreintegration> gyrwheel_ptr,  std::shared_ptr<Sensors> sensors_ptr,
                                       const gtsam::SharedNoiseModel& noise_model, gtsam::Key key0, gtsam::Key key1)
  : gyrwheel_ptr_(gyrwheel_ptr),
    sensors_ptr_(sensors_ptr),
    NoiseModelFactor2(noise_model, key0, key1) {

}

gtsam::Vector GyroDiffWheelFactor::evaluateError(const gtsam::Pose3 &pose0, const gtsam::Pose3 &pose1, boost::optional<gtsam::Matrix&> H0,
                                                boost::optional<gtsam::Matrix&> H1) const {
    utils::Transform3d Tiw0(pose0.rotation().matrix(), pose0.translation());
    utils::Transform3d Tiw1(pose1.rotation().matrix(), pose1.translation());
    
    const utils::Transform3d &Tib = sensors_ptr_->imu.Tib;

    utils::Transform3d Tbw0 = Tiw0 * sensors_ptr_->imu.Tib.Inverse();
    utils::Transform3d Tbw1 = Tiw1 * sensors_ptr_->imu.Tib.Inverse();

    GyroWheel3Dfactorij factor;
    factor.Evaluate(Tbw0, Tbw1, sensors_ptr_->diff_wheel.Tob, gyrwheel_ptr_);
    
    if (H0) {
        // dr_dTiw = [dr_dRbw, dr_dtbw] * [dRbw_dRiw, dRbw_dtiw; dtbw_dRiw, dtbw_dtiw] = [dr_dRbw * dRbw_dRiw + dr_dtbw * dtbw_dRiw, dr_dtbw * dtbw_dtiw]
        // [Rbw*dRbw, tbw + dtbw] = [Riw0, tiw0] * [dR, dt] * [Rbi, tbi] = [Riw0 * dR, Riw0 * dt + tiw0] * [Rbi, tbi]
        // = [Riw0 * dR * Rbi, Riw0 * dR * tbi + Riw0 * dt + tiw0]
        // tbw + dtbw = Riw0 * dR * tbi + Riw0 * dt + tiw0 = Riw0 * (I + drv^) * tbi + Riw0 * dt + tiw0
        // = Riw0 * tbi + tiw0 + Riw0 * drv^ * tbi + Riw0 * dt
        // dtbw = Riw0 * drv^ * tbi + Riw0 * dt = -Riw0 * tbi^ * drv + Riw0 * dt = -Riw0 * (-Rib^t * tib)^ * drv + Riw0 * dt
        // = Riw0 * (Rib^t * tib)^ * drv + Riw0 * dt
        // Rbw * dRbw = Riw0 * dR * Rbi = Riw0 * Rbi * exp(Rbi^t * drv) = Riw0 * Rib^t * exp(Rib * drv)
        Eigen::Matrix<double, 3, 6> dr_dTiw0;
        dr_dTiw0.leftCols(3) = factor.Ji.leftCols(3) * Tib.R();
        dr_dTiw0.leftCols(3) += factor.Ji.rightCols(3) * Tiw0.R() * utils::DMmath::SkewMatrix(Tib.R().transpose() * Tib.t());
        dr_dTiw0.rightCols(3) = factor.Ji.rightCols(3) * Tiw0.R();
        (*H0) = dr_dTiw0;
    }
    if (H1) {
        Eigen::Matrix<double, 3, 6> dr_dTiw1;
        dr_dTiw1.leftCols(3) = factor.Jj.leftCols(3) * Tib.R();
        dr_dTiw1.leftCols(3) += factor.Jj.rightCols(3) * Tiw1.R() * utils::DMmath::SkewMatrix(Tib.R().transpose() * Tib.t());
        dr_dTiw1.rightCols(3) = factor.Jj.rightCols(3) * Tiw1.R();
        (*H1) = dr_dTiw1;
    }

    return factor.residual;
}
} // slam