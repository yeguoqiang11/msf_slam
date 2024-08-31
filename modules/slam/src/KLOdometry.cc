#include "slam/KLOdometry.h"

namespace slam {
KLDivergenceOdometry::KLDivergenceOdometry(const nlohmann::json &config, std::shared_ptr<Sensors> sensor_ptr) {
    sensor_ptr_ = std::move(sensor_ptr);

    double map_voxel_size = config["localmap_voxel_size"];
    localmap_ptr_ = std::make_shared<LocalMap>(map_voxel_size);

    damping_val_ = 1.0e-05;
    param_.huber_thres_ = 0.07;
    param_.GM_kernel_rho_ = 0.003;
    outlier_thres_ = 0.3;
    voxelpoint_num_thres_ = 3;
    surface_ratio_thres_ = 0.15;
    plane_shape_weight_ = 0.005;

    Reset();

    LOG(INFO) << "local map voxel size: " << map_voxel_size;
    LOG(INFO) << "damping val: " << damping_val_;
    LOG(INFO) << "KLodometry huber threshold: " << param_.huber_thres_;
}

void KLDivergenceOdometry::Reset() {
    localmap_ptr_->Reset();

    is_keyframe_ = false;
    is_initialized_ = false;
    timestamp_ = -1;

    Vw_.setZero();
    Tbw_.SetIdentity();
    last_Tbw_.SetIdentity();
    last_keyTbw_.SetIdentity();
}

// frame point cloud: left hand coordinate: x: right; y: down; z: forward
void KLDivergenceOdometry::LoadFrame(std::shared_ptr<Frame> frame_ptr, const double &timestamp) {
    timestamp_ = timestamp;

    if (!is_initialized_) {
        Tbw_ = frame_ptr->Tbw();
        localmap_ptr_->Tbw = frame_ptr->Tbw();
        is_initialized_ = true;
    }

    utils::Timer timer;
    timer.Start();
    double dt;

    if (!localmap_ptr_->map.Empty()) {
        // Optimization(frame);
        SurfaceBasedOptimization(frame_ptr);
        dt = timer.End("kl optimization");
        LOG(INFO) << "kl optimization time cost: " << dt;
    }

    LocalMapUpdate(frame_ptr);
    dt = timer.End("local map updation time cost");
    LOG(INFO) << "local map updation time cost: " << dt;
    std::cout << "-------local map size: " << localmap_ptr_->map.Size() << std::endl;
    LOG(INFO) << "local map size: " << localmap_ptr_->map.Size();
}

void KLDivergenceOdometry::LocalMapUpdate(std::shared_ptr<Frame> frame_ptr) {
    is_keyframe_ = false;
    utils::Transform3d dT = frame_ptr->Tbw().Inverse() * localmap_ptr_->Tbw;
    double dt = dT.t().norm();
    double dtheta = utils::DMmath::LogSO3(dT.R()).norm();

    if (!localmap_ptr_->map.Empty() && dtheta < 5. * DEG2RAD && dt < 0.15) {
        AddFrame2LocalMap(frame_ptr, frame_ptr->Tbw());
        frames_.push_back(frame_ptr);
        return;
    }

    // update local map Tbw
    localmap_ptr_->Reset(frame_ptr->Tbw());

    std::vector<std::shared_ptr<Frame>> frames;
    frames.swap(frames_);
    // for (int i = 0; i < frames.size() && i < 3; i++) {
    //     std::shared_ptr<Frame> tmp_frame_ptr = frames[frames.size() - i - 1];
    //     const utils::Transform3d &Tbw = tmp_frame_ptr->Tbw();
    //     AddFrame2LocalMap(tmp_frame_ptr, Tbw);
    // }

    LOG(INFO) << "local map updating with size: " << localmap_ptr_->map.Size();
    const utils::Transform3d &Tbw = frame_ptr->Tbw();
    AddFrame2LocalMap(frame_ptr, Tbw);
    LOG(INFO) << "local map updating of new frame with size: " << localmap_ptr_->map.Size();
    frames_.push_back(frame_ptr);
    is_keyframe_ = true;
    frame_ptr->SetKeyFrame(true);
    last_keyTbw_ = Tbw;
}

void KLDivergenceOdometry::AddFrame2LocalMap(std::shared_ptr<Frame> frame_ptr, const utils::Transform3d &Tbw) {
    VoxelGrid &voxelgrid_ref = frame_ptr->PointCloudFrameRef().VoxelGridObject();
    VoxelGrid::MapType::iterator iter = voxelgrid_ref.Begin();
    // [Rwb, twb] = [Rbw^t, -Rbw^t * tbw]
    // [Rwl, twl] * [Rbw, tbw] = [Rlw^t, -Rlw^t * tlw] * [Rbw, tbw]
    // = [Rlw^t * Rbw, Rlw^t * tbw - Rlw^t * tlw] = [Rlw^t * Rbw, Rlw^t * (tbw - tlw)]
    utils::Transform3d Tbl = localmap_ptr_->Tbw.Inverse() * Tbw;
    for(; iter != voxelgrid_ref.End(); iter++) {
        // pt0 = R10 * pt1 + t10; pt0_mean = R10 * pt1_mean + t10 
        // dpt0 = pt0 - pt0_mean = R10 * pt1 + t10 - R10 * pt1_mean - t10 = R10 * (pt1 - pt1_mean)
        // cov0 = sum(dpt0 * dpt0^t) = sum(R10 * (pt1 - pt1_mean) * (pt1 - pt1_mean)^t * R10^t)
        // = R10 * sum((pt1 - pt1_mean) * (pt1 - pt1_mean)^t) * R10^t = R10 * cov1 * R10^t
        Eigen::Vector3d c1 = Tbl.transform(iter->second.center);
        Eigen::Matrix3d cov1 = Tbl.R() * iter->second.cov * Tbl.R().transpose();
        localmap_ptr_->map.AddDistribution(c1, cov1, iter->second.N);
    }
}

static std::vector<double> costs;
void KLDivergenceOdometry::SurfaceBasedOptimization(std::shared_ptr<Frame> frame_ptr) {
    std::cout << "***********KLOdometry*************" << std::endl;
    Eigen::Matrix<double, 9, 9> H, I9; H.setZero(); I9.setIdentity();
    Eigen::Matrix<double, 9, 1> b; b.setZero();
    double cost = 0.0;
    utils::Transform3d Tbw = frame_ptr->Tbw();
    utils::Timer ticker;

    // calculate information matrix
    Eigen::Matrix3d wheel_cov = frame_ptr->WheelSharedPtr()->Cov() + 1.0e-15 * Eigen::Matrix3d::Identity();
    Eigen::Matrix3d gyr_wheel_cov = frame_ptr->GyroWheelSharedPtr()->Cov() + 1.0e-15 * Eigen::Matrix3d::Identity();
    Eigen::Matrix<double, 15, 15> imu_cov = frame_ptr->ImuSharedPtr()->Cov();
    Eigen::Matrix<double, 6, 6> gyro_cov;
    gyro_cov.topLeftCorner(3, 3) = imu_cov.topLeftCorner(3, 3);
    gyro_cov.topRightCorner(3, 3) = imu_cov.block<3, 3>(0, 9);
    gyro_cov.bottomLeftCorner(3, 3) = imu_cov.block<3, 3>(9, 0);
    gyro_cov.bottomRightCorner(3, 3) = imu_cov.block<3, 3>(9, 9);

    double surface_info = sensor_ptr_->tofcams[0].noise(0) + sensor_ptr_->tofcams[0].noise(1) +
        sensor_ptr_->tofcams[0].noise(2);//1000.;
    surface_info *= surface_info;
    surface_info /= 9.0;
    surface_info = 0. / surface_info;
    Eigen::Matrix<double, 3, 3> gyr_wheel_info = 1. * gyr_wheel_cov.inverse();
    Eigen::Matrix<double, 4, 4> ground_info = 10000. * surface_info * Eigen::Matrix<double, 4, 4>::Identity();
    Eigen::Matrix<double, 6, 6> gyro_info = 0. * gyro_cov.inverse();

    std::cout << "surface info: " << surface_info << std::endl;
    std::cout << "gyr wheel info: " << gyr_wheel_info << std::endl;

    // prior pose calculation
    utils::Transform3d Tlw = localmap_ptr_->Tbw;
    utils::Transform3d Tbl = Tlw.Inverse() * Tbw;
    utils::Transform3d last_Tbl = Tlw.Inverse() * last_Tbw_;
    Eigen::Vector3d bg, ba;
    bg = frame_ptr->Bg();

    // save points for show
    origin_pts_.clear();
    matched_pts_.clear();
    origin_ground_.fill(-1000);
    matched_ground_.fill(-1000);

    // surface constraint
    int inlier_count = SurfaceCost(frame_ptr, H, b, cost, Tbl, surface_info);
    // if (inlier_count >= 5) {
    //     cost /= inlier_count;
    //     H /= inlier_count;
    //     b /= inlier_count;
    // }

    // std::cout << "cost: " << cost << "," << inlier_count << std::endl;

    GroundConstraint(frame_ptr, H, b, cost, Tbl, Tlw, ground_info);
    GyroCost(frame_ptr, H, b, cost, Tbl.R(), last_Tbl.R(), bg, gyro_info);
    GyroWheelCost(frame_ptr, H, b, cost, Tbl, last_Tbl, gyr_wheel_info);

    std::cout << "init cost inlier_count: " << cost << "; " << inlier_count << std::endl;
    std::cout << "------------------------------" << std::endl;
    origin_pts1_.swap(origin_pts_);
    matched_pts1_.swap(matched_pts_);

    double lambda = 1.0;
    for (int i = 0; i < 6; i++) {
        origin_pts1_.clear();
        matched_pts1_.clear();
        origin_ground1_.fill(-1000);
        matched_ground1_.fill(-1000);

        // calculate increment
        Eigen::Matrix<double, 9, 9> D = H.diagonal().asDiagonal();
        D += I9;
        Eigen::Matrix<double, 9, 1> dx = (H + lambda * I9).ldlt().solve(-b);

        // calculate new pose
        utils::Transform3d Tbl_hat = Tbl * dx.topRows(6);
        Eigen::Vector3d bg_hat = bg + dx.bottomRows(3);
        Eigen::Matrix<double, 9, 9> H_hat; H_hat.setZero();
        Eigen::Matrix<double, 9, 1> b_hat; b_hat.setZero();
        double cost0 = 0.;

        // add voxel ndt constraint
        inlier_count = SurfaceCost(frame_ptr, H_hat, b_hat, cost0, Tbl_hat, surface_info);
        // if (inlier_count >= 5) {
        //     cost0 /= inlier_count;
        //     H_hat /= inlier_count;
        //     b_hat /= inlier_count;
        // }

        // add ground plane constraint
        GroundConstraint(frame_ptr, H_hat, b_hat, cost0, Tbl_hat, Tlw, ground_info);
        GyroCost(frame_ptr, H_hat, b_hat, cost0, Tbl_hat.R(), last_Tbl.R(), bg_hat, gyro_info);
        GyroWheelCost(frame_ptr, H_hat, b_hat, cost0, Tbl_hat, last_Tbl, gyr_wheel_info);

        if (cost0 < cost) {
            cost = cost0;
            H = H_hat;
            b = b_hat;
            Tbl = Tbl_hat;
            bg = bg_hat;

            lambda = lambda < 1.e-06? 1.e-06 : lambda * 0.1;

            origin_ground_.swap(origin_ground1_);
            matched_ground_.swap(matched_ground1_);
            origin_pts_.swap(origin_pts1_);
            matched_pts_.swap(matched_pts1_);
            origin_pts1_.clear();
            matched_pts1_.clear();
        } else {
            lambda = lambda > 1000? 1000 : lambda * 10;
        }
        std::cout << i << "th cost and best cost: " << cost0 << "; " << cost << ";lambda: " << lambda << "; inlier num: " << inlier_count << std::endl;
        std::cout << "------------------------------------" << std::endl;
    }

    // costs.push_back(cost);
    // std::sort(costs.begin(), costs.end(), std::greater<double>());
    // std::cout << "biggest cost: " << costs.front() << std::endl;
    // std::cout << "final Tbl: " << Tbl << std::endl;
    // utils::Transform3d dT; dT.SetIdentity();
    // Eigen::Matrix<double, 6, 1> dv;
    // dv << 0., 0.05, 0., 0.02, 0., -0.02;
    // dT *= dv;
    // std::cout << "dT: " << dT.Inverse() << std::endl;


    Tbw = Tlw * Tbl;
    Tbw.Normalization();
    std::cout << "final Tbw: " << Tbw << std::endl;

    std::cout << "bg: " << bg.transpose() << std::endl;
    LOG(INFO) << "odom Tbw: " << Tbw;

    frame_ptr->SetPose(Tbw.R(), Tbw.t());
    frame_ptr->SetPose(Tbw); 
    // frame.SetBga(Eigen::Vector3d(0., 0., 0.), bg);
    frame_ptr->SetBg(bg);
    Tbw_ = Tbw;
    last_Tbw_ = Tbw_;
}

void KLDivergenceOdometry::GroundConstraint(std::shared_ptr<Frame> frame_ptr, Eigen::Matrix<double, 9, 9> &H, Eigen::Matrix<double, 9, 1> &b, double &cost,
                                            const utils::Transform3d &Tbl, const utils::Transform3d &Tlw, Eigen::Matrix<double, 4, 4> &info_mat) {
    std::shared_ptr<GroundInfo> ground = frame_ptr->PointCloudFrameRef().GroundPtr();
    if (ground == nullptr) return;
    std::shared_ptr<GroundInfo> world_ground = grounds_ptr_->FindGroundPlane(ground);
    if (world_ground == nullptr) return;

    utils::Transform3d Tbw = Tlw * Tbl;

    Eigen::Vector3d center0 = Tbw.transform(ground->Center());
    Eigen::Vector3d nv0 = Tbw.R() * ground->NormalVector();
    Eigen::Vector3d center_w = world_ground->Center();
    Eigen::Vector3d nv_w = world_ground->NormalVector();
    Eigen::Vector3d dist = center0 - center_w;

    // residual  
    Eigen::Vector4d residual;
    residual.topRows(3) = nv_w.cross(nv0);
    residual(3) = nv_w.dot(dist);

    // Jacobian
    // r + dr = nvw^ * nv0 = nvw^ * Rlw * Rbl * (I + drv^) * nv
    // = nvw^ * Rlw * Rbl * nv + nvw^ * Rlw * Rbl * drv^ * nv
    // = r - nvw^ * Rlw * Rbl * nv^ * drv
    // dr = -nvw^ * Rlw * Rbl * nv^ * drv
    // dr/drv = -nvw^ * Rlw * Rbl * nv^ = -nvw^ * Rbw * nv^
    // r + dr = nvw^t * dist = nvw^t * (c0 + dc0 - cw)
    // = nvw^t * (Rlw * Rbl * (I + drv^) * c + Rlw * (tbl + dt) + tlw - cw)
    // = nvw^t * (c0 - cw) + nvw^t * (Rlw * Rbl * drv^ * c + Rlw * dt)
    // = r - nvw^t * (Rlw * Rbl * c^ * drv) + nvw^t * Rlw * dt;
    // dr/drv = -nvw^t * Rlw * Rbl * c^
    // dr/dt = nvw^t * Rlw
    Eigen::Matrix<double, 4, 6> jacobian;
    jacobian.setZero();
    jacobian.topLeftCorner(3, 3) = -utils::DMmath::SkewMatrix(nv_w) * Tbw.R() * utils::DMmath::SkewMatrix(ground->NormalVector());
    jacobian.bottomLeftCorner(1, 3) = -nv_w.transpose() * Tbw.R() * utils::DMmath::SkewMatrix(ground->Center());
    jacobian.bottomRightCorner(1, 3) = nv_w.transpose() * Tlw.R();

    H.topLeftCorner(6, 6) += jacobian.transpose() * info_mat * jacobian;
    b.topRows(6) += jacobian.transpose() * info_mat * residual;
    cost += residual.transpose() * info_mat * residual;

    origin_ground1_ << nv0, center0;
    matched_ground1_ << nv_w, center_w;
}

int KLDivergenceOdometry::SurfaceShapeCost(std::shared_ptr<Frame> frame_ptr, Eigen::Matrix<double, 6, 6> &H, Eigen::Matrix<double, 6, 1> &b, double &cost,
                                   const Eigen::Matrix3d &Rbl, const Eigen::Vector3d &tbl) {
    int inlier_count = 0;
    Eigen::Matrix3d I3; I3.setIdentity();
    std::vector<double> angles;
    Eigen::Matrix3d vt;
    Eigen::Vector3d eig, nvec;
    VoxelTreeMap::VoxelType *result;
    VoxelGrid::MapType::iterator iter = frame_ptr->PointCloudFrameRef().VoxelGridObject().Begin();
    for (; iter != frame_ptr->PointCloudFrameRef().VoxelGridObject().End(); iter++) {
        if (!iter->second.eig_cal_flag_) continue;
        const Eigen::Vector3d &pt0 = iter->second.center;
        Eigen::Vector3d ptl = Rbl * pt0 + tbl;
        result = localmap_ptr_->map.findNeighborsByHash(ptl);
        if (result != nullptr) {
            if (!result->GetEigenInfo(eig, nvec)) {                
                utils::DMmath::Matrix3dEigenDecomposition(result->cov, eig, vt);
                nvec = vt.row(2).transpose();
                result->SetEigenInfo(eig, nvec);
            }

            const Eigen::Vector3d &center = result->center;

            Eigen::Vector3d eig0, nv0;
            iter->second.GetEigenInfo(eig0, nv0);

            if (fabs(eig(1)) < 1.e-05) continue;
            double ratio = eig(2) / eig(1);
            if (ratio > surface_ratio_thres_) continue;

            // r0 = (ptl - Rbl * pt - tbl) * nv;
            // r1 = (nv X (Rbl * nv0)).norm();
            Eigen::Vector4d residual;
            Eigen::Vector3d dist = center - ptl;
            residual(0) = (dist.transpose() * nvec);
            Eigen::Vector3d ev = nvec.cross(Rbl * nv0);
            residual.bottomRows(3) = plane_shape_weight_ * ev;

            double cost0 = residual.squaredNorm();
            double w = 1.0;
            if (cost0 > param_.huber_thres_) {
                w = param_.huber_thres_ / cost0;
            }

            // jacobian
            // r0 + dr0 = (ptl - Rbl * (I + drv^) * pt - tbl - dt)^t * nv
            // = (ptl - Rbl * pt - tbl)^t * nv - (Rbl * drv^ * pt - dt)^t * nv
            // = r0 - (Rbl * drv^ * pt)^t * nv - dt^t * nv
            // dr0 = -nv * Rbl * drv^ * pt - nv^t * dt = nv * Rbl * pt^ * drv - nv^t * dt
            // v = nv^ * (Rbl * nv0)
            // r1  = sqrt(v0 * v0 + v1 * v1 + v2 * v2)
            // dr1 / dv = 0.5 * [2 * v0, 2 * v1, 2 * v2] / ||v|| = v^t / ||v||
            // v + dv = nv^ * (Rbl * (I + drv^) * nv0) = nv^ * (Rbl * nv0 + Rbl * drv^ * nv0)
            // = v + nv^ * Rbl * drv^ * nv0
            // dv = nv^ * Rbl * drv^ * nv0 = -nv^ * Rbl * nv0^ * drv
            Eigen::Matrix<double, 4, 6> dr_dx;
            dr_dx.topLeftCorner(1, 3) = nvec.transpose() * Rbl * utils::DMmath::SkewMatrix(pt0);
            dr_dx.topRightCorner(1, 3) = -nvec.transpose();

            Eigen::Matrix<double, 3, 3> dev_dxv = -utils::DMmath::SkewMatrix(nvec) * Rbl * utils::DMmath::SkewMatrix(nv0);
            dr_dx.bottomLeftCorner(3, 3) = plane_shape_weight_ * dev_dxv;
            dr_dx.bottomRightCorner(3, 3).setZero();

            H += w * dr_dx.transpose() * dr_dx;
            b += w * dr_dx.transpose() * residual;
            cost += w * cost0;
            inlier_count++;
        }
    }

    return inlier_count;
}

int KLDivergenceOdometry::SurfaceCost(std::shared_ptr<Frame> frame_ptr, Eigen::Matrix<double, 9, 9> &H, Eigen::Matrix<double, 9, 1> &b, double &cost,
                                      const utils::Transform3d &Tbl, double info_mat) {
    int inlier_count = 0;
    Eigen::Matrix3d I3; I3.setIdentity();
    const utils::Transform3d Tlw = localmap_ptr_->Tbw;

    // std::cout << "----------------" << std::endl;
    Eigen::Matrix3d vt;
    Eigen::Vector3d eig, nvec;
    VoxelTreeMap::VoxelType *result;
    VoxelGrid::MapType::iterator iter = frame_ptr->PointCloudFrameRef().VoxelGridObject().Begin();
    for (; iter != frame_ptr->PointCloudFrameRef().VoxelGridObject().End(); iter++) {
        if (iter->second.N < voxelpoint_num_thres_) continue;
        int N0 = iter->second.N;
        Eigen::Vector3d pt0 = iter->second.center;

        double depth_inv = 1.; //1. / pt0(2);
        Eigen::Vector3d ptl = Tbl.transform(pt0);
        result = localmap_ptr_->map.findNeighborsByHash(ptl);
        if (result != nullptr) {
            if (!result->GetEigenInfo(eig, nvec)) {
                result->EigenCal();
                if (!result->GetEigenInfo(eig, nvec)) continue;
            }

            const Eigen::Vector3d &center = result->center;

            // C * v = v * S * vt * v = v * S
            // C * vn = sn * vn
            Eigen::Vector3d nv = nvec;// vt.row(2).transpose();
            if (fabs(nv(1)) > 0.3) continue;

            // r = (ptl - Rbl * pt - tbl) * nv;
            Eigen::Vector3d dist = center - ptl;
            if (dist.squaredNorm() > outlier_thres_) continue;
            double residual = dist.dot(nv);
            residual *= depth_inv;

            double cost0 = residual * info_mat * residual;

            double w = 1.0;
            if (fabs(residual) > param_.huber_thres_) {
                w = param_.huber_thres_ / fabs(residual);
            }

            // jacobi
            // e = ptl - Rbl * pt - tbl
            // r = e^t * nv => r + dr = (e^t + de^t) * nv = e^t * nv + de^t * nv;
            // dr = de^t * nv = nv^t * de
            // e + de = ptl - Rbl * exp(drv) * pt - tbl - dt
            // e + de = plt - Rbl * (I + drv^) * pt - tbl - dt
            // e + de = plt - Rbl * pt - tbl - Rbl * drv^ * pt - dt
            // de = -Rbl * drv^ * pt - dt
            // de = Rbl * pt^ * drv - dt
            // r +dr = r + J * dx = 0;
            Eigen::Matrix<double, 1, 3> dr_de = nv.transpose() * depth_inv;
            Eigen::Matrix<double, 3, 6> de_dx;
            de_dx.topLeftCorner(3, 3) = Tbl.R() * utils::DMmath::SkewMatrix(pt0);
            de_dx.topRightCorner(3, 3) = -I3;
            Eigen::Matrix<double, 1, 6> dr_dx = dr_de * de_dx;

            H.topLeftCorner(6, 6) += w * dr_dx.transpose() * info_mat * dr_dx;
            b.topRows(6) += w * dr_dx.transpose() * info_mat * residual;
            cost += w * cost0;
            inlier_count++;
            origin_pts1_.push_back(Tlw.transform(ptl));
            matched_pts1_.push_back(Tlw.transform(center));
        }
    }
    return inlier_count;
}

void KLDivergenceOdometry::GyroCost(std::shared_ptr<Frame> frame_ptr, Eigen::Matrix<double, 9, 9> &H, Eigen::Matrix<double, 9, 1> &b, double &cost,
    const Eigen::Matrix3d &Rbl, const Eigen::Matrix3d &last_Rbl, const Eigen::Vector3d &bg, Eigen::Matrix<double, 6, 6> &info_mat) {
    std::shared_ptr<Imu> imu_ptr = frame_ptr->ImuSharedPtr();
    if (imu_ptr == nullptr) return;
    if (imu_ptr->Dt() <= 0.0001) return;
    const Eigen::Matrix3d &Rib = sensor_ptr_->imu.Tib.R();

    Eigen::Vector3d bgi = imu_ptr->Bg();
    GyroFactor factor;
    factor.Evaluate(last_Rbl, Rbl, bg, bgi, Rib, imu_ptr);

    Eigen::Matrix<double, 6, 9> J; J.setZero();
    J.topLeftCorner(3, 3) = factor.Jj.topLeftCorner(3, 3);
    J.rightCols(3) = factor.Ji.rightCols(3);
    
    H += J.transpose() * info_mat * J;
    b += J.transpose() * info_mat * factor.residual;
    cost += factor.residual.transpose() * info_mat * factor.residual;
    // H.topLeftCorner(3, 3) += factor.Jj.topLeftCorner(3, 3).transpose() * info_mat.topLeftCorner(3, 3) * factor.Jj.topLeftCorner(3, 3);
    // // H.bottomRightCorner(3, 3) += factor.Ji.bottomRightCorner(3, 3).transpose() * info_mat.bottomLeftCorner(3, 3)
    // b.topRows(3) += factor.Jj.topLeftCorner(3, 3).transpose() * info_mat.topLeftCorner(3, 3) * factor.residual.topRows(3);
    // cost += factor.residual.topRows(3).transpose() * info_mat.topLeftCorner(3, 3) * factor.residual.topRows(3);
}

void KLDivergenceOdometry::WheelCost(std::shared_ptr<Frame> frame_ptr, Eigen::Matrix<double, 9, 9> &H, Eigen::Matrix<double, 9, 1> &b, double &cost,
                                     const utils::Transform3d &Tbl, const utils::Transform3d &last_Tbl, Eigen::Matrix<double, 3, 3> &info_mat) {
    std::shared_ptr<EulerWMPreintegration> wheel_ptr = frame_ptr->WheelSharedPtr();
    if (wheel_ptr == nullptr) return;
    DiffWheelModel3DFactor factor;
    factor.Evalute(last_Tbl.R(), last_Tbl.t(), Tbl.R(), Tbl.t(), sensor_ptr_->diff_wheel.Tob.R(),
                   sensor_ptr_->diff_wheel.Tob.t(), wheel_ptr);
    H.topLeftCorner(6, 6) += factor.Jj.transpose() * info_mat * factor.Jj;
    b.topRows(6) += factor.Jj.transpose() * info_mat * factor.residual;
    cost += factor.residual.transpose() * info_mat * factor.residual;
}

void KLDivergenceOdometry::GyroWheelCost(std::shared_ptr<Frame> frame_ptr, Eigen::Matrix<double, 9, 9> &H, Eigen::Matrix<double, 9, 1> &b, double &cost,
                                         const utils::Transform3d &Tbl, const utils::Transform3d &last_Tbl, Eigen::Matrix<double, 3, 3> &info_mat) {
    std::shared_ptr<GyroWheelModelPreintegration> gyr_wheel_ptr = frame_ptr->GyroWheelSharedPtr();
    if (gyr_wheel_ptr == nullptr) return;
    GyroWheel3DFactor factor;
    factor.Evaluate(last_Tbl, Tbl, sensor_ptr_->diff_wheel.Tob, gyr_wheel_ptr);
    H.topLeftCorner(6, 6) += factor.Jj.transpose() * info_mat * factor.Jj;
    b.topRows(6) += factor.Jj.transpose() * info_mat * factor.residual;
    cost += factor.residual.transpose() * info_mat * factor.residual;
}

int KLDivergenceOdometry::DistributionsCost(std::shared_ptr<Frame> frame_ptr, Eigen::Matrix<double, 6, 6> &H, Eigen::Matrix<double, 6, 1> &b, double &cost,
                                            const Eigen::Matrix3d &Rbl, const Eigen::Vector3d &tbl) {
    int inlier_count = 0;
    Eigen::Matrix3d I3; I3.setIdentity();
    VoxelTreeMap::VoxelType result;
    VoxelGrid::MapType::iterator iter = frame_ptr->PointCloudFrameRef().VoxelGridObject().Begin();
    for (; iter != frame_ptr->PointCloudFrameRef().VoxelGridObject().End(); iter++) {
        if (iter->second.N < voxelpoint_num_thres_) continue;
        const Eigen::Vector3d pt0 = iter->second.center;
        double depth = pt0.norm();
        Eigen::Vector3d ptl = Rbl * pt0 + tbl;
        if (localmap_ptr_->map.findNeighborsByHash(ptl, result)) {
            Eigen::Vector3d residual = result.center - ptl;
            if (residual.norm() > outlier_thres_) continue;
            residual /= depth;

            double cov_norm = result.cov.norm();
            if (cov_norm < 1.e-5) continue;
            Eigen::Matrix3d Cpq;
            Eigen::Matrix3d Ci = utils::DMmath::Matrix3dInverse(result.cov * (1. / cov_norm) + damping_val_ * I3);
            if (Ci.norm() <= std::numeric_limits<double>::epsilon() * 1000) continue;
            Cpq = Ci / Ci.norm();

            double cost0 = residual.transpose() * Cpq * residual;
            double w = 1.0;
            if (cost0 > param_.huber_thres_) {
                w = param_.huber_thres_ / cost0;
            }

            Eigen::Vector3d residual0 = Cpq * residual;
            origin_pts_.push_back(localmap_ptr_->Tbw.transform(ptl));
            matched_pts_.push_back(localmap_ptr_->Tbw.transform(result.center));

            Eigen::Matrix<double, 3, 6> J;
            J.topLeftCorner(3, 3) = -Rbl * utils::DMmath::SkewMatrix(iter->second.center);
            J.topRightCorner(3, 3).setIdentity();
            J /= depth;

            H += w * J.transpose() * Cpq * J;
            b += w * J.transpose() * Cpq * residual;
            cost += w * cost0;
            inlier_count++;
        }
    }

    return inlier_count;
}

void KLDivergenceOdometry::MotionConstraintCost(const Eigen::Matrix3d &Rbl, const Eigen::Vector3d &dtl, Eigen::Matrix<double, 6, 6> &H,
                                                Eigen::Matrix<double, 6, 1> &b, double &cost, double w) {
    Eigen::Vector3d dtb = Rbl.transpose() * dtl;
    // motion constraint
    // dtl = tbl - tbl_last;
    // => dtb = Rbl^t * dtl => dtb(0) = 0, dtb(1) = 0;
    // dtb + ddtb = (I - dR) * Rbl^t * (tbl + dt - tbl_last) = Rbl^t * dtl + Rbl^t * dt - dR * Rbl^t * dtl
    // = Rbl^t * dtl + Rbl^t * dt + (Rbl^t * dtl)^ * drvec
    // ddtb = Rbl^t * dt + dtb^ * drvec
    // r = [dtb(0); dtb(1)]
    // dr / dtb = [1, 0, 0; 0, 1, 0]
    // dtb / dt = Rbl^t
    // dtb / drvec = (Rbl^t * dtl)^ = dtb^
    Eigen::Vector2d residual_b = dtb.topRows(2);
    Eigen::Matrix<double, 2, 3> dr_dtb; dr_dtb.fill(0.);
    dr_dtb(0, 0) = 1.; dr_dtb(1, 1) = 1.;

    Eigen::Matrix<double, 3, 6> dtb_dx; dtb_dx.fill(0.);
    dtb_dx.topLeftCorner(3, 3) = utils::DMmath::SkewMatrix(dtb);
    dtb_dx.topRightCorner(3, 3) = Rbl.transpose();
    Eigen::Matrix<double, 2, 6> J = dr_dtb * dtb_dx;
    H += w * J.transpose() * J;
    b += w * J.transpose() * residual_b;
    cost += w * residual_b.squaredNorm();
}

void KLDivergenceOdometry::GetCurrentFramePoints(std::vector<Eigen::Vector3d> &pts, std::vector<Eigen::Vector3d> &colors) {
    // VoxelGrid::MapType::iterator iter = voxelgrid_ptr_->Begin();
    // for (; iter != voxelgrid_ptr_->End(); iter++) {
    //     Eigen::Vector3d pt = iter->second.center;
    //     pts.push_back(Rbw_ * iter->second.center + tbw_);
    //     colors.push_back(Eigen::Vector3d(1, 1, 0));
    // }
    for (size_t i = 0; i < current_pts_.size(); i++) {
        pts.push_back(Tbw_.transform(current_pts_[i]));
        colors.push_back(Eigen::Vector3d(1, 0, 0));
    }
}

void KLDivergenceOdometry::GetLocalMapPoints(std::vector<Eigen::Vector3d> &pts, std::vector<Eigen::Vector3d> &colors) {
    std::cout << "local map1 size: " << localmap_ptr_->map.Size() << "; frame num: " << frames_.size() << std::endl;
    VoxelTreeMap::MapType::iterator iter = localmap_ptr_->map.begin();
    const utils::Transform3d Tbw = localmap_ptr_->Tbw;
    for (; iter != localmap_ptr_->map.end(); iter++) {
        for (size_t i = 0; i < iter->second.Size(); i++) {
            if (iter->second.Voxel(i).N >= 3) {
                pts.push_back(Tbw.transform(iter->second.Voxel(i).center));
                colors.push_back(Eigen::Vector3d(0, 1, 0));
            }
        }
    }
}

void KLDivergenceOdometry::GetMatchedPoints(std::vector<Eigen::Vector3d> &points, std::vector<Eigen::Vector3d> &colors) {
    for (size_t i = 0; i < origin_pts_.size(); i++) {
        points.push_back(origin_pts_[i]);
        colors.push_back(Eigen::Vector3d(1, 0, 0));
    }

    for (size_t i = 0; i < matched_pts_.size(); i++) {
        points.push_back(matched_pts_[i]);
        colors.push_back(Eigen::Vector3d(0, 0, 1));
    }
}

void KLDivergenceOdometry::GetGroundPoints(std::vector<Eigen::Vector3d> &points, std::vector<Eigen::Vector3d> &colors) {
    if (origin_ground_.norm() > 1000) return;
    double grid_size = 0.01;
    int num = 30;
    Eigen::Vector3d x(1, 0, 0);
    Eigen::Vector3d nv = origin_ground_.topRows(3);
    Eigen::Vector3d pt0 = origin_ground_.bottomRows(3);
    Eigen::Vector3d y = nv.cross(x);
    if (y.norm() <= 0.01) {
        x << 0, 0, 1;
        y = nv.cross(x);
    }
    y.normalize();
    x = nv.cross(y);
    for (int i = -num; i < num; i++) {
        for (int j = -num; j < num; j++) {
            Eigen::Vector3d pt = pt0 + i * grid_size * x + j * grid_size * y;
            points.push_back(pt);
            colors.push_back(Eigen::Vector3d(0, 1, 0));
        }
    }

    Eigen::Vector3d nv1 = matched_ground_.topRows(3);
    Eigen::Vector3d pt1 = matched_ground_.bottomRows(3);

    // nv X (pt - pt0) = 0; nv^ * pt = nv^ * pt0;
    // nv * (pt - pt1) = 0; nv^t * pt = nv^t * pt1;
    Eigen::Matrix<double, 4, 3> H;
    Eigen::Matrix<double, 4, 1> b;
    H.topLeftCorner(3, 3) = utils::DMmath::SkewMatrix(nv1);
    H.bottomLeftCorner(1, 3) = nv1.transpose();
    b.topRows(3) = H.topLeftCorner(3, 3) * pt0;
    b(3) = nv1.transpose() * pt1;
    Eigen::Vector3d ptc = (H.transpose() * H).lu().solve(H.transpose() * b);
    
    // direction
    x = nv1.cross(y);
    x.normalize();
    y = nv1.cross(x);
    for (int i = -num; i < num; i++) {
        for (int j = -num; j < num; j++) {
            Eigen::Vector3d pt = ptc + i * grid_size * x + j * grid_size * y;
            points.push_back(pt);
            colors.push_back(Eigen::Vector3d(0, 1, 1));
        }
    }
}

void KLDivergenceOdometry::GetMatchedDirection(std::vector<Eigen::Vector3d> &points, std::vector<Eigen::Vector3d> &dir) {
    if (points.size() != dir.size()) return;
    for (size_t i = 0; i < matched_pts_.size(); i++) {
        // if (matched_pts_[i](1) > 0.03) continue;
        // if (matched_pts_[i](0) > -0.75 || matched_pts_[i](0) < -0.76) continue;
        // if (matched_pts_[i](2) > 0.99 || matched_pts_[i](2) < 0.98) continue;
        points.push_back(matched_pts_[i]);
        dir.push_back(matched_dir_[i]);
    }
}

void KLDivergenceOdometry::RectifyLocalMapPose(const utils::Transform3d &Tbw) {
    utils::Transform3d Tbw0, Tbwi, dTi0;
    Tbw0 = localmap_ptr_->Tbw;
    localmap_ptr_->Tbw = Tbw;
    // for (size_t i = 0; i < frames_.size() - 1; i++) {
    //     Tbwi = frames_[i]->Tbw();
    //     dTi0 = Tbw0.Inverse() * Tbwi;
    //     Tbwi = Tbw * dTi0;
    //     frames_[i]->SetPose(Tbwi);
    // }
    // frames_.back()->SetPose(Tbw);
    Tbw_ = Tbw;
    last_Tbw_ = Tbw;
}
} // namespace slam