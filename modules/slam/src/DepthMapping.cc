#include "slam/DepthMapping.h"

namespace slam {
VoxelMapping::VoxelMapping(std::shared_ptr<GlobalVoxelMap> map_ptr, std::shared_ptr<Sensors> sensors_ptr)
: map_ptr_(std::move(map_ptr)),
  sensor_ptr_(std::move(sensors_ptr)) {
    posegraph_ptr_ = std::make_shared<PoseGraph>(map_ptr_, sensor_ptr_);
    damping_value_ = 1.0e-5;
    huber_thres_ = 0.03;
    pcd_num_thres_ = 6;
    surface_ratio_thres_ = 0.1;
    GM_rho_ = 0.003;
    last_loop_idx_ = 0;
}

void VoxelMapping::LoopOptimization(std::shared_ptr<Frame> frame_ptr) {
    int idx = LoopDetection(frame_ptr);
    // if (map_ptr_->FrameSize() >= 3) {
    //     idx = 0;
    // }
    Eigen::Matrix3d Rbjbi;
    Eigen::Vector3d tbjbi;
    if (LoopTransformationCal(frame_ptr, idx, Rbjbi, tbjbi)) {
        // if (LoopPoseGraph(frame_ptr, idx, Rbjbi, tbjbi)) {
        //     map_ptr_->ReMapFromKeyFrames();
        // }
    }
    std::cout << "loop matched idx: " << idx << "," << map_ptr_->FrameSize() << std::endl;
}

// Rs = [R(0), R(1), ..., R(n+1)]
// ts = [t(0), t(1), ..., t(n+1)]
// dRs = [R(0)^t * R(1), R(1)^t * R(2), ..., R(n)^t * R(n+1)]
// H = [x(1), x(2), ..., x(n+1)]
void VoxelMapping::LoopPoseCost(const std::vector<Eigen::Matrix3d> &Rs, const std::vector<Eigen::Vector3d> &ts, const std::vector<Eigen::Matrix3d> &dRs,
                                const std::vector<Eigen::Vector3d> &dts, const Eigen::Matrix3d &Rbjbi, const Eigen::Vector3d &tbjbi, Eigen::MatrixXd &H,
                                Eigen::VectorXd &b, double &cost) {
    Eigen::Matrix<double, 6, 6> Ji, Jj;
    Eigen::Matrix3d dR, Jr_inv;
    Eigen::Matrix<double, 6, 1> r;
    for (int n = 0; n < dRs.size() ; n++) {
        if (n == 0) {
            // (1)th pose to fixed(0 th) pose constraint
            // exp(r) * exp(Jr * dr) = [R1 * exp(drv1)]^t * R0 * dR
            // exp(r) * exp(Jr * dr) = exp(-drv1) * R1^t * R0 * dR
            // exp(r) * exp(Jr * dr) = exp(-drv1) * exp(r)
            // exp(Jr * dr) = exp(-r) * exp(-drv1) * exp(r) = exp(-exp(-r) * drv1)
            // Jr * dr = -exp(-r) * drv1 => dr = -Jr_inv * exp(-r) * drv1
            // r = log(R1^t * R0 * dR)
            dR = Rs[n + 1].transpose() * Rs[n] * dRs[n];
            r.topRows(3) = utils::DMmath::LogSO3(dR);
            Jr_inv = utils::DMmath::RightJacobianInverse(r.topRows(3));
            Ji.setZero();
            Ji.topLeftCorner(3, 3) = -Jr_inv * dR.transpose();

            // r = R0^t * (t1 - t0) - dt
            // r + dr = R0^t * (t1 + dt1 - t0)
            // dr = -R0^t * dt0
            r.bottomRows(3) = Rs[n].transpose() * (ts[n + 1] - ts[n]) - dts[n];
            Ji.bottomRightCorner(3, 3) = Rs[n].transpose();
            H.block<6, 6>(6 * n, 6 * n) += Ji.transpose() * Ji;
            b.block<6, 1>(6 * n, 0) += Ji.transpose() * r;
            cost += r.squaredNorm();
            continue;
        }
        if (n == dRs.size() - 1) {
            // loop constraint
            // exp(r) = Rn^t * R0 * Rbjbi
            // exp(r) * exp(Jr * dr) = exp(-drv0) * exp(r)
            // exp(Jr * dr) = exp(-exp(-r) * drv0)
            // Jr * dr = -exp(-r) * drv0 => dr = -Jr_inv * exp(-r) * drv0
            dR = Rs[n + 1].transpose() * Rs[0] * Rbjbi;
            r.topRows(3) = utils::DMmath::LogSO3(dR);
            Jr_inv = utils::DMmath::RightJacobianInverse(r.topRows(3));
            Ji.setZero();
            Ji.topLeftCorner(3, 3) = -Jr_inv * dR.transpose();

            // r = R0^t * (tn - t0) - tbjbi
            // r + dr = R0^t * (tn + dtn - t0)
            // dr = R0^t * dt0
            double w = dRs.size();
            r.bottomRows(3) = Rs[0].transpose() * (ts[n + 1] - ts[0]) - tbjbi;
            Ji.bottomRightCorner(3, 3) = Rs[0].transpose();
            H.block<6, 6>(6 * n, 6 * n) += w * Ji.transpose() * Ji;
            b.block<6, 1>(6 * n, 0) += w * Ji.transpose() * r;
            cost += w * r.squaredNorm();
            if (n == 1) continue;
        }

        // odometry constraint
        dR = Rs[n + 1].transpose() * Rs[n] * dRs[n];
        r.topRows(3) = utils::DMmath::LogSO3(dR);
        Jr_inv = utils::DMmath::RightJacobianInverse(r.topRows(3));
        // exp(r) = Rj^t * Ri * dRji
        // exp(r) * exp(Jr * dr) = exp(-drvj) * Rj^t * Ri * dRji
        // exp(r) * exp(Jr * dr) = exp(-drvj) * exp(r)
        // exp(Jr * dr) = exp(-r) * exp(-drvj) * exp(r) = exp(-exp(-r) * drvj)
        // Jr * dr = -exp(-r) * drvj => dr = -Jr_inv * exp(-r) * drvj
        Jj.setZero();
        Jj.topLeftCorner(3, 3) = -Jr_inv * dR.transpose();
        // exp(r) * exp(Jr * dr) = Rj^t * Ri * exp(drvi) * dRji
        // exp(r) * exp(Jr * dr) = Rj^t * Ri * dRji * exp(dRji^t * drvi) = exp(r) * exp(dRji^t * drvi)
        // Jr * dr = dRji^t * drvi => dr = Jr_inv * dRji^t * dRvi
        Ji.setZero();
        Ji.topLeftCorner(3, 3) = Jr_inv * dRs[n].transpose();

        // r = Ri^t * (tj - ti) - dtji
        r.bottomRows(3) = Rs[n].transpose() * (ts[n + 1] - ts[n]) - dts[n];
        // r + dr = exp(-drvi) * Ri^t * (tj - ti) - dtji
        // r + dr = (I - drvi^) * [Ri^t * (tj - ti)] - dtji
        // = r - drvi^ * [Ri^t * (tj - ti)]
        // dr = [Ri^t * (tj - ti)]^ * drvi
        Ji.bottomLeftCorner(3, 3) = utils::DMmath::SkewMatrix(Rs[n].transpose() * (ts[n + 1] - ts[n]));
        // r + dr = Ri^t * (tj + dtj - ti - dti) - dtji
        // r + dr = Ri^t * (tj - ti) - dtji + Ri^t * dtj - Ri^t * dti
        Ji.bottomRightCorner(3, 3) = -Rs[n].transpose();
        Jj.bottomRightCorner(3, 3) = Rs[n].transpose();

        H.block<6, 6>(6 * (n - 1), 6 * (n - 1)) += Ji.transpose() * Ji;
        H.block<6, 6>(6 * (n - 1), 6 * n) += Ji.transpose() * Jj;
        H.block<6, 6>(6 * n, 6 * (n - 1)) += Jj.transpose() * Ji;
        H.block<6, 6>(6 * n, 6 * n) += Jj.transpose() * Jj;
        b.block<6, 1>(6 * (n - 1), 0) += Ji.transpose() * r;
        b.block<6, 1>(6 * n, 0) += Jj.transpose() * r;
        cost += r.squaredNorm();
    }
}

bool VoxelMapping::LoopPoseGraph(std::shared_ptr<Frame> frame_ptr, int idx, const Eigen::Matrix3d &Rbjbi, const Eigen::Vector3d &tbjbi) {
    if (idx < 0) return false;
    std::cout << "---------------------loop pose graph----------------------" << std::endl;
    std::vector<std::shared_ptr<Frame>> &keyframes = map_ptr_->KeyFrames();
    int dim = keyframes.size() - idx;
    std::vector<Eigen::Matrix3d> Rs, dRs;
    std::vector<Eigen::Vector3d> ts, dts;
    Eigen::Matrix3d R, dR;
    Eigen::Vector3d t, dt;
    int idx0;
    int start_id = keyframes.size() - dim;
    // Rs = [R(0), R(1), ..., R(n+1)]
    // ts = [t(0), t(1), ..., t(n+1)]
    for (int n = 0; n < dim; n++) {
        idx0 = start_id + n;
        keyframes[idx0]->GetPose(R, t);
        Rs.push_back(R);
        ts.push_back(t);
    }

    frame_ptr->GetPose(R, t);
    Rs.push_back(R);
    ts.push_back(t);

    // dRs = [R(0)^t * R(1), R(1)^t * R(2), ..., R(n)^t * R(n+1)]
    for (int n = 0; n < Rs.size() - 1; n++) {
        dR = Rs[n].transpose() * Rs[n + 1];
        dt = Rs[n].transpose() * (ts[n + 1] - ts[n]);
        dRs.push_back(dR);
        dts.push_back(dt);
    }

    Eigen::MatrixXd H(dim * 6, dim * 6), Ix(dim * 6, dim * 6); Ix.setIdentity();
    Eigen::VectorXd b(dim * 6);
    H.setZero(); b.setZero();
    double cost = 0.;
    LoopPoseCost(Rs, ts, dRs, dts, Rbjbi, tbjbi, H, b, cost);

    double lambda = 1;
    Eigen::MatrixXd H_hat(dim * 6, dim * 6);
    Eigen::VectorXd b_hat(dim * 6);
    std::vector<Eigen::Matrix3d> Rs_hat; Rs_hat.resize(Rs.size());
    std::vector<Eigen::Vector3d> ts_hat; ts_hat.resize(ts.size());
    Rs_hat[0] = Rs[0];
    ts_hat[0] = ts[0];
    std::cout << "init cost: " << cost << std::endl;
    for (int i = 0; i < 6; i++) {
        Eigen::VectorXd dx = (H + lambda * Ix).ldlt().solve(-b);

        for (size_t j = 0; j < dim; j++) {
            Eigen::Matrix3d dR = utils::DMmath::RotationVector2Matrix(dx.middleRows(6 * j, 3));
            Rs_hat[j + 1] = Rs[j + 1] * dR;
            ts_hat[j + 1] = ts[j + 1] + dx.middleRows(6 * j + 3, 3);
        }
        double cost_hat = 0.;
        H_hat.setZero();
        b_hat.setZero();
        LoopPoseCost(Rs_hat, ts_hat, dRs, dts, Rbjbi, tbjbi, H_hat, b_hat, cost_hat);
        if (cost_hat < cost) {
            Rs.swap(Rs_hat);
            ts.swap(ts_hat);

            H = H_hat;
            b = b_hat;
            cost = cost_hat;

            lambda = lambda < 1.0e-05? lambda : lambda * 0.1;
        } else {
            lambda = lambda > 1000? lambda : lambda * 10;
        }
        std::cout << "loop pose graph cost and min cost: " << cost_hat << ";" << cost << "; lambda: " << lambda << std::endl;
    }

    for (int i = 0; i < dim; i++) {
        idx0 = start_id + i;
        keyframes[idx0]->SetPose(Rs[i], ts[i]);
    }
    frame_ptr->SetPose(Rs.back(), ts.back());

    return true;
}

bool VoxelMapping::LoopTransformationCal(std::shared_ptr<Frame> frame_ptr, int idx, Eigen::Matrix3d &Rbjbi, Eigen::Vector3d &tbjbi) {
    if (idx < 0) return false;
    std::cout << "--------------------loop optimization-----------------------------" << std::endl;
    std::shared_ptr<Frame> &frame_ptri = map_ptr_->KeyFrames()[idx];
    Eigen::Matrix3d Rbiw, Rbjw, Rbjbi0;
    Eigen::Vector3d tbiw, tbjw, tbjbi0;
    frame_ptr->GetPose(Rbjw, tbjw);
    frame_ptri->GetPose(Rbiw, tbiw);
    Rbjbi0 = Rbiw.transpose() * Rbjw;
    tbjbi0 = Rbiw.transpose() * (tbjw - tbiw);

    double pt_info_mat = 10.;
    double ground_Info_mat = 100.;

    std::cout << "init Rbjbi: \n" << Rbjbi0 << std::endl;
    std::cout << "init tbjbi: " << tbjbi0.transpose() << std::endl;

    Eigen::Matrix<double, 6, 6> H, I6; H.setZero(); I6.setIdentity();
    Eigen::Matrix<double, 6, 1> b; b.setZero();
    double cost = 0.;
    int inlier_count = FramesCost(frame_ptri, frame_ptr, Rbjbi0, tbjbi0, H, b, cost, pt_info_mat);
    if (inlier_count < 5) return false;
    cost /= inlier_count;
    H /= inlier_count;
    b /= inlier_count;
    FrameGroundCost(frame_ptri, frame_ptr, Rbjbi0, tbjbi0, H, b, cost, ground_Info_mat);
    Eigen::JacobiSVD<Eigen::Matrix3d> t_svd(H.bottomRightCorner(3, 3), Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::JacobiSVD<Eigen::Matrix3d> r_svd(H.topLeftCorner(3, 3), Eigen::ComputeFullU | Eigen::ComputeFullV);
    std::cout << "t H eig: " << t_svd.singularValues().transpose() << std::endl;
    std::cout << "r H eig: " << r_svd.singularValues().transpose() << std::endl;
    std::cout << "t nv: " << t_svd.matrixV().col(2).transpose() << std::endl;

    double lambda = 0.1;
    for (int i = 0; i < 6; i++) {
        loop_matched_pts_.clear();
        loop_origin_pts_.clear();

        Eigen::Matrix<double, 6, 1> dx = (H + lambda * I6).ldlt().solve(-b);

        Eigen::Matrix3d Rbjbi_hat = Rbjbi0 * utils::DMmath::RotationVector2Matrix(dx.topRows(3));
        Eigen::Vector3d tbjbi_hat = tbjbi0 + dx.bottomRows(3);
        double cost0 = 0.;
        Eigen::Matrix<double, 6, 6> H_hat; H_hat.setZero();
        Eigen::Matrix<double, 6, 1> b_hat; b_hat.setZero();
        inlier_count = FramesCost(frame_ptri, frame_ptr, Rbjbi_hat, tbjbi_hat, H_hat, b_hat, cost0, pt_info_mat);

        if (inlier_count < 5) return false;
        cost0 /= inlier_count;
        H_hat /= inlier_count;
        b_hat /= inlier_count;
        FrameGroundCost(frame_ptri, frame_ptr, Rbjbi_hat, tbjbi_hat, H_hat, b_hat, cost0, ground_Info_mat);

        if (cost0 < cost) {
            H = H_hat;
            b = b_hat;
            cost = cost0;

            Rbjbi0 = Rbjbi_hat;
            tbjbi0 = tbjbi_hat;

            lambda = lambda < 1.0e-05? lambda : lambda * 0.1;
        } else {
            lambda = lambda > 1000? lambda : lambda * 10;
        }
        std::cout << "loop cost and min cost: " << cost0 << ";" << cost << ";lambda: " << lambda << ";inlier count: " << inlier_count << std::endl;
    }

    std::cout << "final Rbjbi: \n" << Rbjbi0 << std::endl;
    std::cout << "final tbjbi: " << tbjbi0.transpose() << std::endl;
    Rbjbi = Rbjbi0;
    tbjbi = tbjbi0;
    return true;
}

void VoxelMapping::FrameGroundCost(std::shared_ptr<Frame> frame_ptri, std::shared_ptr<Frame> frame_ptrj, const Eigen::Matrix3d &Rbjbi, const Eigen::Vector3d &tbjbi,
                         Eigen::Matrix<double, 6, 6> &H, Eigen::Matrix<double, 6, 1> &b, double &cost, double info_mat) {
    if (frame_ptri->PointCloudFrameRef().GroundPtr() == nullptr |
        frame_ptrj->PointCloudFrameRef().GroundPtr() == nullptr) return;
    Eigen::Vector3d ci = frame_ptri->PointCloudFrameRef().GroundPtr()->Center();
    Eigen::Vector3d nvi = frame_ptri->PointCloudFrameRef().GroundPtr()->NormalVector();
    Eigen::Vector3d cj = frame_ptrj->PointCloudFrameRef().GroundPtr()->Center();
    Eigen::Vector3d nvj = frame_ptrj->PointCloudFrameRef().GroundPtr()->NormalVector();

    Eigen::Vector3d ci_hat = Rbjbi * cj + tbjbi;
    Eigen::Vector3d nvi_hat = Rbjbi * nvj;

    double r = nvi.dot(ci_hat - ci);

    // r = nvi^t * (Rbjbi * exp(drv) * cj + tbjbi + dt - ci)
    // r = nvi^t * (Rbjbi * (I + drv^) * cj + tbjbi + dt - ci)
    // r = nvi^t * (Rbjbi * drv^ * cj + tbjbi - ci + Rbjbi * drv^ * cj + dt)
    // dr = nvi^t * (Rbjbi * drv^ * cj + dt) = nvi^t * Rbjbi * drv^ * cj  + nvi^t * dt
    // = -nvi^t * Rbjbi * cj^ * drv + nvi^t * dt
    Eigen::Matrix<double, 1, 6> J;
    J.topLeftCorner(1, 3) = -nvi.transpose() * Rbjbi * utils::DMmath::SkewMatrix(cj);
    J.topRightCorner(1, 3) = nvi.transpose();

    H += J.transpose() * info_mat * J;
    b += J.transpose() * info_mat * r;
    cost += r * info_mat * r;
}

int VoxelMapping::FramesCost(std::shared_ptr<Frame> frame_ptri, std::shared_ptr<Frame> frame_ptrj, const Eigen::Matrix3d &Rbjbi, const Eigen::Vector3d &tbjbi,
                             Eigen::Matrix<double, 6, 6> &H, Eigen::Matrix<double, 6, 1> &b, double &cost, double info_mat) {
    int inlier_count = 0;
    Eigen::Matrix3d Rbiw;
    Eigen::Vector3d tbiw;
    frame_ptri->GetPose(Rbiw, tbiw);
    VoxelGrid &gridi = frame_ptri->PointCloudFrameRef().VoxelGridObject();
    VoxelGrid &gridj = frame_ptrj->PointCloudFrameRef().VoxelGridObject();
    VoxelGrid::MapType::iterator iter = gridj.Begin();
    Eigen::Vector3d pti, pti_hat, eigi, eigj, nvi, nvi_hat, nvj;
    for (; iter != gridj.End(); iter++) {
        if (!iter->second.GetEigenInfo(eigj, nvj)) {
            iter->second.EigenCal();
            if (!iter->second.GetEigenInfo(eigj, nvj)) {
                nvj.setZero();
            }
        }
        pti_hat = Rbjbi * iter->second.center + tbjbi;
        nvi_hat = Rbjbi * nvj;
        VoxelGrid::VoxelType *result = gridi.FindNeighbors(pti_hat, nvi_hat, 3);
        if (result == nullptr) continue;
        if (!result->GetEigenInfo(eigi, nvi)) {
            result->EigenCal();
            if (!result->GetEigenInfo(eigi, nvi)) continue;
        }

        Eigen::Vector3d dist = pti_hat - result->center;
        double residual = nvi.dot(dist);
        double cost0 = residual * residual;

        // GM robust kernel function
        double c_inv = 1. / (GM_rho_ + cost0);
        cost0 = 0.5 * cost0 * c_inv;
        double dc_dr = GM_rho_ * fabs(residual) * c_inv * c_inv;

        // jacobians
        // r + dr = nvi^t * (Rbjbi * (I + drv^) * ptj + tbjbi + dt - pi)
        // r + dr = nvi^t * (Rbjbi * ptj + tbjbi - pi + Rbjbi * drv^ * ptj + dt)
        // = r + nvi^t * (-Rbjbi * ptj^ * drv + dt)
        // dr = -nvi^t * Rbjbi * ptj^ * drv + nvi^t * dt
        Eigen::Matrix<double, 1, 6> jacob;
        jacob.topLeftCorner(1, 3) = -nvi.transpose() * Rbjbi * utils::DMmath::SkewMatrix(iter->second.center);
        jacob.topRightCorner(1, 3) = nvi.transpose();

        H += dc_dr * jacob.transpose() * info_mat * jacob;
        b += dc_dr * jacob.transpose() * info_mat * residual;
        cost += cost0;
        inlier_count++;

        loop_matched_pts_.push_back(Rbiw * pti_hat + tbiw);
        loop_origin_pts_.push_back(Rbiw * result->center + tbiw);
    }
    return inlier_count;
}

int VoxelMapping::LoopDetection(std::shared_ptr<Frame> frame_ptr) {
    std::vector<std::shared_ptr<Frame>> &key_frames = map_ptr_->KeyFrames();
    int loop_dist_num = 10;
    if (key_frames.size() < 3) return -1;
    if (key_frames.size() - last_loop_idx_ < loop_dist_num) return -1;

    utils::Transform3d Tbw, Tbw0, dT;
    frame_ptr->GetPose(Tbw);

    bool loop_flag = false;
    double angle_thres = 5. * DEG2RAD; angle_thres *= angle_thres;
    double t_thres = 1.; t_thres *= t_thres;
    std::vector<size_t> candids;
    std::vector<double> dangles, dts;
    Eigen::Vector3d drv;
    for (size_t i = 0; i < key_frames.size() - loop_dist_num && i < key_frames.size(); i++) {
        size_t idx = key_frames.size() - 1 - i;
        key_frames[idx]->GetPose(Tbw0);
        dT = Tbw0.Inverse() * Tbw;
        drv = utils::DMmath::LogSO3(dT.R());

        if (drv.squaredNorm() < angle_thres && dT.t().squaredNorm() < t_thres) {
            candids.push_back(idx);
            dangles.push_back(drv.squaredNorm());
            dts.push_back(dT.t().squaredNorm());
        }
    }
    std::cout << "loop candid size: " << candids.size() << std::endl;

    double min_score = 10000., score;
    int min_idx = -1;
    for (size_t i = 0; i < candids.size(); i++) {
        score = 30. * dangles[i] + dts[i];
        if (score < min_score) {
            min_score = score;
            min_idx = candids[i];
        }
    }
    if (key_frames.size() - min_idx < 10) return -1;
    if (min_idx != -1) last_loop_idx_ = key_frames.size();
    return min_idx;
}

void VoxelMapping::StateUpdate(std::shared_ptr<Frame> frame_ptr) {
    map_ptr_->FusingFrame(frame_ptr);
    if (frame_ptr->ImuSharedPtr() != nullptr) {
        frame_ptr->ImuSharedPtr()->Initialize();
    }
    if (frame_ptr->WheelSharedPtr() != nullptr) {
        frame_ptr->WheelSharedPtr()->Initialization();
    }
    if (frame_ptr->GyroWheelSharedPtr() != nullptr) {
        frame_ptr->GyroWheelSharedPtr()->Initialization();
    }
    map_ptr_->LastKeyFrameRePreintegration();

    frame_ptr->SetFrameId(map_ptr_->FrameSize());
}

void VoxelMapping::SetGlobalGround(std::shared_ptr<GroundPlanes> ground_ptr)
 { 
    grounds_ptr_ = ground_ptr;
    posegraph_ptr_->SetGroundPtr(ground_ptr);
}



void VoxelMapping::InsertKeyFrame(std::shared_ptr<Frame> frame_ptr) {
    // KeyFrameOptimization(frame);
    StateUpdate(frame_ptr);

    utils::Timer ticker;
    ticker.Start();
    // SurfaceBasedOptimization(frame_ptr);
    // LoopOptimization(frame_ptr);
    PoseGraphOptimization(frame_ptr);

    ticker.End("key frame optimiazaion");
    map_ptr_->InsertKeyFrame(frame_ptr);
    ticker.End("key frame mapping");
}

void VoxelMapping::FusingFrame(std::shared_ptr<Frame> frame_ptr) {
    map_ptr_->FusingFrame(frame_ptr);
}

void VoxelMapping::GetFramesPtis(std::vector<Eigen::Vector3d> &pts, std::vector<Eigen::Vector3d> &colors) {
    pts.clear();
    colors.clear();
    for (size_t i = 0; i < frames_ptis_.size(); i++) {
        pts.push_back(frames_ptis_[i]);
        colors.emplace_back(0.0, 1., 0.);
    }
}

void VoxelMapping::GetFramesPtjs(std::vector<Eigen::Vector3d> &pts, std::vector<Eigen::Vector3d> &colors) {
    pts.clear();
    colors.clear();
    for (size_t i = 0; i < frames_ptjs_.size(); i++) {
        pts.push_back(frames_ptjs_[i]);
        colors.emplace_back(1., 0.0, 0.);
    }
}



static std::vector<double> bcosts;
void VoxelMapping::SurfaceBasedOptimization(std::shared_ptr<Frame> frame_ptr) {
    std::cout << "**********keyframe pose optimization*************" << std::endl;
    LOG(INFO) << "***************keyframe pose optimization**************";
    if (map_ptr_->Empty()) return;
    Eigen::Matrix<double, 9, 9> H, I9; H.setZero(); I9.setIdentity();
    Eigen::Matrix<double, 9, 1> b; b.setZero();
    double cost = 0.;
    Eigen::Matrix3d Rbw, last_Rbw;
    Eigen::Vector3d tbw, last_tbw;
    frame_ptr->GetPose(Rbw, tbw);
    std::shared_ptr<Frame> last_frame_ptr = map_ptr_->KeyFrames().back();
    last_frame_ptr->GetPose(last_Rbw, last_tbw);
    Eigen::Vector3d bg;
    if (frame_ptr->ImuSharedPtr() != nullptr) {
        bg = frame_ptr->Bg();
    }

    // calculate information matrix
    Eigen::Matrix3d wheel_cov = last_frame_ptr->WheelSharedPtr()->Cov() + 1.0e-09 * Eigen::Matrix3d::Identity();
    Eigen::Matrix<double, 15, 15> imu_cov = last_frame_ptr->ImuSharedPtr()->Cov();
    Eigen::Matrix<double, 6, 6> gyro_cov;
    gyro_cov.topLeftCorner(3, 3) = imu_cov.topLeftCorner(3, 3);
    gyro_cov.topRightCorner(3, 3) = imu_cov.block<3, 3>(0, 9);
    gyro_cov.bottomLeftCorner(3, 3) = imu_cov.block<3, 3>(9, 0);
    gyro_cov.bottomRightCorner(3, 3) = imu_cov.block<3, 3>(9, 9);

    double surface_info = sensor_ptr_->tofcams[0].noise(0) + sensor_ptr_->tofcams[0].noise(1) +
        sensor_ptr_->tofcams[0].noise(2);//1000.;
    surface_info /= 3.0;
    surface_info = 1. * 500. / surface_info;
    Eigen::Matrix<double, 3, 3> wheel_info = 0. * wheel_cov.inverse();//1. * Eigen::Matrix3d::Identity();
    Eigen::Matrix<double, 4, 4> ground_info = 100. * Eigen::Matrix<double, 4, 4>::Identity();
    Eigen::Matrix<double, 6, 6> gyro_info = 0. * gyro_cov.inverse();//1. * Eigen::Matrix<double, 6, 6>::Identity();

    LOG(INFO) << "key frame init Rbw: " << Rbw;
    LOG(INFO) << "key frame init tbw: " << tbw.transpose();

    utils::Timer ticker;
    int inlier_count = SurfaceCost(frame_ptr, H, b, cost, Rbw, tbw, surface_info);
    if (inlier_count >= 6) {
        cost /= inlier_count;
        H /= inlier_count;
        b /= inlier_count;
    }
    WheelCost(last_frame_ptr, H, b, cost, Rbw, tbw, last_Rbw, last_tbw, wheel_info);
    GyroCost(last_frame_ptr, H, b, cost, Rbw, last_Rbw, bg, gyro_info);
    GroundConstraint(frame_ptr, H, b, cost, Rbw, tbw, ground_info);

    LOG(INFO) << "key frame init cost and inlier count: " << cost << "; " << inlier_count;
    std::cout << "init cost: " << cost << ";" << inlier_count << std::endl;
    double lambda = 1.0;
    for (int i = 0; i < 6; i++) {
        matched_pts_.clear();
        origin_pts_.clear();
        Eigen::Matrix<double, 9, 1> dx = (H + lambda * I9).ldlt().solve(-b);

        Eigen::Matrix3d Rbw_hat = Rbw * utils::DMmath::RotationVector2Matrix(dx.topRows(3));
        Eigen::Vector3d tbw_hat = tbw + dx.middleRows(3, 3);
        Eigen::Vector3d bg_hat = bg + dx.bottomRows(3);
        Eigen::Matrix<double, 9, 9> H_hat; H_hat.setZero();
        Eigen::Matrix<double, 9, 1> b_hat; b_hat.setZero();

        double cost0 = 0.;
        inlier_count = SurfaceCost(frame_ptr, H_hat, b_hat, cost0, Rbw_hat, tbw_hat, surface_info);
        if (inlier_count >= 6) {
            cost0 /= inlier_count;
            H_hat /= inlier_count;
            b_hat /= inlier_count;
        }
        WheelCost(last_frame_ptr, H_hat, b_hat, cost0, Rbw_hat, tbw_hat, last_Rbw, last_tbw, wheel_info);
        GyroCost(last_frame_ptr, H_hat, b_hat, cost0, Rbw_hat, last_Rbw, bg_hat, gyro_info);
        GroundConstraint(frame_ptr, H_hat, b_hat, cost0, Rbw_hat, tbw_hat, ground_info);

        if (cost0 < cost) {
            cost = cost0;
            H = H_hat;
            b = b_hat;
            Rbw = Rbw_hat;
            tbw = tbw_hat;
            bg = bg_hat;
            lambda = lambda < 1.e-06? 1.e-06 : lambda * 0.1;
        } else {
            lambda = lambda > 1000? 1000 : lambda * 10;
        }
        
        std::cout << i << "th cost and min cost: " << cost0 << "; " << cost << "; lambda: " << lambda << "; inlier count: " << inlier_count << std::endl;
        LOG(INFO) << "cost and min cost: " << cost0 << ": " << cost << "; inlier count: " << inlier_count;
    }
    bcosts.push_back(cost);
    std::sort(bcosts.begin(), bcosts.end(), std::greater<double>());
    std::cout << "biggiest cost: " << bcosts.front() << std::endl;

    Rbw = utils::DMmath::RotationNormalization(Rbw);
    std::cout << "key Rbw: " << Rbw << std::endl;
    std::cout << "key tbw: " << tbw.transpose() << std::endl;
    std::cout << "bg: " << bg.transpose() << std::endl;
    LOG(INFO) << "optimized key frame Rbw: " << Rbw;
    LOG(INFO) << "optimized key frame tbw: " << tbw.transpose();
    frame_ptr->SetPose(Rbw, tbw);
    frame_ptr->SetBg(bg);
}

void VoxelMapping::PoseGraphOptimization(std::shared_ptr<Frame> frame_ptr) {
    // std::vector<std::shared_ptr<Frame>> &frames = map_ptr_->KeyFrames();
    // const utils::Transform3d &Tob = sensor_ptr_->diff_wheel.Tob;
    // const utils::Transform3d Tib = sensor_ptr_->imu.Tib;
    // utils::Transform3d last_Tbw, Tbw, Tbw0;
    // if (!frames.empty()) {
    //     for (int i = 0; i < static_cast<int>(frames.size()) - 1; i++) {
    //         std::shared_ptr<Frame> &frame0 = frames[i];
    //         std::shared_ptr<Frame> &frame1 = frames[i + 1];

    //         frame0->GetPose(last_Tbw);
    //         frame0->GyroWheelSharedPtr()->PoseCalFromLastPose(last_Tbw, Tob, Tbw);
    //         frame1->GetPose(Tbw0);

    //         utils::Transform3d dT = Tbw.Inverse() * Tbw0;
    //     }
    // }
    posegraph_ptr_->LoadFrame(frame_ptr);
    if (grounds_ptr_->GravitySharedPtr() != nullptr) {
        std::cout << "-------Gi: " << (*grounds_ptr_->GravitySharedPtr()).transpose() << std::endl;
    }
}

void VoxelMapping::KeyFrameOptimization(std::shared_ptr<Frame> frame_ptr) {
    std::cout << "#####keyframe optimization####################" << std::endl;
    LOG(INFO) << "*******************key frame optimization***********";
    if (map_ptr_->Empty()) return;
    Eigen::Matrix<double, 6, 6> H, I6; H.fill(0.); I6.setIdentity();
    Eigen::Matrix<double, 6, 1> b; b.fill(0.);
    double cost = 0;
    Eigen::Matrix3d Rbw;
    Eigen::Vector3d tbw;
    frame_ptr->GetPose(Rbw, tbw);

    // std::cout << "key init Rbw: " << Rbw << std::endl;
    // std::cout << "key init tbw: " << tbw.transpose() << std::endl;
    LOG(INFO) << "key frame init Rbw: " << Rbw;
    LOG(INFO) << "key frame init tbw: " << tbw.transpose();
    
    int inlier_count = DistributionCost(frame_ptr, H, b, cost, Rbw, tbw);
    if (inlier_count < 6) return;
    cost /= inlier_count;
    int max_iteration = 7;
    double lambda = 1.;
    LOG(INFO) << "key frame init cost and inlier count: " << cost << "; " << inlier_count;
    for (int i = 0; i < max_iteration; i++) {
        // calculate incrementment
        Eigen::Matrix<double, 6, 1> dx = (H + lambda * I6).ldlt().solve(b);

        // update 
        Eigen::Matrix3d Rbw_hat = Rbw * utils::DMmath::RotationVector2Matrix(dx.topRows(3));
        Eigen::Vector3d tbw_hat = tbw + dx.bottomRows(3);
        Eigen::Matrix<double, 6, 6> H_hat; H_hat.setZero();
        Eigen::Matrix<double, 6, 1> b_hat; b_hat.setZero();

        double cost0 = 0;
        inlier_count = DistributionCost(frame_ptr, H_hat, b_hat, cost0, Rbw_hat, tbw_hat);
        if (inlier_count < 6) continue;
        cost0 /= inlier_count;
        if (cost0 < cost) {
            cost = cost0;
            H = H_hat;
            b = b_hat;
            Rbw = Rbw_hat;
            tbw = tbw_hat;
            lambda = lambda < 1.e-03? lambda : lambda * 0.1;
        } else {
            lambda = lambda > 2000? lambda : lambda * 10;
        }
        // std::cout << "cost and min cost: " << cost0 << "; " << cost << "; inlier count: " << inlier_count << std::endl;
        LOG(INFO) << "cost and min cost: " << cost0 << ": " << cost << "; inlier count: " << inlier_count;
    }
    Rbw = utils::DMmath::RotationNormalization(Rbw);
    std::cout << "key Rbw: " << Rbw << std::endl;
    std::cout << "key tbw: " << tbw.transpose() << std::endl;
    LOG(INFO) << "optimized key frame Rbw: " << Rbw;
    LOG(INFO) << "optimized key frame tbw: " << tbw.transpose();
    frame_ptr->SetPose(Rbw, tbw);
}

void VoxelMapping::WheelCost(std::shared_ptr<Frame> frame_ptr, Eigen::Matrix<double, 9, 9> &H, Eigen::Matrix<double, 9, 1> &b, double &cost,
                                     const Eigen::Matrix3d &Rbw, const Eigen::Vector3d &tbw, const Eigen::Matrix3d &last_Rbw,
                                     const Eigen::Vector3d &last_tbw, Eigen::Matrix<double, 3, 3> &info_mat) {
    std::shared_ptr<EulerWMPreintegration> wheel_ptr = frame_ptr->WheelSharedPtr();
    if (wheel_ptr == nullptr) return;
    DiffWheelModel3DFactor factor;
    factor.Evalute(last_Rbw, last_tbw, Rbw, tbw, sensor_ptr_->diff_wheel.Rob,
                   sensor_ptr_->diff_wheel.tob, wheel_ptr);
    H.topLeftCorner(6, 6) += factor.Jj.transpose() * info_mat * factor.Jj;
    b.topRows(6) += factor.Jj.transpose() * info_mat * factor.residual;
    cost += factor.residual.transpose() * info_mat * factor.residual;
}

void VoxelMapping::GyroCost(std::shared_ptr<Frame> frame_ptr, Eigen::Matrix<double, 9, 9> &H, Eigen::Matrix<double, 9, 1> &b, double &cost,
    const Eigen::Matrix3d &Rbw, const Eigen::Matrix3d &last_Rbw, const Eigen::Vector3d &bg, Eigen::Matrix<double, 6, 6> &info_mat) {
    std::shared_ptr<Imu> imu_ptr = frame_ptr->ImuSharedPtr();
    if (imu_ptr == nullptr) return;
    if (imu_ptr->Dt() <= 0.0001) return;
    Eigen::Matrix3d Rib = sensor_ptr_->imu.Rib;

    Eigen::Vector3d bgi = imu_ptr->Bg();
    GyroFactor factor;
    factor.Evaluate(last_Rbw, Rbw, bg, bgi, Rib, imu_ptr);

    Eigen::Matrix<double, 6, 9> J; J.setZero();
    J.topLeftCorner(3, 3) = factor.Jj.topLeftCorner(3, 3);
    J.rightCols(3) = factor.Ji.rightCols(3);
    
    H += J.transpose() * info_mat * J;
    b += J.transpose() * info_mat * factor.residual;
    cost += factor.residual.transpose() * info_mat * factor.residual;
}

void VoxelMapping::GroundConstraint(std::shared_ptr<Frame> frame_ptr, Eigen::Matrix<double, 9, 9> &H, Eigen::Matrix<double, 9, 1> &b, double &cost,
                                    const Eigen::Matrix3d &Rbw, const Eigen::Vector3d &tbw, Eigen::Matrix<double, 4, 4> &info_mat) {
    std::shared_ptr<GroundInfo> ground = frame_ptr->PointCloudFrameRef().GroundPtr();
    if (ground == nullptr) return;
    std::shared_ptr<GroundInfo> world_ground = grounds_ptr_->FindGroundPlane(ground);
    if (world_ground == nullptr) return;
    Eigen::Vector3d center0 = Rbw * ground->Center() + tbw;
    Eigen::Vector3d nv0 = Rbw * ground->NormalVector();
    Eigen::Vector3d center_w = world_ground->Center();
    Eigen::Vector3d nv_w = world_ground->NormalVector();
    Eigen::Vector3d dist = center0 - center_w;

    // residual  
    Eigen::Vector4d residual;
    residual.topRows(3) = nv_w.cross(nv0);
    residual(3) = nv_w.dot(dist);

    // Jacobian
    // r + dr = nvw^ * nv0 = nvw^ * Rbw * (I + drv^) * nv
    // = nvw^ * Rbw * nv + nvw^ * Rbw * drv^ * nv
    // = r - nvw^ * Rbw * nv^ * drv;
    // dr = -nvw^ * Rbw * nv^ * drv
    Eigen::Matrix<double, 4, 6> jacobian;
    jacobian.setZero();
    jacobian.topLeftCorner(3, 3) = -utils::DMmath::SkewMatrix(nv_w) * Rbw * utils::DMmath::SkewMatrix(ground->NormalVector());
    // r + dr = nvw^t * dist = nvw^t * (Rbw * (I + drv^) * c + tbw + dt - cw)
    // = nvw^t * (c0 - cw + Rbw * drv^ * c + dt)
    // = r + nvw^t * Rbw * drv^ * c + nvw^t * dt
    // = r - nvw^t * Rbw * c^ * drv + nvw^t * dt
    jacobian.bottomLeftCorner(1, 3) = -nv_w.transpose() * Rbw * utils::DMmath::SkewMatrix(ground->Center());
    jacobian.bottomRightCorner(1, 3) = nv_w.transpose();

    H.topLeftCorner(6, 6) += jacobian.transpose() * info_mat * jacobian;
    b.topRows(6) += jacobian.transpose() * info_mat * residual;
    cost += residual.transpose() * info_mat * residual;
}

int VoxelMapping::SurfaceCost(std::shared_ptr<Frame> frame_ptr, Eigen::Matrix<double, 9, 9> &H, Eigen::Matrix<double, 9, 1> &b, double &cost,
                              const Eigen::Matrix3d &Rbw, const Eigen::Vector3d &tbw, double info_mat) {
    int inlier_count = 0;
    Eigen::Matrix3d I3; I3.setIdentity();

    Eigen::Matrix3d vt;
    Eigen::Vector3d eig, nvec;
    double w, cost0;
    VoxelTreeMap::VoxelType *result;
    VoxelGrid::MapType::iterator iter = frame_ptr->PointCloudFrameRef().VoxelGridObject().Begin();
    for (; iter != frame_ptr->PointCloudFrameRef().VoxelGridObject().End(); iter++) {
        if (iter->second.N < pcd_num_thres_) continue;
        const Eigen::Vector3d pt0 = iter->second.center;
        double depth_inv = 1. / pt0.norm();
        Eigen::Vector3d ptw = Rbw * pt0 + tbw;
        result = map_ptr_->MapRef().findNeighborsByHash(ptw);
        if (result != nullptr) {
            if (!result->GetEigenInfo(eig, nvec)) {
                result->EigenCal();
                if (!result->GetEigenInfo(eig, nvec)) continue;
            }

            const Eigen::Vector3d &center = result->center;                
            
            Eigen::Vector3d nv = nvec;
            Eigen::Vector3d dist = center - ptw;
            if (dist.squaredNorm() > 0.1) continue;
            double residual = dist.dot(nv);
            residual *= depth_inv;

            cost0 = residual * info_mat * residual;

            double w = 1.;
            if (fabs(residual) > huber_thres_) {
                w = huber_thres_ / residual;
            }

            // jacob
            // r + dr = (e^t +de^t) * nv = e^t * nv + de^t * nv
            // dr = nv^t * de
            // e + de = ptw - Rbw * exp(drv) * pt - tbw - dt
            // e + de = ptw - Rbw * (drv^ + I) * pt - tbw - dt
            // e + de = ptw - Rbw * pt - tbw - Rbw * drv^ * pt - dt
            // de = Rbw * pt^ * drv - dt
            Eigen::Matrix<double, 1, 3> dr_de = nv.transpose() * depth_inv;
            Eigen::Matrix<double, 3, 6> de_dx;
            de_dx.topLeftCorner(3, 3) = Rbw * utils::DMmath::SkewMatrix(pt0);
            de_dx.topRightCorner(3, 3) = -I3;
            
            Eigen::Matrix<double, 1, 6> dr_dx = dr_de * de_dx;

            H.topLeftCorner(6, 6) += w * dr_dx.transpose() * info_mat * dr_dx;
            b.topRows(6) += w * dr_dx.transpose() * info_mat * residual;
            cost += w * cost0;

            inlier_count++;
            matched_pts_.push_back(center);
            origin_pts_.push_back(ptw);
        }
    }
    return inlier_count;
}

int VoxelMapping::DistributionCost(std::shared_ptr<Frame> frame_ptr, Eigen::Matrix<double, 6, 6> &H, Eigen::Matrix<double, 6, 1> &b, double &cost,
                           const Eigen::Matrix3d &Rbw, const Eigen::Vector3d &tbw) {
    int inlier_count = 0;
    Eigen::Matrix3d I3; I3.setIdentity();
    VoxelTreeMap::VoxelType result;
    VoxelGrid::MapType::iterator iter = frame_ptr->PointCloudFrameRef().VoxelGridObject().Begin();
    for (; iter != frame_ptr->PointCloudFrameRef().VoxelGridObject().End(); iter++) {
        if (iter->second.N < 60) continue;
        const Eigen::Vector3d pt0 = iter->second.center;
        double depth = pt0.norm();
        Eigen::Vector3d ptw = Rbw * pt0 + tbw;
        if (map_ptr_->MapRef().findNeighborsByHash(ptw, result)) {
            Eigen::Vector3d residual = result.center - ptw;
            if (residual.norm() > 0.2) continue;
            residual /= depth;
            
            double cov_norm = result.cov.norm();
            if (cov_norm < std::numeric_limits<double>::epsilon() * 100) continue;
            Eigen::Matrix3d Cpq;
            Eigen::Matrix3d Ci = utils::DMmath::Matrix3dInverse(result.cov * result.N + damping_value_ * I3);
            Cpq = Ci / Ci.norm();
            double cost0 = residual.transpose() * Cpq * residual;
            double w = 1.0;
            if (cost0 > huber_thres_) {
                w = huber_thres_ / cost0;
            }
            // if (pt0(2) > 1 && pt0(1) > 0.055) continue;

            Eigen::Matrix<double, 3, 6> J;
            J.topLeftCorner(3, 3) = -Rbw * utils::DMmath::SkewMatrix(pt0);
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

void VoxelMapping::GetMatchedPoints(std::vector<Eigen::Vector3d> &pts, std::vector<Eigen::Vector3d> &colors) {
    for (size_t i = 0; i < matched_pts_.size(); i++) {
        pts.push_back(matched_pts_[i]);
        colors.push_back(Eigen::Vector3d(1., 0., 0.));
    }

    for (size_t i = 0; i < origin_pts_.size(); i++) {
        pts.push_back(origin_pts_[i]);
        colors.push_back(Eigen::Vector3d(0., 0., 1.));
    }
}

void VoxelMapping::GetLoopMatchedPoints(std::vector<Eigen::Vector3d> &pts, std::vector<Eigen::Vector3d> &colors) {
    for (int i = 0; i < loop_matched_pts_.size(); i++) {
        pts.push_back(loop_matched_pts_[i]);
        colors.push_back(Eigen::Vector3d(0., 0., 1.));
    }

    for (int i = 0; i < loop_origin_pts_.size(); i++) {
        pts.push_back(loop_origin_pts_[i]);
        colors.push_back(Eigen::Vector3d(1., 0., 0.));
    }
    loop_matched_pts_.clear();
    loop_origin_pts_.clear();
}

} // namespace slam