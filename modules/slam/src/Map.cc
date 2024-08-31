#include "slam/Map.h"

namespace slam {

PointCloudFrame::PointCloudFrame(double voxelgrid_size): voxelgrid_(voxelgrid_size) {
    ground_ptr_ = nullptr;
}

uint32_t Frame::frame_count_ = 0;
Frame::Frame(double voxelgrid_size, std::shared_ptr<Imu> imu_ptr, std::shared_ptr<EulerWMPreintegration> wheel_ptr)
: pointcloud_frame_(voxelgrid_size),
  imu_ptr_(std::move(imu_ptr)),
  wheel_ptr_(wheel_ptr),
  is_keyframe_(false) {
    Rbw_.setIdentity();
    tbw_.setZero();
    Tbw_.SetIdentity();
    Vw_.setZero();
    if (imu_ptr_ != nullptr) {
        ba_ = imu_ptr_->Ba();
        bg_ = imu_ptr_->Bg();
    } else {
        ba_.setZero();
        bg_.setZero();
    }
}
void Frame::SetImuSharedPtr(std::shared_ptr<Imu> imu_ptr) {
    imu_ptr_ = std::move(imu_ptr);
    ba_ = imu_ptr->Ba();
    bg_ = imu_ptr->Bg();
}

void Frame::ImuRePreintegration() {
    if (imu_ptr_ != nullptr) {
        imu_ptr_->SetBg(bg_);
        imu_ptr_->RePreintegration();
    }
}

void Frame::WheelRePreintegration() {
    if (wheel_ptr_ != nullptr) {
        wheel_ptr_->RePreintegration();
    }
}

void Frame::GyroWheelRePreintegration() {
    if (gyro_wheel_ptr_ != nullptr) {
        gyro_wheel_ptr_->RePreintegration();
    }
}

void GlobalVoxelMap::FusingFrame(std::shared_ptr<Frame> frame_ptr) {
    if (keyframe_set_.empty()) return;

    std::shared_ptr<Imu> imu_ptr = frame_ptr->ImuSharedPtr();
    if (imu_ptr != nullptr) {
        keyframe_set_.back()->ImuSharedPtr()->InsertImuWithoutPreintegration(imu_ptr);
    }

    std::shared_ptr<EulerWMPreintegration> wh_ptr = frame_ptr->WheelSharedPtr();
    if (wh_ptr != nullptr) {
        keyframe_set_.back()->WheelSharedPtr()->FusingWMwithoutPreintegration(wh_ptr);
    }

    std::shared_ptr<GyroWheelModelPreintegration> gyr_wh_ptr = frame_ptr->GyroWheelSharedPtr();
    if (gyr_wh_ptr != nullptr) {
        keyframe_set_.back()->GyroWheelSharedPtr()->FusingGyrWheelWithoutPreintegration(gyr_wh_ptr);
    }
}

void GlobalVoxelMap::LastKeyFrameRePreintegration() {
    if (keyframe_set_.empty()) return;
    keyframe_set_.back()->ImuRePreintegration();
    keyframe_set_.back()->WheelRePreintegration();
    keyframe_set_.back()->GyroWheelRePreintegration();
}

void GlobalVoxelMap::ReMapFromKeyFrames() {
    map_.Clear();
    std::vector<std::shared_ptr<Frame>> frames;
    frames.swap(keyframe_set_);
    for (int i = 0; i < frames.size(); i++) {
        InsertKeyFrame(frames[i]);
    }
}

void GlobalVoxelMap::InsertKeyFrame(std::shared_ptr<Frame> frame_ptr) {
    // inital boundary
    double x_min = 100000, x_max = -100000;
    double y_min = 100000, y_max = -100000;
    double z_min = 100000, z_max = -100000;
    dynamic_pts.clear();

    const utils::Transform3d &Tbw = frame_ptr->Tbw();
    VoxelGrid::MapType::iterator iter = frame_ptr->PointCloudFrameRef().VoxelGridObject().Begin();

    for(; iter != frame_ptr->PointCloudFrameRef().VoxelGridObject().End(); iter++) {
        Eigen::Vector3d c1 = Tbw.transform(iter->second.center);
        Eigen::Matrix3d cov1 = Tbw.R() * iter->second.cov * Tbw.R().transpose();

        // update map based on point depth from cam
        // far point has low weight, otherwise near point has hight weight
        double depth = iter->second.center.norm();
        int pt_num = static_cast<int>(6 * depth_based_weight_ * iter->second.N / depth);

        map_.AddDistributionAndOccupancyUpdation(c1, cov1, pt_num, Tbw.t());

        // boundary
        if (c1(0) > x_max) x_max = c1(0);
        if (c1(0) < x_min) x_min = c1(0);

        if (c1(1) > y_max) y_max = c1(1);
        if (c1(1) < y_min) y_min = c1(1);

        if (c1(2) > z_max) z_max = c1(2);
        if (c1(2) < z_min) z_min = c1(2);
    }
    keyframe_set_.push_back(frame_ptr);

    // load boundary
    std::vector<double> boundary = {x_min, x_max, y_min, y_max, z_min, z_max};
    map_.LoadBoundary(boundary);
}

void GlobalVoxelMap::GetGlobalMapPoints(std::vector<Eigen::Vector3d> &points, std::vector<Eigen::Vector3d> &colors) {
    VoxelTreeMap::MapType::iterator iter = map_.begin();
    for (; iter != map_.end(); iter++) {
        for (size_t i = 0; i < iter->second.Size(); i++) {
            if (iter->second.Voxel(i).N > 3) {
                points.push_back(iter->second.Voxel(i).center);
                colors.push_back(Eigen::Vector3d(0, 1, 0));
            }
        }
    }

    for (int i = 0; i < dynamic_pts.size(); i++) {
        points.push_back(dynamic_pts[i]);
        colors.push_back(Eigen::Vector3d(1, 1, 1));
    }
}
// cov = [v0, v1, vgd] * diag([1; 1; 0]) * [v0; v1; vgd^t]
GroundExtraction::GroundExtraction(const Eigen::Vector3d &ground_dir, double noise)
: ground_dir_(ground_dir), weight_(1. / noise) {
    dir_thres_ = 0.1;

    Eigen::Vector3d v0(0, 0, 1);
    ground_dir_ /= ground_dir_.norm();
    double vg = v0.dot(ground_dir_);
    if (fabs(vg - 1.) < 1.0e-05) {
        v0 << 1, 0, 0;
        vg = v0.dot(ground_dir_);
    }
    v0 = v0 - vg * ground_dir_;
    v0 /= v0.norm();

    Eigen::Vector3d v1 = ground_dir_.cross(v0);
    v1 /= v1.norm();

    Eigen::Matrix3d V;
    V << v0, v1, ground_dir_;

    ground_cov_ = V * Eigen::Vector3d(0.707107, 0.707107, 1.0e-05).asDiagonal() * V.transpose();
    ground_cov_.normalize();
    ground_cov_inv_ = utils::DMmath::Matrix3dInverse(ground_cov_);
    ground_cov_inv_.normalize();

    plane_inlier_thres_ = 0.23;
}

bool GroundExtraction::Extract(std::shared_ptr<Frame> frame_ptr) {

    Eigen::Vector3d pt(0., 0., 0.);
    Eigen::Vector3d nv(0., 1.0, 0.);
    ground_.SetInfo(pt, nv, 300);
    return true;

    Eigen::Matrix3d Rbw;
    Eigen::Vector3d tbw;
    frame_ptr->GetPose(Rbw, tbw);
    Eigen::Vector3d gd_vec = ground_dir_;

    VoxelGrid::MapType::iterator iter = frame_ptr->PointCloudFrameRef().VoxelGridObject().Begin();
    std::vector<Eigen::Matrix<double, 6, 1>> planes;
    std::vector<int> weights;
    for (; iter != frame_ptr->PointCloudFrameRef().VoxelGridObject().End(); iter++) {
        if (iter->second.N < 5) continue;
        const Eigen::Vector3d &pt = iter->second.center;
        if (pt(2) < 0.6 && iter->second.eig_cal_flag_) {
            Eigen::Vector3d eig0, nv0;
            iter->second.GetEigenInfo(eig0, nv0);
            double cost = 1. - fabs(gd_vec.dot(nv0));
            if (cost > 0.07) continue;
            Eigen::Matrix<double, 6, 1> plane;
            plane.topRows(3) = nv0;
            plane.bottomRows(3) = pt;
            planes.push_back(plane);
            weights.push_back(iter->second.N);
            // std::cout << "plane: " << plane.transpose() << std::endl;
        }
    }

    int max_inlier = 0;
    std::vector<int> inliers;
    for (int i = 0; i < planes.size(); i++) {
        const Eigen::Matrix<double, 6, 1> &cand = planes[i];

        std::vector<int> inlier_idxs;
        for (size_t j = 0; j < planes.size(); j++) {
            double cost = MatchedPlaneCost(cand, planes[j]);
            // std::cout << i << "th cost: " << cost << std::endl;
            if (cost < plane_inlier_thres_) {
                inlier_idxs.push_back(j);
            }
        }
        if  (inlier_idxs.size() > max_inlier) {
            // std::cout << "selected idx: " << i << std::endl;
            max_inlier = inlier_idxs.size();
            inliers.swap(inlier_idxs);
        }
            // std::cout << "----------" << std::endl;

        // if (max_inlier >= 0.6 * planes.size() && i >= 5) break;
    }
    if (max_inlier < 2) return false;

    Eigen::Matrix<double, 6, 1> ground_plane;
    Eigen::Matrix3d H; H.setZero();
    Eigen::Vector3d center; center.setZero();
    double w = 0;
    // pi X groud = 0 => pi^ * ground = 0
    for (auto idx : inliers) {
        // std::cout << "idx: " << idx << std::endl;
        Eigen::Matrix3d j_hat = utils::DMmath::SkewMatrix(planes[idx].topRows(3));
        H += j_hat.transpose() * j_hat * weights[idx] * weights[idx];
        center += weights[idx] * planes[idx].bottomRows(3);
        w += weights[idx];
    }
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    ground_plane.topRows(3) = svd.matrixV().col(2).normalized();
    ground_plane.bottomRows(3) = center / w;
    // for (size_t i = 0; i < planes.size(); i++) {
    //     double cost = MatchedPlaneCost(planes[i], ground_plane);
    //     std::cout << "cost: " << cost << std::endl;
    // }
    Eigen::Vector4d gdplane;
    gdplane << ground_plane.topRows(3), -ground_plane.topRows(3).dot(ground_plane.bottomRows(3));
    // std::cout << "ground plane: " << ground_plane.transpose() << std::endl;
    std::cout << "ground plane: " << gdplane.transpose() << std::endl;

    ground_.SetInfo(ground_plane.bottomRows(3), ground_plane.topRows(3), static_cast<int>(w));
    return true;
}

double GroundExtraction::MatchedPlaneCost(const Eigen::Matrix<double, 6, 1> &plane0, const Eigen::Matrix<double, 6, 1> &plane1) {
    double a = plane0.topRows(3).dot(plane1.topRows(3));
    Eigen::Vector3d dist = plane0.bottomRows(3) - plane1.bottomRows(3);
    // double b = fabs(plane0.topRows(3).dot(dist)) + fabs(plane1.topRows(3).dot(dist));
    // std::cout << "cost: " << (1 - fabs(a)) << ";" << b << std::endl;
    return (1 - fabs(a)) * 5 + fabs(plane0.topRows(3).dot(dist)) + fabs(plane1.topRows(3).dot(dist));
}

void GroundInfo::SetInfo(const Eigen::Vector3d &pt, const Eigen::Vector3d &nv, int N) {
    center_ = pt;
    normal_vector_ = nv;
    N_ = N;
}

GroundInfo::GroundInfo(std::shared_ptr<GroundInfo> &ground_ptr) {
    center_ = ground_ptr->center_;
    normal_vector_ = ground_ptr->normal_vector_;
    N_ = ground_ptr->N_;
    plane_matched_thres_ = ground_ptr->plane_matched_thres_;
}

GroundInfo::GroundInfo(std::shared_ptr<GroundInfo> &ground_ptr, const Eigen::Matrix3d &Rbw, const Eigen::Vector3d &tbw) {
    center_ = Rbw * ground_ptr->center_ + tbw;
    normal_vector_ = Rbw * ground_ptr->normal_vector_;
    N_ = ground_ptr->N_;
    plane_matched_thres_ = ground_ptr->plane_matched_thres_;
}

void GroundInfo::Transform(const Eigen::Matrix3d &Rbw, const Eigen::Vector3d &tbw) {
    center_ = Rbw * center_ + tbw;
    normal_vector_ = Rbw * normal_vector_;
}

bool GroundInfo::GroundUpdate(const Eigen::Vector3d &center, const Eigen::Vector3d &nv, int N) {
    // pt = (N0 * pt + N * pt0) / (N0 + N)
    double s = 1. / (N + N_);
    center_ = s * (N * center + N_ * center_);
    normal_vector_ = s * (N * nv + N_ * normal_vector_);
    N_ += N;
    return true;
}

bool GroundInfo::GroundUpdate(const GroundInfo &gi, const Eigen::Matrix3d &Rbw, const Eigen::Vector3d &tbw) {
    int N = gi.N_;
    Eigen::Vector3d center = Rbw * gi.center_ + tbw;
    Eigen::Vector3d nv = Rbw * gi.normal_vector_;

    double s = 1. / (N + N_);
    center_ = s * (N * center + N_ * center_);
    normal_vector_ = s * (N * nv + N_ * normal_vector_);
    normal_vector_.normalize();
    N_ = N_ > 100000000? N_ : N_ + N;
    return true;
}

bool GroundInfo::GroundUpdate(const std::shared_ptr<GroundInfo> &gi, const Eigen::Matrix3d &Rbw, const Eigen::Vector3d &tbw) {
    int N = gi->N_;
    Eigen::Vector3d center = Rbw * gi->center_ + tbw;
    Eigen::Vector3d nv = Rbw * gi->normal_vector_;
    double s = 1. / (N + N_);
    center_ = s * (N * center + N_ * center_);
    normal_vector_ = s * (N * nv + N_ * normal_vector_);
    normal_vector_.normalize();
    N_ = N_ > 100000000? N_ : N_ + N;
    return true;
}

GroundPlanes::GroundPlanes()
  : Gw_ptr_(nullptr) {
    matched_thres_ = 0.23;
    matched_angle_thres = cos(10. * M_PI / 180.);
    matched_dist_thres_ = 0.06;
    outlier_plane_thres_ = 10000;
}

double GroundPlanes::MatchedPlaneCost(GroundInfo &gd0, GroundInfo &gd1) {
    double a = gd0.NormalVector().dot(gd1.NormalVector());
    Eigen::Vector3d dist = gd0.Center() - gd1.Center();
    return (1. - fabs(a)) * 5. + fabs(gd0.NormalVector().dot(dist)) + fabs(gd1.NormalVector().dot(dist));
}

std::shared_ptr<GroundInfo> GroundPlanes::FindGroundPlane(std::shared_ptr<GroundInfo> &ground) {
    if (ground == nullptr || grounds_.empty()) return nullptr;
    for (size_t i = 0; i < grounds_.size(); i++) {
        double angle = grounds_[i]->NormalVector().dot(ground->NormalVector());
        Eigen::Vector3d t = grounds_[i]->Center() - ground->Center();
        double dist = (fabs(grounds_[i]->NormalVector().dot(t)));
        if (dist < matched_dist_thres_ && fabs(angle) > matched_angle_thres) {
            return grounds_[i];
        }
    }

    return nullptr;
}

void GroundPlanes::GroundUpdate(std::shared_ptr<Frame> frame_ptr) {
    std::shared_ptr<GroundInfo> ground = frame_ptr->PointCloudFrameRef().GroundPtr();
    if (ground == nullptr) return;
    Eigen::Matrix3d Rbw;
    Eigen::Vector3d tbw;
    frame_ptr->GetPose(Rbw, tbw);
    if (grounds_.empty()) {
        grounds_.emplace_back(std::make_shared<GroundInfo>(ground, Rbw, tbw));
        return;
    }

    // for (auto it : grounds_) {
    //     std::cout << "plane info: " << it->NormalVector().transpose() << ";" << it->Center().transpose()
    //               << ";N: " << it->PointNum() << std::endl;
    // }

    Eigen::Vector3d center = Rbw * ground->Center() + tbw;
    Eigen::Vector3d nv = Rbw * ground->NormalVector();

    for (size_t i = 0; i < grounds_.size(); i++) {
        double angle = grounds_[i]->NormalVector().dot(nv);
        Eigen::Vector3d t = grounds_[i]->Center() - center;
        double dist = (fabs(grounds_[i]->NormalVector().dot(t)));
        // double cost = MatchedPlaneCost(grounds_[i], ground);
        // std::cout << "+++++center0: " << center.transpose() << std::endl;
        // std::cout << "angle and dist: " << angle << ";nv * t: " << dist << ";t norm: " << t.norm() << std::endl;
        // std::cout << "thres: " << matched_angle_thres << ";" << matched_dist_thres_ << std::endl;
        std::cout << "updated plane info: " << grounds_[i]->NormalVector().transpose() << "," << grounds_[i]->Center().transpose()
                    << ";N:" << grounds_[i]->PointNum() << std::endl;
        if (dist < matched_dist_thres_ && fabs(angle) > matched_angle_thres) {

            grounds_[i]->GroundUpdate(ground, Rbw, tbw);
            OutlierGroundPlaneRemovement();
            return;
        }
    }

    grounds_.emplace_back(std::make_shared<GroundInfo>(ground, Rbw, tbw));
    OutlierGroundPlaneRemovement();
}

void GroundPlanes::OutlierGroundPlaneRemovement() {
    if (grounds_.size() < 2) return;
    std::vector<std::shared_ptr<GroundInfo>>::iterator iter = grounds_.end();
    iter -= 2;
    if ((*iter)->PointNum() < outlier_plane_thres_) {
        std::cout << "erase plane point num: " << (*iter)->PointNum() << std::endl;
        grounds_.erase(iter);
    }
}

} // namespace slam