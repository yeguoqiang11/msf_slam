#include "slam/VoxelMap.h"

namespace slam {
std::vector<Eigen::Vector3d> dynamic_pts;

double VoxelMap::voxel_size_ = 0.10;
double VoxelBox::voxel_size_ = VoxelMap::voxel_size_;
int VoxelBox::subvoxel_num_ = VoxelMap::voxel_size_ / 0.00666 ;
double VoxelBox::subvoxel_size_ = VoxelBox::voxel_size_ / VoxelBox::subvoxel_num_;
double VoxelTree::subvoxel_size_ = 0.2;
double VoxelTree::damping_val_ = 1.0e-09;
int VoxelTree::SubVoxel::eigen_decomposition_num_thres_ = 5;

inline Eigen::Vector3d PivotPoint(const Eigen::Vector3d &pt, double voxel_size) {
    Eigen::Vector3d xyz;
    for (int i = 0; i < 3; i++) {
        xyz(i) = static_cast<double>(static_cast<int64_t>(pt(i) / voxel_size));
    }
    return xyz * voxel_size;
}

// ---------------build voxel box-----------------
VoxelBox::VoxelBox(const FeatType &feat) {
    center_ = feat;
    features_.push_back(feat);
    pivot_point_ = PivotPoint(feat.point, voxel_size_);
    features_set_.push_back(std::vector<FeatType>{feat});
}

void VoxelBox::InsertFeature(const FeatType &feat) {
    if (features_.size() == 0) {
        center_ = feat;
        features_.push_back(feat);
        pivot_point_ = PivotPoint(feat.point, voxel_size_);
        InsertFeature2Set(feat);
        return;
    }

    InsertFeature2Set(feat);
    if (features_.size() > 20) return;

    for (size_t i = 0; i < features_.size(); i++) {
        Eigen::Vector3d diff = features_[i].point - feat.point;
        if (diff.norm() < 0.005) return;
    }
    features_.push_back(feat);
}

void VoxelBox::InsertFeature2Set(const FeatType &feat) {
    double thres; // = subvoxel_size_ * subvoxel_size_;
    // thres = thres > 0.0004? thres : 0.0004;
    thres = 0.0004;
    for (size_t i = 0; i < features_set_.size(); i++) {
        Eigen::Vector3d diff = features_set_[i][0].point - feat.point;
        if (diff.squaredNorm() < thres) {
                features_set_[i].push_back(feat);
            return;
        }
    }

    if (features_set_.size() < subvoxel_num_) {
        features_set_.push_back(std::vector<FeatType>{feat});
    }
}

const FeatType &VoxelBox::operator()(const FeatType &feat) const {
    double min_dist = 1000;
    int idx = -1;
    for (size_t i = 0; i < features_set_.size(); i++) {
        Eigen::Vector3d diff = features_set_[i][0].point - feat.point;
        if (diff.squaredNorm() < min_dist) {
            min_dist = diff.squaredNorm();
            idx = i;
        }
    }
    if (idx == -1) exit(0);

    return features_set_[idx][0];
}

void VoxelBox::GetNearbyFeatures(const FeatType &feat, std::vector<FeatType> &outs) const {
    for (size_t i = 0; i < features_set_.size(); i++) {
        outs.push_back(features_set_[i][0]);
    }
}

void VoxelBox::GetNearbyPoint(const Eigen::Vector3d &pt, std::vector<FeatType> &outs) const {
    for (size_t i = 0; i < features_set_.size(); i++) {
        outs.push_back(features_set_[i][0]);
    }
}

// ----------------build voxel map---------------
VoxelKey VoxelMap::Key(const FeatType &feat) const {
    int64_t xyz[3];
    for (int i = 0; i < 3; i++) {
        xyz[i] = feat.point(i) / voxel_size_;
    }

    return VoxelKey(xyz[0], xyz[1], xyz[2]);
}

VoxelKey VoxelMap::Key(const Eigen::Vector3d &pt) const {
    int64_t xyz[3];
    xyz[0] = pt(0) / voxel_size_;
    xyz[1] = pt(1) / voxel_size_;
    xyz[2] = pt(2) / voxel_size_;
    return VoxelKey(xyz[0], xyz[1], xyz[2]);
}


void VoxelMap::InsertFeature(const FeatType &feat) {
    VoxelKey key = Key(feat);
    auto iter = maps_.find(key);
    if (iter == maps_.end()) {
        keys_.push_back(key);
        VoxelBox voxel(feat);
        maps_[key] = voxel;
    } else {
        iter->second.InsertFeature(feat);
    }
}

double VoxelMap::kdtree_get_pt(const size_t &idx, const size_t &dim) const {
    std::unordered_map<VoxelKey, VoxelBox>::const_iterator iter = maps_.find(keys_[idx]);
    return iter->second.Center()(dim);
}

VoxelBox &VoxelMap::operator()(const FeatType &feat) {
    VoxelKey key = Key(feat);
    auto iter = maps_.find(key);
    if (iter == maps_.end()) {
        maps_[key] = VoxelBox();
        return maps_.find(key)->second;
    } else {
        return iter->second;
    }
}

const VoxelBox &VoxelMap::operator()(const size_t &idx) const {
    const VoxelKey &key = keys_[idx];
    std::unordered_map<VoxelKey, VoxelBox>::const_iterator iter = maps_.find(keys_[idx]);
    return iter->second;
}

const VoxelBox &VoxelMap::QueryVoxelBox(const Eigen::Vector3d &pt) const {
    VoxelKey key = Key(pt);
    std::unordered_map<VoxelKey, VoxelBox>::const_iterator iter = maps_.find(key);
    return iter->second;
}


bool VoxelMap::Find(const FeatType &feat) const {
    VoxelKey key = Key(feat);
    std::unordered_map<VoxelKey, VoxelBox>::const_iterator iter = maps_.find(key);
    if (iter == maps_.end()) {
        return false;
    } else {
        return true;
    }
}

bool VoxelMap::Find(const Eigen::Vector3d &pt) const {
    VoxelKey key = Key(pt);
    std::unordered_map<VoxelKey, VoxelBox>::const_iterator iter = maps_.find(key);
    if (iter == maps_.end()) {
         return false;
    } else {
        return true;
    }
}


const VoxelBox &MapAdaptor::operator()(const size_t &idx) const {
    return map_(idx);
}

// ---------------------build map kdtree----------------
MapKdTree::MapKdTree(int dim, size_t capacity) {
    // dim = 0, 1 => horizon or vertical feature indexing
    if (dim == 0 || dim == 1) {
        scale_ = 0.3;
    } else if (dim == 2) {
        // dim = 3 => plane feature indexing
        scale_ = 0.7;
    } else {
        scale_ = 1.;
    }
    dim_ = dim;
    map_adaptor_ptr_ = std::make_shared<MapAdaptor>(voxel_map_, dim, scale_);
    map_kdtree_ptr_ = std::make_shared<MapKDTreeType>(3, *map_adaptor_ptr_,
        nanoflann::KDTreeSingleIndexAdaptorParams(10));
    built_count = 0;

    query_capacity_ = capacity;
    ret_index_ = new size_t[capacity];
    ret_dist_ = new double[capacity];
}

MapKdTree::~MapKdTree() {
    delete []ret_index_;
    delete []ret_dist_;
}


void MapKdTree::IndexRebuild() {
    int num = voxel_map_.MapSize() - 1;
    if (num < built_count || num <= 0) return;
    map_kdtree_ptr_->addPoints(built_count, num);
    built_count = num + 1;
}

std::vector<FeatType> MapKdTree::findNeighbors(const FeatType &feat, int index_num) {
    std::vector<FeatType> features;
    size_t num = index_num;
    if (num > query_capacity_) num = query_capacity_;
    query_result_.ResizeCapacity(num);
    query_result_.init(ret_index_, ret_dist_);

    Eigen::Vector3d query_pt = feat.point;
    if (dim_ == 0 || dim_ == 1 || dim_ == 2) {
        query_pt(dim_) *= scale_;
    }

    map_kdtree_ptr_->findNeighbors(query_result_, query_pt.data(), nanoflann::SearchParams(10));
    for (size_t j = 0; j < query_result_.size(); j++) {
        voxel_map_(ret_index_[j]).GetNearbyFeatures(feat, features);
        // features.push_back(voxel_map_(ret_index_[j]).CenterFeature());
        // if (features.size() >= index_num) {
        //     features.resize(index_num);
        //     return features;
        // }
    }
    std::vector<size_t> idxs;
    std::vector<double> dists;
    for (size_t i = 0; i < features.size(); i++) {
        double dist = (features[i].point - feat.point).squaredNorm();
        idxs.push_back(i);
        dists.push_back(dist);
    }
    std::vector<FeatType> tmp;
    if (idxs.size() >= 2) {
        tmp.swap(features);
        std::sort(idxs.begin(), idxs.end(),  [&dists](size_t i1, size_t i2) { return dists[i1] < dists[i2]; });
        for (size_t i = 0; i < index_num && i < idxs.size(); i++) {
            features.push_back(tmp[idxs[i]]);
        }
    }
    return features;
}

void MapKdTree::Init() {
    voxel_map_.Released();
    map_kdtree_ptr_.reset();
    map_kdtree_ptr_ = std::make_shared<MapKDTreeType>(3, *map_adaptor_ptr_,
        nanoflann::KDTreeSingleIndexAdaptorParams(10));
    built_count = 0;
}

const VoxelBox &MapKdTree::operator()(const size_t &idx) const {
    return voxel_map_(idx);
}

std::vector<FeatType> MapKdTree::findNeighborsByHash(const Eigen::Vector3d &pt, int index_num) {
    // VoxelKey key = Key(feat);
    std::vector<FeatType> outs;
    voxel_map_.QueryVoxelBox(pt).GetNearbyFeatures(pt, outs);

    // search nearby voxel
    if (outs.size() < index_num) {
        Eigen::Vector3d pt0 = pt;
        pt0(2) += 0.1;
        if (voxel_map_.Find(pt0)) {
            voxel_map_.QueryVoxelBox(pt0).GetNearbyFeatures(pt, outs);
        }
        pt0(2) -= 0.2;
        if (voxel_map_.Find(pt0)) {
            voxel_map_.QueryVoxelBox(pt0).GetNearbyFeatures(pt, outs);
        }
        pt0(2) += 0.1;

        pt0(0) += 0.1;
        if (voxel_map_.Find(pt0)) {
            voxel_map_.QueryVoxelBox(pt0).GetNearbyFeatures(pt, outs);
        }
        pt0(0) -= 0.2;
        if (voxel_map_.Find(pt0)) {
            voxel_map_.QueryVoxelBox(pt0).GetNearbyFeatures(pt, outs);
        }
    }

    // sort by distance
    std::vector<size_t> idxs;
    std::vector<double> dists;
    for (size_t i = 0; i < outs.size(); i++) {
        idxs.push_back(i);
        dists.push_back((outs[i].point - pt).squaredNorm());
    }

    if (idxs.size() >= 2) {
        std::sort(idxs.begin(), idxs.end(), [&dists](size_t i1, size_t i2) { return dists[i1] < dists[i2]; });
        std::vector<FeatType> tmp;
        tmp.swap(outs);
        for (size_t i = 0; i < idxs.size() && i < index_num; i++) {
            outs.push_back(tmp[idxs[i]]);
        }
    }
    return outs;
}

// ------------------------------build voxel tree-----------------
VoxelTree::SubVoxel::SubVoxel(const Eigen::Vector3d &pt) {
    center = pt;
    cov.setZero();
    cov_inv.setZero();
    N = 1;
    eig_cal_flag_ = false;
    odds_ = 1;
}

VoxelTree::SubVoxel::SubVoxel() {
    center.setZero();
    cov.setZero();
    cov_inv.setZero();
    N = 0;
    eig_cal_flag_ = false;
    odds_ = 1;
}

void VoxelTree::SubVoxel::AddPoint(const Eigen::Vector3d &pt) {
    // u1 = (m * u0 + pt) / (m + 1) = ((m + 1) * u0 + (pt - u0)) / (m + 1)
    // u1 = u0 + (pt - u0) / (m + 1)
    double s = 1.0 / (N + 1);
    Eigen::Vector3d tmp = pt - center;
    center += tmp * s;
    cov *= N;
    cov += tmp * tmp.transpose() * N * s;
    cov *= s;
    N++;
    N = N > 300? 300 : N;
}

void VoxelTree::SubVoxel::AddPoint(const Eigen::Vector3d &pt, const Eigen::Matrix3d &R, const Eigen::Vector3d &t) {
    double s = 1.0 / (N + 1);
    Eigen::Vector3d tmp = R * pt + t - center;
    center += tmp * s;
    cov *= N;
    cov += tmp * tmp.transpose() * N * s;
    cov *= s;
    N++;
    N = N > 300? 300 : N;
}


void VoxelTree::SubVoxel::AddDistribution(const Eigen::Vector3d &pt0, const Eigen::Matrix3d &cov0, int N0) {
    // u1 = (m * u0 + n * u1) / (m + n) = ((m + n) * u0 + n * (u1 - u0)) / (m + n)
    // u1 = u0 + n * (u1 - u0) / (m + n)
    double s = 1.0 / (N + N0);
    Eigen::Vector3d tmp = pt0 - center;
    center += N0 * tmp * s;
    cov *= N;
    cov += cov0 * N0 + tmp * tmp.transpose() * N * N0 * s;
    cov *= s;
    N += N0;
    N = N > 10000? 10000 : N;

    eig_cal_flag_ = false;
}

void VoxelTree::SubVoxel::AddDistribution(const Eigen::Vector3d &pt0, const Eigen::Matrix3d &cov0, int N0,
                                          const Eigen::Matrix3d &R, const Eigen::Vector3d &t) {
    double s = 1.0 / (N + N0);
    Eigen::Vector3d tmp = R * pt0 + t - center;
    Eigen::Matrix3d tmp_cov = R * cov0 * R.transpose();
    center += N0 * tmp * s;
    cov *= N;
    cov += tmp_cov * N0 + tmp * tmp.transpose() * N * N0 * s;
    cov *= s;
    N += N0;
    N = N > 10000? 10000 : N;

    eig_cal_flag_ = false;
}

void VoxelTree::SubVoxel::SetEigenInfo(const Eigen::Vector3d &eig0, const Eigen::Vector3d &nv) {
    eig_cal_flag_ = true;

    eig = eig0;
    normal_vec_ = nv;
}

bool VoxelTree::SubVoxel::GetEigenInfo(Eigen::Vector3d &eig0, Eigen::Vector3d &nv) {
    if (eig_cal_flag_) {
        eig0 = eig;
        nv = normal_vec_;
    }
    return eig_cal_flag_;
}

void VoxelTree::SubVoxel::EigenCal() {
    if (!eig_cal_flag_ && N >= eigen_decomposition_num_thres_) {
        Eigen::Matrix3d vt;
        utils::DMmath::Matrix3dEigenDecomposition(cov, eig, vt);
        if (fabs(eig(1)) < 1.e-04 || fabs(eig(2)) > 2.e-05) return;
        if (fabs(eig(2) / eig(1)) > 0.1) return;
        normal_vec_ = vt.row(2).transpose();
        eig_cal_flag_ = true;
    }
}

void VoxelTree::SubVoxel::Nullification() {
    N  = 0.5 * N;
}

VoxelTree::VoxelTree(const Eigen::Vector3d &pivot, double voxel_size) {
    subvoxel_size_inv_ = 1.0 / subvoxel_size_;
    subvoxel_dim_ = std::ceil(voxel_size * subvoxel_size_inv_);
    subvoxel_dim_ = subvoxel_dim_ > 0? subvoxel_dim_ : 1;
    size_t cap = subvoxel_dim_ * subvoxel_dim_ * subvoxel_dim_ + 1;
    subvoxels_.resize(cap);
    pivot_ = pivot;

    query_thres_ = 10;
}

size_t VoxelTree::Index(const Eigen::Vector3d &pt) {
    Eigen::Vector3i idxs = ((pt - pivot_) * subvoxel_size_inv_).cast<int>();
    return (idxs(0) + idxs(1) * subvoxel_dim_ + idxs(2) * subvoxel_dim_ * subvoxel_dim_);
}

void VoxelTree::AddPoint(const Eigen::Vector3d &pt) {
    int idx = Index(pt);
    subvoxels_[idx].AddPoint(pt);
}

void VoxelTree::AddPoint(const Eigen::Vector3d &pt, const Eigen::Matrix3d &R, const Eigen::Vector3d &t) {
    int idx = Index(pt);
    subvoxels_[idx].AddPoint(pt, R, t);
}

void VoxelTree::AddDistribution(const Eigen::Vector3d &pt0, const Eigen::Matrix3d &cov0, int N0) {
    int idx = Index(pt0);
    subvoxels_[idx].AddDistribution(pt0, cov0, N0);
}

void VoxelTree::AddDistribution(const Eigen::Vector3d &pt0, const Eigen::Matrix3d &cov0, int N0,
                                const Eigen::Matrix3d &R, const Eigen::Vector3d &t) {
    int idx = Index(pt0);
    subvoxels_[idx].AddDistribution(pt0, cov0, N0, R, t);
}

bool VoxelTree::findNeighbors(const Eigen::Vector3d &pt, SubVoxel &result) {
    int idx = -1;
    double min_dist = 1000;
    for (size_t i = 0; i < subvoxels_.size(); i++) {
        const auto &tmp = subvoxels_[i];
        if (tmp.N > query_thres_) {
            double dist = (tmp.center - pt).squaredNorm();
            if (dist < min_dist) {
                idx = i;
                min_dist = dist;
            }
        }
    }
    if (idx >= 0) {
        result = subvoxels_[idx];
        return true;
    } else {
        return false;
    }
}

VoxelTree::SubVoxel *VoxelTree::findNeighbors(const Eigen::Vector3d &pt) {
    int idx = -1;
    double min_dist = 1000;
    for (size_t i = 0; i < subvoxels_.size(); i++) {
        const auto &tmp = subvoxels_[i];
        if (tmp.N >= tmp.eigen_decomposition_num_thres_) {
            double dist = (tmp.center - pt).squaredNorm();
            if (dist < min_dist) {
                idx = i;
                min_dist = dist;
            }
        }
    }
    if (idx >= 0) {
        return &subvoxels_[idx];
    } else {
        return nullptr;
    }
}


bool VoxelTree::findVoxel(const Eigen::Vector3d &pt, size_t &idx) {
    int idx0 = Index(pt);
    idx = static_cast<size_t>(idx0);
    if (subvoxels_[idx].N > 1) {
        return true;
    } else {
        return false;
    }
}

VoxelTree::SubVoxel *VoxelTree::findSubVoxel(const Eigen::Vector3d &pt) {
    int idx0 = Index(pt);
    return &subvoxels_[idx0];
}

// ----------------------------build voxel tree map---------------------------
VoxelTreeMap::VoxelTreeMap(double voxel_size): voxel_size_(voxel_size) {
    voxel_size_inv_ = 1.0 / voxel_size_;

    x_min_ = 100000;
    y_min_ = 100000;
    z_min_ = 100000;

    x_max_ = -100000;
    y_max_ = -100000;
    z_max_ = -100000;
}

VoxelKey VoxelTreeMap::Key(const Eigen::Vector3d &pt) const {
    int64_t xyz[3];
    xyz[0] = std::floor(pt(0) * voxel_size_inv_);
    xyz[1] = std::floor(pt(1) * voxel_size_inv_);
    xyz[2] = std::floor(pt(2) * voxel_size_inv_);
    return VoxelKey(xyz[0], xyz[1], xyz[2]);
}

Eigen::Vector3d VoxelTreeMap::Pivot(const Eigen::Vector3d &pt) const {
    int64_t xyz[3];
    xyz[0] = std::floor(pt(0) * voxel_size_inv_);
    xyz[1] = std::floor(pt(1) * voxel_size_inv_);
    xyz[2] = std::floor(pt(2) * voxel_size_inv_);
    return Eigen::Vector3d(xyz[0] * voxel_size_, xyz[1] * voxel_size_, xyz[2] * voxel_size_);
}

void VoxelTreeMap::AddPoint(const Eigen::Vector3d &pt) {
    VoxelKey key = Key(pt);
    auto iter = maps_.find(key);
    if (iter == maps_.end()) {
        Eigen::Vector3d pivot = Pivot(pt);
        maps_.insert(std::pair<VoxelKey, VoxelTree>(key, VoxelTree(pivot, voxel_size_)));
        maps_[key].AddPoint(pt);
    } else {
        iter->second.AddPoint(pt);
        // maps_[key].AddPoint(pt);
    }
}

void VoxelTreeMap::AddPoint(const Eigen::Vector3d &pt, const Eigen::Matrix3d &R, const Eigen::Vector3d &t) {
    VoxelKey key = Key(pt);
    auto iter = maps_.find(key);
    if (iter == maps_.end()) {
        Eigen::Vector3d pivot = Pivot(pt);
        maps_.insert(std::pair<VoxelKey, VoxelTree>(key, VoxelTree(pivot, voxel_size_)));
        maps_[key].AddPoint(pt, R, t);
    } else {
        iter->second.AddPoint(pt, R, t);
        // maps_[key].AddPoint(pt, R, t);
    }
}


void VoxelTreeMap::AddDistribution(const Eigen::Vector3d &pt0, const Eigen::Matrix3d &cov0, int N0) {
    VoxelKey key = Key(pt0);
    auto iter = maps_.find(key);
    if (iter == maps_.end()) {
        Eigen::Vector3d pivot = Pivot(pt0);
        maps_.insert(std::pair<VoxelKey, VoxelTree>(key, VoxelTree(pivot, voxel_size_)));
        maps_[key].AddDistribution(pt0, cov0, N0);
    } else {
        iter->second.AddDistribution(pt0, cov0, N0);
        // maps_[key].AddDistribution(pt0, cov0, N0);
    }
}

void VoxelTreeMap::AddDistributionAndOccupancyUpdation(const Eigen::Vector3d &pt0, const Eigen::Matrix3d &cov0, int N0, const Eigen::Vector3d &pos) {
    AddDistribution(pt0, cov0, N0);
    RayCastingOccupancyUpdation(pos, pt0);
}

void VoxelTreeMap::AddDistribution(const Eigen::Vector3d &pt0, const Eigen::Matrix3d &cov0, int N0,
                                   const Eigen::Matrix3d &R, const Eigen::Vector3d &t) {
    VoxelKey key = Key(pt0);
    auto iter = maps_.find(key);
    if (iter == maps_.end()) {
        Eigen::Vector3d pivot = Pivot(pt0);
        maps_.insert(std::pair<VoxelKey, VoxelTree>(key, VoxelTree(pivot, voxel_size_)));
        maps_[key].AddDistribution(pt0, cov0, N0, R, t);
    } else {
        maps_[key].AddDistribution(pt0, cov0, N0, R, t);
    }
}

// raycast from pt0 to pt1; pt0: robot pose center, pt1: observation point
// reference: bresenham 3D line drawing
// https://github.com/SainsburyWellcomeCentre/aMAP_validation/blob/master/externalModules/NIfTI_20140122/bresenham_line3d.m
void VoxelTreeMap::RayCastingOccupancyUpdation(const Eigen::Vector3d &pt0, const Eigen::Vector3d &pt1) {
    int sub = 16;
    double precision = sub / voxel_size_inv_ / VoxelTree::subvoxel_size_;
    double precision_inv = 1. / precision;
    int64_t xyz[3], xyz0[3], xyz1[3], dxyz[3], axyz[3], sxyz[3];
    xyz0[0] = std::floor(precision * pt0(0) * voxel_size_inv_);
    xyz0[1] = std::floor(precision * pt0(1) * voxel_size_inv_);
    xyz0[2] = std::floor(precision * pt0(2) * voxel_size_inv_);

    xyz1[0] = std::floor(precision * pt1(0) * voxel_size_inv_);
    xyz1[1] = std::floor(precision * pt1(1) * voxel_size_inv_);
    xyz1[2] = std::floor(precision * pt1(2) * voxel_size_inv_);

    dxyz[0] = xyz1[0] - xyz0[0];
    dxyz[1] = xyz1[1] - xyz0[1];
    dxyz[2] = xyz1[2] - xyz0[2];

    axyz[0] = abs(dxyz[0]) * 2;
    axyz[1] = abs(dxyz[1]) * 2;
    axyz[2] = abs(dxyz[2]) * 2;

    sxyz[0] = dxyz[0] >= 0? 1 : -1;
    sxyz[1] = dxyz[1] >= 0? 1 : -1;
    sxyz[2] = dxyz[2] >= 0? 1 : -1;

    xyz[0] = xyz0[0];
    xyz[1] = xyz0[1];
    xyz[2] = xyz0[2];

    // select dominant axis
    int idxs[3];
    idxs[0] = 0;
    idxs[1] = 1;
    idxs[2] = 2;
    int max = axyz[0];
    if (axyz[1] > max) {
        idxs[0] = 1;
        idxs[1] = 0;
        idxs[2] = 2;
        max = axyz[1];
    }

    if (axyz[2] > max) {
        idxs[0] = 2;
        idxs[1] = 0;
        idxs[2] = 1;
    }

    int d0 = axyz[idxs[1]] - axyz[idxs[0]] / 2;
    int d1 = axyz[idxs[2]] - axyz[idxs[0]] / 2;

    int subvoxel_dim_ = std::ceil(voxel_size_ / VoxelTree::subvoxel_size_);
    int end_pt = xyz1[idxs[0]] - 1.25 * sub * sxyz[idxs[0]];

    if (sxyz[idxs[0]] * (end_pt - xyz[idxs[0]]) <= 0) return;
    int64_t res[3];
    while (1) {
        res[0] = std::floor(xyz[0] * precision_inv);
        res[1] = std::floor(xyz[1] * precision_inv);
        res[2] = std::floor(xyz[2] * precision_inv);

        int64_t tmp[3];
        tmp[0] = (xyz[0] - res[0] * precision) / sub;
        tmp[1] = (xyz[1] - res[1] * precision) / sub;
        tmp[2] = (xyz[2] - res[2] * precision) / sub;

        int idx = tmp[0] + tmp[1] * subvoxel_dim_ + tmp[2] * subvoxel_dim_ * subvoxel_dim_;

        // Eigen::Vector3d pt0;
        // pt0 << xyz[0], xyz[1], xyz[2];
        // pt0 = pt0 * voxel_size_ / precision;

        // VoxelKey key0 = Key(pt0);
        VoxelKey key(res[0], res[1], res[2]);

        auto iter = maps_.find(key);
        if (iter != maps_.end()) {
            // todo: missing voxel updation
            if (iter->second.Voxel(idx).N > 3) {
                iter->second.Voxel(idx).Nullification();
                iter->second.Voxel(idx).N = 1000;
            }
        }

        if (xyz[idxs[0]] == end_pt) break;

        if (d0 >= 0) {
            xyz[idxs[1]] += sxyz[idxs[1]];
            d0 -= axyz[idxs[0]];
        }

        if (d1 >= 0) {
            xyz[idxs[2]] += sxyz[idxs[2]];
            d1 -= axyz[idxs[0]];
        }

        xyz[idxs[0]] += sxyz[idxs[0]];
        d0 += axyz[idxs[1]];
        d1 += axyz[idxs[2]];
    }
}

bool VoxelTreeMap::findNeighborsByHash(const Eigen::Vector3d &pt, VoxelType &result) {
    VoxelKey key = Key(pt);
    auto iter = maps_.find(key);
    if (iter == maps_.end()) {
        return false;
    } else {
        if (iter->second.findNeighbors(pt, result)) {
            return true;
        } else {
            return false;
        }
    }
}

VoxelTreeMap::VoxelType *VoxelTreeMap::findNeighborsByHash(const Eigen::Vector3d &pt) {
    VoxelKey key = Key(pt);
    auto iter = maps_.find(key);
    if (iter == maps_.end()) {
        return nullptr;
    } else {
        return iter->second.findNeighbors(pt);
    }
}

VoxelTreeMap::VoxelType *VoxelTreeMap::FindVoxel(const Eigen::Vector3d &pt) {
    VoxelKey key = Key(pt);
    auto iter = maps_.find(key);
    if (iter == maps_.end()) {
        return nullptr;
    } else {
        return iter->second.findSubVoxel(pt);
    }
}

bool VoxelTreeMap::Find(const Eigen::Vector3d &pt) {
    VoxelKey key = Key(pt);
    if (maps_.find(key) == maps_.end()) {
        return false;
    } else {
        return true;
    }
}

// boundary = [x_min, x_max, y_min, y_max, z_min, z_max]
void VoxelTreeMap::LoadBoundary(const std::vector<double> &boundary) {
    if (boundary.size() < 6) return;
    if (boundary[0] < x_min_) x_min_ = boundary[0];
    if (boundary[1] > x_max_) x_max_ = boundary[1];

    if (boundary[2] < y_min_) y_min_ = boundary[2];
    if (boundary[3] > y_max_) y_max_ = boundary[3];

    if (boundary[4] < z_min_) z_min_ = boundary[4];
    if (boundary[5] > z_max_) z_max_ = boundary[5];
}

// --------------------------------------build voxel grid---------------------------------------------
VoxelGrid::VoxelGrid(double voxel_size) {
    voxel_size_ = voxel_size;
    voxel_size_inv_ = 1.0 / voxel_size;
}

void VoxelGrid::SetGridSize(const double &voxel_size) {
    voxel_size_ = voxel_size;
    voxel_size_inv_ = 1.0 / voxel_size;
}


VoxelKey VoxelGrid::Key(const Eigen::Vector3d &pt) const {
    int64_t xyz[3];
    xyz[0] = pt(0) * voxel_size_inv_;
    xyz[1] = pt(1) * voxel_size_inv_;
    xyz[2] = pt(2) * voxel_size_inv_;
    return VoxelKey(xyz[0], xyz[1], xyz[2]);
}

void VoxelGrid::AddPoint(const Eigen::Vector3d &pt) {
    VoxelKey key = Key(pt);
    auto iter = maps_.find(key);
    if (iter == maps_.end()) {
        maps_[key] = VoxelTree::SubVoxel(pt);
    } else {
        maps_[key].AddPoint(pt);
    }
}

VoxelGrid::VoxelType *VoxelGrid::FindNeighbors(const Eigen::Vector3d &pt, const Eigen::Vector3d &nv, int range) {
    VoxelKey key = Key(pt);
    auto iter = maps_.find(key);
    if (iter == maps_.end()) {
        Eigen::Vector3d pt0;
        for (int i = 1; i < range; i++) {
            pt0 = pt + voxel_size_ * i * nv;
            key = Key(pt0);
            iter = maps_.find(key);
            if (iter == maps_.end()) {
                pt0 = pt - voxel_size_ * i * nv;
                key = Key(pt0);
                iter = maps_.find(key);
                if (iter != maps_.end()) {
                    return &iter->second;
                }
            } else {
                return &iter->second;
            }
        }
    } else {
        return &iter->second;
    }
    return nullptr;
}

void VoxelGrid::AddPointCloud(const std::vector<Eigen::Vector3d> &pointcloud) {
    for (size_t i = 0; i < pointcloud.size(); i++) {
        VoxelKey key = Key(pointcloud[i]);
        auto iter = maps_.find(key);
        if (iter == maps_.end()) {
            maps_[key] = VoxelTree::SubVoxel(pointcloud[i]);
        } else {
            maps_[key].AddPoint(pointcloud[i]);
        }
    }
}

void VoxelGrid::GetBoundary(std::vector<double> &boundary) {
    MapType::iterator iter = maps_.begin();
    double x_min = 1000000, x_max = -1000000;
    double y_min = 1000000, y_max = -1000000;
    double z_min = 1000000, z_max = -1000000;
    for (; iter != maps_.end(); iter++) {
        Eigen::Vector3d &pt = iter->second.center;
        if (pt(0) > x_max) x_max = pt(0);
        if (pt(0) < x_min) x_min = pt(0);

        if (pt(1) > y_max) y_max = pt(1);
        if (pt(1) < y_min) y_min = pt(1);

        if (pt(2) > z_max) z_max = pt(2);
        if (pt(2) < z_min) z_min = pt(2);
    }
    boundary.clear();
    boundary.push_back(x_min);
    boundary.push_back(x_max);
    boundary.push_back(y_min);
    boundary.push_back(y_max);
    boundary.push_back(z_min);
    boundary.push_back(z_max);
}

// void LocalMap::Reset(const Eigen::Matrix3d &Rbw0, const Eigen::Vector3d &tbw0) {
//     map.Clear();
//     Rbw = Rbw0;
//     tbw = tbw0;
// }

void LocalMap::Reset(const utils::Transform3d &Tbw0) {
    map.Clear();
    Tbw = Tbw0;
}

void LocalMap::Reset() {
    Tbw.SetIdentity();
    map.Clear();
}

void VoxelMapSetting(const nlohmann::json &config) {
    VoxelTree::subvoxel_size_ = config["subvoxel_size"];
    VoxelTree::SubVoxel::eigen_decomposition_num_thres_ = config["eigen_decomposition_num_thres"];
}

} // namespace slam