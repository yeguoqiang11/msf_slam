/**
 * @file VoxelMap.h
 * @author Guoqiang Ye (yegq@dreame.tech)
 * @brief 
 * @version 0.1
 * @date 2022-01-18
 * 
 * @copyright Copyright (c) 2022 Dreame Technology Co.Ltd. All rights reserved
 * 
 */
#ifndef MODULES_SLAM_VOXEL_MAP
#define MODULES_SLAM_VOXEL_MAP
#include <iostream>
#include <memory>
#include <unordered_map>
#include <vector>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/SVD>

#include "utils/json.hpp"
#include "utils/nanoflann.hpp"
#include "utils/DMmath.h"
namespace slam {
extern std::vector<Eigen::Vector3d> dynamic_pts;
inline Eigen::Vector3d PivotPoint(const Eigen::Vector3d &pt, double voxel_size = 0.05);
// key for voxel indexing
class VoxelKey {
  public:
    int64_t x_, y_, z_;
    VoxelKey(int64_t x = 0, int64_t y = 0, int64_t z = 0): x_(x), y_(y), z_(z) {}
    
    bool operator== (const VoxelKey &in) const {
        return ((x_ == in.x_) && (y_ == in.y_) && (z_ == in.z_));
    }
};
struct FeatType{
    FeatType() { point.fill(-1.); direction.fill(0.); }
    FeatType(const Eigen::Vector3d &pt, Eigen::Vector3d dir = Eigen::Vector3d(0, 0, 0)): point(pt), direction(dir) {}
    FeatType(const Eigen::Vector3d &pt0, const Eigen::Vector3d &pt_trans, Eigen::Vector3d dir = Eigen::Vector3d(0, 0, 0)):
        point_orig(pt0), point(pt_trans), direction(dir) {}
    void SetOriginPoint(const Eigen::Vector3d &pt) { point_orig = pt; }
    Eigen::Vector3d point_orig;
    Eigen::Vector3d point;
    Eigen::Vector3d direction;
    bool is_inlier;
};

// voxel box
class VoxelBox {
  public:
    VoxelBox() {}
    VoxelBox(const FeatType &feat);
    void SetCenter(Eigen::Vector3d pt) { center_.point = pt; }
    const Eigen::Vector3d &Center() const { return center_.point; }
    const FeatType &CenterFeature() const { return center_; }
    void InsertFeature(const FeatType &feat);
    size_t VoxelBoxSize() { return features_.size(); }
    void InsertFeature2Set(const FeatType &feat);
    const FeatType &operator()(const FeatType &feat) const;
    void GetNearbyFeatures(const FeatType &feat, std::vector<FeatType> &outs) const;
    void GetNearbyPoint(const Eigen::Vector3d &pt, std::vector<FeatType> &outs) const;
  private:
    FeatType center_;
    std::vector<FeatType> features_;
    std::vector<std::vector<FeatType>> features_set_;
    Eigen::Vector3d pivot_point_;
  public:
    static double voxel_size_;
    static int subvoxel_num_;
    static double subvoxel_size_;
};
}

// std hash key for voxel
namespace std {
    template<>
    struct hash<slam::VoxelKey> {
        size_t operator() (const slam::VoxelKey &s) const {
            using std::size_t; using std::hash;
            return ((hash<int64_t>()(s.x_) ^ (hash<int64_t>()(s.y_) << 1)) >> 1) ^
                (hash<int64_t>()(s.z_ << 1));
        }
    };
}
namespace slam {
// voxel map
class VoxelMap {
  public:
    VoxelMap() {}
    VoxelKey Key(const FeatType &feat) const;
    VoxelKey Key(const Eigen::Vector3d &pt) const;
    void InsertFeature(const FeatType &feat);
    size_t MapSize() const { return maps_.size(); }
    double kdtree_get_pt(const size_t &idx, const size_t &dim) const;
    VoxelBox &operator()(const FeatType &feat);
    bool GetVoxelBox(VoxelBox &voxel, const FeatType &feat);
    const VoxelBox &operator()(const size_t &idx) const;
    const VoxelBox &QueryVoxelBox(const Eigen::Vector3d &pt) const;
    void Released() { maps_.clear(); keys_.clear(); }
    bool Empty() { return (maps_.size() == 0); }
    bool Find(const FeatType &feat) const;
    bool Find(const Eigen::Vector3d &pt) const;

  private:
    std::unordered_map<VoxelKey, VoxelBox> maps_;
    std::vector<VoxelKey> keys_;
  public:
    static double voxel_size_;
};

// map adaptor for kdtree
class MapAdaptor {
  public:
    const VoxelMap &map_;
    int dim_;
    double scale_;
    MapAdaptor(const VoxelMap &map, int dim = -1, double scale = 1.0) : map_(map), dim_(dim), scale_(scale) {}
    inline size_t kdtree_get_point_count() const { return map_.MapSize(); }
    inline double kdtree_get_pt(const size_t idx, const size_t dim) const {
        if (dim == dim_) return map_.kdtree_get_pt(idx, dim) * scale_;
        return map_.kdtree_get_pt(idx, dim);
    }
    const VoxelBox &operator()(const size_t &idx) const;
    template<class BBOX>
    bool kdtree_get_bbox(BBOX&) const { return false; }
};

typedef nanoflann::KDTreeSingleIndexDynamicAdaptor<nanoflann::L2_Simple_Adaptor<double, MapAdaptor>, MapAdaptor, 3> MapKDTreeType;

// map kdtree for knn
class MapKdTree {
  public:
    MapKdTree(int dim = -1, size_t capacity = 5);
    ~MapKdTree();
    void InsertFeature(const FeatType &feature) { voxel_map_.InsertFeature(feature); }
    void IndexRebuild();
    std::vector<FeatType> findNeighbors(const FeatType &feat, int index_num = 3);
    std::vector<FeatType> findNeighborsByHash(const Eigen::Vector3d &pt, int index_num = 3);
    void Init();
    bool Empty() { return voxel_map_.Empty();}
    size_t Size() const { return voxel_map_.MapSize(); }
    const VoxelBox &operator()(const size_t &idx) const;
    VoxelBox &operator()(const FeatType &feat) { return voxel_map_(feat); }
    bool Find(const FeatType &feat) { return voxel_map_.Find(feat); }
  private:
    VoxelMap voxel_map_;
    std::shared_ptr<MapAdaptor> map_adaptor_ptr_;
    std::shared_ptr<MapKDTreeType> map_kdtree_ptr_;
    size_t built_count;
    size_t dim_;
    double scale_;
    size_t query_capacity_;
    size_t *ret_index_;
    double *ret_dist_;
    nanoflann::KNNResultSet<double> query_result_;
};

// voxel tree
class VoxelTree {
  public:
    struct SubVoxel {
        SubVoxel();
        SubVoxel(const Eigen::Vector3d &pt);
        void AddPoint(const Eigen::Vector3d &pt);
        void AddPoint(const Eigen::Vector3d &pt, const Eigen::Matrix3d &R, const Eigen::Vector3d &t);
        void AddDistribution(const Eigen::Vector3d &pt0, const Eigen::Matrix3d &cov0, int N0);
        void AddDistribution(const Eigen::Vector3d &pt0, const Eigen::Matrix3d &cov0, int N0,
                            const Eigen::Matrix3d &R, const Eigen::Vector3d &t);
        void Nullification();
        void SetEigenInfo(const Eigen::Vector3d &eig0, const Eigen::Vector3d &nv);
        bool GetEigenInfo(Eigen::Vector3d &eig0, Eigen::Vector3d &nv);
        void EigenCal();
        Eigen::Vector3d center;
        Eigen::Matrix3d cov, cov_inv;
        int N;
        std::vector<Eigen::Vector3d> pts;
        bool eig_cal_flag_;
        Eigen::Vector3d eig, normal_vec_;
        double odds_;
        static int eigen_decomposition_num_thres_;
    };
    VoxelTree() {std::cout << "this constructor of voxel tree should not be called" << std::endl; exit(0); }
    VoxelTree(const Eigen::Vector3d &pivot, double voxel_size = 0.1);
    size_t Index(const Eigen::Vector3d &pt);
    void AddPoint(const Eigen::Vector3d &pt);
    void AddPoint(const Eigen::Vector3d &pt, const Eigen::Matrix3d &R, const Eigen::Vector3d &t);
    void AddDistribution(const Eigen::Vector3d &pt0, const Eigen::Matrix3d &cov0, int N0);
    void AddDistribution(const Eigen::Vector3d &pt0, const Eigen::Matrix3d &cov0, int N0,
                        const Eigen::Matrix3d &R, const Eigen::Vector3d &t);
    bool findNeighbors(const Eigen::Vector3d &pt, SubVoxel &result);
    VoxelTree::SubVoxel *findNeighbors(const Eigen::Vector3d &pt);
    bool findVoxel(const Eigen::Vector3d &pt, size_t &idx);
    SubVoxel *findSubVoxel(const Eigen::Vector3d &pt);
    size_t Size() { return subvoxels_.size(); }
    SubVoxel &Voxel(size_t i) { return subvoxels_[i]; }
    Eigen::Vector3d Pivot() { return pivot_; }
    static double subvoxel_size_;
    static double damping_val_;
    static double p_hit_, p_miss_;

    std::vector<SubVoxel> subvoxels_;

  private:
    double voxel_size_, subvoxel_size_inv_;
    int subvoxel_dim_;
    Eigen::Vector3d pivot_;
    int query_thres_;
};

class VoxelTreeMap {
  public:
    typedef VoxelTree::SubVoxel VoxelType;
    typedef std::unordered_map<VoxelKey, VoxelTree> MapType;

    VoxelTreeMap(double voxel_size = 0.3);
    VoxelKey Key(const Eigen::Vector3d &pt) const;
    Eigen::Vector3d Pivot(const Eigen::Vector3d &pt) const;
    void AddPoint(const Eigen::Vector3d &pt);
    void AddPoint(const Eigen::Vector3d &pt, const Eigen::Matrix3d &R, const Eigen::Vector3d &t);
    void AddDistribution(const Eigen::Vector3d &pt0, const Eigen::Matrix3d &cov0, int N0);
    void AddDistributionAndOccupancyUpdation(const Eigen::Vector3d &pt0, const Eigen::Matrix3d &cov0, int N0, const Eigen::Vector3d &pos);

    void AddDistribution(const Eigen::Vector3d &pt0, const Eigen::Matrix3d &cov0, int N0,
                         const Eigen::Matrix3d &R, const Eigen::Vector3d &t);
    void RayCastingOccupancyUpdation(const Eigen::Vector3d &pt0, const Eigen::Vector3d &pt1);
    bool Empty() { return maps_.size() == 0; }
    size_t Size() { return maps_.size(); }
    bool findNeighborsByHash(const Eigen::Vector3d &pt, VoxelType &result);
    VoxelTreeMap::VoxelType *findNeighborsByHash(const Eigen::Vector3d &pt);

    VoxelType *FindVoxel(const Eigen::Vector3d &pt);
    bool Find(const Eigen::Vector3d &pt);
    MapType::iterator begin() { return maps_.begin(); }
    MapType::iterator end() { return maps_.end(); }
    void Clear() { 
        maps_.clear();
        x_min_ = 100000;
        y_min_ = 100000;
        z_min_ = 100000;

        x_max_ = -100000;
        y_max_ = -100000;
        z_max_ = -100000;
    }
    void LoadBoundary(const std::vector<double> &boundary);

  private:
    double voxel_size_, voxel_size_inv_;
    std::unordered_map<VoxelKey, VoxelTree> maps_;
    double x_min_, x_max_;
    double y_min_, y_max_;
    double z_min_, z_max_;
};

class VoxelGrid {
  public:
    typedef VoxelTree::SubVoxel VoxelType;
    typedef std::unordered_map<VoxelKey, VoxelTree::SubVoxel> MapType;
    VoxelGrid(double voxel_size = 0.1);
    void SetGridSize(const double &voxel_size);
    VoxelKey Key(const Eigen::Vector3d &pt) const;
    void AddPoint(const Eigen::Vector3d &pt);
    void AddPointCloud(const std::vector<Eigen::Vector3d> &pointcloud);
    void Initialize() { maps_.clear(); }
    VoxelType *FindNeighbors(const Eigen::Vector3d &pt, const Eigen::Vector3d &nv, int range);
    MapType::iterator End() { return maps_.end(); }
    MapType::iterator Begin() { return maps_.begin(); }
    MapType::const_iterator CBegin() const { return maps_.cbegin(); }
    MapType::const_iterator CEnd() const { return maps_.cend(); }
    size_t Size() { return maps_.size(); }
    void GetBoundary(std::vector<double> &boundary); // boundary: [x_min, x_max, y_min, y_max, z_min, z_max];

  private:
    double voxel_size_, voxel_size_inv_;
    MapType maps_;
};

struct LocalMap {
  LocalMap(double voxel_size = 0.3): map(voxel_size) {}
  VoxelTreeMap map;
  utils::Transform3d Tbw;
  void Reset(const utils::Transform3d &Tbw);
  void Reset();
};

void VoxelMapSetting(const nlohmann::json &config);
}
#endif // MODULES_SLAM_VOXEL_MAP