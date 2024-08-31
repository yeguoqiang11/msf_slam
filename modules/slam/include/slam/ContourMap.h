/**
 * @file ContourMap.h
 * @author Guoqiang Ye (yegq@dreame.tech)
 * @brief 
 * @version 0.1
 * @date 2022-01-18
 * 
 * @copyright Copyright (c) 2022 Dreame Technology Co.Ltd. All rights reserved
 * 
 */
#ifndef SLAM_CONTOURMAP_H
#define SLAM_CONTOURMAP_H
#include <opencv2/opencv.hpp>

#include "slam/VoxelMap.h"
#include "utils/DMmath.h"

namespace slam {
class GridKey2D {
  public:
    int64_t x_, y_;
    GridKey2D(int64_t x = 0, int64_t y = 0): x_(x), y_(y) {}
    
    bool operator== (const GridKey2D &in) const {
        return ((x_ == in.x_) && (y_ == in.y_));
    }
};
} // namespace slam

namespace std {
    template<>
    struct hash<slam::GridKey2D> {
        size_t operator() (const slam::GridKey2D &s) const {
            using std::size_t; using std::hash;
            return (hash<int64_t>()(s.x_) ^ (hash<int64_t>()(s.y_) << 1));
        }
    };
} // namespace std

namespace slam {
class Grid {
  public:
    virtual void UpdateMaxHeight(const double &x, const double &y, const double &h, const double &resolution) = 0;
    virtual bool IsMatched(const double &h, const double &resolution) = 0;
    virtual Eigen::Vector2d Center() = 0;
    virtual int MaxHeight() = 0;
};

class MaxHeightGrid: public Grid {
  public:
    MaxHeightGrid(): N(0), center_(0, 0) {}
    void UpdateMaxHeight(const double &x, const double &y, const double &h, const double &resolution) {
        double s = 1.0 / (N + 1);
        center_ += Eigen::Vector2d(x - center_(0), y - center_(1)) * s;

        int tmp_height = static_cast<int>(h / resolution + 0.5);
        if(tmp_height > height_) height_ = tmp_height;
        N = N > 100? 100 : N + 1;
    }

    bool IsMatched(const double &h, const double &resolution) {
        return (static_cast<int>(h / resolution + 0.5) == height_);
    }

    Eigen::Vector2d Center() { return center_; }
    int MaxHeight() { return height_; }

    Eigen::Vector2d center_;
    int N;
    int height_;
};

class PyramidGrid: public Grid {
  public:
    PyramidGrid():N(0), center_(0, 0) {}
    void UpdateMaxHeight(const double &x, const double &y, const double &h, const double &resolution) {
        double s = 1.0 / (N + 1);
        center_ += Eigen::Vector2d(x - center_(0), y - center_(1)) * s;
        heights_.insert(static_cast<int>(h / resolution + 0.5));
        N = N > 100? 100 : N + 1;
    }

    bool IsMatched(const double &h, const double &resolution) {
        return (heights_.find(static_cast<int>(h / resolution + 0.5)) != heights_.end());
    }

    Eigen::Vector2d Center() { return center_; }
    int MaxHeight() { return *heights_.crbegin(); }
    Eigen::Vector2d center_;
    int N;
    std::set<int> heights_;
};

class ContourMapBase {
  public:
    ContourMapBase(const double &max_height, const double &grid_size, const double &height_resolution);
    GridKey2D Key(const double &x, const double &y);
    double GridSize() { return grid_size_; }
    virtual size_t Size() = 0;
    virtual cv::Mat ShowMat(int img_size = 500, double ratio = 1.) = 0;
    double HeightResolution() { return height_resolution_; }
    virtual bool IsMatched(const double &x, const double &y, const double &h) = 0;
    double MaxHeight() { return max_height_; }
  protected:
    double max_height_;
    double grid_size_, height_resolution_;
};

class MaxHeightMap: public ContourMapBase {
  public:
    typedef std::unordered_map<GridKey2D, MaxHeightGrid> MapType;
    MaxHeightMap(double grid_size, double max_height, double height_resolution);
    void AddPoint(const double &x, const double &y, const double &h);
    MapType::iterator Begin() { return map_.begin(); }
    MapType::iterator End() { return map_.end(); }
    size_t Size() { return map_.size(); }
    cv::Mat ShowMat(int img_size = 500, double ratio = 1.);
    bool IsMatched(const double &x, const double &y, const double &h);
    void AddGrid(VoxelGrid &grid, Eigen::Matrix3d Rbw = Eigen::Matrix3d::Identity(), Eigen::Vector3d tbw = Eigen::Vector3d(0, 0, 0));
  private:
    std::unordered_map<GridKey2D, MaxHeightGrid> map_;
};

class PyramidMap: public ContourMapBase {
  public:
    typedef std::unordered_map<GridKey2D, PyramidGrid> MapType;
    PyramidMap(double grid_size, double max_height, double height_resolution);
    void AddPoint(const double &x, const double &y, const double &h);
    MapType::iterator Begin() { return map_.begin(); }
    MapType::iterator End() { return map_.end(); }
    size_t Size() { return map_.size(); }
    cv::Mat ShowMat(int img_size = 500, double ratio = 1.);
    bool IsMatched(const double &x, const double &y, const double &h);
  private:
    std::unordered_map<GridKey2D, PyramidGrid> map_;
};

class ContourMapStack {
  public:
    typedef MaxHeightMap MapType0;
    typedef PyramidMap Maptypen;
    ContourMapStack();
    ContourMapStack(double raw_grid_size, double ratio, size_t max_depth);
    void Init();
    void AddGrid(VoxelGrid &grid, const Eigen::Matrix3d &Rbw, const Eigen::Vector3d &tbw);
    size_t Size() { return pyramid_maps_.size(); }
    std::vector<double> Boundary();
    MaxHeightMap &HeightMap() { return map0_; }
    PyramidMap &PyramidMapIndexing(int i) {
      i = i >= pyramid_maps_.size()? pyramid_maps_.size() - 1 : i;
      return pyramid_maps_[i];
    }
    void MapShow();
  private:
    double raw_grid_size_, ratio_;
    size_t max_depth_;
    double x_min_, x_max_, y_min_, y_max_, z_min_, z_max_;
    MaxHeightMap map0_;
    std::vector<PyramidMap> pyramid_maps_;
};

struct PoseAndScore {
    PoseAndScore(const double &score0, const double &x0, const double &y0, const double &yaw0):
      score(score0), x(x0), y(y0), yaw(yaw0) {}

    void SetScoreAndPose(const double &score0, const double &x0, const double &y0, const double &yaw0) {
      score = score0;
      x = x0;
      y = y0;
      yaw = yaw0;
    }

    double score, upper_bound;
    double x, y, yaw;
    double step, angle_step;
    int level;
    bool operator>(const PoseAndScore &other) const { return score > other.score; }
    bool operator<(const PoseAndScore &other) const { return score < other.score; }
};

class BranchAndBound {
  public:
    BranchAndBound(double range = 0.1, double raw_grid_size = 0.1, double ratio = 1.5, size_t max_depth = 3);
    BranchAndBound(std::shared_ptr<ContourMapStack> maps, double range = 0.1);
    void PoseIndexing(VoxelGrid &grid, Eigen::Matrix3d &Rbw, Eigen::Vector3d &tbw);
    MaxHeightMap VoxelGrid2MaxHeightMap(VoxelGrid &grid, Eigen::Matrix3d &Rbw, Eigen::Vector3d &tbw);
    std::vector<MaxHeightMap> VoxelGrid2MaxHeightMap(VoxelGrid &grid, Eigen::Matrix3d &Rbw, Eigen::Vector3d &tbw, double ratio = 1.5, double level = 3);
    void BranchAndBound2DMap(MaxHeightMap &query_map, Eigen::Matrix2d &Rbw, Eigen::Vector2d &tbw);
    int PyramidMapMatchScore(int map_idx, MaxHeightMap &query_map, const Eigen::Matrix2d &Rbw, const Eigen::Vector2d &tbw);
    int MaxHeightMapMatchScore(MaxHeightMap &query_map, const Eigen::Matrix2d &Rbw, const Eigen::Vector2d &tbw);
    std::vector<double> MaxHeightMapBoundCalc(MaxHeightMap &query_map);
    bool IsInsideMap(const Eigen::Vector2d &pt, const std::vector<double> &bound);
  private:
    double range_;
    std::shared_ptr<ContourMapStack> pyramid_maps_ptr_;
    double height_inlier_range_;
    double yaw_resolution_;
    double x_resolution_, y_resolution_;
    double ratio_;
    std::vector<double> errors_;
};
} // namespace slam
#endif // SLAM_CONTOURMAP_H