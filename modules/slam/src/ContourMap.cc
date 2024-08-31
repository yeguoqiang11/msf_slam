#include "slam/ContourMap.h"

namespace slam {
//--------------------------------contour map base-------------------------
ContourMapBase::ContourMapBase(const double &max_height, const double &grid_size, const double &height_resolution)
  : max_height_(max_height),
    grid_size_(grid_size),
    height_resolution_(height_resolution) {

}

GridKey2D ContourMapBase::Key(const double &x, const double &y) {
    int64_t xy[2];
    xy[0] = static_cast<int64_t>(x / grid_size_);
    xy[1] = static_cast<int64_t>(y / grid_size_);

    return GridKey2D(xy[0], xy[1]);
}

//------------------------------------max height map------------------------
MaxHeightMap::MaxHeightMap(double grid_size, double max_height,  double height_resolution)
  : ContourMapBase(max_height, grid_size, height_resolution) {
}

void MaxHeightMap::AddPoint(const double &x, const double &y, const double &h) {
    GridKey2D key = Key(x, y);
    double h0 = h > max_height_? max_height_ : h;
    MapType::iterator iter = map_.find(key);
    if (iter != map_.end()) {
        iter->second.UpdateMaxHeight(x, y, h0, height_resolution_);
    } else {
        map_.insert(std::pair<GridKey2D, MaxHeightGrid>(std::move(key), std::move(MaxHeightGrid())));
        map_[key].UpdateMaxHeight(x, y, h0, height_resolution_);
    }
}

cv::Mat MaxHeightMap::ShowMat(int img_size, double ratio) {
    cv::Mat mat;
    mat.create(img_size, img_size, CV_8U);
    mat.setTo(0);
    int x0 = img_size / 2.0;
    int y0 = img_size / 2.0;
    MapType::iterator iter = map_.begin();
    double reso = 255 / max_height_ - 10;
    for (; iter != map_.end(); iter++) {
        const Eigen::Vector2d &pt = iter->second.center_;
        GridKey2D key = Key(pt(0), pt(1));
        double h = iter->second.height_ * height_resolution_;
        if (h > 0.3) {
            int gray = reso * h;
            int x = key.x_ + x0;
            int y = key.y_ + y0;
            x = x < 0? 0 : x < img_size? x : img_size;
            y = y < 0? 0 : y < img_size? y : img_size;
            mat.at<uchar>(x, y) = gray;
        }
    }
    return mat;
}

bool MaxHeightMap::IsMatched(const double &x, const double &y, const double &h) {
    GridKey2D key = Key(x, y);
    MapType::iterator iter = map_.find(key);
    if (iter != map_.end()) {
        return iter->second.IsMatched(h, height_resolution_);
    } else {
        return false;
    }
}

void MaxHeightMap::AddGrid(VoxelGrid &grid, Eigen::Matrix3d Rbw, Eigen::Vector3d tbw) {
    VoxelGrid::MapType::iterator grid_iter = grid.Begin();
    for (; grid_iter != grid.End(); grid_iter++) {
        Eigen::Vector3d pt = Rbw * grid_iter->second.center + tbw;
        AddPoint(pt(0), pt(2), -pt(1));
    }
}

//-------------------------------pyramid map------------------------------
PyramidMap::PyramidMap(double grid_size, double max_height, double height_resolution)
  : ContourMapBase(max_height, grid_size, height_resolution) {
}

void PyramidMap::AddPoint(const double &x, const double &y, const double &h) {
    GridKey2D key = Key(x, y);
    MapType::iterator iter = map_.find(key);
    if (iter != map_.end()) {
        iter->second.UpdateMaxHeight(x, y, h, height_resolution_);
    } else {
        map_.insert(std::pair<GridKey2D, PyramidGrid>(key, std::move(PyramidGrid())));
        map_[key].UpdateMaxHeight(x, y, h, height_resolution_);
    }
}

cv::Mat PyramidMap::ShowMat(int img_size, double ratio) {
    cv::Mat mat;
    mat.create(img_size, img_size, CV_8U);
    mat.setTo(0);
    int x0 = img_size / 2.0;
    int y0 = img_size / 2.0;
    MapType::iterator iter = map_.begin();
    double reso = 255 / max_height_ - 10;
    for (; iter != map_.end(); iter++) {
        const Eigen::Vector2d &pt = iter->second.center_;
        GridKey2D key = Key(pt(0), pt(1));
        double h = (iter->second.MaxHeight()) * height_resolution_;
        if (h > 0.3) {
            int gray = reso * h;
            int x = key.x_ * ratio + x0;
            int y = key.y_ * ratio + y0;
            x = x < 0? 0 : x < img_size? x : img_size;
            y = y < 0? 0 : y < img_size? y : img_size;
            mat.at<uchar>(x, y) = gray;
        }
    }
    return mat;
}

bool PyramidMap::IsMatched(const double &x, const double &y, const double &h) {
    GridKey2D key = Key(x, y);
    MapType::iterator iter = map_.find(key);
    if (iter != map_.end()) {
        return iter->second.IsMatched(h, height_resolution_);
    } else {
        return false;
    }
}

//------------------------contour map stack------------------------------
ContourMapStack::ContourMapStack()
  : map0_(0.1, 3, 0.2) {
    raw_grid_size_ = 0.10;
    ratio_ = 1.5;
    max_depth_ = 3;

    Init();
}

ContourMapStack::ContourMapStack(double raw_grid_size, double ratio, size_t max_depth)
  : raw_grid_size_(raw_grid_size),
    ratio_(ratio),
    max_depth_(max_depth),
    map0_(raw_grid_size_, 3, 0.1) {
    Init();
}

void ContourMapStack::Init() {
    for (size_t i = 1; i < max_depth_; i++) {
        PyramidMap map(raw_grid_size_ * std::pow(ratio_, i), 0.5, 0.1);
        pyramid_maps_.push_back(std::move(map));
    }

    x_min_ = 10000;
    x_max_ = -10000;
    y_min_ = 10000;
    y_max_ = -10000;
    z_min_ = 10000;
    z_max_ = -10000;
}

void ContourMapStack::AddGrid(VoxelGrid &grid, const Eigen::Matrix3d &Rbw, const Eigen::Vector3d &tbw) {
    // first level map updation
    VoxelGrid::MapType::iterator iter = grid.Begin();
    for (; iter != grid.End(); iter++) {
        Eigen::Vector3d pt = Rbw * iter->second.center + tbw;
        if (-pt(1) > 1) continue;
        map0_.AddPoint(pt(0), pt(2), -pt(1));

        if (pt(0) < x_min_) x_min_ = pt(0);
        if (pt(0) > x_max_) x_max_ = pt(0);
        if (pt(2) < y_min_) y_min_ = pt(2);
        if (pt(2) > y_max_) y_max_ = pt(2);
        if (-pt(1) < z_min_) z_min_ = -pt(1);
        if (-pt(1) > z_max_) z_max_ = -pt(1);
    }

    // pyramid map updation
    MaxHeightMap::MapType::iterator map0_iter = map0_.Begin();
    for (; map0_iter != map0_.End(); map0_iter++) {
        const Eigen::Vector2d &pt = map0_iter->second.Center();
        double h = map0_iter->second.height_ * map0_.HeightResolution();
        for (size_t i = 0; i < pyramid_maps_.size(); i++) {
            pyramid_maps_[i].AddPoint(pt(0), pt(1), h);
        }
    }
}

std::vector<double> ContourMapStack::Boundary() {
    std::vector<double> bound = {x_min_, x_max_, y_min_, y_max_, z_min_, z_max_};
    return std::move(bound);
}

void ContourMapStack::MapShow() {
    std::string txt;
    for (int i = 0; i < pyramid_maps_.size(); i++) {
        txt = std::to_string(i) + "th map";
        cv::Mat map = pyramid_maps_[i].ShowMat(500, 5);
        cv::imshow(txt, map);
    }
}

//----------------------------branch and bound------------------------------------------
BranchAndBound::BranchAndBound(double range, double raw_grid_size, double ratio, size_t max_depth)
  : range_(range) {
    height_inlier_range_ = 0.2;

    pyramid_maps_ptr_ = std::make_shared<ContourMapStack>(raw_grid_size, ratio, max_depth);

    // search step resolution
    yaw_resolution_ = 3 * M_PI / 180.;
    x_resolution_ = raw_grid_size;
    y_resolution_ = raw_grid_size;    
}

BranchAndBound::BranchAndBound(std::shared_ptr<ContourMapStack> maps, double range)
  : pyramid_maps_ptr_(std::move(maps)),
    range_(range) {
    height_inlier_range_ = 0.2;

    // searching step length
    double raw_grid_size = pyramid_maps_ptr_->HeightMap().GridSize();
    yaw_resolution_ = 3 * M_PI / 180.;
    x_resolution_ = raw_grid_size;
    y_resolution_ = raw_grid_size;    
}

void BranchAndBound::PoseIndexing(VoxelGrid &grid, Eigen::Matrix3d &Rbw, Eigen::Vector3d &tbw) {
    // build query z-projection map
    double grid_size = pyramid_maps_ptr_->HeightMap().GridSize();
    MaxHeightMap query_map(grid_size, 3, 0.2);
    query_map.AddGrid(grid);

    Eigen::Matrix3d Rbh; Rbh.setIdentity();
    Eigen::Vector3d tbh; tbh.setZero();
    std::vector<MaxHeightMap> query_maps = VoxelGrid2MaxHeightMap(grid, Rbh, tbh, 1.5, 5);

    Eigen::Matrix2d query_Rbw;
    Eigen::Vector2d query_tbw;
    BranchAndBound2DMap(query_map, query_Rbw, query_tbw);
    return;
    // ---------------------the below is for test and is not important---------------------------

    // test
    utils::Timer timecount;
    timecount.Start();
    std::vector<PoseAndScore> candidate_vec;
    // query
    int test_idx = pyramid_maps_ptr_->Size() - 2;
    test_idx = test_idx >= 0? test_idx : 0;
    int query_idx = query_maps.size() - 5;
    query_idx = query_idx >= 0? query_idx : 0;

    int max_score = -1000;
    Eigen::Matrix2d best_Rbw;
    Eigen::Vector2d best_tbw;
    double best_yaw;
    std::vector<double> bound = pyramid_maps_ptr_->Boundary();
    double step = pyramid_maps_ptr_->PyramidMapIndexing(test_idx).GridSize();
    std::cout << "step: " << step << "; map idx: " << test_idx << std::endl;
    // int query_map_size = query_maps[test_idx].Size();
    for (double x = bound[0]; x < bound[1]; x += step) {
        for (double y = bound[2]; y < bound[3]; y += step) {
            for (double yaw = 0; yaw < 2 * M_PI; yaw += yaw_resolution_ * 2) {
                Eigen::Vector2d tbw(x, y);
                Eigen::Matrix2d Rbw = utils::DMmath::Yaw2Matrix(yaw);
                int score = MaxHeightMapMatchScore(query_map, Rbw, tbw);
                // int score = PyramidMapMatchScore(test_idx, query_maps[query_idx], Rbw, tbw);
                PoseAndScore candidate(score, x, y, yaw);
                candidate_vec.push_back(std::move(candidate));

                if (score > max_score) {
                    best_Rbw = Rbw;
                    best_tbw = tbw;
                    best_yaw = yaw;
                    max_score = score;
                }
            }
        }
    }

    std::sort(candidate_vec.begin(), candidate_vec.end(), std::greater<PoseAndScore>());
    for (size_t i = 0; i < candidate_vec.size() && i < 30; i++) {
        const PoseAndScore &tmp = candidate_vec[i]; 
        std::cout << tmp.score << "th pose and score: " << tmp.x << ", " << tmp.y << ", " << tmp.yaw << std::endl;
    }
    if (candidate_vec.size() > 0) {
        const PoseAndScore &tmp = candidate_vec[0];
        Eigen::Vector2d tbw(tmp.x, tmp.y);
        Eigen::Matrix2d Rbw = utils::DMmath::Yaw2Matrix(tmp.yaw);
        for (size_t i = 0; i < pyramid_maps_ptr_->Size(); i++) {
            int score = PyramidMapMatchScore(i, query_map, Rbw, tbw);
            std::cout << i << "th pyramid match score: " << score << std::endl;
        }
    }

    timecount.End("candidate search");
    std::cout << "best Rbw: " << best_Rbw << std::endl;
    std::cout << "best tbw: " << best_tbw.transpose() << std::endl;

    Eigen::Matrix2d true_Rbw;
    Eigen::Vector2d true_tbw;
    true_Rbw << Rbw(0, 0), Rbw(0, 2), Rbw(2, 0), Rbw(2, 2);
    true_tbw << tbw(0), tbw(2);
    Eigen::Vector2d e_tbw = best_tbw - true_tbw;
    Eigen::Matrix2d e_Rbw = true_Rbw.transpose() * best_Rbw;
    std::cout << "---error tbw: " << e_tbw.transpose() << std::endl;
    std::cout << "===error Rbw: " << e_Rbw << std::endl;
    errors_.push_back(e_tbw.norm());
    if (errors_.size() > 342) {
        std::sort(errors_.begin(), errors_.end(), std::less<double>());
        int inlier = 0;
        for (size_t i = 0; i < errors_.size(); i++) {
            if (errors_[i] > step) {
                break;
            }
            inlier++;
        }
        std::cout << "inlier ratio: " << inlier * 1.0 / errors_.size() << std::endl;
        std::cout << "median error: " << errors_[errors_.size() * 0.5] << std::endl;
    }
    std::cout << "error size: " << errors_.size() << std::endl;

    cv::Mat img0 = pyramid_maps_ptr_->HeightMap().ShowMat();
    cv::imshow("max height map", img0);
    for (int i = 0; i < pyramid_maps_ptr_->Size(); i++) {
        cv::Mat img = pyramid_maps_ptr_->PyramidMapIndexing(i).ShowMat();
        std::string name = std::to_string(i) + "th pyramid map";
        cv::imshow(name, img);
    }
    cv::Mat frame = query_maps[query_idx].ShowMat();
    cv::imshow("current frame", frame);
}

MaxHeightMap BranchAndBound::VoxelGrid2MaxHeightMap(VoxelGrid &grid, Eigen::Matrix3d &Rbw, Eigen::Vector3d &tbw) {
    MaxHeightMap map(pyramid_maps_ptr_->HeightMap().GridSize(), pyramid_maps_ptr_->HeightMap().MaxHeight(),
                     pyramid_maps_ptr_->HeightMap().HeightResolution());
    map.AddGrid(grid, Rbw, tbw);
    return std::move(map);
}

std::vector<MaxHeightMap> BranchAndBound::VoxelGrid2MaxHeightMap(VoxelGrid &grid, Eigen::Matrix3d &Rbw, Eigen::Vector3d &tbw, double ratio, double level) {
    std::vector<MaxHeightMap> maps;
    double grid_size = pyramid_maps_ptr_->HeightMap().GridSize();
    double max_height = pyramid_maps_ptr_->HeightMap().MaxHeight();
    double height_reso = pyramid_maps_ptr_->HeightMap().HeightResolution();

    // build pyramid height map
    MaxHeightMap map0(grid_size, max_height, height_reso);
    map0.AddGrid(grid, Rbw, tbw);
    grid_size *= ratio;
    maps.push_back(std::move(map0));
    for (int i = 1; i < level; i++) {
        MaxHeightMap map(grid_size, max_height, height_reso);        
        grid_size *= ratio;
        maps.push_back(std::move(map));        
    }

    MaxHeightMap::MapType::iterator iter = maps[0].Begin();
    for (; iter != maps[0].End(); iter++) {
        const Eigen::Vector2d pt = iter->second.center_;
        double h = iter->second.MaxHeight() * height_reso;
        for (size_t i = 1; i < maps.size(); i++) {
            maps[i].AddPoint(pt(0), pt(1), h);
        }
    }

    return std::move(maps);
}


int BranchAndBound::MaxHeightMapMatchScore(MaxHeightMap &query_map, const Eigen::Matrix2d &Rbw, const Eigen::Vector2d &tbw) {
    MaxHeightMap::MapType::iterator map_iter = query_map.Begin();

    double height_resolution = query_map.HeightResolution();
    int matched_count = 0;
    for (; map_iter != query_map.End(); map_iter++) {
        const Eigen::Vector2d pt = Rbw * map_iter->second.center_ + tbw;
        if (pyramid_maps_ptr_->HeightMap().IsMatched(pt(0), pt(1), map_iter->second.MaxHeight() * height_resolution)) {
            matched_count++;
        }
    }
    return matched_count;
}

int BranchAndBound::PyramidMapMatchScore(int map_idx, MaxHeightMap &query_map, const Eigen::Matrix2d &Rbw, const Eigen::Vector2d &tbw) {
    MaxHeightMap::MapType::iterator map_iter = query_map.Begin();

    double height_resolution = query_map.HeightResolution();
    int matched_count = 0;
    PyramidMap &index_map = pyramid_maps_ptr_->PyramidMapIndexing(map_idx);

    for (; map_iter != query_map.End(); map_iter++) {
        const Eigen::Vector2d pt = Rbw * map_iter->second.center_ + tbw;
        if (index_map.IsMatched(pt(0), pt(1), map_iter->second.MaxHeight() * height_resolution)) {
            matched_count++;
        }
    }
    return matched_count;
}

void BranchAndBound::BranchAndBound2DMap(MaxHeightMap &query_map, Eigen::Matrix2d &Rbw, Eigen::Vector2d &tbw) {
    std::vector<double> query_boundary = MaxHeightMapBoundCalc(query_map);
    Eigen::Vector2d pt0, pt1;
    pt0 << query_boundary[1], query_boundary[3];
    pt1 << query_boundary[0], query_boundary[2];

    int first_level = pyramid_maps_ptr_->Size() - 1;
    std::vector<double> bound = pyramid_maps_ptr_->Boundary();
    std::cout << "pt0: " << pt0.transpose() << std::endl;
    std::cout << "pt1: " << pt1.transpose() << std::endl;
    double step = pyramid_maps_ptr_->PyramidMapIndexing(first_level).GridSize();
    bound[0] -= 1.5 * step;
    bound[1] += 1.5 * step;
    bound[2] -= 1.5 * step;
    bound[2] += 1.5 * step;
    std::cout << "***bound: " << bound[0] << ";" << bound[1] << ";" << bound[2] << ";" << bound[3] << "; step: " << step << std::endl;

    std::vector<PoseAndScore> candidates;
    for (double x = bound[0]; x < bound[1]; x += step) {
        for (double y = bound[2]; y < bound[3]; y += step) {
            for (double yaw = 0; yaw < 2 * M_PI; yaw += yaw_resolution_ * 2) {
                Eigen::Vector2d tmp_tbw(x, y);
                Eigen::Matrix2d tmp_Rbw = utils::DMmath::Yaw2Matrix(yaw);
                Eigen::Vector2d ptb0 = tmp_Rbw * pt0 + tmp_tbw;
                Eigen::Vector2d ptb1 = tmp_Rbw * pt1 + tmp_tbw;
                if (!IsInsideMap(ptb0, bound) || !IsInsideMap(ptb1, bound)) continue;
                int score = PyramidMapMatchScore(first_level, query_map, tmp_Rbw, tmp_tbw);
                PoseAndScore cand(score, x, y, yaw);
                cand.level = first_level;
                cand.step = step;
                cand.angle_step = yaw_resolution_;
                candidates.push_back(std::move(cand));
            }
        }
    }

    std::sort(candidates.begin(), candidates.end(), std::less<PoseAndScore>());
    for (size_t i = 0; i < candidates.size() && i < 10; i++) {
        const PoseAndScore &tmp = candidates[candidates.size() - 1 - i]; 
        std::cout << tmp.score << "th pose and score: " << tmp.x << ", " << tmp.y << ", " << tmp.yaw << std::endl;
    }

    std::cout << "candidates size: " << candidates.size() << std::endl;

    // branch and bound for best candidate score
    PoseAndScore best_candid(0, 0, 0, 0);
    while (!candidates.empty()) {
        PoseAndScore tmp_pose = candidates.back();
        candidates.pop_back();

        if (tmp_pose.score > best_candid.score) {
            if (tmp_pose.level == -1) {
                best_candid = tmp_pose;
            } else {
                double dx = tmp_pose.step * 0.5;
                std::vector<double> xs = {dx, 0, -dx}, yaws;
                for (int i = 0; i < 3; i++) {
                    yaws.push_back(tmp_pose.yaw + i * yaw_resolution_);
                    yaws.push_back(tmp_pose.yaw - i * yaw_resolution_);
                }

                // calculate new candidates for next level of pyramid map
                int level = tmp_pose.level - 1;
                std::vector<PoseAndScore> next_candids;
                for (size_t i = 0; i < xs.size(); i++) {
                    for (size_t j = 0; j < xs.size(); j++) {
                        for (size_t k = 0; k < yaws.size(); k++) {
                            Eigen::Vector2d tmp_tbw(xs[i] + tmp_pose.x, xs[j] + tmp_pose.y);
                            Eigen::Matrix2d tmp_Rbw = utils::DMmath::Yaw2Matrix(yaws[k]);
                            int score;
                            if (level == -1) {
                                score = MaxHeightMapMatchScore(query_map, tmp_Rbw, tmp_tbw);
                            } else {
                                score = PyramidMapMatchScore(level, query_map, tmp_Rbw, tmp_tbw);
                            }
                            PoseAndScore tmp(score, tmp_tbw(0), tmp_tbw(1), yaws[k]);
                            tmp.level = level;
                            tmp.step = dx;
                            tmp.angle_step = yaw_resolution_;
                            next_candids.push_back(std::move(tmp));
                        }
                    }
                }
                std::sort(next_candids.begin(), next_candids.end(), std::less<PoseAndScore>());
                candidates.insert(candidates.end(), next_candids.begin(), next_candids.end());
            }
        }
    }
    std::cout << best_candid.score << "th best pose: " << best_candid.x << "; " << best_candid.y << "; " << best_candid.yaw 
              << "; step: " << best_candid.step << std::endl;    
}

// bound [min_x, max_x, min_y, max_y]
std::vector<double> BranchAndBound::MaxHeightMapBoundCalc(MaxHeightMap &query_map) {
    MaxHeightMap::MapType::iterator iter = query_map.Begin();

    double x_min = 10000;
    double x_max = -10000;
    double y_min = 10000;
    double y_max = -10000;
    for (; iter != query_map.End(); iter++) {
        const Eigen::Vector2d &pt = iter->second.center_;
        if (pt(0) < x_min) x_min = pt(0);
        if (pt(0) > x_max) x_max = pt(0);
        if (pt(1) < y_min) y_min = pt(1);
        if (pt(1) > y_max) y_max = pt(1);
    }
    return (std::vector<double>{x_min, x_max, y_min, y_max});
}

bool BranchAndBound::IsInsideMap(const Eigen::Vector2d &pt, const std::vector<double> &bound) {
    if (pt(0) < bound[0] || pt(0) > bound[1]) return false;
    if (pt(1) < bound[2] || pt(1) > bound[3]) return false;
    return true;
}
} // namespace slam