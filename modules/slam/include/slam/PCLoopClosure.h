/**
 * @file PCLoopClosure.h
 * @author Guoqiang Ye (yegq@Yeguoqiang.tech)
 * @brief 
 * @version 0.1
 * @date 2022-01-18
 * 
 * @copyright Copyright (c) 2022 Yeguoqiang Technology Co.Ltd. All rights reserved
 * 
 */
#ifndef SRC_SLAM_PCLOOPCLOSURE_H
#define SRC_SLAM_PCLOOPCLOSURE_H
#include <eigen3/Eigen/Core>

#include "slam/ContourMap.h"
#include "slam/Map.h"
#include "slam/VoxelMap.h"

namespace slam {
class VoxelMapLoopClosure {
  public:
    VoxelMapLoopClosure(std::shared_ptr<GlobalVoxelMap> map_ptr);
    void InsertFrame(std::shared_ptr<Frame> frame_ptr);
    void CalculateCentroidAndPCA(std::shared_ptr<Frame> frame_ptr);
  private:
    std::shared_ptr<GlobalVoxelMap> map_ptr_;
    double min_depth_, max_depth_;
    double bin_size_;
    std::shared_ptr<ContourMapStack> pyramid_maps_ptr_;
    std::shared_ptr<BranchAndBound> indexing_ptr_;

};

} // namespace slam

#endif // SRC_SLAM_PCLOOPCLOSURE_H