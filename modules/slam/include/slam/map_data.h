// map data with basic operation.
//
// 地图包含两部分信息。
// 一部分是pose graph，包含节点的ID、位姿、相对位姿、协方差等信息。
// 加载出来后可以进行相关的查询操作，例如地图数量、ID等。
// 第一部分信息加载完成后，可以进行pose graph优化操作。
// 第二部分是Node内含的信息，和具体的实现有关。
// 对于VSLAM，它可以包含特征点、描述子、深度等信息；
// 对于TOFSLAM，它可以包含voxel，深度图等信息；
// 对于LIDAR SLAM，它可以包含scan等信息。
// 参考了RTabMap(https://github.com/introlab/rtabmap)的实现。
//
// 将数据分为两部分的好处是：
// 1. 提高第一部分加载速度，这样开机后可以马上查询地图信息；
// 2. 第二部分信息可以按需加载，例如在有先验的重定位时，可以只加载先验范围内的点云数据，减少计算量；
// 3. 可以实现大规模地图；
//
// Copyright 2021 Yeguoqiang Co.Ltd. All rights reserved.

#ifndef MODULES_SLAM_INCLUDE_SLAM_MAP_DATA_H_
#define MODULES_SLAM_INCLUDE_SLAM_MAP_DATA_H_

#include <map>
#include <vector>

namespace dmslam {

class MapElement {
 public:
  virtual void SerialToFile(const std::string& folder_path, const std::string& file_name)  = 0;
  virtual void ParseFromFile(const std::string& folder_path, const std::string& file_name) = 0;
};

class MapData {
 public:
  struct Brief {
    int id;
    int mem_size;
    int maturity;
  };
  /*
   * 从磁盘中读取地图的摘要。
   */
  virtual std::map<int, Brief> Summary() = 0;

  /*
   * 加载指定地图的pose graph信息。
   */
  virtual void LoadStructure(const std::vector<int>& trajectories) = 0;
  virtual void Save() const                                        = 0;
  virtual void Delete(int trajectory_id)                           = 0;

 protected:
};
}  // namespace dmslam

#endif  // MODULES_SLAM_INCLUDE_SLAM_MAP_DATA_H_
