// pose graph interface
// Copyright 2021 Yeguoqiang Co.Ltd. All rights reserved.
#ifndef MODULES_SLAM_INCLUDE_SLAM_GRAPH_H_
#define MODULES_SLAM_INCLUDE_SLAM_GRAPH_H_

#include <memory>
#include <array>
#include "sophus/se3.hpp"

namespace dmslam {

using QuatTrans = std::array<double, 7>;

struct NodeInfo {
  int merge_to;
  QuatTrans pose;
};

class Node {
 public:
  using Ptr = std::shared_ptr<Node>;
  using WeakPtr = std::weak_ptr<Node>;

  int id() const { return id_; }
  int trajectory_id() const { return trajectory_id_; }
  Sophus::SE3d pose() { return pose_; }

 protected:
  static int cnt_;
  int id_;
  int trajectory_id_;
  Sophus::SE3d pose_;
};

class Edge {
 public:
  using Ptr = std::shared_ptr<Edge>;

 private:
};

class Graph {
 public:
 private:
};

}  // namespace dmslam
#endif  // MODULES_SLAM_INCLUDE_SLAM_GRAPH_H_
