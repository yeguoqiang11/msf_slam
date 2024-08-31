// sensor data interface.
// Copyright 2021 Dreame Co.Ltd. All rights reserved.
#ifndef MODULES_SLAM_INCLUDE_SLAM_SENSOR_DATA_H_
#define MODULES_SLAM_INCLUDE_SLAM_SENSOR_DATA_H_

#include <memory>
namespace dmslam {
class SensorData {
 public:
  using Ptr = std::shared_ptr<SensorData>;
  using ConstPtr = std::shared_ptr<const SensorData>;

  int64_t timestamp() const { return timestamp_; }

 protected:
  int64_t timestamp_;
};

}  // namespace dmslam
#endif
