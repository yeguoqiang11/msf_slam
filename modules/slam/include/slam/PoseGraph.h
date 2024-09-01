/**
 * @file PoseGraph.h
 * @author Guoqiang Ye (yegq@Yeguoqiang.tech)
 * @brief 
 * @version 0.1
 * @date 2022-06-18
 * 
 * @copyright Copyright (c) 2022 Yeguoqiang Technology Co.Ltd. All rights reserved
 * 
 */
#ifndef SLAM_POSEGRAPH_H
#define SLAM_POSEGRAPH_H
#include <boost/program_options.hpp>

#include <eigen3/Eigen/Core>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Key.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include "utils/DMmath.h"
#include "utils/logs.h"
#include "slam/Map.h"
#include "slam/VoxelMap.h"

using gtsam::symbol_shorthand::B; // bias = (ba_x, ba_y, ba_z, bg_x, bg_y, bg_z)
using gtsam::symbol_shorthand::V; // velocity = (vx, vy, vz)
using gtsam::symbol_shorthand::X; // pose3d (t, R)
namespace slam {
extern std::vector<Eigen::Vector3d> frames_ptis_;
extern std::vector<Eigen::Vector3d> frames_ptjs_;
struct FramesTransformSolver {
    FramesTransformSolver();
    bool FramesTransformCal(std::shared_ptr<Frame> framei_ptr, std::shared_ptr<Frame> framej_ptr, utils::Transform3d &dTji);

    int FramesVoxelCost(std::shared_ptr<Frame> framei_ptr, std::shared_ptr<Frame> framej_ptr, const utils::Transform3d &dTji,
                        Eigen::Matrix<double, 6, 6> &H, Eigen::Matrix<double, 6, 1> &b, double &cost, double info_mat);
    
    void FramesGroundConstraintCost(std::shared_ptr<Frame> framei_ptr, std::shared_ptr<Frame> framej_ptr, const utils::Transform3d &dTji,
                                    Eigen::Matrix<double, 6, 6> &H, Eigen::Matrix<double, 6, 1> &b, double &cost, double info_mat);
    
    double voxel_cost_huber_thres_;
    double outlier_thres;
};

class PoseGraph {
  struct LoopOptions {
        int last_loop_idx = 0; // last-time loop frame
        int min_frame_dis_from_lastloop = 3; // 10 frames distance from last loop frame;
        int skip_num = 3; // frame of redetection after being detection
        double angle_thres = 5. * DEG2RAD; // rad
        double dtz_thres = 2.5; // moving direction meters
        double dtxy_thres = 0.5; // 
  };
  public:
    PoseGraph(std::shared_ptr<GlobalVoxelMap> map_ptr, std::shared_ptr<Sensors> sensors_ptr);
    void SetGroundPtr(std::shared_ptr<GroundPlanes> grounds_ptr);

    void LoadFrame(std::shared_ptr<Frame> frame_ptr);

    void ImuParamInitialization();

    void AddInitialValueAndFirstPriorConstraint(uint32_t frame_id, const utils::Transform3d &Tiw, const Eigen::Vector3d &Vw,
                                                const Eigen::Vector3d &ba, const Eigen::Vector3d &bg);

    void AddOdomConstraint(const utils::Transform3d &Tiw, uint32_t frame_id);

    void AddGyroWheelConstraint(uint32_t frame1_id);

    void AddImuConstraint(uint32_t frame1_id, const Eigen::Vector3d &ba, const Eigen::Vector3d &bg);

    void AddLoopConstraint(std::shared_ptr<Frame> framej_ptr, uint32_t framej_id);

    gtsam::ISAM2Result Optimization(std::shared_ptr<Frame> frame_ptr);

    void StateUpdate(std::shared_ptr<Frame> frame_ptr, gtsam::ISAM2Result &result);

    std::vector<int> LoopCandidateDetection(std::shared_ptr<Frame> frame_ptr);

    std::vector<std::pair<int, utils::Transform3d>> LoopClosure(std::shared_ptr<Frame> frame_ptr);

    bool LoopTransformCal(int idx, std::shared_ptr<Frame> framej_ptr, utils::Transform3d &Tji);

  private:
    std::shared_ptr<GlobalVoxelMap> map_ptr_;
    std::shared_ptr<Sensors> sensors_ptr_;
    std::shared_ptr<GroundPlanes> grounds_ptr_;
    boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> imu_param_ptr_;
    gtsam::NonlinearFactorGraph *graph_;
    gtsam::ISAM2* isam2_;
    gtsam::Values initial_values_;

    bool is_initialized_;
    double timestamp_;

    LoopOptions loop_options_;

    // frames transform solver
    std::shared_ptr<FramesTransformSolver> frames_solver_ptr_;
};

class GyroDiffWheelFactor: public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3> {
  public:
    GyroDiffWheelFactor(std::shared_ptr<GyroWheelModelPreintegration> gyrwheel_ptr, std::shared_ptr<Sensors> sensors_ptr,
                       const gtsam::SharedNoiseModel& noise_model, gtsam::Key key0, gtsam::Key key1);
    gtsam::Vector evaluateError(const gtsam::Pose3 &pose0, const gtsam::Pose3 &pose1, boost::optional<gtsam::Matrix&> H0 = boost::none,
                                boost::optional<gtsam::Matrix&> H1 = boost::none) const;
  private:
    std::shared_ptr<GyroWheelModelPreintegration> gyrwheel_ptr_;
    std::shared_ptr<Sensors> sensors_ptr_;
};
} // namespace slam
#endif // SLAM_POSEGRAPH_H