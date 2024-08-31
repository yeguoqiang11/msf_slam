#include <eigen3/Eigen/Core>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

class EdgeFactor: public gtsam::NoiseModelFactor1<gtsam::Pose3> {
  public:
    EdgeFactor(const gtsam::SharedNoiseModel& model):
        gtsam::NoiseModelFactor1<gtsam::Pose3>(model, 1) {}
  private:
    Eigen::Vector3d direction;
};