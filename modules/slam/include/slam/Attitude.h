/**
 * @file Attitude.h
 * @author Guoqiang Ye (yegq@dreame.tech)
 * @brief 
 * @version 0.1
 * @date 2022-01-18
 * 
 * @copyright Copyright (c) 2022 Dreame Technology Co.Ltd. All rights reserved
 * 
 */
#ifndef MODULES_SLAM_ATTITUDE_H
#define MODULES_SLAM_ATTITUDE_H
#include <iostream>
#include <vector>

#include <eigen3/Eigen/Core>
#include <Eigen/Geometry>

#include "slam/Sensors.h"
#include "utils/DMmath.h"
#include "utils/logs.h"

namespace slam {
/**
 * @brief base class for attitude calculation
 * navigation coordinate is based on gravity definition
 */
class AttitudeBase {
  public:
    /**
     * @brief Construct a new Attitude Base object
     * 
     * @param sensor_ptr extrinsic from imu coordinate to body coordinate
     * @param Gravity gravity vector in navigation coordinate
     */
    AttitudeBase(std::shared_ptr<Sensors> sensor_ptr, Eigen::Vector3d Gravity = Eigen::Vector3d(0, -9.8, 0)) {
        Rib_ = sensor_ptr->imu.Rib;
        Gravity_ = Gravity;
    }

    /**
     * @brief calculate attitude from imu groups
     * 
     * @param imus  vector of [time, accx, accy, accz, gyrox, gyroy, gyroz]
     * @return Eigen::Vector2d [pitch; roll]
     */
    virtual Eigen::Vector2d LoadImus(std::vector<Eigen::Matrix<double, 1, 7>> imus) = 0;
    
    /**
     * @brief calculate attitude from imu
     * 
     * @param imu [time, accx, accy, accz, gyrox, gyroy, gyroz]
     * @return Eigen::Vector2d [pitch; roll]
     */
    virtual Eigen::Vector2d LoadImu(Eigen::Matrix<double, 1, 7> imu) = 0;
  protected:
    /**
     * @brief extrinsic from imu coordinate to body coordinate
     * 
     */
    Eigen::Matrix3d Rib_;
    Eigen::Vector3d Gravity_;
};

class MEKFAttitude: public AttitudeBase {
  public:
    // @param gravity - gravity vector in navigation coordinate
    MEKFAttitude(std::shared_ptr<Sensors> sensor_ptr, Eigen::Vector3d gravity = Eigen::Vector3d(0., 0., 9.8));

    bool IsStatic(Eigen::Matrix<double, 1, 7> imu);

    bool Initialization(Eigen::Matrix<double, 1, 7> imu);

    /**
     * @brief  estimate attitude from imu groups
     * 
     * @param imus vector of imu = [timestamp, ax, ay, az, gx, gy, gz]
     * @return Eigen::Vector2d attitude = [roll, pitch]
     */
    virtual Eigen::Vector2d LoadImus(std::vector<Eigen::Matrix<double, 1, 7>> imus);

    virtual Eigen::Vector2d LoadImu(Eigen::Matrix<double, 1, 7> imu);

    /**
     * @brief EKF estimation from imu = [timestamp, ax, ay, az, gx, gy, gz]
     * 
     * @param imu imu data = [timetamp, ax, ay, az, gx, gy, gz]
     */
    void InsertImu(Eigen::Matrix<double, 1, 7> imu);

    Eigen::Matrix3d EstimateRotationFromAcc(Eigen::Vector3d acc);

    /**
     * @brief output Euler angle from rotation matrix
     * 
     * @return Eigen::Vector3d euler angle = [roll, pitch, yaw]
     */
    Eigen::Vector3d EulerAngle() const;

    Eigen::Matrix3d Rbw() { return Rib_.transpose() * Rin_ * Rib_; }

    bool IsInitialized() { return is_initialized_; }

    // gravity vector in robot navigation coordinate
    std::shared_ptr<Eigen::Vector3d> GravitySharedPtr() { return Gw_ptr_; }
  private:
    Eigen::Vector3d bg_, ba_;
    Eigen::Matrix3d Rin_; // rotation from imu to navigation coordinate

    // gravity vector in robot navigation coordinate
    std::shared_ptr<Eigen::Vector3d> Gw_ptr_;

    // initial things
    bool is_initialized_;
    int init_num_;
    Eigen::Vector3d init_gyro_mean_, init_acc_mean_;
    Eigen::Matrix3d init_gyro_cov_, init_acc_cov_;

    Eigen::Matrix<double, 1, 7> last_imu_;

    // EKF paramters
    Eigen::Matrix3d Q, R, P;
};
} // namespace slam
#endif // MUODUES_SLAM_ATTITUDE_H