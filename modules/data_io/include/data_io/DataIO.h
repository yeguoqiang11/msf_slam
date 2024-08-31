#ifndef SRC_DATAIO_H
#define SRC_DATAIO_H
#include <algorithm>
#include <dirent.h>
#include <fstream>
#include <iostream>
#include <string>
#include <sstream>
#include <unistd.h>

#include <eigen3/Eigen/Core>
#include <opencv2/opencv.hpp>

namespace DataIO {
namespace DataSetType {
void GetFilesFromFolder(std::string folder, std::vector<std::string> &files, std::string format = "");
void GetAllVectorsFromFile(std::string file, std::vector<std::vector<double>> &vecs_set, char separator = ' ');

// tum data reader
class TumData {
  public:
    TumData(const std::string& tum_folder);

    bool GetData(cv::Mat &depth_image, cv::Mat &image, double &img_time, double &depth_time,
                 Eigen::Vector3d &acc, double &acc_time);

    bool GetPointFromImage(const cv::Mat &depth_image, const cv::Mat &image,
                           std::vector<Eigen::Vector3d> &points,
                           std::vector<Eigen::Vector3d> &colors);

    Eigen::Matrix3d Intrinsic() const;
    double fx() const { return fx_; }
    double fy() const { return fy_; }
    double cx() const { return cx_; }
    double cy() const { return cy_; }

    cv::Mat DepthImage(){ return depth_image_; }
  private:
    std::string folder_path_;
    std::string image_lists_path_;
    std::string depth_lists_path_;
    std::string acc_file_;
    std::ifstream depth_stream_, image_stream_, acc_stream_;
    double fx_, fy_, cx_, cy_, factor_;
    std::vector<Eigen::Vector3d> point_color_;
    int image_count_, depth_count_;
    std::string root_direct_;
    cv::Mat depth_image_;
};

// simulation data reader
class SimulationData {
  public:
    SimulationData(std::string dir);
    bool GetDepth(cv::Mat &depth, cv::Mat &show_img, double &timestamp);
    bool GetDepths(std::vector<cv::Mat> &depths, std::vector<cv::Mat> &shows, std::vector<double> &timestamps);
    bool GetPoints(cv::Mat &depth, std::vector<Eigen::Vector3d> &points, std::vector<Eigen::Vector3d> &colors) const;
    bool GetGTPose(const double &time, Eigen::Matrix<double, 8, 1> &pose);
    bool GetImus(std::vector<Eigen::Matrix<double, 1, 7>> &imus, double t0, double t1);
  private:
    double fx_, fy_, cx_, cy_;
    std::vector<std::string> depth_set_, depth_back_set_;
    std::vector<double> depth_time_set_, depth_back_time_set_;
    std::vector<Eigen::Matrix<double, 8, 1>> true_pose_set_;
    std::vector<Eigen::Matrix<double, 1, 7>> imu_set_;
    int count_;
    Eigen::Matrix3d Rcb0_, Rcb1_;
    Eigen::Vector3d tcb0_, tcb1_;
};

// orbtech data reader
class OrbbecData {
  public:
    OrbbecData(std::string dir);
    bool GetDepths(std::vector<cv::Mat> &depths, std::vector<cv::Mat> &shows, std::vector<double> &timestamps);
    void GetPoinClouds(const cv::Mat &depth, std::vector<Eigen::Vector3d> &points, std::vector<Eigen::Vector3d> &colors, bool is_front = true);

  private:
    double fx_, fy_, cx_, cy_;
    std::vector<std::string> depth_vec_, depth_back_vec_;
    std::vector<double> depth_time_vec_, depth_back_time_vec_;
    int count_;
};

class OrbMachineData {
// [timestamp, gx, gy, gz, ax, ay, az, wheel_left, wheel_right, optic_flow_x, optic_flow_y, yaw]
typedef Eigen::Matrix<double, 1, 12> SensorsDataType;
  public:
    OrbMachineData(const std::string &folder, bool is_IR = false);
    bool GetImages(std::vector<cv::Mat> &images, double &timestamp);
    void GetPointCloud(const cv::Mat &image, std::vector<Eigen::Vector3d> &pcd);
    /**
     * @brief Get the Imus between t0 and t1
     * 
     * @param imus vector of imu, imu: [timestamp, gx, gy, gz, ax, ay, az]
     * @param t0 begin timestamp
     * @param t1 end timestamp
     * @return true got successully
     * @return false fail to get
     */
    bool GetImus(std::vector<Eigen::Matrix<double, 1, 7>> &imus, double t0, double t1);
    bool GetWheels(std::vector<Eigen::Matrix<double, 1, 3>> &wheels, double t0, double t1);
    bool GetInitGyroBias(Eigen::Vector3d &gyro);
    inline Eigen::Vector3d TransformLeftDepth2Point(const int &w, const int &h, const double &d);
    inline Eigen::Vector3d TransformRightDepth2Point(const int &w, const int &h, const double &d);
    double Scale() { return scale0_; }
  private:
    std::vector<std::string> front_depths_, rear_depths_;
    std::vector<SensorsDataType> sensors_data_;
    double fx0_, fy0_, cx0_, cy0_;
    double fx0_inv_, fy0_inv_;
    double fx1_, fy1_, cx1_, cy1_;
    double fx1_inv_, fy1_inv_;
    double scale0_, scale1_;
    unsigned long out_idx_;
    int width_, height_;
};

class SunnyMachine {
  public:
    SunnyMachine(std::string folder_path);
    bool GetPointCloud(std::vector<std::vector<Eigen::Vector3d>> &pts, double &timestamp);
    bool GetOdomData(std::vector<Eigen::Matrix<double, 1, 9>> &odoms, double t0, double t1);
    bool GetImuAndWheelData(std::vector<Eigen::Matrix<double, 1, 7>> &imus, std::vector<Eigen::Matrix<double, 1, 3>> &wheels,
                            double t0, double t1);
  private:
    std::vector<std::string> pc_files_;
    std::vector<std::vector<double>> odom_vecs_;
    int count_;
    int odom_count_;
};

class OfilmMachine {
  public:
    OfilmMachine(std::string folder_path);
    bool GetPointCloud(std::vector<std::vector<Eigen::Vector3d>> &pts, double &timestamp);
    bool GetDepthImage(std::vector<cv::Mat> &depths, double &timestamp);
    bool ReadPly(const std::string &file, std::vector<Eigen::Vector3d> &pts);
    bool GetImuAndWheelData(std::vector<Eigen::Matrix<double, 1, 7>> &imus, std::vector<Eigen::Matrix<double, 1, 3>> &wheels,
                            double t0, double t1);
  private:
    std::vector<std::string> pc_files_, depth_files_;
    std::vector<std::vector<double>> odom_vecs_;
    int count_;
    int depth_count_;
    int odom_count_;
    int depth_width_, depth_height_;
};

void GenerateListsTxt(std::string dir);

cv::Mat ReadOrbRawDataImage(std::string image_file, int raw_data_height, int raw_data_width);

void ReadPlyPoints(std::string file, std::vector<Eigen::Vector3d> &pts, std::vector<Eigen::Vector3d> &colors);

} // namespace DataSetType
} // namespace slam
#endif // SRC_DATAIO_H
