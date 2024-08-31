#include "data_io/DataIO.h"
#include "utils/json.hpp"

namespace DataIO {
namespace DataSetType {

bool FileNameCompare(std::string &name0, std::string &name1) {
    if (name0.size() == name1.size()) {
        return name0 < name1;
    } else {
        return name0.size() < name1.size();
    }
}

void GetAllFiles(std::string folder, std::vector<std::string> &files) {
    DIR* dir;
    if (access(folder.c_str(), 0)) {
        std::cout << "folder does not exist with name: " << folder << std::endl;
        exit(0);
    }
    dir = opendir(folder.c_str());
    if (readdir(dir) == NULL) {
        std::cout << "cannot find any file at: " << folder << std::endl;
    }
    struct dirent* dir_ptr;
    while((dir_ptr = readdir(dir)) != NULL) {
        if (dir_ptr->d_name[0] == '.') continue;
        files.push_back(dir_ptr->d_name);
    }
    std::sort(files.begin(), files.end(), FileNameCompare);
}

void GetFilesFromFolder(std::string folder, std::vector<std::string> &files, std::string format) {
    size_t s = format.size();
    DIR* dir;
    if (access(folder.c_str(), 0)) {
        std::cout << "folder does not exist with name:" << folder << std::endl;
        exit(0);
    }
    dir = opendir(folder.c_str());
    if (readdir(dir) == NULL) {
        std::cout << "cannot find any file at: " << folder << std::endl;
        return;
    }
    struct dirent* dir_ptr;
    while((dir_ptr = readdir(dir)) != NULL) {
        if (dir_ptr->d_name[0] == '.') continue;
        std::string name = dir_ptr->d_name;
        if (name.size() <= s) continue;
        std::string d = name.substr(name.size() - s, s);
        if (!d.compare(format)) {
            files.push_back(dir_ptr->d_name);
        }
    }
    std::sort(files.begin(), files.end(), FileNameCompare);
}

void GetAllVectorsFromFile(std::string file, std::vector<std::vector<double>> &vecs_set, char separator) {
    std::fstream file_stream(file);
    std::vector<double> datas;
    std::string line, data_txt;

    while (!file_stream.eof()) {
        std::getline(file_stream, line);
        std::stringstream ss(line);
        datas.clear();
        int i = 0;
        while (std::getline(ss, data_txt, separator)) {
            double a = std::stod(data_txt);
            datas.push_back(a);
        }
        if (datas.empty()) continue;
        vecs_set.push_back(datas);
    }
}

// Tum data reading
TumData::TumData(const std::string& tum_folder) {
    acc_file_ = tum_folder + "/accelerometer.txt";
    depth_lists_path_ = tum_folder + "/depth.txt";
    image_lists_path_ = tum_folder + "/rgb.txt";

    acc_stream_.open(acc_file_);
    depth_stream_.open(depth_lists_path_);
    image_stream_.open(image_lists_path_);
    if (!acc_stream_.is_open() || !depth_stream_.is_open() || !image_stream_.is_open()) {
        std::cout << "Cannot Open: " << tum_folder << std::endl;
        return;
    }
    std::string header;
    std::getline(image_stream_, header);
    std::getline(image_stream_, header);
    std::getline(image_stream_, header);

    std::getline(depth_stream_, header);
    std::getline(depth_stream_, header);
    std::getline(depth_stream_, header);

    std::getline(acc_stream_, header);
    std::getline(acc_stream_, header);
    std::getline(acc_stream_, header);

    fx_ = 525.0;
    fy_ = 525.0;
    cx_ = 319.5;
    cy_ = 239.5;
    factor_ = 5000;

    image_count_ = 0;
    depth_count_ = 0;

    root_direct_ = tum_folder;
}

bool TumData::GetData(cv::Mat &depth_image, cv::Mat &image, double &img_time, double &depth_time,
                      Eigen::Vector3d &acc, double &acc_time) {
    if (image_stream_.eof() || depth_stream_.eof() || acc_stream_.eof()) {
        std::cout << "Cannot Get Tum Data!!!" << std::endl;
        return false;
    }
    std::string img_file, depth_file;
    image_stream_ >> img_time >> img_file;
    depth_stream_ >> depth_time >> depth_file;
    std::cout.precision(15);
    std::cout << "-----timestamp: " << depth_time << std::endl;


    img_file = root_direct_ + "/" + img_file;
    depth_file = root_direct_ + "/" + depth_file;
    image = cv::imread(img_file, cv::IMREAD_UNCHANGED);
    if (image.data == NULL) { 
        std::cout << "Cannot Get Tum Image Data!!!" << std::endl;
        return false;
    }

    depth_image = cv::imread(depth_file, cv::IMREAD_UNCHANGED);
    depth_image.convertTo(depth_image, CV_64F);
    depth_image /= 5000.0;

    cv::Size imsize = depth_image.size();
    depth_image_.create(imsize, CV_8U);
    for (int r = 0; r < imsize.height; r++) {
        for (int c = 0; c < imsize.width; c++) {
            depth_image_.at<uchar>(r, c) = static_cast<uchar>(std::min((depth_image.at<double>(r, c)), 7.0) * 255 / 7);
        }
    }
    
    if (depth_image.data == NULL) {
        std::cout << "Cannot Get Tum Depth Image Data!!!" << std::endl;
        return false;
    }

    acc_stream_ >> acc_time >> acc[0] >> acc[1] >> acc[2];
    return true;
}

bool TumData::GetPointFromImage(const cv::Mat &depth_image, const cv::Mat &image,
                                std::vector<Eigen::Vector3d> &points,
                                std::vector<Eigen::Vector3d> &colors) {
    points.clear();
    colors.clear();

    int height = depth_image.size().height;
    int width = depth_image.size().width;
    for (int r = 0; r < height; r++) {
        for (int c = 0; c < width; c++) {
            cv::Vec3b pixel = image.at<cv::Vec3b>(r, c);
            double depth = depth_image.at<double>(r, c);
            if (depth < 0.1 || depth > 4) continue;

            Eigen::Vector3d rgb(pixel[0] / 255.0, pixel[1] / 255.0, pixel[2] / 255.0);

            double y = (static_cast<double>(r) - cy_) * depth / fy_;
            double x = (static_cast<double>(c) - cx_) * depth / fx_;
            Eigen::Vector3d point(-x, -y, depth);
            points.push_back(point);
            colors.push_back(rgb);
        }
    }
    return true;
}

Eigen::Matrix3d TumData::Intrinsic() const {
    Eigen::Matrix3d out;
    out << fx_, 0, cx_,
           0, fy_, cy_,
           0, 0, 1;
    return out;
}

SimulationData::SimulationData(std::string dir) {
    std::string depth_lists_path = dir + "/depth_front_noise.txt";
    std::string depth_back_lists_file = dir + "/depth_back_noise.txt";
    std::string imu_file = dir + "/imu.txt";
    std::string tf_file = dir + "/odom.txt";
    std::ifstream depth_stream, depth_back_stream, pose_stream, imu_stream;
    depth_stream.open(depth_lists_path);
    depth_back_stream.open(depth_back_lists_file);
    pose_stream.open(tf_file);
    imu_stream.open(imu_file);

    if (!depth_stream.is_open()) {
        std::cout << "cannot open simulation data!!!" << std::endl;
        exit(0);
    }
    double depth_timestamp, depth_back_timestamp;
    std::string depth_file, depth_back_file;
    while(!depth_stream.eof() && depth_stream.is_open()) {
        depth_stream >> depth_timestamp >> depth_file;
        depth_file = dir + "/" + depth_file;
        depth_time_set_.push_back(depth_timestamp);
        depth_set_.push_back(depth_file);
    }
    while(!depth_back_stream.eof() && depth_back_stream.is_open()) {
        depth_back_stream >> depth_back_timestamp >> depth_back_file;
        depth_back_file = dir + "/" + depth_back_file;
        depth_back_time_set_.push_back(depth_back_timestamp);
        depth_back_set_.push_back(depth_back_file);
    }

    Eigen::Matrix<double, 8, 1> pose;
    while (!pose_stream.eof() && pose_stream.is_open()) {
        pose_stream >> pose(0) >> pose(1) >> pose(2) >> pose(3) >>
            pose(4) >> pose(5) >> pose(6) >> pose(7);
        // std::cout << "true pose: " << pose.transpose() << std::endl;
        true_pose_set_.push_back(pose);
    }
    Eigen::Matrix<double, 1, 7> imus;
    while (!imu_stream.eof()) {
        imu_stream >> imus(0) >> imus(1) >> imus(2) >> imus(3) >> imus(4)
            >> imus(5) >> imus(6);
        imu_set_.push_back(imus);
    }

    count_ = 0;
    fx_ = 312.195129;
    fy_ = 312.195129;
    cx_ = 320.;
    cy_ = 240.;

    tcb0_ << 0., 0.0765, 0.18;
    tcb1_ << 0., 0.0765, -0.18;
    Rcb0_.setIdentity();
    Rcb1_ << -1, 0, 0,
             0, 1, 0,
             0, 0, -1;
}

bool SimulationData::GetDepth(cv::Mat &depth, cv::Mat &show_img, double &timestamp) {
    if (count_ < depth_time_set_.size()) {
        depth = cv::imread(depth_set_[count_], cv::IMREAD_UNCHANGED);
        depth.convertTo(depth, CV_64F);
        depth /= 10000;
        timestamp = depth_time_set_[count_];
        show_img = depth * 255 / 3;
        show_img.convertTo(show_img, CV_8U);
        count_++;
        return true;
    }
    return false;
}

bool SimulationData::GetDepths(std::vector<cv::Mat> &depths, std::vector<cv::Mat> &shows, std::vector<double> &timestamps) {
    depths.clear();
    shows.clear();
    timestamps.clear();

    if (count_ < depth_time_set_.size()) {
        cv::Mat depth = cv::imread(depth_set_[count_], cv::IMREAD_UNCHANGED);
        depth.convertTo(depth, CV_64F);
        depth /= 10000;
        depths.push_back(depth);
        timestamps.push_back(depth_time_set_[count_]);
        cv::Mat show = depth * 255 / 3;
        show.convertTo(show, CV_8U);
        shows.push_back(show);
    }
    if (count_ < depth_back_time_set_.size()) {
        cv::Mat depth = cv::imread(depth_back_set_[count_], cv::IMREAD_UNCHANGED);
        depth.convertTo(depth, CV_64F);
        depth /= 10000;
        depths.push_back(depth);
        timestamps.push_back(depth_back_time_set_[count_]);
        cv::Mat show = depth * 255 / 3;
        show.convertTo(show, CV_8U);
        shows.push_back(show);
    }

    if (depths.empty()) return false;
    count_+= 8;
    return true;
}

bool SimulationData::GetPoints(cv::Mat &depths, std::vector<Eigen::Vector3d> &points, std::vector<Eigen::Vector3d> &colors) const {
    if (depths.empty()) return false;

    int width = depths.size().width;
    int height = depths.size().height;

    for (size_t r = 0; r < height; r++) {
        for (size_t c = 0; c < width; c++) {
            if (depths.at<double>(r, c) > 0) {
                double depth = depths.at<double>(r, c);
                double y = (r - cy_) * depth / fy_;
                double x = (c - cx_) * depth / fx_;
                points.push_back(Eigen::Vector3d(x, y, depth));
                colors.push_back(Eigen::Vector3d(1, 1, 0));
            }
        }
    }
    return true;
}

bool SimulationData::GetGTPose(const double &time, Eigen::Matrix<double, 8, 1> &pose) {
    for (size_t i = 0; i < true_pose_set_.size(); i++) {
        if (true_pose_set_[i](0) < time) {
            continue;
        } else {
            pose = true_pose_set_[i];
            return true;
        }
    }
    return false;
}

bool SimulationData::GetImus(std::vector<Eigen::Matrix<double, 1, 7>> &imus, double t0, double t1) {
    imus.clear();
    for (size_t i = 0; i < imu_set_.size(); i++) {
        if (imu_set_[i](0) >= t0 && imu_set_[i](0) <= t1) {
            imus.push_back(imu_set_[i]);
        }
        if (imu_set_[i](0) > t1) break;
    }
    return true;
}

//------------------------------------------orbbec technology---------------------------------------------
OrbbecData::OrbbecData(std::string dir)
  : count_(0) {
    std::string depth_lists_path = dir + "/tof/depth.txt";
    std::string depth_back_lists_file = dir + "/tof/depth_back.txt";
    std::string imu_file = dir + "/imu/imu.txt";
    std::ifstream depth_stream, depth_back_stream, imu_stream;
    depth_stream.open(depth_lists_path);
    depth_back_stream.open(depth_back_lists_file);
    imu_stream.open(imu_file);

    if (!depth_stream.is_open() || !depth_back_stream.is_open()) {
        std::cout<< "cannot open orbbec datasets!!!" << std::endl;
        exit(0);
    }

    double depth_timestamp, depth_back_timestamp;
    std::string depth_file, depth_back_file;
    while(!depth_stream.eof()) {
        depth_stream >> depth_timestamp >> depth_file;
        depth_file = dir + "/tof/" + depth_file;
        depth_time_vec_.push_back(depth_timestamp);
        depth_vec_.push_back(depth_file);
    }

    while(!depth_back_stream.eof()) {
        depth_back_stream >> depth_back_timestamp >> depth_back_file;
        depth_back_file = dir + "/tof/" + depth_back_file;
        depth_back_time_vec_.push_back(depth_back_timestamp);
        depth_back_vec_.push_back(depth_back_file);
    }
    fx_ = 514.7033;
    fy_ = 514.494;
    cx_ = 324.32614135742188;
    cy_ = 248.0206298828125;
}

bool OrbbecData::GetDepths(std::vector<cv::Mat> &depths, std::vector<cv::Mat> &shows, std::vector<double> &timestamps) {
    depths.clear();
    shows.clear();
    timestamps.clear();

    if (count_ < depth_time_vec_.size()) {
        cv::Mat depth = cv::imread(depth_vec_[count_], cv::IMREAD_UNCHANGED);
        depth.convertTo(depth, CV_64F);
        depth /= 10000;
        depths.push_back(depth);
        timestamps.push_back(depth_time_vec_[count_]);
        cv::Mat show = depth * 255 / 8;
        show.convertTo(show, CV_8U);
        shows.push_back(show);
    }
    if (count_ < depth_back_time_vec_.size()) {
        cv::Mat depth = cv::imread(depth_back_vec_[count_], cv::IMREAD_UNCHANGED);
        depth.convertTo(depth, CV_64F);
        depth /= 10000;
        depths.push_back(depth);
        timestamps.push_back(depth_back_time_vec_[count_]);
        cv::Mat show = depth * 255 / 8;
        show.convertTo(show, CV_8U);
        shows.push_back(show);
    }

    if (depths.empty()) return false;
    count_+= 1;
    return true;
}

void OrbbecData::GetPoinClouds(const cv::Mat &depth_img, std::vector<Eigen::Vector3d> &points, std::vector<Eigen::Vector3d> &colors, bool is_front) {
    // points.clear();
    // colors.clear();
    if (depth_img.empty()) {
        std::cout << "orbbec depth image is empty!!!" << std::endl;
        return;
    }

    int height = depth_img.size().height;
    int width = depth_img.size().width;
    for (int r = 0; r < height; r++) {
        for (int c = 0; c < width; c++) {
            double depth = depth_img.at<double>(r, c);
            if (depth > 0.1) {
                double x = (c - cx_) * depth / fx_;
                double y = (r - cy_) * depth / fy_;
                if (is_front) {
                    points.push_back(Eigen::Vector3d(-x, y, depth));
                    colors.push_back(Eigen::Vector3d(1, 0, 0));
                } else {
                    points.push_back(Eigen::Vector3d(x, y, -depth));
                    colors.push_back(Eigen::Vector3d(1, 0, 0));
                }
            }
        }
    }
}

OrbMachineData::OrbMachineData(const std::string &folder, bool is_IR) {
    std::string imu_file = folder + "/sensor_data/imu/data.txt";
    std::string tof_front_folder = folder + "/sensor_data/tof/depth/";
    // std::string tof_rear_folder = folder + "/tof_rear";

    std::vector<std::string> depths_front, depths_rear;
    GetAllFiles(tof_front_folder, depths_front);
    // GetAllFiles(tof_rear_folder, depths_rear);

    if (depths_front.empty()){// || depths_rear.empty()) {
        std::cout << "no front depth image from " << tof_front_folder << std::endl;
        // std::cout << "no rear depth image from " << tof_rear_folder << std::endl;
        exit(0);
    }

    // while (depths_front[0] < depths_rear[0])
    //     depths_front.erase(depths_front.begin());
    // while (depths_rear[0] < depths_front[0])
    //     depths_rear.erase(depths_rear.begin());

    for (size_t i = 0; i < depths_front.size(); i++) {
        std::string front_depth = tof_front_folder + "/" + depths_front[i];
        // std::string rear_depth = tof_rear_folder + "/" + depths_rear[i];
        front_depths_.push_back(front_depth);
        // rear_depths_.push_back(rear_depth);
    }

    std::fstream imu_stream(imu_file);
    if (!imu_stream.is_open()) {
        std::cout << "ERROR: no exit imu file from " << imu_file << std::endl;
        exit(0);
    }

    Eigen::Vector3d angle(0., 0., 0.);
    SensorsDataType data;
    double t = -1;
    std::vector<double> datas;
    std::string line, data_txt;

    while (!imu_stream.eof()) {
        std::getline(imu_stream, line);
        std::stringstream ss(line);
        datas.clear();
        int i = 0;
        while (std::getline(ss, data_txt, ' ')) {
            double a = std::stod(data_txt);
            datas.push_back(a);
            data[i] = a;
            i++;
        }
        Eigen::Vector3d tmp = data.middleCols(1, 3);
        data.middleCols(1, 3) = data.middleCols(4, 3) * 9.80;
        data.middleCols(4, 3) = tmp;
        
        sensors_data_.push_back(data);
    }

    std::cout << "image size: " << front_depths_.size() << std::endl;

    out_idx_ = 0;
    width_ = 320;
    height_ = 240;

    cx0_ = 320.7877197265625 / 2.;
    cy0_ = 260.12588500976563 / 2.;
    fx0_ = 228.31242370605469 / 2.;
    fy0_ = 228.3023681640625 / 2.;
    fx0_inv_ = 1. / fx0_;
    fy0_inv_ = 1. / fy0_;
    scale0_ = 0.001;

    cx1_ = 316.0771484375 / 2.;
    cy1_ = 254.39895629882813 / 2.;
    fx1_ = 228.51287841796875 / 2.;
    fy1_ = 228.47869873046875 / 2.;
    fx1_inv_ = 1. / fx1_;
    fy1_inv_ = 1. / fy1_;
    scale1_ = 0.001;
}

bool OrbMachineData::GetImages(std::vector<cv::Mat> &images, double &timestamp) {
    images.clear();
    if (out_idx_ >= front_depths_.size()) {
        std::cout << "depth data meet the end!!!" << std::endl;
        return false;
    }
    cv::Mat image_front = ReadOrbRawDataImage(front_depths_[out_idx_], height_, width_);
    // cv::Mat image_rear = ReadOrbRawDataImage(rear_depths_[out_idx_], height_, width_);
    images.push_back(image_front);
    // images.push_back(image_rear);
    std::string name = front_depths_[out_idx_];
    int front_pos = name.find_last_of("/") + 1;
    int last_pos = name.find_last_of(".");
    std::string time = name.substr(front_pos, last_pos - front_pos);
    timestamp = std::atof(time.c_str());

    out_idx_+= 1;
    return true;
}

bool OrbMachineData::GetImus(std::vector<Eigen::Matrix<double, 1, 7>> &imus, double t0, double t1) {
    imus.clear();
    for (size_t i = 0; i < sensors_data_.size(); i++) {
        if (sensors_data_[0](0) > t0) return false;
        if (sensors_data_[i](0) < t0 && sensors_data_[i + 1](0) >= t0) {
            Eigen::Matrix<double, 1, 7> tmp;
            tmp(0) = t0;
            tmp.rightCols(6) = sensors_data_[i + 1].middleCols(1, 6);
            imus.push_back(tmp);
        }
        if (sensors_data_[i](0) <= t1 && sensors_data_[i](0) > t0) {
            imus.push_back(sensors_data_[i].leftCols(7));
        }
        if (sensors_data_[i](0) <= t1 && sensors_data_[i + 1](0) > t1) {
            Eigen::Matrix<double, 1, 7> tmp;
            tmp(0) = t1;
            tmp.rightCols(6) = sensors_data_[i].middleCols(1, 6);
            imus.push_back(tmp);
            return true;
        }
    }
    return false;
}

bool OrbMachineData::GetWheels(std::vector<Eigen::Matrix<double, 1, 3>> &wheels, double t0, double t1) {
    wheels.clear();
    Eigen::Matrix<double, 1, 3> tmp;
    for (size_t i = 0; i < sensors_data_.size(); i++) {
        if (sensors_data_[0](0) > t0) return false;
        if (sensors_data_[i](0) < t0 && sensors_data_[i + 1](0) >= t0) {
            tmp(0) = t0;
            tmp.rightCols(2) = sensors_data_[i + 1].middleCols(7, 2) / 0.035;
            wheels.push_back(tmp);
        }
        if (sensors_data_[i](0) <= t1 && sensors_data_[i](0) > t0) {
            tmp(0) = sensors_data_[i](0);
            tmp.rightCols(2) = sensors_data_[i].middleCols(7, 2) / 0.035;
            wheels.push_back(tmp);
        }
        if (sensors_data_[i](0) <= t1 && sensors_data_[i + 1](0) > t1) {
            tmp(0) = t1;
            tmp.rightCols(2) = sensors_data_[i].middleCols(7, 2) / 0.035;
            wheels.push_back(tmp);
            return true;
        }
    }
    return false;
}

bool OrbMachineData::GetInitGyroBias(Eigen::Vector3d &gyro) {
    Eigen::Vector3d mean(0., 0., 0.);
    std::vector<double> std_group;
    std::vector<Eigen::Vector3d> means;
    for (int i = 0; i < sensors_data_.size() - 50; i++) {
        for (int j = 0; j < 50; j++) {
            mean += sensors_data_[i + j].middleCols(1, 3).transpose();
        }
        mean /= 50.;
        double std = 0.;
        // std::cout << "mean: " << mean.transpose() << std::endl;
        for (int j = 0; j < 50; j++) {
            std += (sensors_data_[i + j].middleCols(1, 3).transpose() - mean).norm();
        }
        std /= 49.;
        std_group.push_back(std);
        means.push_back(mean);
        // if (std < 0.1) {
        //     std::cout << "imu bias time: " << sensors_data_[i + 25](0) << std::endl;
        //     gyro = mean;
        //     return true;
        // }
    }
    std::vector<double>::iterator min_iter = std::min_element(std_group.begin(), std_group.end());
    int idx = std::distance(std_group.begin(), min_iter);
    std::cout << "gyro min std: " << *min_iter << std::endl;
    if (*min_iter < 0.1) {
        gyro = means[idx];
        std::cout << "min timestamp: " << std::setprecision(13) << sensors_data_[idx](0) << ", idx: " << idx 
            << ",total size: " << std_group.size() << std::endl;
        return true;
    }

    return false;
}

SunnyMachine::SunnyMachine(std::string folder_path) {
    std::string odom_file = folder_path + "/sensor_data/imu/data.txt";
    std::string pc_folder = folder_path + "/sensor_data/sunny_tof/";

    std::string format = ".txt";
    GetFilesFromFolder(pc_folder, pc_files_, format);
    for (auto &it : pc_files_) {
        it = pc_folder + it;
    }

    char separator = ' ';
    GetAllVectorsFromFile(odom_file, odom_vecs_, separator);

    count_ = 0;
    odom_count_ = 1;
}

bool SunnyMachine::GetPointCloud(std::vector<std::vector<Eigen::Vector3d>> &pts, double &timestamp) {
    pts.clear();
    if (count_ >= pc_files_.size()) return false;
    std::string file = pc_files_[count_];

    int front_pos = file.find_last_of("/") + 1;
    int last_pos = file.find_last_of(".");
    std::string time = file.substr(front_pos, last_pos - front_pos);
    timestamp = std::atof(time.c_str());

    std::vector<std::vector<double>> vecs;
    GetAllVectorsFromFile(file, vecs, ';');
    Eigen::Vector3d pt;
    std::vector<Eigen::Vector3d> points;
    for (auto &it : vecs) {
        pt(0) = it[0];
        pt(1) = it[1];
        pt(2) = it[2];
        points.push_back(pt);
    }
    pts.push_back(points);

    count_++;
    return true;
}

bool SunnyMachine::GetOdomData(std::vector<Eigen::Matrix<double, 1, 9>> &odoms, double t0, double t1) {
    odoms.clear();
    if (t1 - t0 < 0.001) return false;
    odom_count_ = odom_count_ > 1? odom_count_ - 1 : odom_count_;
    Eigen::Matrix<double, 1, 9> odom;
    for (; odom_count_ < odom_vecs_.size() - 1; odom_count_++) {
        auto &data = odom_vecs_[odom_count_];
        if (data[0] < t0) continue;
        if (data[0] >= t0 && odom_vecs_[odom_count_ - 1][0] < t0) {
            memcpy(odom.data(), &data[0], 9 * sizeof(double));
            odom.middleCols(1, 3).swap(odom.middleCols(4, 3));
            odoms.push_back(odom);
            continue;
        }
        if (data[0] > t0 && data[0] < t1) {
            memcpy(odom.data(), &data[0], 9 * sizeof(double));
            odom.middleCols(1, 3).swap(odom.middleCols(4, 3));
            odoms.push_back(odom);
        }
        if (data[0] < t1 && odom_vecs_[odom_count_ + 1][0] >= t1) {
            memcpy(odom.data(), &data[0], 9 * sizeof(double));
            odom.middleCols(1, 3).swap(odom.middleCols(4, 3));
            odoms.push_back(odom);
            break;
        }
        if (data[0] > t1) break;
    }
    if (odoms.size() < 2) return false;
    double dt = odoms.back()(0) - odoms.front()(0);
    if (dt < (t1 - t0 - 0.001)) return false;

    for (size_t i = 0; i < odoms.size() - 1; i++) {
        dt = odoms[i + 1](0) - odoms[i](0);
        if (dt > 0.05) {
            std::cout << "time interval between imu is too big with " << dt <<std::endl;
            // exit(0);
        }
    }

    return true;
}

bool SunnyMachine::GetImuAndWheelData(std::vector<Eigen::Matrix<double, 1, 7>> &imus, std::vector<Eigen::Matrix<double, 1, 3>> &wheels,
                            double t0, double t1) {
    imus.clear();
    wheels.clear();
    if (t1 - t0 < 0.001) return false;
    odom_count_ = odom_count_ > 1? odom_count_ - 1 : odom_count_;
    Eigen::Matrix<double, 1, 7> imu;
    Eigen::Matrix<double, 1, 3> wheel;
    for (; odom_count_ < odom_vecs_.size() - 1; odom_count_++) {
        auto &data = odom_vecs_[odom_count_];
        if (data[0] < t0) continue;
        if (data[0] >= t0 && odom_vecs_[odom_count_ - 1][0] < t0) {
            memcpy(imu.data(), &data[0], 7 * sizeof(double));
            imu.middleCols(1, 3).swap(imu.middleCols(4, 3));
            wheel << t0, data[7] * 2. * M_PI, data[8] * 2. * M_PI;
            imu(0) = t0;
            imus.push_back(imu);
            wheels.push_back(wheel);
            continue;
        }
        if (data[0] > t0 && data[0] < t1) {
            memcpy(imu.data(), &data[0], 7 * sizeof(double));
            imu.middleCols(1, 3).swap(imu.middleCols(4, 3));
            wheel << data[0], data[7] * 2. * M_PI, data[8] * 2. * M_PI;
            imus.push_back(imu);
            wheels.push_back(wheel);
        }
        if (data[0] < t1 && odom_vecs_[odom_count_ + 1][0] >= t1) {
            memcpy(imu.data(), &data[0], 7 * sizeof(double));
            imu.middleCols(1, 3).swap(imu.middleCols(4, 3));
            wheel << t1, data[7] * 2. * M_PI, data[8] * 2. * M_PI;
            imu(0) = t1;
            imus.push_back(imu);
            wheels.push_back(wheel);
            break;
        }
        if (data[0] > t1) break;
    }
    if (imus.size() < 2) return false;
    double dt = imus.back()(0) - imus.front()(0);
    if (dt < (t1 - t0 - 0.001)) return false;

    for (size_t i = 0; i < imus.size() - 1; i++) {
        dt = imus[i + 1](0) - imus[i](0);
        if (dt > 0.05) {
            std::cout << "t0,t1: " << t0 << "," << t1 << std::endl;
            std::cout << "imu j: " << imus[i+1] << std::endl;
            std::cout << "imu i: " << imus[i] << std::endl;
            std::cout << "time interval between imu is too big with " << dt <<std::endl;
            exit(0);
        }
    }

    // std::cout << "t0 t1: " << t0 << "," << t1 << std::endl;
    // for (size_t i = 0; i < imus.size(); i++) {
    //     std::cout << "imu: " << imus[i] << ", wheel: " << wheels[i] << std::endl;
    // }
    // if (fabs(wheels.back()(1)) > 0.1)
    //     exit(0);

    return true;
}

OfilmMachine::OfilmMachine(std::string folder_path) {
    std::string odom_file = folder_path + "/sensor_data/imu/data.txt";
    std::string pc_folder = folder_path + "/sensor_data/tof/PointCloud/";
    std::string flood_folder = folder_path + "/sensor_data/tof/depth/";

    std::string format = ".ply";
    GetFilesFromFolder(pc_folder, pc_files_, format);
    for (auto &it : pc_files_) {
        it = pc_folder + it;
    }

    std::string flood_format = ".raw";
    GetFilesFromFolder(flood_folder, depth_files_, flood_format);
    for (auto &it : depth_files_) {
        it = flood_folder + it;
    }

    char separator = ' ';
    GetAllVectorsFromFile(odom_file, odom_vecs_, separator);

    count_ = 0;
    depth_count_ = 0;
    odom_count_ = 1;
    depth_width_ = 240;
    depth_height_ = 180;
}

bool OfilmMachine::GetPointCloud(std::vector<std::vector<Eigen::Vector3d>> &pts, double &timestamp) {
    pts.clear();
    if (count_ >= pc_files_.size()) return false;
    std::string pc_file = pc_files_[count_];
    pts.emplace_back();
    if (!ReadPly(pc_file, pts[0])) {
        pts.clear();
        return false;
    }
    int endpos = pc_file.find_last_of(".");
    int startpos = pc_file.find_last_of("/");
    std::string time_txt = pc_file.substr(startpos + 1, endpos - startpos - 1);
    timestamp = std::atof(time_txt.c_str());
    count_++;
    return true;
}

bool OfilmMachine::GetDepthImage(std::vector<cv::Mat> &depths, double &timestamp) {
    depths.clear();
    if (depth_count_ >= depth_files_.size()) return false;
    std::string depth_file = depth_files_[depth_count_];
    cv::Mat depth = ReadOrbRawDataImage(depth_file, depth_height_, depth_width_);
    if (depth.empty()) return false;
    // get depth data and remove confidence data bit
    Eigen::Map<Eigen::Matrix<uint16_t, Eigen::Dynamic, Eigen::Dynamic,
            Eigen::RowMajor>> tmp((uint16_t*)depth.data, depth_height_, depth_width_);
    for (int r = 0; r < depth_height_; r++) {
        for (int c = 0; c < depth_width_; c++) {
            tmp(r, c) <<= 3;
            tmp(r, c) >>= 3;
        }
    }

    depths.push_back(depth);
    int endpos = depth_file.find_last_of(".");
    int startpos = depth_file.find_last_of("/");
    std::string time_txt = depth_file.substr(startpos + 1, endpos - startpos - 1);
    timestamp = std::atof(time_txt.c_str());
    depth_count_++;
    return true;
}

bool OfilmMachine::ReadPly(const std::string &file, std::vector<Eigen::Vector3d> &pts) {
    FILE *fp = fopen(file.c_str(), "r");
    if (fp == nullptr) {
        std::cout << "cannot open file: " << file << std::endl;
        exit(0);
    }

    Eigen::Vector3f pt;
    char buf[255];
    int param_num;
#define READLINE() if (!fgets(buf, 255, fp)) goto ply_read_error
#define LINECMP(txt) !strncasecmp(buf, txt, std::strlen(txt))

    READLINE();
    if (!LINECMP("ply")) {
        std::cout << "it is not ply of file: " << file << std::endl;
        exit(0);
    }

    while(1) {
        READLINE();
        if (LINECMP("element vertex")) break;
    }

    int point_num;
    sscanf(buf, "element vertex %d\n", &point_num);
    while(1) {
        READLINE();
        if (LINECMP("end_header")) break;
    }

    while (1) {
        param_num = fscanf(fp, "%f %f %f", &pt(0), &pt(1), &pt(2));

        if (param_num != 3) break;
        pts.push_back(pt.cast<double>());
    }

    return true;
ply_read_error:
    fclose(fp);
    std::cout << "read error of ply file: " << file << std::endl;
    exit(0);
    return false;
}

bool OfilmMachine::GetImuAndWheelData(std::vector<Eigen::Matrix<double, 1, 7>> &imus, std::vector<Eigen::Matrix<double, 1, 3>> &wheels,
                                      double t0, double t1) {
    imus.clear();
    wheels.clear();
    if (t1 - t0 < 0.001) return false;
    odom_count_ = odom_count_ > 1? odom_count_ - 1 : odom_count_;
    Eigen::Matrix<double, 1, 7> imu;
    Eigen::Matrix<double, 1, 3> wheel;
    for (; odom_count_ < odom_vecs_.size() - 1; odom_count_++) {
        auto &data = odom_vecs_[odom_count_];
        if (data[0] < t0) continue;
        if (data[0] >= t0 && odom_vecs_[odom_count_ - 1][0] < t0) {
            memcpy(imu.data(), &data[0], 7 * sizeof(double));
            imu.middleCols(1, 3).swap(imu.middleCols(4, 3));
            imu.middleCols(1, 3) *= 9.8;
            wheel << t0, data[7] / 0.035, data[8] / 0.035;
            imu(0) = t0;
            imus.push_back(imu);
            wheels.push_back(wheel);

            memcpy(imu.data(), &data[0], 7 * sizeof(double));
            imu.middleCols(1, 3).swap(imu.middleCols(4, 3));
            imu.middleCols(1, 3) *= 9.8;
            wheel << data[0], data[7] / 0.035, data[8] / 0.035;
            imu(0) = data[0];
            imus.push_back(imu);
            wheels.push_back(wheel);
            continue;
        }
        if (data[0] > t0 && data[0] < t1) {
            memcpy(imu.data(), &data[0], 7 * sizeof(double));
            imu.middleCols(1, 3).swap(imu.middleCols(4, 3));
            imu.middleCols(1, 3) *= 9.8;
            wheel << data[0], data[7] / 0.035, data[8] / 0.035;
            imus.push_back(imu);
            wheels.push_back(wheel);
        }
        if (data[0] < t1 && odom_vecs_[odom_count_ + 1][0] >= t1) {
            memcpy(imu.data(), &data[0], 7 * sizeof(double));
            imu.middleCols(1, 3).swap(imu.middleCols(4, 3));
            imu.middleCols(1, 3) *= 9.8;
            wheel << t1, data[7] / 0.035, data[8] / 0.035;
            imu(0) = t1;
            imus.push_back(imu);
            wheels.push_back(wheel);
            break;
        }
        if (data[0] > t1) break;
    }
    if (imus.size() < 2) return false;
    double dt = imus.back()(0) - imus.front()(0);
    if (dt < (t1 - t0 - 0.001)) return false;

    std::cout.precision(10);
    for (size_t i = 0; i < imus.size() - 1; i++) {
        dt = imus[i + 1](0) - imus[i](0);
        // std::cout << "imu: " << imus[i] << std::endl;
        if (dt > 0.1) {
            std::cout << "t0,t1: " << t0 << "," << t1 << std::endl;
            std::cout << "imu j: " << imus[i+1] << std::endl;
            std::cout << "imu i: " << imus[i] << std::endl;
            std::cout << "time interval between imu is too big with " << dt <<std::endl;
            // exit(0);
        }
    }

    // std::cout << "t0 t1: " << t0 << "," << t1 << std::endl;
    // for (size_t i = 0; i < imus.size(); i++) {
    //     std::cout << "imu: " << imus[i] << ", wheel: " << wheels[i] << std::endl;
    // }
    // if (fabs(wheels.back()(1)) > 0.1)
    //     exit(0);

    return true;
}

cv::Mat ReadOrbRawDataImage(std::string image_file, int raw_data_height, int raw_data_width) {
    FILE *file_ptr;
    file_ptr = fopen(image_file.c_str(), "rb+");

    if (file_ptr == nullptr) {
        std::cout << "cannot open raw file image from " << image_file << std::endl;
        exit(0);
    }

    cv::Mat raw_data(raw_data_height, raw_data_width, CV_16UC1, cv::Scalar(0));
    size_t res = fread(raw_data.data, sizeof(uint16_t), raw_data_height * raw_data_width, file_ptr);

    fclose(file_ptr);
    return std::move(raw_data);
}

void GenerateListsTxt(std::string dir) {
    std::vector<std::string> depth, depth_back;
    std::string depth_file = dir + "tof/depth";
    std::string depth_back_file = dir + "tof/depth_back";

    GetAllFiles(depth_file, depth);
    GetAllFiles(depth_back_file, depth_back);
}

void ReadPlyPoints(std::string file, std::vector<Eigen::Vector3d> &pts, std::vector<Eigen::Vector3d> &colors) {
    Eigen::Vector3f pt;
    Eigen::Vector3d ct;
    int other_property_len = 0;
    int points_num = 0, result, color_index = 0, bank = 0;
    bool has_color = false;
    uchar color[3];

    FILE *f = fopen(file.c_str(), "r");
    if (!f) {
        std::cout << "cannot open: " << file << std::endl;
        exit(0);
    }
    char buf[255];
    if (!fgets(buf, 255, f) || strncmp(buf, "ply", 3)) {
        std::cout << "buf: " << buf << std::endl;
        std::cout << "it is not a ply file: " << file << std::endl;
        exit(0);
    }

#define GETLINE() if (!fgets(buf, 255, f)) goto plyreaderror
#define LINEIS(txt) !strncasecmp(buf, txt, std::strlen(txt))

    while(1) {
        GETLINE();
        if (LINEIS("element vertex")) break;
    }

    result = sscanf(buf, "element vertex %d\n", &points_num);

    GETLINE();
    if (!LINEIS("property float x")) goto plyreaderror;
    GETLINE();
    if (!LINEIS("property float y")) goto plyreaderror;
    GETLINE();
    if (!LINEIS("property float z")) goto plyreaderror;

    GETLINE();
    while (LINEIS("property")) {
        if (LINEIS("property uchar red")) {
            color_index = other_property_len;
            has_color = true;
        }
        if (LINEIS("property char") || LINEIS("property uchar")) other_property_len += 1;
        else if (LINEIS("property int") || LINEIS("property uint") || LINEIS("property float")) other_property_len += 4;
        else goto plyreaderror;
        GETLINE();
    }
    GETLINE();
    while (!LINEIS("end_header")) {
        GETLINE();
    }

    if (has_color) {
        other_property_len -= color_index + 3;
    }
    for (int i = 0; i < points_num; i++) {
        if (!fread((void*)pt.data(), 12, 1, f)) goto plyreaderror;
        if (has_color) {
            if (color_index && !fread((void*)buf, color_index, 1, f)) goto plyreaderror;
            if (!fread((void*)color, 3, 1, f)) goto plyreaderror;
            ct(0) = static_cast<double>(color[0]);
            ct(1) = static_cast<double>(color[1]);
            ct(2) = static_cast<double>(color[2]);
            colors.push_back(ct / ct.norm());
        }
        if (other_property_len && !fread((void*)buf, other_property_len, 1, f)) goto plyreaderror;
        pts.push_back(pt.cast<double>());
    }

    fclose(f);
    return;
plyreaderror:
    fclose(f);
    std::cout << "read error of file: " << file << std::endl;
}

} // DataSetType
} // slam
