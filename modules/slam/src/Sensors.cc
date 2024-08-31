#include "slam/Sensors.h"

namespace slam {
PinHoleCam::PinHoleCam(const Eigen::Vector4d &intrinsic, int width, int height, Eigen::Matrix<double, 5, 1> distort)
    : intrinsic_(intrinsic), distort_(distort), width_(width), height_(height) {
    D_ = (cv::Mat_<double>(5, 1) << distort_(0), distort_(1), distort_(2), distort_(3), distort_(4));
    K_ = (cv::Mat_<double>(3, 3) << intrinsic_(0), 0, intrinsic_(2),
                                    0, intrinsic_(1), intrinsic_(3),
                                    0, 0, 1);
    cv::Size img_size;
    img_size.width = width_;
    img_size.height = height_;
    cv::initUndistortRectifyMap(K_, D_, cv::Mat(), cv::Mat(), img_size, CV_32FC1, mapx_, mapy_);
}
cv::Mat PinHoleCam::Undistort(cv::Mat image) {
    cv::Mat out;
    cv::remap(image, out, mapx_, mapy_, cv::INTER_LINEAR);
    return std::move(out);
}

void PinHoleCam::UndistortPoint2fs(const std::vector<cv::Point2f> &points, std::vector<cv::Point2f> &undistorted) {
    cv::undistortPoints(points, undistorted, K_, D_);
}

void PinHoleCam::IntrinsicZooming(const Eigen::Vector4d &intrinsic, Eigen::Vector4d &out, double s) {
    // u = fx * x / z + cx => x = s * (u - cx) * z / (s * fx) = (s * u - s * cx) * z / (s * fx)
    // => x = (u^ - cx^) * z / fx^; u^ = s * u, cx^ = s * cx, fx^ = s * fx
    // the same: fy^ = s * fy, cy^ = s * cy;
    out = intrinsic * s;
}

void PinHoleCam::DistortCoeffZooming(const Eigen::Matrix<double, 5, 1> &dist_coef, Eigen::Matrix<double, 5, 1> &out, double s) {
    out = dist_coef;
}
void TofCamera::IntrinsicZooming(const Eigen::Vector4d &intrinsic, Eigen::Vector4d &out, double s) {
    // u = fx * x / z + cx => x = s * (u - cx) * z / (s * fx) = (s * u - s * cx) * z / (s * fx)
    // => x = (u^ - cx^) * z / fx^; u^ = s * u, cx^ = s * cx, fx^ = s * fx
    // the same: fy^ = s * fy, cy^ = s * cy;
    out = intrinsic * s;
}

void TofCamera::DistortCoeffZooming(const Eigen::Matrix<double, 5, 1> &dist_coef, Eigen::Matrix<double, 5, 1> &out, double s) {
    out = dist_coef;
}

Sensors::Sensors(const nlohmann::json &config) {
    // imu config
    for (int i = 0; i < 3; i++) {
        imu.na(i) = config["imu"]["Na"][i];
        imu.ng(i) = config["imu"]["Ng"][i];
        imu.ba(i) = config["imu"]["ba"][i];
        imu.bg(i) = config["imu"]["bg"][i];
        imu.nba(i) = config["imu"]["Nba"][i];
        imu.nbg(i) = config["imu"]["Nbg"][i];
    }

    for (size_t i = 0; i < 3; i++) {
        for (size_t j = 0; j < 3; j++) {
            imu.Rib(i, j) = config["imu"]["Rib"][3 * i + j];
            imu.Tib.R(i, j) = config["imu"]["Rib"][3 * i + j];
        }
        imu.tib(i) = config["imu"]["tib"][i];
        imu.Tib.t(i) = config["imu"]["tib"][i];
    }

    // tof camera config
    int cam_num = config["cam_num"];    
    for (size_t i = 0; i < cam_num; i++) {
        std::string cam_name = "cam" + std::to_string(i);
        TofCamera cam;
        cam.fx = config[cam_name]["intrinsic"][0];
        cam.fy = config[cam_name]["intrinsic"][1];
        cam.cx = config[cam_name]["intrinsic"][2];
        cam.cy = config[cam_name]["intrinsic"][3];

        cam.fx_inv = 1.0 / cam.fx;
        cam.fy_inv = 1.0 / cam.fy;
        for (size_t i = 0; i < 3; i++) {
            for (size_t j = 0; j < 3; j++) {
                cam.Rcb(i, j) = config[cam_name]["Rcb"][3 * i + j];
                cam.Tcb.R(i, j) = config[cam_name]["Rcb"][3 * i + j];
            }
            cam.tcb(i) = config[cam_name]["tcb"][i];
            cam.Tcb.t(i) = config[cam_name]["tcb"][i];
        }
        cam.max_depth = config[cam_name]["max_depth"];
        cam.min_depth = config[cam_name]["min_depth"];
        cam.width = config[cam_name]["image_size"][0];
        cam.height = config[cam_name]["image_size"][1];
        for (size_t i = 0; i < 3; i++) {
            cam.noise(i) = config[cam_name]["noise"][i];
        }
        tofcams.push_back(cam);
    }

    // differetial wheel config
    diff_wheel.distance = config["wheel"]["distance"];
    diff_wheel.radius = config["wheel"]["radius"];
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            diff_wheel.Rob(i, j) = config["wheel"]["Rob"][3 * i + j];
            diff_wheel.Tob.R(i, j) = config["wheel"]["Rob"][3 * i + j];
        }
        diff_wheel.tob(i) = config["wheel"]["tob"][i];
        diff_wheel.Tob.t(i) = config["wheel"]["tob"][i];
    }
    diff_wheel.noise_left = config["wheel"]["noise_left"];
    diff_wheel.noise_right = config["wheel"]["noise_right"];

    SensorsConfigPrint();
}

void Sensors::SensorsConfigPrint() {
    LOG(INFO) << "***********sensor configuration*************";
    LOG(INFO) << "imu na: " << imu.na.transpose() << "; ng: " << imu.ng.transpose();
    LOG(INFO) << "imu ba: " << imu.ba.transpose() << "; bg: " << imu.bg.transpose();
    LOG(INFO) << "imu nba: " << imu.nba.transpose() << "; nbg: " << imu.nbg.transpose();
    LOG(INFO) << "Tib: " << imu.Tib;

    for (size_t i = 0; i < tofcams.size(); i++) {
        const TofCamera &cam = tofcams[i];
        LOG(INFO) << i << "th tof cam: ";
        LOG(INFO) << "(fx, fy, cx, cy): " << cam.fx << ", " << cam.fy << ", " << cam.cx << ", " << cam.cy;
        LOG(INFO) << "max and min depth: " << cam.max_depth << ", " << cam.min_depth;
        LOG(INFO) << "width and height: " << cam.width << ", " << cam.height;
        LOG(INFO) << "Tcb: " << cam.Tcb;
    }
    LOG(INFO) << "differential wheel distance: " << diff_wheel.distance;
    LOG(INFO) << "wheel radius: " << diff_wheel.radius;
    LOG(INFO) << "Tob: " << diff_wheel.Tob;
    // std::cout << "****sensor configuration" << std::endl;
    // std::cout.width(8);
    // std::cout << "imu: " << std::endl;
    // std::cout.width(12);
    // std::cout << "na: " << imu.na.transpose() << "; ng: " << imu.ng.transpose() << std::endl;
    // std::cout.width(12);
    // std::cout << "ba: " << imu.ba.transpose() << "; bg: " << imu.bg.transpose() << std::endl;
    // std::cout.width(12);
    // std::cout << "nba: " << imu.nba.transpose() << "; nbg: " << imu.nbg.transpose() << std::endl;
    // std::cout.width(12);
    // std::cout << "Rib: " << imu.Rib.row(0) << std::endl;
    // std::cout.width(12);
    // std::cout << " " << imu.Rib.row(1) << std::endl;
    // std::cout.width(13);
    // std::cout << " " << imu.Rib.row(2) << std::endl;
    // std::cout.width(12);
    // std::cout << "tib: " << imu.tib.transpose() << std::endl;
    // for (size_t i = 0; i < tofcams.size(); i++) {
    //     const TofCamera &cam = tofcams[i];
    //     std::cout << "   " << i << "th tof cam: " << std::endl;
    //     std::cout << "       (fx, fy, cx, cy): " << cam.fx << ", " << cam.fy << ", " << cam.cx << ", " << cam.cy << std::endl;
    //     std::cout << "       max and min depth: " << cam.max_depth << ", " << cam.min_depth << std::endl;
    //     std::cout << "       width and height: " << cam.width << ", " << cam.height << std::endl;
    //     std::cout.width(12);
    //     std::cout << "Rcb: " << cam.Rcb.row(0) << std::endl;
    //     std::cout.width(12);
    //     std::cout << " " << cam.Rcb.row(1) << std::endl;
    //     std::cout.width(12);
    //     std::cout << " " << cam.Rcb.row(2) << std::endl;
    //     std::cout.width(12);
    //     std::cout << "tcb: " << cam.tcb.transpose() << std::endl;
    // }
}

} // namespace slam