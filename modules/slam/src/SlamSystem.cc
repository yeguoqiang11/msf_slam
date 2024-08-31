#include "slam/SlamSystem.h"

namespace slam {
SlamSystem::SlamSystem(const std::string &config, bool is_feature_based) {
    std::ifstream config_ss(config);
    nlohmann::json config_json = nlohmann::json::parse(config_ss);
    
    // sensor object
    sensors_ptr_ = std::make_shared<Sensors>(config_json["Sensors"]);
    Imu::GlobalParametersInitialize(sensors_ptr_);

    // map manager
    voxelmap_ptr_ = std::make_shared<GlobalVoxelMap>(config_json["VoxelMap"]);
    voxelgrid_size_ = config_json["VoxelMap"]["griding_voxel_size"];

    // mapping object
    voxelmapping_ptr_ = std::make_shared<VoxelMapping>(voxelmap_ptr_, sensors_ptr_);

    // make kl divergence odometry object
    KL_odom_ptr_ = std::make_shared<KLDivergenceOdometry>(config_json["KLOdometry"], sensors_ptr_);

    // loop closure
    pcloopclosure_ptr_ = std::make_shared<VoxelMapLoopClosure>(voxelmap_ptr_);

    // attitude object
    attitude_ptr_ = std::make_shared<MEKFAttitude>(sensors_ptr_);

    // ground extractor
    ground_ptr_ = std::make_shared<GroundExtraction>(Eigen::Vector3d(0, 1., 0));

    // ground planes
    groundplanes_ptr_ = std::make_shared<GroundPlanes>();
    KL_odom_ptr_->SetGroundPtr(groundplanes_ptr_);
    voxelmapping_ptr_->SetGlobalGround(groundplanes_ptr_);

    // voxel map setting
    VoxelMapSetting(config_json["VoxelMap"]);

    // shower
    shower_ptr_ = std::make_shared<Shower>(voxelmap_ptr_, sensors_ptr_);

    // init odometry pose
    Tbw_.SetIdentity();
    Vw_.setZero();
    ba_.setZero();
    bg_.setZero();
    last_timestamp_ = -1;
    timestamp_ = -1;

    is_feature_based_ = is_feature_based;
}

std::shared_ptr<GyroWheelModelPreintegration> SlamSystem::GyroWheelObject(SensorsData &data) {
    if (data.imus.empty() || data.wheels.empty()) return nullptr;

    Eigen::Vector3d noise;
    noise(0) = sensors_ptr_->diff_wheel.noise_left;
    noise(1) = sensors_ptr_->diff_wheel.noise_right;
    noise(2) = (sensors_ptr_->imu.Rib * sensors_ptr_->imu.ng)(1);
    double r = sensors_ptr_->diff_wheel.radius;
    std::shared_ptr<GyroWheelModelPreintegration> imu_wheel_ptr =
        std::make_shared<GyroWheelModelPreintegration>(r, r, 0., noise);
    
    Eigen::Matrix3d Rio = sensors_ptr_->diff_wheel.Rob.transpose() * sensors_ptr_->imu.Rib;
    std::vector<Eigen::RowVector2d> gyros;
    for (size_t i = 0; i < data.imus.size(); i++) {
        Eigen::Vector3d g = Rio * data.imus[i].rightCols(3).transpose();
        gyros.emplace_back(data.imus[i](0), g(2));
    }
    imu_wheel_ptr->InsertDatas(gyros, data.wheels);
    return std::move(imu_wheel_ptr);
}

/**
 * @brief load vector of images and imus to pose calculation
 * 
 * @param images depth image type of CV_64F
 * @param imus vector of imu [timestamp, ax, ay, az, gx, gy, gz]
 * @param timestamps vector of timestamp
 */
void SlamSystem::LoadData(SensorsData &data) {
    NdtBasedMethod(data);
    // DepthImageBasedMethod(images, imus, timestamps);
}

std::shared_ptr<Frame> SlamSystem::CreateFrame(SensorsData &data) {
    // timer ticker
    utils::Timer timecount;
    timecount.Start();

    // set gravity in imu coordinate
    if (groundplanes_ptr_->GravitySharedPtr() == nullptr) {
        if (attitude_ptr_->GravitySharedPtr() != nullptr) {
            groundplanes_ptr_->SetGravityInImuFrame((*attitude_ptr_->GravitySharedPtr()));
            std::cout << "Gw: " << (*groundplanes_ptr_->GravitySharedPtr()).transpose() << std::endl;
        }
    }

    // imu integration
    std::shared_ptr<Imu> imu_ptr = nullptr;
    if (!data.imus.empty()) {
        imu_ptr = std::make_shared<Imu>(ba_, bg_);
        imu_ptr->PreIntegration(data.imus);
        LOG(INFO) << "imu preintegration Rji: \n" << imu_ptr->Rji();
    }
    double dt = timecount.End("imu preintegration");
    LOG(INFO) << "imu preintegration time cost: " << dt;

    // wheel preintegration
    std::shared_ptr<EulerWMPreintegration> wheel_ptr = nullptr;
    if (!data.wheels.empty()) {
        wheel_ptr = std::make_shared<EulerWMPreintegration>(sensors_ptr_->diff_wheel.radius,
            sensors_ptr_->diff_wheel.radius, sensors_ptr_->diff_wheel.distance, sensors_ptr_->diff_wheel.noise_left,
            sensors_ptr_->diff_wheel.noise_right);
        wheel_ptr->LoadWheelDatas(data.wheels);
    }

    // gyro-wheel model preintegration
    std::shared_ptr<GyroWheelModelPreintegration> gyro_wheel_ptr = GyroWheelObject(data);
    utils::Transform3d Tow1(Tbw_);
    if (gyro_wheel_ptr != nullptr) {
        gyro_wheel_ptr->PoseCalFromLastPose(Tbw_, sensors_ptr_->diff_wheel.Tob, Tow1);
    }

    // pose prediction from wheel odom
    utils::Transform3d Tow(Tbw_);
    if (wheel_ptr != nullptr) {
        wheel_ptr->PoseCalFromLastPose(Tbw_, sensors_ptr_->diff_wheel.Tob, Tow);
    }

    timecount.End("wheel preintegration");
    // create frame, load sensor measurements
    std::shared_ptr<Frame> frame_ptr = std::make_shared<Frame>(voxelgrid_size_, imu_ptr, wheel_ptr);
    frame_ptr->SetGyroWheelSharedPtr(gyro_wheel_ptr);
    if (!data.images.empty())
        AddPointCloud2Frame(data.images, data.timestamp, frame_ptr);

    if (!data.points.empty()) {
        AddPointCloud2Frame(data.points, frame_ptr);
    }

    frame_ptr->SetBga(ba_, bg_);
    std::cout << "init ba: " << frame_ptr->Ba().transpose() << std::endl;
    std::cout << "init bg: " << frame_ptr->Bg().transpose() << std::endl;

    dt = timecount.End("griding pointcloud");
    LOG(INFO) << "griding pointcloud time cost: " << dt;

    // ground extractor
    if (ground_ptr_->Extract(frame_ptr)) {
        std::shared_ptr<GroundInfo> tmp_ground = std::make_shared<GroundInfo>(ground_ptr_->Ground());
        frame_ptr->PointCloudFrameRef().SetGround(tmp_ground);
    }

    // tof prediction from imu
    utils::Transform3d Tbw(Tbw_);
    if (imu_ptr != nullptr) {
        Eigen::Vector3d Vw1;
        utils::Transform3d Tbw1;
        imu_ptr->ImuStatePrediction(Tbw_ * sensors_ptr_->imu.Tib, Vw_, Eigen::Vector3d(0, 0, 0), Tbw1, Vw1);
        Tbw1 *= sensors_ptr_->imu.Tib.Inverse();
    }

    // pose prediction setting
    frame_ptr->SetPose(Tow1);
    // frame.SetPose(Row, tow);
    // frame_ptr->SetPose(Rbw_, tbw_);

    frame_ptr->SetTimeStamp(data.timestamp);

    return frame_ptr;
}

void SlamSystem::NdtBasedMethod(SensorsData &data) {
    // timer ticker
    utils::Timer timecount;
    timecount.Start();

    // calculate attitude
    attitude_ptr_->LoadImus(data.imus);

    // sensor data collection and preprocess
    std::shared_ptr<Frame> frame_ptr = CreateFrame(data);

    // multi-sensors odometry
    KL_odom_ptr_->LoadFrame(frame_ptr, data.timestamp);

    double dt = timecount.End("kl odom");
    LOG(INFO) << "kl odom time cost: " << dt;

    // key frame optimization
    if (KL_odom_ptr_->IsKeyFrame()) {
        // global map updation
        voxelmapping_ptr_->InsertKeyFrame(frame_ptr);

        dt = timecount.End("keyframe optimization");
        LOG(INFO) << "keyframe optimization time cost: " << dt;

        KL_odom_ptr_->RectifyLocalMapPose(frame_ptr->Tbw());

        // loop closure
        timecount.Start();
        // pcloopclosure_ptr_->InsertFrame(frame_ptr);
        timecount.End("loop closure cost");
    } else {
        voxelmapping_ptr_->FusingFrame(frame_ptr);
    }

    // system state update
    StateUpdate(frame_ptr);
    timestamp_ = data.timestamp;
    std::cout << "keyframe num: " << voxelmap_ptr_->FrameSize() << std::endl;
}

void SlamSystem::StateUpdate(std::shared_ptr<Frame> frame_ptr) {
    if (frame_ptr->ImuSharedPtr() != nullptr) {
        ba_ = frame_ptr->Ba();
        bg_ = frame_ptr->Bg();
        std::cout << "ba: " << ba_.transpose() << std::endl;
        std::cout << "bg: " << bg_.transpose() << std::endl;
    }

    frame_ptr->GetPose(Tbw_);

    // frame_ptr->GetPose(Rbw_, tbw_);
    // ground update
    groundplanes_ptr_->GroundUpdate(frame_ptr);
    // std::cout << "ground num: " << groundplanes_ptr_->Size() << std::endl;

    is_keyframe_ = frame_ptr->IsKeyFrame();

    Eigen::Vector3d euler_tof = utils::DMmath::RotationMatrix2EulerAngleZXY(Tbw_.R());
    std::cout << "tof euler: " << euler_tof.transpose() * RAD2DEG << std::endl;
    std::cout << "output pose: " << Tbw_ << std::endl;
}

int SlamSystem::DepthCostfunction(const cv::Mat &image0, const cv::Mat &image1, const Eigen::Matrix3d R10, const Eigen::Vector3d &t10,
                                   Eigen::Matrix<double, 6, 6> &H, Eigen::Matrix<double, 6, 1> &b, double &cost) {
    TofCamera &cam = sensors_ptr_->tofcams[0];
    Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> depth1((double*)image1.data, cam.height, cam.width);
    Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> depth0((double*)image0.data, cam.height, cam.width);

    int bank = 30;
    double fx = direct_.K(0, 0);
    double fy = direct_.K(1, 1);
    double cx = direct_.K(0, 2);
    double cy = direct_.K(1, 2);

    int inlier_count = 0;
    for (int r = bank; r < cam.height - bank; r += 2) {
        for (int c = bank; c < cam.width - bank; c += 2) {
            double deptht1 = depth1(r, c);
            if (cam.IsDepthValid(deptht1)) {
                Eigen::Vector3d pt1 = deptht1 * direct_.K_inv * Eigen::Vector3d(c, r, 1);
                Eigen::Vector3d pt0 = R10 * pt1 + t10;
                Eigen::Vector3d u = direct_.K * pt0;

                if (!cam.IsDepthValid(u(2))) continue;
                u /= u(2);

                int c0 = static_cast<int>(u(0));
                int r0 = static_cast<int>(u(1));
                int c1 = static_cast<int>(u(0) + 1.00001);
                int r1 = static_cast<int>(u(1) + 1.00001);

                if (!cam.IsInside(r0, c0)) continue;
                double d0 = depth0(r0,  c0);
                double dc = depth0(r0, c1);
                double dr = depth0(r1, c0);
                double drc = depth0(r1, c1);
                double v0 = fabs(dr - d0) + fabs(dc - d0) + fabs(drc - d0);
                if (v0 > 0.3) continue;

                if (!cam.IsDepthValid(d0) || !cam.IsDepthValid(dc) || !cam.IsDepthValid(dr)) continue;
                // d = (r-r1)*(c-c1)*d0/(r0-r1)(c0-c1) + (r-r1)*(c-c0)*dc/(r0-r1)*(c1-c0) + (r-r0)*(c-c1)*dr/(r1-r0)*(c0-c1) + (r-r0)*(c-c0)*drc/(r1-r0)(c1-c0)
                // d = (r1-r)*(c1-c)*d0/(r1-r0)(c1-c0) + (r1-r)*(c-c0)*dc/(r1-r0)*(c1-c0) + (r-r0)*(c1-c)*dr/(r1-r0)*(c1-c0) + (r-r0)*(c-c0)*drc/(r1-r0)(c1-c0)
                double a = 1. / (r1 - r0) / (c1 - c0);
                double dr0 = u(1) - r0, dr1 = r1 - u(1), dc0 = u(0) - c0, dc1 = c1 - u(0);
                double h0 = dr1 * dc1 * a, h1 = dr1 * dc0 * a, h2 = dr0 * dc1 * a, h3 = dr0 * dc0 * a;
                double d = h0 * d0 + h1 * dc + h2 * dr + h3 * drc; 
                double residual = d - pt0(2);

                double sq_r = residual * residual;
                if (residual > 0.3) continue;

                double huber_thres = 0.02 * 0.02;
                double w = 1.;
                if (sq_r > huber_thres) {
                    w = huber_thres / sq_r;
                }

                Eigen::Vector2d grad;

                if (!cam.IsDepthValid(depth0(r0 + 2, c0)) || !cam.IsDepthValid(depth0(r0, c0 + 2))) continue;
                double gradr = (depth0(r0 + 2, c0) - depth0(r0, c0)) * 0.5;
                double gradc = (depth0(r0, c0 + 2) - depth0(r0, c0)) * 0.5;

                Eigen::Matrix<double, 1, 3> dr_du; dr_du(0) = gradc; dr_du(1) = gradr; dr_du(2) = -1.;
                
                double d_inv = 1. / pt0(2);
                double x = pt0(0);
                double y = pt0(1);
                // z * u = [fx, 0, cx; 0, fy, cy; 0, 0, 1] * [x, y, z]
                // c0 = fx * x/z + cx;
                // r0 = fy * y/z + cy;
                // z = z;
                Eigen::Matrix<double, 3, 3> du_dpt0;
                du_dpt0.row(0) << fx * d_inv, 0., -fx * x * d_inv * d_inv;
                du_dpt0.row(1) << 0., fy * d_inv, -fy * y * d_inv * d_inv;
                du_dpt0.row(2) << 0., 0., 1.;

                // pt0 = R10 * pt1 + t10;
                // pt0 + dpt0 = R10 * exp(drv) * pt1 + t10 + dt
                // pt0 + dpt0 = R10 * (I + drv^) * pt1 + t10 + dt
                // pt0 + dpt0 = R10 * pt1 + t10 + R10 * drv^ * pt1 + dt
                // dpt0 = R10 * drv^ * pt1 + dt
                // dpt0 = -R10 * pt1^ * drv + dt
                Eigen::Matrix<double, 3, 6> dpt0_dx;
                dpt0_dx.topLeftCorner(3, 3) = -R10 * utils::DMmath::SkewMatrix(pt1);
                dpt0_dx.topRightCorner(3, 3).setIdentity();

                Eigen::Matrix<double, 1, 6> jacob = dr_du * du_dpt0 * dpt0_dx;
                H += w * jacob.transpose() * jacob;
                b += w * jacob.transpose() * residual;
                cost += w * residual * residual;

                inlier_count++;
            }
        }
    }
    return inlier_count;
}


// [x; y; z] = z * K_inv * U
// P = [x; y; z]; P0 = R * P + t
// U0 = K * P0 / Z0
// e = G(U0) - Z0
void SlamSystem::DepthImageBasedMethod(const std::vector<cv::Mat> &images, std::vector<Eigen::Matrix<double, 1, 7>> imus,
                                       const std::vector<double> &timestamps) {
    std::cout << "******************direct method****************" << std::endl;
    // timer ticker
    utils::Timer timecount;
    timecount.Start();

    if (direct_.last_img.empty()) {
        direct_.last_img = images[0].clone();
        direct_.K << sensors_ptr_->tofcams[0].fx, 0, sensors_ptr_->tofcams[0].cx,
                     0., sensors_ptr_->tofcams[0].fy, sensors_ptr_->tofcams[0].cy,
                     0, 0, 1;
        direct_.K_inv << 1. / direct_.K(0, 0), 0., -direct_.K(0, 2) / direct_.K(0, 0),
                         0., 1. / direct_.K(1, 1), -direct_.K(1, 2) / direct_.K(1, 1),
                         0., 0., 1.;
        direct_.timestamp = timestamps[0];
        return;
    }

    // imu integration
    std::shared_ptr<Imu> imu_ptr = std::make_shared<Imu>(ba_, bg_);
    imu_ptr->PreIntegration(imus);
    LOG(INFO) << "imu preintegration Rji: \n" << imu_ptr->Rji();
    timecount.End("imu preintegration");

    std::cout << "Rji: " << imu_ptr->Rji() << std::endl;

    // tof prediction from imu
    Eigen::Matrix3d Rbw = direct_.Rbw;
    Eigen::Vector3d tbw = direct_.tbw;
    if (imu_ptr->Dt() > 0) {
        Eigen::Matrix3d Rbw1;
        Eigen::Vector3d tbw1, Vw1;
        // imu_ptr->ImuStatePrediction(Rbw_ * sensors_ptr_->imu.Rib, Vw_, tbw_, Eigen::Vector3d(0, 0, 0),
        //                             Rbw1, Vw1, tbw1);
        Rbw1 = Rbw1 * sensors_ptr_->imu.Rib.transpose();
        Rbw = Rbw1;
    }
    std::cout << "predict Rbw: " << Rbw << std::endl;

    Eigen::Matrix3d R10 = direct_.Rbw.transpose() * Rbw;
    Eigen::Vector3d t10 = direct_.Rbw.transpose() * (direct_.tbw - tbw);

    TofCamera &cam = sensors_ptr_->tofcams[0];
    Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> depth1((double*)images[0].data, cam.height, cam.width);
    Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> depth0((double*)direct_.last_img.data, cam.height, cam.width);

    std::cout << "R10: " << R10 << std::endl;
    std::cout << "t10: " << t10.transpose() << std::endl;
    int bank = 30;
    double max_err = -1;

    double fx = direct_.K(0, 0);
    double fy = direct_.K(1, 1);
    double cx = direct_.K(0, 2);
    double cy = direct_.K(1, 2);

    Eigen::Matrix<double, 6, 6> H; H.setZero();
    Eigen::Matrix<double, 6, 1> b; b.setZero();
    double cost = 0.;
    int inlier_count = DepthCostfunction(direct_.last_img, images[0], R10, t10, H, b, cost);

    std::cout << "init cost: " << cost << ", inlier count: " << inlier_count << std::endl;
    Eigen::Matrix<double, 6, 6> I6; I6.setIdentity();
    double lambda = 0.01;
    for (int i = 0; i < 7; i++) {
        Eigen::Matrix<double, 6, 1> dx = (H + lambda * I6).ldlt().solve(-b);

        Eigen::Matrix3d R10_hat = R10 * utils::DMmath::RotationVector2Matrix(dx.topRows(3));
        Eigen::Vector3d t10_hat = t10 + dx.bottomRows(3);
        Eigen::Matrix<double, 6, 6> H_hat; H_hat.setZero();
        Eigen::Matrix<double, 6, 1> b_hat; b_hat.setZero();
        double cost0 = 0.;
        inlier_count = DepthCostfunction(direct_.last_img, images[0], R10_hat, t10_hat, H_hat, b_hat, cost0);

        if (cost0 < cost) {
            cost = cost0;
            H = H_hat;
            b = b_hat;
            R10 = R10_hat;
            t10 = t10_hat;
            lambda = lambda < 1.0e-05? 1.0e-05 : lambda * 0.1;
        } else {
            lambda = lambda > 1000? 1000 : lambda * 10;
        }
        std::cout << "min cost and cost0: " << cost << "; " << cost0 << "; inlier:" << inlier_count << std::endl;
    }

    direct_.timestamp = timestamps[0];
    direct_.last_img = images[0].clone();
    direct_.Rbw = direct_.Rbw * R10;
    direct_.tbw = direct_.Rbw * t10 + direct_.tbw;
    direct_.Rbw = utils::DMmath::RotationNormalization(direct_.Rbw);
    std::cout << "direct Rbw: \n" << direct_.Rbw << std::endl;
    std::cout << "direc tbw: " << direct_.tbw.transpose() << std::endl;
}

void SlamSystem::AddPointCloud2Frame(const std::vector<cv::Mat> &images, double timestamp, std::shared_ptr<Frame> frame_ptr) {
    PointCloudFrame &pc_frame = frame_ptr->PointCloudFrameRef();
    current_points_.clear();

    int bank = 20;
    // point cloud: left hand coordinate: x: right; y: down; z: forward
    for (size_t i = 0; i < images.size(); i++) {
        TofCamera &cam = sensors_ptr_->tofcams[i];
        Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic,
           Eigen::RowMajor>> depths((double*)images[i].data, cam.height, cam.width);
        for (int r = bank + 0; r < cam.height - bank - 100; r++) {
            for (int c = bank; c < cam.width - bank; c++) {
                const double depth = depths(r, c);
                if (depth < cam.max_depth && depth > cam.min_depth) {
                    Eigen::Vector3d pt = cam.Depth2BodyPt(r, c, depth);
                    // if (pt(1) > 0.10) continue;
                    pc_frame.AddPoint(pt);
                    current_points_.push_back(pt);
                }
            }
        }
    }

    frame_ptr->SetTimeStamp(timestamp);
    LOG(INFO) << "griding voxel size: " << pc_frame.VoxelGridObject().Size();
}

void SlamSystem::AddPointCloud2Frame(const std::vector<std::vector<Eigen::Vector3d>> &points, std::shared_ptr<Frame> frame_ptr) {
    current_points_.clear();
    PointCloudFrame &pc_frame = frame_ptr->PointCloudFrameRef();
    Eigen::Matrix3d Rcb = sensors_ptr_->tofcams[0].Rcb;
    Eigen::Vector3d tcb = sensors_ptr_->tofcams[0].tcb;
    for (size_t i = 0; i < points.size(); i++) {
        TofCamera &cam = sensors_ptr_->tofcams[i];
        const std::vector<Eigen::Vector3d> &pts = points[i];
        const Eigen::Matrix3d &Rcb = cam.Rcb;
        const Eigen::Vector3d &tcb = cam.tcb;
        for (size_t j = 0; j < pts.size(); j++) {
            pc_frame.AddPoint(Rcb * pts[j] + tcb);
            current_points_.push_back(Rcb * pts[j] + tcb);
        }
        std::cout << "point num: " << pts.size() << std::endl;
    }

    LOG(INFO) << "griding voxel size: " << pc_frame.VoxelGridObject().Size();
}

void SlamSystem::GetDepthFeaturePoints(std::vector<Eigen::Vector3d> &points, std::vector<Eigen::Vector3d> &colors) {
    if (is_feature_based_) {

    } else {
        for (size_t i = 0; i < current_points_.size(); i++) {
            points.push_back(current_points_[i]);
            colors.push_back(Eigen::Vector3d(1, 0, 0));
        }
        // KL_odom_ptr_->GetCurrentFramePoints(points, colors);
    }

}

void SlamSystem::GetLocalFeaturesPoints(std::vector<Eigen::Vector3d> &points, std::vector<Eigen::Vector3d> &colors) {
    KL_odom_ptr_->GetLocalMapPoints(points, colors);
}

void SlamSystem::GetMatchedFeaturePoints(std::vector<Eigen::Vector3d> &points, std::vector<Eigen::Vector3d> &colors) {
    KL_odom_ptr_->GetMatchedPoints(points, colors);
}

void SlamSystem::GetGlobalMapoints(std::vector<Eigen::Vector3d> &points, std::vector<Eigen::Vector3d> &colors) {
    voxelmap_ptr_->GetGlobalMapPoints(points, colors);
}

void SlamSystem::GetLoopPoints(std::vector<Eigen::Vector3d> &pts, std::vector<Eigen::Vector3d> &colors) {
    if (!is_feature_based_) {
        voxelmapping_ptr_->GetLoopMatchedPoints(pts, colors);
    }
}

void SlamSystem::GetBackendMatchedPoints(std::vector<Eigen::Vector3d> &points, std::vector<Eigen::Vector3d> &colors) {
    // mapping_ptr_->GetMatchedPoints(points, colors);
    voxelmapping_ptr_->GetMatchedPoints(points, colors);
}

void SlamSystem::GetFramesPtis(std::vector<Eigen::Vector3d> &pts, std::vector<Eigen::Vector3d> &colors) {
    voxelmapping_ptr_->GetFramesPtis(pts, colors);
}

void SlamSystem::GetFramesPtjs(std::vector<Eigen::Vector3d> &pts, std::vector<Eigen::Vector3d> &colors) {
    voxelmapping_ptr_->GetFramesPtjs(pts, colors);
}

void SlamSystem::GetMatchedDirection(std::vector<Eigen::Vector3d> &points, std::vector<Eigen::Vector3d> &dirs) {
    if (!is_feature_based_) {
        KL_odom_ptr_->GetMatchedDirection(points, dirs);
    }
}

void SlamSystem::GetGroundPoints(std::vector<Eigen::Vector3d> &points, std::vector<Eigen::Vector3d> &colors) {
    KL_odom_ptr_->GetGroundPoints(points, colors);
}


bool SlamSystem::IsKeyFrame() {
    return is_keyframe_;
    return KL_odom_ptr_->IsKeyFrame();
}
} // namespace slam
