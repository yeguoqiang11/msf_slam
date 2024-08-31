#include <fstream>
#include <iostream>
#include <string>
#include <thread>

#include <eigen3/Eigen/Core>
#include <opencv2/opencv.hpp>

#include "data_io/DataIO.h"
#include "slam/Map.h"
#include "slam/Sensors.h"
#include "slam/SlamSystem.h"
#include "Viewer/PangoViewer.h"
#include "utils/logs.h"

using namespace DataIO;
using namespace DataIO::DataSetType;
using namespace slam;

int main(int argc, char** argv) {
    dmlog::Glogger log;

    std::string data_folder = argv[1];
    // std::string config_file = "../../config/Machine0Config1.json";
    std::string config_file = "../../config/OrbMachineNarrow.json";

    std::shared_ptr<DataIO::DataSetType::OrbMachineData> data_ptr =
        std::make_shared<DataIO::DataSetType::OrbMachineData>(data_folder);

    // slam object
    std::shared_ptr<SlamSystem> slam_ptr = std::make_shared<SlamSystem>(config_file, false);

    PangoViewer viewer;
    viewer.Start();
    viewer.SetShower(slam_ptr->ShowerPtr());

    cv::namedWindow("front", cv::WINDOW_NORMAL);

    double depth_scale = data_ptr->Scale();
    unsigned long count = 0;
    SlamSystem::SensorsData datas;
    std::vector<cv::Mat> images;
    double image_timestamp, image_timestamp_last = -1;
    while (data_ptr->GetImages(images, image_timestamp)) {
        LOG(INFO) << "-------------------" << count << ":" << image_timestamp << "--------------------";
        if (count < 0) {
            count++;
            continue;
        }
        image_timestamp += 1.55;
        // image_timestamp -= 0.25;

        // for show
        cv::Mat front_image;
        front_image = images[0].clone();
        front_image *= 0.3;

        datas.Initialization();
        datas.timestamp = image_timestamp;
        if (image_timestamp_last > 0) {
            if (data_ptr->GetImus(datas.imus, image_timestamp_last, image_timestamp)
                && data_ptr->GetWheels(datas.wheels, image_timestamp_last, image_timestamp)) {
                cv::Mat depth0;
                images[0].convertTo(depth0, CV_64F);
                depth0 *= depth_scale;
                datas.images.push_back(depth0);

                slam_ptr->LoadData(datas);
            } else {
                std::cout << "cannot get imu data" << std::endl;
                LOG(INFO) << "COULD NOT GET IMU DATA";
            }
        }

        // show ground points
        bool show_ground_points = false;
        if (show_ground_points) {
            std::vector<Eigen::Vector3d> gpts, gcolors;
            slam_ptr->GetGroundPoints(gpts, gcolors);
            viewer.SetPointCloud(gpts, gcolors, 2);
        }

        // show local map
        bool show_localvoxelmap = true;
        if (show_localvoxelmap) {
            std::vector<Eigen::Vector3d> local_fts, local_colors;
            slam_ptr->GetLocalFeaturesPoints(local_fts, local_colors);
            viewer.SetPointCloud(local_fts, local_colors, 2);
        } 

        // show current point cloud
        bool show_current_points = true;
        if (show_current_points) {
            std::vector<Eigen::Vector3d> feats, feat_colors;
            slam_ptr->GetDepthFeaturePoints(feats, feat_colors);
            const utils::Transform3d &Tbw = slam_ptr->CurrentPose();
            for (size_t i = 0; i < feats.size(); i++) {
                feats[i] = Tbw.transform(feats[i]);
            }
            viewer.SetCurrentPointCloud(feats, feat_colors, 2.0);
        }

        // show matched point cloud 
        // matched: low blue -> red; green -> blue
        bool show_local_matched_points = true;
        if (show_local_matched_points) {
            std::vector<Eigen::Vector3d> matched_feats, matched_colors;
            slam_ptr->GetMatchedFeaturePoints(matched_feats, matched_colors);
            viewer.SetPointCloud1(matched_feats, matched_colors);
        }

        // show global point cloud
        bool show_global_map = true;
        if (show_global_map) {
            std::vector<Eigen::Vector3d> global_pts, global_colors;
            slam_ptr->GetGlobalMapoints(global_pts, global_colors);
            viewer.SetPointCloud2(global_pts, global_colors);
        }

        // show backend matched points
        // blue->red
        bool show_backend_matchedpoints = false;
        if (show_backend_matchedpoints) {
            std::vector<Eigen::Vector3d> global_matched_pts, global_matched_colors;
            slam_ptr->GetBackendMatchedPoints(global_matched_pts, global_matched_colors);
            viewer.SetCurrentPointCloud(global_matched_pts, global_matched_colors);
        }

        bool show_loop_matchedpoints = true;
        if (show_loop_matchedpoints) {
            std::vector<Eigen::Vector3d> ptis, ptjs, coloris, colorjs;
            slam_ptr->GetFramesPtis(ptis, coloris);
            viewer.SetPointCloud3(ptis, coloris);

            slam_ptr->GetFramesPtjs(ptjs, colorjs);
            viewer.SetPointCloud4(ptjs, colorjs);
        }

        std::cout << "-------------------" << count << ":" << image_timestamp << "--------------------" << std::endl;
        // draw pose
        const utils::Transform3d Tbw0 = slam_ptr->CurrentPose();
        Eigen::Matrix4d Tbw;
        Tbw.topLeftCorner(3, 3) = Tbw0.R();
        Tbw.topRightCorner(3, 1) = Tbw0.t();
        viewer.SetOdomPose(Tbw, Eigen::Vector3f(1, 0, 0));
        viewer.AddPosition(Tbw0.t().cast<float>());

        front_image.convertTo(front_image, CV_8UC1);
        cv::imshow("front", front_image);
        bool is_key = slam_ptr->IsKeyFrame();
        if (is_key) {
            cv::waitKey(0);
        } else {
            cv::waitKey(10);
        }
        // cv::waitKey(0);

        image_timestamp_last = image_timestamp;
        count++;
    }

    viewer.Stop();
}