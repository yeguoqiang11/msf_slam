#include <bitset>
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
    std::string config_file = "../../config/OfilmMachine.json";

    std::shared_ptr<DataIO::DataSetType::OfilmMachine> data_ptr =
        std::make_shared<DataIO::DataSetType::OfilmMachine>(data_folder);

    // slam object
    std::shared_ptr<SlamSystem> slam_ptr = std::make_shared<SlamSystem>(config_file, false);

    // create show window
    PangoViewer viewer;
    viewer.Start();
    cv::namedWindow("depth", cv::WINDOW_NORMAL);

    unsigned long count = 0;
    SlamSystem::SensorsData datas;
    double timestamp, timestamp_last = -1;
    std::vector<cv::Mat> images;
    while (data_ptr->GetPointCloud(datas.points, timestamp) || data_ptr->GetDepthImage(images, timestamp)) {
        LOG(INFO) << "-------------------" << count << ":" << timestamp << "--------------------";
        if (count < 0) {
            count++;
            continue;
        }

        cv::Mat tmp;
        for (size_t i = 0; i < images.size(); i++) {
            images[i].convertTo(tmp, CV_64FC1);
            datas.images.push_back(tmp.clone() * 1.0e-03);
        }

        datas.timestamp = timestamp;
        if (timestamp_last > 0) {
            if (data_ptr->GetImuAndWheelData(datas.imus, datas.wheels, timestamp_last, timestamp)) {
                slam_ptr->LoadData(datas);
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
        bool show_localvoxelmap = false;
        if (show_localvoxelmap) {
            // viewer.ClearPointCloud();
            std::vector<Eigen::Vector3d> local_fts, local_colors;
            slam_ptr->GetLocalFeaturesPoints(local_fts, local_colors);
            viewer.SetPointCloud(local_fts, local_colors, 2);
        }

        std::vector<Eigen::Vector3d> feats, feat_colors;
        // show current point cloud
        bool show_current_points = true;
        if (show_current_points) {
            feats.clear();
            feat_colors.clear();
            slam_ptr->GetDepthFeaturePoints(feats, feat_colors);
            const utils::Transform3d &Tbw = slam_ptr->CurrentPose();
            for (size_t i = 0; i < feats.size(); i++) {
                feats[i] = Tbw.transform(feats[i]);
            }

            viewer.SetCurrentPointCloud(feats, feat_colors, 2.0);
        }

        // show matched point cloud 
        // matched: low blue -> red; green -> blue
        bool show_local_matched_points = false;
        if (show_local_matched_points) {
            std::vector<Eigen::Vector3d> matched_feats, matched_colors;
            slam_ptr->GetMatchedFeaturePoints(matched_feats, matched_colors);
            feats.insert(feats.end(), matched_feats.begin(), matched_feats.end());
            feat_colors.insert(feat_colors.end(), matched_colors.begin(), matched_colors.end());
            viewer.SetCurrentPointCloud(feats, feat_colors);
        }

        // show global point cloud
        bool show_global_map = true;
        if (show_global_map) {
            std::vector<Eigen::Vector3d> global_pts, global_colors;
            slam_ptr->GetGlobalMapoints(global_pts, global_colors);
            viewer.SetPointCloud(global_pts, global_colors);
        }

        // show backend matched points
        // blue->red
        bool show_backend_matchedpoints = false;
        if (show_backend_matchedpoints) {
            std::vector<Eigen::Vector3d> global_matched_pts, global_matched_colors;
            slam_ptr->GetBackendMatchedPoints(global_matched_pts, global_matched_colors);
            feats.insert(feats.end(), global_matched_pts.begin(), global_matched_pts.end());
            feat_colors.insert(feat_colors.end(), global_matched_colors.begin(), global_matched_colors.end());
            viewer.SetCurrentPointCloud(feats, feat_colors);
        }

        bool show_loop_matchedpoints = false;
        if (show_loop_matchedpoints) {
            std::vector<Eigen::Vector3d> loop_pts, loop_colors;
            slam_ptr->GetLoopPoints(loop_pts, loop_colors);
            feats.insert(feats.end(), loop_pts.begin(), loop_pts.end());
            feat_colors.insert(feat_colors.end(), loop_colors.begin(), loop_colors.end());
            viewer.SetCurrentPointCloud(feats, feat_colors);
        }

        std::cout << "-------------------" << count << ":" << timestamp << "--------------------" << std::endl;
        // draw pose
        const utils::Transform3d &Tbw0 = slam_ptr->CurrentPose();
        Eigen::Matrix4d Tbw;
        Tbw.topLeftCorner(3, 3) = Tbw0.R();
        Tbw.topRightCorner(3, 1) = Tbw0.t();
        viewer.SetOdomPose(Tbw, Eigen::Vector3f(1, 0, 0));
        viewer.AddPosition(Tbw0.t().cast<float>());

        if (!images.empty()) {
            cv::flip(images[0], images[0], 0);
            cv::flip(images[0], images[0], 1);
            cv::imshow("depth", images[0] * 12);
            cv::waitKey(0);
        } else getchar();
        bool is_key = slam_ptr->IsKeyFrame();

        timestamp_last = timestamp;
        datas.Initialization();
        count++;
    }
    viewer.Stop();
    return 0;
}