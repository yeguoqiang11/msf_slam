#include <fstream>
#include <iostream>
#include <string>
#include <thread>
#include <unistd.h>

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

void GetPoinstFromLine(const Eigen::Vector3d &pt0, const Eigen::Vector3d &pt1, std::vector<Eigen::Vector3d> &pts, std::vector<Eigen::Vector3d> &colors);
void GetDiffPoints(std::vector<Eigen::Vector3d> &pts0, std::vector<Eigen::Vector3d> &pts1, std::vector<Eigen::Vector3d> &diffs);

int main(int argc, char** argv) {
    VoxelTree::subvoxel_size_ = 0.2;
    VoxelTreeMap map(0.8);
    Eigen::Vector3d p(0., 0., 0.);
    Eigen::Vector3d pt;
    for (int i = -300; i < 300; i++) {
        for (int j = -200; j < 300; j++) {
            for (int k = -200; k < 200; k++) {
                pt << -i * 0.002, -j * 0.002, -0.3 - 0.002 * k;
                map.AddPoint(pt);
            }
        }
    }

    Eigen::Vector3d pt1(-1., 1.0, -2);
    // pt1 << 0.191139954, 0.04293291213, 0.2247196317;
    Eigen::Vector3d pt0(0., 0., 0.);
    // pt0 << 0.0003407842768, 0, -0.01531654856;

    map.RayCastingOccupancyUpdation(pt0, pt1);

    int pt_n = 0;
    utils::Timer timer;
    // for (int i = -200; i < 200; i++) {
    //     for (int j = -200; j < 200; j++) {
    //         pt << -i * 0.002, -j * 0.002, -0.95;
    //         map.RayCastingOccupancyUpdation(pt0, pt);
    //         map.AddPoint(pt);
    //         pt_n++;
    //     }
    // }
    double dt = timer.End("add 100 0000 pts");
    std::cout << "cost time per 100 points: " << 300. * dt / pt_n << std::endl;

    // show map points
    std::vector<Eigen::Vector3d> pts, colors;
    std::vector<Eigen::Vector3d> indexed_pts;
    VoxelTreeMap::MapType::iterator iter1 = map.begin();
    for (; iter1 != map.end(); iter1++) {
        for (size_t i = 0; i < iter1->second.Size(); i++) {
            int N = iter1->second.Voxel(i).N;
            if (N >= 3 && N < 900) {
                pts.push_back(iter1->second.Voxel(i).center);
                colors.push_back(Eigen::Vector3d(0, 1, 0));
            } else if (N == 1000) {
                // raycasted points
                pts.push_back(iter1->second.Voxel(i).center);
                colors.push_back(Eigen::Vector3d(1., 0., 0.));
                indexed_pts.push_back(iter1->second.Voxel(i).center);
                // std::cout << "raycasting res: " << iter1->second.Voxel(i).center.transpose() << std::endl;
            }
        }
    }
    pts.push_back(pt0);
    colors.push_back(Eigen::Vector3d(1., 0., 0.));
    pts.push_back(pt1);
    colors.push_back(Eigen::Vector3d(1., 0., 0.));

    // indexing points from begin and end of line
    std::vector<Eigen::Vector3d> tmp_pts, tmp_colors;
    std::vector<Eigen::Vector3d> indexs_pts;
    GetPoinstFromLine(pt0, pt1, tmp_pts, tmp_colors);
    for (int i = 0; i < tmp_pts.size(); i++) {
    //    VoxelTreeMap::VoxelType *res = map.findNeighborsByHash(tmp_pts[i]);
    //    if (res != nullptr) {
    //         indexs_pts.push_back(res->center);
    //         pts.push_back(res->center);
    //         colors.push_back(Eigen::Vector3d(1., 0., 0.));
    //         // std::cout << "line search result: " << res->center.transpose() << std::endl;
    //    }

       pts.push_back(tmp_pts[i]);
       colors.push_back(tmp_colors[i]);
    }

    PangoViewer viewer;
    viewer.Start();
    viewer.SetPointCloud(pts, colors, 2);

    pause();
}

void GetPoinstFromLine(const Eigen::Vector3d &pt0, const Eigen::Vector3d &pt1,
                       std::vector<Eigen::Vector3d> &pts, std::vector<Eigen::Vector3d> &colors) {
    double len = 0.03;
    Eigen::Vector3d dpt = pt1 - pt0;
    int n = dpt.norm() / len;
    dpt.normalize();

    for (int i = 0; i < n; i++) {
        pts.push_back(pt0 + i * len * dpt);
        colors.push_back(Eigen::Vector3d(0., 0., 1.));
    }
}

void GetDiffPoints(std::vector<Eigen::Vector3d> &pts0, std::vector<Eigen::Vector3d> &pts1, std::vector<Eigen::Vector3d> &diffs) {
    for (int i = 0; i < pts0.size(); i++) {
        bool is_same = false;
        for (int j = 0; j < pts1.size(); j++) {
            if ((pts0[i] - pts1[j]).norm() < 0.01) {
                is_same = true;
                break;
            }
        }
        if (!is_same) {
            diffs.push_back(pts0[i]);
        }
    }

    for (int i = 0; i < pts1.size(); i++) {
        bool is_same = false;
        for (int j = 0; j < pts0.size(); j++) {
            if ((pts0[j] - pts1[i]).norm() < 0.01) {
                is_same = true;
                break;
            }
        }
        if (!is_same) {
            diffs.push_back(pts1[i]);
        }
    }
}

