#include <iostream>
#include <memory>

#include "slam/WheelModel.h"
#include "slam/VoxelMap.h"
#include "utils/DMmath.h"

struct Test {
    int data0 = 1;
    float data1 = 2;
    double data2 = 3;
};
using namespace slam;
int main(int argc, char** argv) {
    std::vector<Eigen::Vector3d> pts;
    Eigen::Vector3d mean_pt(0., 0., 0.);
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 9; j++) {
            Eigen::Vector3d pt;
            pt << i * 0.02, j * 0.005, 0.00001 * (i + j);
            pts.push_back(pt);
            mean_pt += pt;
        }
    }

    mean_pt /= pts.size();
    Eigen::Matrix3d cov; cov.setZero();
    for (size_t i = 0; i < pts.size(); i++) {
        Eigen::Vector3d pt = pts[i] - mean_pt;
        cov += pt * pt.transpose();
    }
    cov /= pts.size() - 1;
    std::cout << "mean pt: " << mean_pt.transpose() << std::endl;
    std::cout << "cov: \n" << cov << std::endl;
    Eigen::Matrix3d Vt;
    Eigen::Vector3d eig;
    utils::DMmath::Matrix3dEigenDecomposition(cov, eig, Vt);
    Eigen::Vector3d v0, v1;
    double l0, l1;
    std::cout << "eig: " << eig.transpose() << std::endl;
    std::cout <<"sqrt eig: " << sqrt(eig(0)) << "," << sqrt(eig(1)) << "," << sqrt(eig(2)) << std::endl;
    std::cout << "Vt: \n" << Vt << std::endl;
    std::cout <<"sqrt : " << std::pow(0.035 / 3, 2) << "," << std::pow(0.02 / 3.5, 2) << std::endl;

    slam::VoxelTreeMap map(0.5);
    slam::VoxelTree::subvoxel_size_ = 0.1;
    for (size_t i = 0; i < 100; i++) {
        for (size_t j = 0; j < 100; j++) {
            Eigen::Vector3d pt;
            pt << i * 0.01, j * 0.03, 0.0001 * (i + j);
            map.AddPoint(pt);
        }
    }
    slam::VoxelTreeMap::MapType::iterator iter = map.begin();
    Eigen::Vector3d nv;
    for (; iter != map.end(); iter++) {
        for (int i = 0; i < iter->second.Size(); i++) {
            auto &voxel = iter->second.Voxel(i);
            if (iter->second.Voxel(i).N > 10) {
                Eigen::Matrix3d cov0 = voxel.cov;
                utils::DMmath::Matrix3dEigenDecomposition(cov0, eig, Vt);
                // std::cout << "eig: " << eig.transpose() << std::endl;
                // // std::cout << "Vt: \n" << Vt << std::endl;
                // voxel.EigenCal();
                // if (voxel.GetEigenInfo(eig, nv)) {
                //     std::cout << "eig0: " << eig.transpose() << std::endl;
                // }
                // std::cout << "-------------------------" << std::endl;
            }
        }
    }

    return 0;
    double radius = 0.05;
    double d = 0.10;
    double nl = 0.001;
    double nr = 0.001;
    std::shared_ptr<EulerWMPreintegration> wheel_ptr =
        std::make_shared<EulerWMPreintegration>(radius, radius, d, nl, nr);

    double dt = 0.01;
    double w = 20. * DEG2RAD;
    double v = w * radius;
    for (size_t i = 0; i < 1; i++) {
        Eigen::Vector2d wh(v, v);
        wheel_ptr->LoadWheelData(wh, dt);
    }
    std::cout << "v: " << v << std::endl;
    std::cout << "alphaij: " << wheel_ptr->Alpha().transpose() << std::endl;
    std::cout << "cov: \n" << wheel_ptr->Cov() << std::endl;
}