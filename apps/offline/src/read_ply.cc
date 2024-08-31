#include <iostream>
#include <string>
#include <vector>

#include "data_io/DataIO.h"
#include "Viewer/PangoViewer.h"

class Test {
  public:
    template<class T = double>
    static Eigen::Matrix<T, 3, 3> SkewMatrix(const Eigen::Ref<const Eigen::Matrix<T, 3, 1>> vec);
    static Eigen::Matrix3d SkewMatrix1(const Eigen::Vector3d &vec);
    template<class T = double>
    static T Print(const T &a);
    template<class T = double>
    static Eigen::Matrix<T, 3, 1> Print1(const Eigen::Matrix<T, 3, 1> &vec);
};

template<class T>
Eigen::Matrix<T, 3, 3> Test::SkewMatrix(const Eigen::Ref<const Eigen::Matrix<T, 3, 1>> vec) {
    Eigen::Matrix<T, 3, 1> out;
    out << T(0.), -vec(2), vec(1),
           vec(2), T(0.), -vec(0),
           -vec(1), vec(0), T(0.);

    return std::move(out);
}

template<class T = double>
Eigen::Matrix<T, 3, 1> Test::Print1(const Eigen::Matrix<T, 3, 1> &vec) {
    Eigen::Matrix<T, 3, 1> out;
    out.row(0) << T(0.0), -vec(2), vec(1);
    return out;
}

template<class T>
T Test::Print(const T &a) {
    std::cout << "a: " << a << std::endl;
    return a + 1;
}

Eigen::Matrix3d Test::SkewMatrix1(const Eigen::Vector3d &vec) {
    Eigen::Matrix3d out;
    out << 0., -vec(2), vec(1),
           vec(2), 0., -vec(0),
           -vec(1), vec(0), 0.;

    return std::move(out);
}


int main(int argc, char** argv) {
    std::string file_path = "/home/yegq/0data/tofsmahcine0/TestRoom0408.ply";
    std::vector<Eigen::Vector3d> pts, colors;
    DataIO::DataSetType::ReadPlyPoints(file_path, pts, colors);
    PangoViewer viewer;
    viewer.Start();

    std::vector<Eigen::Vector3d> pt1s, color1s;
    for (int i = 0; i < pts.size(); i++) {
        if (i % 100 == 0) {
            pt1s.push_back(pts[i]);
            color1s.push_back(colors[i]);
        }
    }
    std::cout << "pts size: " << pt1s.size() << std::endl;
    std::cout << "color size: " << color1s.size() << std::endl;

    for (int i = 0; i < pt1s.size(); i++) {
        std::cout << i << "th pt1s: " << pt1s[i].transpose() << std::endl;
    }
    viewer.SetCurrentPointCloud(pt1s, color1s, 2.0f);
    
    char buf[255];
    std::cin >> buf;
}