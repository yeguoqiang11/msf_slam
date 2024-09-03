# msf_slam
play it

# 1. Introduction
msf slam is a project of integrating multi-sensors to generate localization and 3D map, it may use laser, imu, camera and maybe tof as well. we start develop by using laser, then extend it to multi-sensor.
we hope it is a fusion library supportting a lot of sensors.

the project also try a new way of odometry by directory estimating pose from depth image. it also include wheel intrinsic and extrinsic calibration. we use wheel preintegration to optimize pose graph. we try euler model, arc model, and so on to test which it is best.

to increase computation of slam, we try a lot include svd optimization, map saving, map indexing. we use map by unordered_map instead of tree(which we have test as well).

# 2. Prerequisites
we use at least C++14 and test it on ubuntu 18.04
## OpenCV
we use opencv to manipulate images and features. Download and install instructions can be found at: http://opencv.org. **Required at least 3.4. Tested with OpenCV 3.4.11.**
## Eigen3
we use eigen3 to manipulate matrix and vector. Download and install instructions can be found at http://eigen.tuxfamily.org. **Tested with eigen 3.2.10**
## GTSAM
use gtsam to optimize trajectory, map and so on. Download and install instructions can be found at https://github.com/borglab/gtsam.
## Pangolin
use Pangolin to show 3D map and trajectory. Download and install instructions can be found at https://github.com/stevenlovegrove/Pangolin.

# 3. Building Projectory
Clone the repository
```
mkdir build
cd build
cmake ..
make -j8
```

# 4. Run Dataset
## Tum Dataset
```
cd examples
./tum_run
```
