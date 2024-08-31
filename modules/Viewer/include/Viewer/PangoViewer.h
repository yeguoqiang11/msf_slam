#ifndef SRC_VIEWER_PANGOVIEWER_H
#define SRC_VIEWER_PANGOVIEWER_H
#include <eigen3/Eigen/Dense>
#include <pangolin/pangolin.h>
#include <opencv2/opencv.hpp>
#include <deque>
#include <thread>
#include <mutex>
#include <memory>
#include <atomic>
#include <chrono>


#include <slam/Shower.h>

#define MAX_POINTS_NUM 100000000
#define MAX_VOXELS_NUM 3000000

struct BBoxInfo {
    BBoxInfo() {}

    BBoxInfo(int id, float x, float y, float z, float l, float w, float h,
              float vx = 0, float vy = 0, float vz = 0,
              float r = 0.f, float g = 1.f, float b = 0.f)
            : id_(id), x_(x), y_(y), z_(z), l_(l), w_(w), h_(h),
              vx_(vx), vy_(vy), vz_(vz), r_(r), g_(g), b_(b) {}

    void SetVertices(std::vector<Eigen::Vector3f> vertices) {
        vertices_ = std::move(vertices);
    }

    int id_;
    float x_, y_, z_;
    float l_, w_, h_;
    float vx_, vy_, vz_;
    float r_, g_, b_;
    std::vector<Eigen::Vector3f> vertices_;
};


class PangoViewer {

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PangoViewer(int width = 1280, int height = 720);

    ~PangoViewer();

    static void Create() {
        if (instance_ == nullptr) {
            instance_ = std::make_shared<PangoViewer>();
        }
    }

    void Start();
    void Stop();
    void RenderProcess();

    // background color
    void SetBackground(const Eigen::Vector4f &color);

    // set shower object from slam
    void SetShower(std::shared_ptr<slam::Shower> shower_ptr) { shower_ptr_ = shower_ptr; }

    // axis helper
    void SetAxisHelper(const Eigen::Vector3f &origin, float length);

    // grid helper
    void SetGridHelper(const Eigen::Vector3f &origin, float grid_size = 0.5, int grid_num = 100);

    // camera pose
    void AddCamPose(const Eigen::Matrix4d &Twc,  const Eigen::Vector3f &color);
    void SetCamPoses(const std::vector<Eigen::Matrix4d> &poses, const std::vector<Eigen::Vector3f> &colors);
    void ClearCamPoses();

    // odometry pose
    void SetOdomPose(const Eigen::Matrix4d &Twb,  const Eigen::Vector3f &color);
    void ClearOdomPoses();

    // trajectoriy
    void AddPosition(const Eigen::Vector3f &pos);
    void SetTrajectory(const std::vector<Eigen::Vector3f> &trajectory, const Eigen::Vector3f &color);
    void SetTrajectoryColor(const Eigen::Vector3f &color);
    void ClearTrajectory();

    // trajectories
    void AddPositions(const std::vector<Eigen::Vector3f> &poses);
    void SetTrajectories(const std::vector<std::vector<Eigen::Vector3f>> &trajectories, const std::vector<Eigen::Vector3f> &colors);
    void SetTrajectoriesColorsAndSize(const std::vector<Eigen::Vector3f> &colors, size_t s);
    void ClearTrajectories();

    // planned path
    void SetPlannedPath(const std::vector<Eigen::Vector3f> &path, const Eigen::Vector3f &color);
    void ClearPlannedPath();

    // goal position
    void SetGoal(const Eigen::Vector3f &pos, const Eigen::Vector3f &color);
    void ClearGoals();

    // 3D object info
    void AddBBoxInfo(const BBoxInfo &bbox);
    void RemoveBBoxInfo(int id);
    void ClearBBoxInfos();

    // landmarks
    void SetLandmarks(const std::vector<Eigen::Vector3f> &landmarks, const std::vector<Eigen::Vector3f> &colors, float point_size = 1);
    void ClearLandmarks();

    // 3D point cloud
    void SetPointCloud(std::vector<float> &points, std::vector<float> &colors, float point_size = 2, bool move = true);
    void SetPointCloud(const std::vector<Eigen::Vector3d> &points, const std::vector<Eigen::Vector3d> &colors, float point_size = 2);
    void ClearPointCloud();

    void SetPointCloud1(const std::vector<Eigen::Vector3d> &points, const std::vector<Eigen::Vector3d> &colors, float point_size = 2);
    void SetPointCloud2(const std::vector<Eigen::Vector3d> &points, const std::vector<Eigen::Vector3d> &colors, float point_size = 2);
    void SetPointCloud3(const std::vector<Eigen::Vector3d> &points, const std::vector<Eigen::Vector3d> &colors, float point_size = 2);
    void SetPointCloud4(const std::vector<Eigen::Vector3d> &points, const std::vector<Eigen::Vector3d> &colors, float point_size = 2);

    // set Current 3D point cloud
    void SetCurrentPointCloud(const std::vector<Eigen::Vector3d> &points, const std::vector<Eigen::Vector3d> &colors, float point_size = 2);

    // set matched point and direction
    void SetMatchedPointCloud(const std::vector<Eigen::Vector3d> &points, const std::vector<Eigen::Vector3d> &dirs, float point_size = 2);

    // 3D voxels
    void SetVoxels(std::vector<float> &points, std::vector<float> &colors, float voxel_size = 0.1, bool move = true);
    void ClearVoxels();

    // Screen shot
    cv::Mat GetScreenShot();

    inline bool ShouldStop() {
        return shouldStop_;
    };

    static std::shared_ptr<PangoViewer> GetInstance() {
        return instance_;
    }

private:
    void DrawAxisHelper();

    void DrawGridHelper();

    void DrawCamPoses();

    void DrawOdomPoses();

    void DrawTrajectory();

    void DrawPlannedPath();

    void DrawBBoxInfos();

    void DrawLandmakrs();

    void DrawPointDirection();

    void DrawGoals();

    void DrawPointCloud();

    void DrawPointCloud1();

    void DrawPointCloud2();

    void DrawPointCloud3();

    void DrawPointCloud4();

    void DrawCurrentPointCloud();

    void DrawVoxels();

    void GetBboxVertices(const BBoxInfo &bbox, std::vector<Eigen::Vector3f> &vertices);

private:
    std::shared_ptr<slam::Shower> shower_ptr_;

    static std::shared_ptr<PangoViewer> instance_;

    // window size
    int width_;
    int height_;

    // background color
    Eigen::Vector4f backgroundColor_;

    // axis helper
    Eigen::Vector3f axisOrigin_;
    float axisLength_;

    // grid helper
    Eigen::Vector3f gridOrigin_;
    float gridSize_;
    int gridNum_;

    // camera poses
    std::vector<Eigen::Matrix4d> camPoses_;
    std::vector<Eigen::Vector3f> camPoseColors_;
    std::mutex camPoseMutex_;

    // odom poses
    std::deque<Eigen::Matrix4d> odomPoses_;
    Eigen::Vector3f odomPoseColor_;
    std::mutex odomPoseMutex_;

    // trajectory
    std::vector<Eigen::Vector3f> trajectory_;
    Eigen::Vector3f trajectoryColor_;
    std::mutex trajectoryMutex_;

    // trajectories
    std::vector<std::vector<Eigen::Vector3f>> trajectories_;
    std::vector<Eigen::Vector3f> trajectories_colors_;
    std::mutex trajectories_mutex_;

    // planned path
    std::vector<Eigen::Vector3f> plannedPath_;
    Eigen::Vector3f plannedPathColor_;
    std::mutex plannedPathMutex_;

    // goal positions
    std::deque<Eigen::Vector3f> goalPositions_;
    Eigen::Vector3f goalColor_;
    std::mutex goalMutex_;

    // 3D object infos
    std::map<int, BBoxInfo> bboxInfos_;
    std::mutex bboxInfoMutex_;

    // landmarks
    std::vector<Eigen::Vector3f> landmarks_;
    std::vector<Eigen::Vector3f> landmarkColors_;
    float landmarkSize_;
    std::mutex landmarkMutex_;

    // point cloud
    std::vector<float> pointCloud_;
    std::vector<float> pointCloudColors_;
    float pointSize_;
    std::atomic<bool> pointCloudUpdated_{false};
    std::mutex pointCloudMutex_;
    pangolin::GlBuffer pcVertexBuffer_;
    pangolin::GlBuffer pcColorBuffer_;

    // point cloud1
    std::vector<float> pointCloud1_;
    std::vector<float> pointCloudColors1_;
    float pointSize1_;
    std::atomic<bool> pointCloudUpdated1_{false};
    std::mutex pointCloudMutex1_;
    pangolin::GlBuffer pcVertexBuffer1_;
    pangolin::GlBuffer pcColorBuffer1_;

    // point cloud2
    std::vector<float> pointCloud2_;
    std::vector<float> pointCloudColors2_;
    float pointSize2_;
    std::atomic<bool> pointCloudUpdated2_{false};
    std::mutex pointCloudMutex2_;
    pangolin::GlBuffer pcVertexBuffer2_;
    pangolin::GlBuffer pcColorBuffer2_;

    // point cloud3
    std::vector<float> pointCloud3_;
    std::vector<float> pointCloudColors3_;
    float pointSize3_;
    std::atomic<bool> pointCloudUpdated3_{false};
    std::mutex pointCloudMutex3_;
    pangolin::GlBuffer pcVertexBuffer3_;
    pangolin::GlBuffer pcColorBuffer3_;

    // point cloud4
    std::vector<float> pointCloud4_;
    std::vector<float> pointCloudColors4_;
    float pointSize4_;
    std::atomic<bool> pointCloudUpdated4_{false};
    std::mutex pointCloudMutex4_;
    pangolin::GlBuffer pcVertexBuffer4_;
    pangolin::GlBuffer pcColorBuffer4_;

    // current frame point cloud
    std::vector<float> curPointCloud_;
    std::vector<float> curPointCloudColors_;
    float curPointSize_;
    std::atomic<bool> curPointCloudUpdated_{false};
    std::mutex curPointCloudMutex_;
    pangolin::GlBuffer pcCurVertexBuffer_;
    pangolin::GlBuffer pccurColorBuffer_;

    // matched direction and points;
    std::vector<Eigen::Vector3d> matchedPoints_;
    std::vector<Eigen::Vector3d> matchedDirections_;
    float matchPointSize_;
    std::atomic<bool> matchPointUpdated_{false};
    std::mutex matchPointMutex_;
    pangolin::GlBuffer pcMatchVertexBuffer_;

    // voxels
    std::vector<float> voxels_;
    std::vector<float> voxelColors_;
    std::vector<float> voxelQuads_;
    std::vector<float> voxelQuadColors_;
    float voxelSize_;
    std::atomic<bool> voxelUpdated_{false};
    std::mutex voxelMutex_;
    pangolin::GlBuffer voxelVertexBuffer_;
    pangolin::GlBuffer voxelColorBuffer_;

    // screen capture
    pangolin::Image<unsigned char> screenImageBuffer_;
    cv::Mat screenImage_;
    std::mutex screenImageMutex_;

    // renderer thread
    std::thread* viewerThread_;
    std::atomic<bool> shouldStop_{false};

};
#endif //SRC_VIEWER_PANGOVIEWER_H
