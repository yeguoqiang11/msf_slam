#include "slam/Shower.h"

namespace slam {
Shower::Shower(std::shared_ptr<GlobalVoxelMap> map_ptr, std::shared_ptr<Sensors> sensors_ptr)
  : map_ptr_(map_ptr), sensors_ptr_(sensors_ptr) {

}

void Shower::GetTrajectory(std::vector<Eigen::Vector3f> &traj) {
    std::unique_lock<std::mutex> locker(mtx_);
    std::vector<std::shared_ptr<Frame>> &keyframes = map_ptr_->KeyFrames();
    for (int i = 0; i < keyframes.size(); i++) {
        traj.push_back(keyframes[i]->Tbw().t().cast<float>());
    }
}

void Shower::GetFrameMap(int idx, Eigen::Vector3d color, std::vector<Eigen::Vector3d> &pts, std::vector<Eigen::Vector3d> &colors) {
    pts.clear();
    colors.clear();
    if (idx < 0) return;
    if (idx > static_cast<int>(map_ptr_->FrameSize()) - 1) return;
    utils::Transform3d Tbw = map_ptr_->KeyFrames()[idx]->Tbw();
    VoxelGrid grid = map_ptr_->KeyFrames()[idx]->PointCloudFrameRef().VoxelGridObject();
    VoxelGrid::MapType::iterator iter = grid.Begin();
    for (; iter != grid.End(); iter++) {
        pts.push_back(Tbw.transform(iter->second.center));
        colors.push_back(color);
    }
}
} // namespace slam