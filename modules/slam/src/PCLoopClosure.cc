#include "slam/PCLoopClosure.h"

namespace slam {
VoxelMapLoopClosure::VoxelMapLoopClosure(std::shared_ptr<GlobalVoxelMap> map_ptr): map_ptr_(std::move(map_ptr)) {
    min_depth_ = 0.1;
    max_depth_ = 6;
    bin_size_ = 0.2;

    // contour pyriamd maps
    pyramid_maps_ptr_ = std::make_shared<ContourMapStack>(0.05, 1.60, 5);
    indexing_ptr_ = std::make_shared<BranchAndBound>(pyramid_maps_ptr_, 0.1);
}

void VoxelMapLoopClosure::InsertFrame(std::shared_ptr<Frame> frame_ptr) {   
    int size = map_ptr_->FrameSize();
    std::vector<std::shared_ptr<Frame>> &frames = map_ptr_->KeyFrames();

    std::cout << "loop closure timestamp: " << frame_ptr->TimeStamp() << std::endl;

    // loop closure
    Eigen::Matrix3d Rbw;
    Eigen::Vector3d tbw;
    frame_ptr->GetPose(Rbw, tbw);

    // if (frame.TimeStamp() > 246 && frame.TimeStamp() < 280) {
        // std::cout << "relocalization indexing!!!" << std::endl;
        // indexing_ptr_->PoseIndexing(frame->PointCloudFrameRef().VoxelGridObject(), Rbw, tbw);
    // }

    // adding grid to z-projection map
    frame_ptr->GetPose(Rbw, tbw);
    pyramid_maps_ptr_->AddGrid(frame_ptr->PointCloudFrameRef().VoxelGridObject(), Rbw, tbw);
    // pyramid_maps_ptr_->MapShow();
}

void VoxelMapLoopClosure::CalculateCentroidAndPCA(std::shared_ptr<Frame> frame_ptr) {
    Eigen::Vector3d mean(0, 0, 0);
    int count = 0;

    // grid_width
    int grid_size = std::ceil(2.0 * max_depth_ / bin_size_);
    double num = grid_size * grid_size;
    std::vector<double> max_heights(num, -100);

    VoxelGrid::MapType::iterator iter = frame_ptr->PointCloudFrameRef().VoxelGridObject().Begin();
    for (; iter != frame_ptr->PointCloudFrameRef().VoxelGridObject().End(); iter++) {
        mean += iter->second.center * iter->second.N;
        count += iter->second.N;
    }
    mean /= count;
}

} // namespace slam