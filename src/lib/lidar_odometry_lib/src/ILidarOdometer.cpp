#include "ILidarOdometer.hpp"

namespace vlo {
ILidarOdometer::ILidarOdometer() {}
ILidarOdometer::~ILidarOdometer() {}

Transform3D ILidarOdometer::getLastPose() const {
  return current_lidar_frame_->pose_;
}

Transform3D ILidarOdometer::getFrameToFrameTransformation() const {
  if (history_frames_.size() < 2)
  {
    std::cout << "returning identity " << std::endl;
    Transform3D frameToframeTf = Transform3D::Identity();
    return frameToframeTf;
  }
  return history_frames_.end()[-2]->pose_.inverse() *
         history_frames_.end()[-1]->pose_;
}

void ILidarOdometer::pushFrameToHistory(IFrame::Ptr lidar_frame) {
  // map_->insertKeyFrame(frame);
  history_frames_.push_back(lidar_frame);
  reference_lidar_frame_ = lidar_frame;
  std::cout << "history frames: " << history_frames_.size() << std::endl;
  if (history_frames_.size() > 10)
  {
      history_frames_.erase(history_frames_.begin());
  }
}

void ILidarOdometer::takeLocalMap(PointCloud & map)
{

    for ( auto & frame : history_frames_)
    {

        map = map + frame->point_cloud_;
    }
}

void ILidarOdometer::hintForICP(const Transform3D &)
{}

};  // namespace vlo
