#include "IVisualOdometer.hpp"


namespace vlo {

IVisualOdometer::IVisualOdometer(const Transform3D &initial_pose)
    : initial_pose_(initial_pose) {}

IVisualOdometer::~IVisualOdometer() {}

Transform3D IVisualOdometer::getLastPose() const {
  return current_frame_->pose_;
}

void IVisualOdometer::pushFrameToHistory(IFrame::Ptr &camera_frame) {
  // map_->insertKeyFrame(frame);
  history_frames_.push_back(camera_frame);
  reference_frame_ = camera_frame;
  std::cout << "history frames: " << history_frames_.size() << std::endl;
  if (history_frames_.size() > 3)
  {
      history_frames_.erase(history_frames_.begin());
  }
}

Transform3D IVisualOdometer::getFrameToFrameTransformation() const {
  if (history_frames_.size() < 2)
  {
    std::cout << "returning identity " << std::endl;
    return Transform3D::Identity();
  }
  return history_frames_.end()[-2]->pose_.inverse() *
         history_frames_.end()[-1]->pose_;
}
void IVisualOdometer::calculateScale(const Vec3 &)
{}

};  // namespace vlo
