#include "VLO6dof/VLO6dof.hpp"

namespace vlo {
VLO6dof::VLO6dof(const VO_FrameToFrame_5point::Ptr &visual_odometer,
                 const LidarOdometerFrameToFrame::Ptr &lidar_odometer,
                 const Transform3D &camera_lidar_transform,
                 const Transform3D &initial_pose)
    : visual_odometer_(visual_odometer),
      lidar_odometer_(lidar_odometer),
      camera_lidar_transform_(camera_lidar_transform) {
  trajectory_.push_back(initial_pose);
}

vlo::Transform3D VLO6dof::getFrameToFrameTransformation() const {
  return trajectory_.end()[-2].inverse() * trajectory_.end()[-1];
}

void VLO6dof::calculateScaleFromGroundTruthVLO(const Vec3 &d_ground_truth) {
  scale_xyz_ = d_ground_truth;
  auto length = d_ground_truth.norm();
  scale_gt_ = length;
}

void VLO6dof::addVisualAndLidarFrames(IFrame::Ptr &current_frame_) {
  if (state_ == VLO_STATE::INIT) {
    addKeyFrame(current_frame_);
    state_ = VLO_STATE::RUNNING;

  } else if (state_ == VLO_STATE::RUNNING) {
    auto [frame_to_frame_transform, trust_result] =
        findFrameToFrameTransformBetween(current_frame_, reference_frame_);

    Transform3D current_pose = trajectory_.back() * frame_to_frame_transform;

    trajectory_.push_back(current_pose);
    scale_ = frame_to_frame_transform.translation().norm();
    scale_xyz_ = camera_lidar_transform_.rotation() *
                 frame_to_frame_transform.translation();
    addKeyFrame(current_frame_);
  }
}

std::tuple<Transform3D, bool> VLO6dof::findFrameToFrameTransformBetween(
    IFrame::Ptr &current_frame, IFrame::Ptr &reference_frame) {
  auto [frame_to_frame_transform_vo, trust_rotation] =
      visual_odometer_->findFrameToFrameTransformBetween(current_frame,
                                                         reference_frame);

  //    frame_to_frame_transform_vo.matrix().block<3, 1>(0, 3) =
  //        scale_ * frame_to_frame_transform_vo.matrix().block<3, 1>(0,
  //        3).array();
  if (verbose_) {
    std::cout << " before scaling " << std::endl;
    std::cout << frame_to_frame_transform_vo.matrix() << std::endl;
  }
  frame_to_frame_transform_vo.matrix().block<3, 1>(0, 3) =
      scale_xyz_.array() *
      frame_to_frame_transform_vo.matrix().block<3, 1>(0, 3).array();
  if (verbose_) {
    std::cout << "Frame to Frame vo " << std::endl;
    std::cout << frame_to_frame_transform_vo.matrix() << std::endl;

    std::cout << "scale_ " << std::endl;
    std::cout << scale_ << std::endl;
    std::cout << "scale xyz: " << std::endl;
    std::cout << scale_xyz_ << std::endl;
  }

  lidar_odometer_->hintForICP(camera_lidar_transform_.inverse() *
                              frame_to_frame_transform_vo *
                              camera_lidar_transform_);
  auto [frame_to_frame_transform_lo, trust_result] = lidar_odometer_->findFrameToFrameTransformBetween(current_frame, reference_frame);

  return {frame_to_frame_transform_lo, trust_result};
}

void VLO6dof::addKeyFrame(const IFrame::Ptr &frame) {
  pushFrameToHistory(frame);
  reference_frame_ = frame;
}
double VLO6dof::calculateCrossCheckingError(const IFrame::Ptr & current_frame, const IFrame::Ptr & reference_frame)
{

}
void VLO6dof::pushFrameToHistory(const IFrame::Ptr &camera_frame) {
  // map_->insertKeyFrame(frame);
  trajectory_nodes_.push_back(camera_frame);
  reference_frame_ = camera_frame;
}

};  // namespace vlo
