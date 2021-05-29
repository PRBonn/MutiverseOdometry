#include "HuangsVLO/HuangVLO.hpp"

#include "common_vlo.hpp"
namespace vlo {
HuangVLO::HuangVLO(const VO_FrameToFrame_5point::Ptr &visual_odometer,
                   const Transform3D &initial_pose)
    : visual_odometer_(visual_odometer){
  initial_pose_ = initial_pose;
  trajectory_.push_back(initial_pose);
}

vlo::Transform3D HuangVLO::getFrameToFrameTransformation() const {
  return trajectory_.end()[-2].inverse() * trajectory_.end()[-1];
}

void HuangVLO::addVisualAndLidarFrames(IFrame::Ptr &current_frame_) {
  if (state_ == VLO_STATE::INIT) {
    current_frame_->pose_ = initial_pose_;
    addKeyFrame(current_frame_);
    state_ = VLO_STATE::RUNNING;

  } else if (state_ == VLO_STATE::RUNNING) {
    auto [frame_to_frame_transform, trust_result] =
        findFrameToFrameTransformBetween(current_frame_, reference_frame_);
    Transform3D current_pose = trajectory_.back();
    if (trust_result) {
      current_pose = trajectory_.back() * frame_to_frame_transform;
      current_frame_->pose_ = current_pose;
    } else {
      std::cout << "Constant Velocity model!" << std::endl;

      auto last_transform = getFrameToFrameTransformation();
      if (verbose_) {
        std::cout << " last transform " << std::endl;
        std::cout << last_transform.matrix() << std::endl;
      }
      last_transform.translation() = last_transform.translation().normalized();
      last_transform.matrix().block<3, 3>(0, 0) = Mat33::Identity();
      lidar_odometer_.directionHintICP(last_transform);
      auto scale = lidar_odometer_.getScale(current_frame_, reference_frame_);
      frame_to_frame_transform = last_transform;
      frame_to_frame_transform.translation() =
          scale * last_transform.translation();
      if (verbose_) {
        std::cout << "frame to frame tf" << std::endl;
        std::cout << frame_to_frame_transform.matrix() << std::endl;
        std::cout << "reference frame pose " << std::endl;
        std::cout << reference_frame_->pose_.matrix() << std::endl;
      }
      current_pose = reference_frame_->pose_ * frame_to_frame_transform;
      if (verbose_) {
        std::cout << "Current pose: " << std::endl;
        std::cout << current_pose.matrix() << std::endl;
      }
      Eigen::Quaterniond q_current(current_pose.rotation().matrix());
      q_current.normalize();
      current_pose.matrix().block<3, 3>(0, 0) = q_current.toRotationMatrix();
      current_frame_->pose_ = current_pose;
    }
    std::cout << "Huang's transform: " << frame_to_frame_transform.matrix() << std::endl;
    trajectory_.push_back(current_pose);
    addKeyFrame(current_frame_);
  }
}

std::tuple<Transform3D, bool> HuangVLO::findFrameToFrameTransformBetween(
    IFrame::Ptr &current_frame, IFrame::Ptr &reference_frame) {
  std::cout << "Huang's method looks for transformation!" << std::endl;
  auto [frame_to_frame_transform, trust_rotation] =
      visual_odometer_->findFrameToFrameTransformBetween(current_frame,
                                                         reference_frame);
  if (trust_rotation) {
    lidar_odometer_.directionHintICP(frame_to_frame_transform);
    auto scale = lidar_odometer_.getScale(current_frame, reference_frame);
    if (verbose_) {
      std::cout << "Huang's scale " << std::endl;
      std::cout << scale << std::endl;
      std::cout << "frame to frame translation" << std::endl;
      std::cout << frame_to_frame_transform.translation() << std::endl;
    }
    frame_to_frame_transform.translation() =
        scale * frame_to_frame_transform.translation();
    if (verbose_) {
      std::cout << "frame to frame translation, after scaling" << std::endl;
      std::cout << frame_to_frame_transform.translation() << std::endl;
    }
      std::cout << "trust rotation?" << std::endl;
    return {frame_to_frame_transform, trust_rotation};
  }
  if (verbose_) {
    std::cout << " UNTRUSTED! " << std::endl;
  }
  frame_to_frame_transform = Transform3D::Identity();
  if (verbose_) {
    std::cout << frame_to_frame_transform.translation() << std::endl;
  }

  return {frame_to_frame_transform, trust_rotation};
}

Transform3D HuangVLO::getLastPose() const { return trajectory_.back(); }

void HuangVLO::addKeyFrame(const IFrame::Ptr &frame) {
  pushFrameToHistory(frame);
  reference_frame_ = frame;
}

double HuangVLO::calculateCrossCheckingError(const IFrame::Ptr & current_frame, const IFrame::Ptr & reference_frame)
{
    std::cout << "huang!" << std::endl;
    return 2.0;
}


void HuangVLO::pushFrameToHistory(const IFrame::Ptr &camera_frame) {
  // map_->insertKeyFrame(frame);
  trajectory_nodes_.push_back(camera_frame);
  reference_frame_ = camera_frame;
}

OdometerStruct getPointerAndOrientationToHuangsMethodAndInitialize(
    const vlo::VO_FrameToFrame_5point::Ptr &vo_5dof,
    const Transform3D &reference_pose, bool debug, bool verbose) {
    vlo::IOdometer::Ptr huangs_vlo = std::make_shared<vlo::HuangVLO>(
        vo_5dof, reference_pose);
    huangs_vlo->debug_ = debug;
    huangs_vlo->verbose_ = verbose;

    return OdometerStruct{huangs_vlo, vlo::Transform3D::Identity()};
}

};  // namespace vlo
