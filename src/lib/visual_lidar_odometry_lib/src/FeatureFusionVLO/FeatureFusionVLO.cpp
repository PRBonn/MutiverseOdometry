#include "FeatureFusionVLO/FeatureFusionVLO.hpp"

#include "common_vlo.hpp"
namespace vlo {
FeatureFusionVLO::FeatureFusionVLO(    const Transform3D &camera_lidar_transform, const Transform3D &initial_pose)
    :   camera_lidar_transform_(camera_lidar_transform) {
  initial_pose_ = initial_pose;
  trajectory_.push_back(initial_pose);

}

vlo::Transform3D FeatureFusionVLO::getFrameToFrameTransformation() const {
  return trajectory_.end()[-2].inverse() * trajectory_.end()[-1];
}

void FeatureFusionVLO::addVisualAndLidarFrames(IFrame::Ptr &current_frame_) {
  if (state_ == VLO_STATE::INIT) {
    current_frame_->pose_ = initial_pose_;
    addKeyFrame(current_frame_);
    state_ = VLO_STATE::RUNNING;

  } else if (state_ == VLO_STATE::RUNNING) {
    auto [frame_to_frame_transform, trust_result] =
        findFrameToFrameTransformBetween(current_frame_, reference_frame_);
    Transform3D current_pose = Transform3D::Identity();
    if (trust_result) {
      std::cout << "Result is fine!" << std::endl;
      current_pose = trajectory_.back() * frame_to_frame_transform;
      current_frame_->pose_ = current_pose;
    } else {

    }
    //    if (sanity_checker_.check(current_frame_, trajectory_nodes_) or not
    //    trust_rotation) {
    //     std::cout << " I don't trust the camera result!" <<  std::endl;
    //      auto last_transform = getFrameToFrameTransformation();

    //      last_transform.matrix().block<3, 1>(0, 3) =
    //      last_transform.translation(); last_transform.matrix().block<3, 3>(0,
    //      0) = Mat33::Identity();

    //      current_pose = trajectory_.back() * last_transform;
    //      Eigen::Quaterniond q_current(current_pose.rotation().matrix());
    //      q_current.normalize();
    //      current_pose.matrix().block<3, 3>(0, 0) =
    //      q_current.toRotationMatrix();
    // current_frame_->pose_ = current_pose;
    //    }

    trajectory_.push_back(current_pose);
    std::cout << "Current pose: " << std::endl;
    std::cout << current_frame_->pose_.matrix() << std::endl;
    addKeyFrame(current_frame_);
  }
}
double FeatureFusionVLO::calculateCrossCheckingError(const IFrame::Ptr & current_frame, const IFrame::Ptr & reference_frame)
{

}
std::tuple<Transform3D, bool>
FeatureFusionVLO::findFrameToFrameTransformBetween(
    IFrame::Ptr &current_frame, IFrame::Ptr &reference_frame) {



    FeatureFusionMatcher feature_matcher;
    std::vector<cv::DMatch> good_matches;
    feature_matcher.match(current_frame, reference_frame,camera_lidar_transform_, good_matches);




    Transform3D frame_to_frame_transform = Transform3D::Identity();
    bool trust_rotation;



    return {frame_to_frame_transform, trust_rotation};
}

Transform3D FeatureFusionVLO::getLastPose() const { return trajectory_.back(); }

void FeatureFusionVLO::addKeyFrame(const IFrame::Ptr &frame) {
  pushFrameToHistory(frame);
  reference_frame_ = frame;
}

void FeatureFusionVLO::pushFrameToHistory(const IFrame::Ptr &camera_frame) {
  // map_->insertKeyFrame(frame);
  trajectory_nodes_.push_back(camera_frame);
  reference_frame_ = camera_frame;
}

};  // namespace vlo
