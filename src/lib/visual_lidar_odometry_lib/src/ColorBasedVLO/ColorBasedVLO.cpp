#include "ColorBasedVLO/ColorBasedVLO.hpp"

#include <open3d/pipelines/registration/ColoredICP.h>

#include "common_vlo.hpp"
namespace vlo {
ColorBasedVLO::ColorBasedVLO(const Transform3D &initial_pose){
  initial_pose_ = initial_pose;
  trajectory_.push_back(initial_pose);
}

vlo::Transform3D ColorBasedVLO::getFrameToFrameTransformation() const {
  return trajectory_.end()[-2].inverse() * trajectory_.end()[-1];
}

void ColorBasedVLO::addVisualAndLidarFrames(IFrame::Ptr &current_frame_) {
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

std::tuple<Transform3D, bool> ColorBasedVLO::findFrameToFrameTransformBetween(
    IFrame::Ptr &current_frame, IFrame::Ptr &reference_frame) {
    std::cout << "Colored ICP looks for the transformation " << std::endl;
    if (reference_frame->colored == false or reference_frame->colored == false)
    {
        std::cout << "You need to setup color version of the frame" << std::endl;
        exit(EXIT_FAILURE);
    }
  auto frame_to_frame_transform = Transform3D::Identity();
  bool trust_rotation = true;

  std::cout << "Time for matching: "
            << current_frame->point_cloud_colored_.points_.size() << std::endl;

  std::cout << "Time for matching: "
            << reference_frame->point_cloud_colored_.points_.size() << std::endl;

//  current_frame->point_cloud_colored_.EstimateNormals();
//  reference_frame->point_cloud_colored_.EstimateNormals();

    auto result = open3d::pipelines::registration::RegistrationColoredICP(
        current_frame->point_cloud_colored_, reference_frame->point_cloud_colored_,
        1.0, Eigen::Matrix4d::Identity());



//  open3d::visualization::DrawGeometries(
//      {std::make_shared<PointCloud>(current_frame->point_cloud_),
//       std::make_shared<PointCloud>(reference_frame->point_cloud_)},
//      "Name", 1600, 900);

//    auto pc_result =
//    reference_frame->point_cloud_.Transform(result.transformation_.inverse());

//    open3d::visualization::DrawGeometries({std::make_shared<PointCloud>(current_frame->point_cloud_),
//                                           std::make_shared<PointCloud>(pc_result)},
//                                           "Name", 1600, 900);

//    std::cout << result.transformation_ << std::endl;

    frame_to_frame_transform = result.transformation_;

  return {frame_to_frame_transform, trust_rotation};
}

Transform3D ColorBasedVLO::getLastPose() const { return trajectory_.back(); }

void ColorBasedVLO::addKeyFrame(const IFrame::Ptr &frame) {
  pushFrameToHistory(frame);
  reference_frame_ = frame;
}
double ColorBasedVLO::calculateCrossCheckingError(const IFrame::Ptr & current_frame, const IFrame::Ptr & reference_frame)
{

}
void ColorBasedVLO::pushFrameToHistory(const IFrame::Ptr &camera_frame) {
  // map_->insertKeyFrame(frame);
  trajectory_nodes_.push_back(camera_frame);
  reference_frame_ = camera_frame;
  std::cout << "trajectory node: " << trajectory_nodes_.size() << std::endl;
  if (trajectory_nodes_.size() > 3)
  {
      trajectory_nodes_.erase(trajectory_nodes_.begin());
  }
}

OdometerStruct getColorBasedVLOFramAndInitialize(Transform3D reference_pose,
                                              bool debug, bool verbose) {
  std::shared_ptr<vlo::IOdometer> colorBasedVLO =
      std::make_shared<vlo::ColorBasedVLO>(reference_pose);
  colorBasedVLO->verbose_ = debug;
  colorBasedVLO->debug_ = verbose;

  return vlo::OdometerStruct{colorBasedVLO, Transform3D::Identity()};
}

};  // namespace vlo
