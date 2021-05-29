#include "LidarOdometerFrameToFrame.hpp"

#include "open3d/pipelines/registration/Registration.h"
namespace vlo {
LidarOdometerFrameToFrame::LidarOdometerFrameToFrame(
    const vlo::Transform3D &initial_pose, const std::string &config_file)
    : initial_pose_(initial_pose){
          //  std::ifstream ifs(config_file.c_str());
          //  icp.loadFromYaml(ifs);
      };

void LidarOdometerFrameToFrame::processFrame(IFrame::Ptr lidar_frame) {
  current_lidar_frame_ = lidar_frame;

  if (lo_state_ == VLO_STATE::INIT) {
    current_lidar_frame_->pose_ = initial_pose_;
    pushFrameToHistory(current_lidar_frame_);
    lo_state_ = VLO_STATE::RUNNING;
  } else if (lo_state_ == VLO_STATE::RUNNING) {
    bool map = false;

    //    if (!map)
    //    {

    auto [frame_to_frame_transform, trust_result] =
        findFrameToFrameTransformBetween(current_lidar_frame_,
                                         reference_lidar_frame_);
    //} else {
    //              std::vector<Vec3> colors;
    //              colors.push_back(Vec3(1.0, 0.0, 0.0));  // red
    //              colors.push_back(Vec3(1.0, 0.5, 0.0));  // orange
    //              colors.push_back(Vec3(1.0, 1.0, 0.0));  // yellow
    //              colors.push_back(Vec3(0.8, 0.0, 0.0));  // red
    //              colors.push_back(Vec3(0.8, 0.5, 0.0));  // orange
    //              colors.push_back(Vec3(0.8, 0.8, 0.0));  // yellow
    //              int i = 0;
    //              PointCloud map;
    //              for (auto &frame : history_frames_) {
    //                  auto transform = frame->pose_.inverse() *
    //                  reference_lidar_frame_->pose_.matrix(); std::cout << "
    //                  transform in map:" << std::endl; std::cout <<
    //                  transform.matrix() << std::endl;
    //                frame->point_cloud_.PaintUniformColor(colors[i]);
    //                i++;
    //                map = map +
    //                frame->point_cloud_.Transform(transform.inverse().matrix());
    //              }
    //              auto [frame_to_frame_transform, trust_result] =
    //                  findFrameToFrameTransformBetween(current_lidar_frame_,
    //                  map);
    //}

    current_lidar_frame_->pose_.matrix() =
        reference_lidar_frame_->pose_.matrix() *
        frame_to_frame_transform.matrix();
    scale_ = frame_to_frame_transform.translation();

    pushFrameToHistory(current_lidar_frame_);
  }
}

void open3DtoPM(const PointCloud &input_point_cloud,
                PM::DataPoints &output_point_cloud) {
  PM::Matrix features(4, input_point_cloud.points_.size());
  for (auto i = 0; i < input_point_cloud.points_.size(); i++) {
    features(0, i) = input_point_cloud.points_[i](0);
    features(1, i) = input_point_cloud.points_[i](1);
    features(2, i) = input_point_cloud.points_[i](2);
    features(3, i) = 1.0;
  }
  output_point_cloud.addFeature("x", features.row(0));
  output_point_cloud.addFeature("y", features.row(1));
  output_point_cloud.addFeature("z", features.row(2));
  output_point_cloud.addFeature("pad", features.row(3));
}

std::tuple<Transform3D, bool>
LidarOdometerFrameToFrame::findFrameToFrameTransformBetween(
    IFrame::Ptr &current_frame, IFrame::Ptr &reference_frame) {
  std::cout << "Point2Plane ICP looks for the transformation " << std::endl;
  std::cout << "Point2Plane ICP: Last transform: " << std::endl;
  std::cout << last_transform_.matrix() << std::endl;
  auto result = open3d::pipelines::registration::RegistrationICP(
      current_frame->point_cloud_, reference_frame->point_cloud_, 0.1,
      last_transform_,
      open3d::pipelines::registration::TransformationEstimationPointToPlane());
  Transform3D frame_to_frame_transformation(result.transformation_);
  std::cout << "Point2Plane ICP: Current transform: " << std::endl;
  std::cout << frame_to_frame_transformation.matrix() << std::endl;
  last_transform_ = frame_to_frame_transformation.matrix();

  auto current = std::make_shared<PointCloud>(current_frame->point_cloud_);//->point_cloud_.Transform(frame_to_frame_transformation.matrix());
  current->PaintUniformColor(Vec3(0,0,1));
  current->Transform(frame_to_frame_transformation.matrix());
  auto reference = std::make_shared<PointCloud>(reference_frame->point_cloud_);
  reference->PaintUniformColor(Vec3(0,1,0));

  auto together = *current + *reference;


open3d::io::WritePointCloudToPCD("p2p.pcd" , together, {true});

  return {frame_to_frame_transformation, true};
}

std::tuple<Transform3D, bool>
LidarOdometerFrameToFrame::findFrameToFrameTransformBetween(
    IFrame::Ptr &current_frame, PointCloud &map) {
  std::cout << "Last transform: " << std::endl;
  std::cout << last_transform_.matrix() << std::endl;
  //  open3d::visualization::DrawGeometries(
  //      {std::make_shared<PointCloud>(map),
  //       std::make_shared<PointCloud>(current_frame->point_cloud_)},
  //      "map vs current scan", 1600, 900);

  auto result = open3d::pipelines::registration::RegistrationICP(
      current_frame->point_cloud_, map, 0.1, last_transform_,
      open3d::pipelines::registration::TransformationEstimationPointToPlane());
  Transform3D frame_to_frame_transformation(result.transformation_);
  std::cout << "Transformastions found: \n"
            << result.transformation_.matrix() << std::endl;
  last_transform_ = frame_to_frame_transformation.matrix();

  return {frame_to_frame_transformation, true};
}

double LidarOdometerFrameToFrame::calculateCrossCheckingError(
    const IFrame::Ptr &current_frame, const IFrame::Ptr &reference_frame) {
  std::cout << "lidar!" << std::endl;
  return 5.0;
}

void LidarOdometerFrameToFrame::hintForICP(const Transform3D &guess_for_icp) {
  guess_for_icp_ = guess_for_icp;
}

OdometerStruct getLOFrameToFrameAndInitialize(Transform3D reference_pose,
                                              std::string config_file,
                                              bool debug, bool verbose) {
  std::shared_ptr<vlo::LidarOdometerFrameToFrame> point2plane_lo =
      std::make_shared<vlo::LidarOdometerFrameToFrame>(reference_pose,
                                                       config_file);
  point2plane_lo->verbose_ = debug;
  point2plane_lo->debug_ = verbose;

  return vlo::OdometerStruct{point2plane_lo, Transform3D::Identity()};
}

};  // namespace vlo
