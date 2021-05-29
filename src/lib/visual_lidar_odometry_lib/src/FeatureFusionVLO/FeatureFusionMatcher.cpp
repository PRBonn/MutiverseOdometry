#include "FeatureFusionVLO/FeatureFusionMatcher.hpp"

#include <opencv2/ml.hpp>
namespace vlo {

FeatureFusionMatcher::FeatureFusionMatcher() {}

std::vector<DescriptorWithPoint> FeatureFusionMatcher::matchWithinTheFrame(
    IFrame::Ptr &frame, const Transform3D &camera_lidar_transform) {
  cv::Mat key_points_mat;
  for (auto keypoint : frame->getGoodKeyPoints()) {
    key_points_mat.push_back(cv::Mat(keypoint.pt).t());
  }
 auto  camera_lidar_transform2  = Transform3D::Identity();
  PointCloud point_cloud_of_descriptor;
  //  point_cloud_of_descriptor.points_.reserve(frame->point_cloud_.points_.size());
  //  point_cloud_of_descriptor.colors_.reserve(frame->point_cloud_.points_.size());
  cv::Mat_<cv::Point2f> projected_points3d_mat;
  std::vector<cv::KeyPoint> projected_points3d_mat_keypoints;
  for (size_t i = 0; i < frame->point_cloud_.points_.size(); i++) {
    auto point_camera_frame =
        camera_lidar_transform2 * frame->point_cloud_.points_[i];
    auto projected_point_camera_frame = projectPointToImagePlane(
        point_camera_frame, frame->sensor_->getSensorMatrix());

    if (isInImage(frame->image_, projected_point_camera_frame) and
        point_camera_frame[2] > 0.0) {
      projected_points3d_mat.push_back(
          cv::Mat(cv::Point2f(projected_point_camera_frame)).t());
      cv::KeyPoint point;
      point.pt = cv::Point2f(projected_point_camera_frame);
      projected_points3d_mat_keypoints.push_back(point);
      point_cloud_of_descriptor.points_.push_back(point_camera_frame);
      auto img_color = getRGBvalueFromImageNormalized(
          frame->image_, projected_point_camera_frame);
      point_cloud_of_descriptor.colors_.push_back(
          Vec3(img_color[2], img_color[1], img_color[0]));
    }
  }

  cv::Ptr<cv::DescriptorMatcher> matcher =
      cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
  key_points_mat.convertTo(key_points_mat, CV_32F);
  projected_points3d_mat.convertTo(projected_points3d_mat, CV_32F);
  matcher->add(projected_points3d_mat);
  std::vector<cv::DMatch> matches, good_matches;
  matcher->match(key_points_mat, matches);

  PointCloud important_3d_points;
  std::vector<DescriptorWithPoint> descriptor_with_point3d;
  std::vector<int> good_indexes;
  for (const auto match : matches) {
    std::cout << match.distance << std::endl;
    if (match.distance < 10) {
      good_matches.push_back(match);

      important_3d_points.points_.push_back(
          frame->point_cloud_.points_[match.trainIdx]);

      DescriptorWithPoint descriptor_with_point;
      descriptor_with_point.descriptor =
          frame->getGoodDescriptors().row(match.queryIdx);
      descriptor_with_point.point3d =
          frame->point_cloud_.points_[match.trainIdx];
      descriptor_with_point.idx = frame->good_indexes_[match.queryIdx];
      good_indexes.push_back(descriptor_with_point.idx);
      descriptor_with_point3d.push_back(descriptor_with_point);
    }
  }

  cv::Mat img_show;
  cv::drawMatches(frame->image_, frame->getGoodKeyPoints(), frame->image_,
                  projected_points3d_mat_keypoints, good_matches, img_show);

  cv::imshow("window_name", img_show);

  KeyPoints keypoints;

  for (const auto &good_match : good_matches) {
    keypoints.push_back(frame->getGoodKeyPoints()[good_match.queryIdx]);
  }

  cv::Mat keypoint_img;
  cv::drawKeypoints(frame->image_, keypoints, keypoint_img);
  cv::imshow("img keypoint", keypoint_img);

  frame->updateGoodIndexes(good_indexes);
  return descriptor_with_point3d;
}

void FeatureFusionMatcher::match(IFrame::Ptr &current_frame,
                                 IFrame::Ptr &reference_frame,
                                 Transform3D &camera_lidar_transform,
                                 std::vector<cv::DMatch> &matches) {
  cv::Ptr<cv::DescriptorMatcher> matcher_ =
      cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
  std::vector<std::vector<cv::DMatch>> knn_matches_;
  std::vector<cv::DMatch> good_matches;
  matcher_->clear();
  matcher_->add(reference_frame->descriptors_);
  matcher_->match(current_frame->descriptors_, good_matches);

  cv::Mat out_out_img;
  cv::drawMatches(current_frame->image_, current_frame->key_points_,
                  reference_frame->image_, reference_frame->key_points_,
                  good_matches, out_out_img);
  cv::imshow("casual", out_out_img);

  std::vector<int> reference_indexes;
  std::vector<int> current_indexes;

  for (auto &good_match : good_matches) {
    current_indexes.push_back(good_match.queryIdx);
    reference_indexes.push_back(good_match.trainIdx);
  }

  reference_frame->updateGoodIndexes(reference_indexes);
  current_frame->updateGoodIndexes(current_indexes);

  auto current_descriptors =
      matchWithinTheFrame(current_frame, camera_lidar_transform);
  auto reference_descriptors =
      matchWithinTheFrame(reference_frame, camera_lidar_transform);
  cv::Mat current_mat_descriptors;
  cv::Mat reference_mat_descriptors;

  std::vector<cv::DMatch> good_matches_with_depth;
  matcher_->clear();
  matcher_->add(reference_frame->getGoodDescriptors());
  matcher_->match(current_frame->getGoodDescriptors(), good_matches_with_depth);

  std::cout << "sizes comparison: " << current_descriptors.size() << " vs "
            << current_frame->getGoodDescriptors().size() << std::endl;

  std::cout << "sizes comparison: " << reference_descriptors.size() << " vs "
            << reference_frame->getGoodDescriptors().size() << std::endl;

  PointCloud current_pointcloud;
  PointCloud reference_pointcloud;

  for (size_t i = 0; i < current_descriptors.size(); i++) {
    current_pointcloud.points_.push_back(current_descriptors[i].point3d);
  }
  current_pointcloud.PaintUniformColor(Vec3(255.,0.0,0.0));
  for (size_t i = 0; i < reference_descriptors.size(); i++) {
    reference_pointcloud.points_.push_back(reference_descriptors[i].point3d);
  }
  reference_pointcloud.PaintUniformColor(Vec3(0.0,255.0,0.0));
  std::cout << reference_pointcloud.points_.size() << std::endl;
    std::cout << reference_pointcloud.points_.size() << std::endl;

  open3d::visualization::DrawGeometries(
      {std::make_shared<PointCloud>(current_pointcloud),
       std::make_shared<PointCloud>(reference_pointcloud)},
      "Name", 1600, 900);

  //  for (const auto & good_match_with_depth: good_matches_with_depth)
  //  {

  //  }

  cv::Mat img_show;
  cv::drawMatches(current_frame->image_, current_frame->getGoodKeyPoints(),
                  reference_frame->image_, reference_frame->getGoodKeyPoints(),
                  good_matches_with_depth, img_show);

  cv::imshow("main", img_show);
  cv::waitKey(0);
}

}  // namespace vlo
