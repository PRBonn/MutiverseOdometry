#include "VisualLidarFusionFrame.hpp"

#include <random>

#include "geometry/Camera.hpp"
namespace vlo {

struct DescriptorWithPoint{
    cv::Mat descriptor;
    Vec3 point3d;
};

VisualLidarFusionFrame::VisualLidarFusionFrame(
    const VisualFrame::Ptr &visual_frame, const LidarFrame::Ptr &lidar_frame,
    const Transform3D &camera_lidar_transform) {
  image_ = visual_frame->image_;
  key_points_ = visual_frame->key_points_;
  point_cloud_ = lidar_frame->point_cloud_;
  cv::Mat key_points_mat;
  for (auto keypoint : key_points_) {
    key_points_mat.push_back(cv::Mat(keypoint.pt).t());
  }

  good_indexes_ = visual_frame->good_indexes_;
  sensor_ = visual_frame->sensor_;

  PointCloud point_cloud_of_descriptor;
  point_cloud_of_descriptor.points_.reserve(
      lidar_frame->point_cloud_.points_.size());
  point_cloud_of_descriptor.colors_.reserve(
      lidar_frame->point_cloud_.points_.size());
  auto t_start = std::chrono::high_resolution_clock::now();
  cv::Mat_<cv::Point2f> projected_points3d_mat;
  std::vector<cv::KeyPoint> projected_points3d_mat_keypoints;
  for (size_t i = 0; i < lidar_frame->point_cloud_.points_.size(); i++) {
    auto point_camera_frame =
        camera_lidar_transform * lidar_frame->point_cloud_.points_[i];
    auto projected_point_camera_frame = projectPointToImagePlane(
        point_camera_frame, sensor_->getSensorMatrix());

    if (isInImage(image_, projected_point_camera_frame) and
        point_camera_frame[2] > 0.0) {
      projected_points3d_mat.push_back(
          cv::Mat(cv::Point2f(projected_point_camera_frame)).t());
      cv::KeyPoint point;
      point.pt = cv::Point2f(projected_point_camera_frame);
      projected_points3d_mat_keypoints.push_back(point);
      point_cloud_of_descriptor.points_.push_back(point_camera_frame);
      auto img_color =
          getRGBvalueFromImageNormalized(image_, projected_point_camera_frame);
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
  matcher->match(key_points_mat, matches, 1);

  PointCloud important_3d_points;
  std::vector<DescriptorWithPoint> descriptor_with_point3d;
  std::cout << "good descriptors " << descriptors_.size() << std::endl;
  for (const auto &match : matches) {
    if (match.distance < 10) {
      good_matches.push_back(match);
      good_indexes_.push_back(match.queryIdx);
      important_3d_points.points_.push_back(
          lidar_frame->point_cloud_.points_[match.trainIdx]);
      descriptors_.push_back(visual_frame->descriptors_.row(match.queryIdx));


      DescriptorWithPoint descriptor_with_point;
      descriptor_with_point.descriptor = visual_frame->descriptors_.row(match.queryIdx);
      descriptor_with_point.point3d = lidar_frame->point_cloud_.points_[match.trainIdx];

      descriptor_with_point3d.push_back(descriptor_with_point);

    }
  }

  auto t_end = std::chrono::high_resolution_clock::now();
  double elapsed_time_ms =
      std::chrono::duration<double, std::milli>(t_end - t_start).count();
  std::cout << "It takes " << elapsed_time_ms
            << ", good matches: " << good_matches.size() << std::endl;

  //  if (false)
  //  {
  //    std::cout << "good escriptors2 " << good_indexes_.size() << std::endl;
  //  std::cout << " before point cloud : " << point_cloud_.points_.size() <<
  //  std::endl; std::cout << " colors: " << point_cloud_.colors_.size() <<
  //  std::endl;

  //  std::cout << " before point cloud : " << point_cloud_.points_.size() <<
  //  std::endl; std::cout << " colors: " << point_cloud_.colors_.size() <<
  //  std::endl;
  //  }
  //  if (false) {
  //    cv::Mat img_out;
  //    cv::drawKeypoints(image_, visual_frame->key_points_, img_out,
  //                      cv::Scalar(0, 0, 255));
  //    cv::imshow("window_name", img_out);
  //    cv::waitKey(0);
  //    cv::Mat img_show;
  //    cv::Mat img_keypoints1;
  //    cv::Mat img_keypoints2;
  //    KeyPoints rgb_keypoint, keypoints3d;
  //    for (int i = 0; i < good_matches.size(); i++) {
  //      rgb_keypoint.push_back(key_points_[good_matches[i].queryIdx]);
  //      keypoints3d.push_back(
  //          projected_points3d_mat_keypoints[good_matches[i].trainIdx]);
  //    }
  //    cv::drawKeypoints(image_, rgb_keypoint, img_keypoints1,
  //                      cv::Scalar(0, 0, 255));
  //    cv::imshow("img_keypoints1", img_keypoints1);

  //    cv::drawKeypoints(image_, keypoints3d, img_keypoints2,
  //                      cv::Scalar(0, 255, 255));
  //    cv::imshow("img_keypoints3d", img_keypoints2);
  cv::Mat img_show;
  cv::drawMatches(image_, visual_frame->key_points_, image_,
                  projected_points3d_mat_keypoints, good_matches, img_show);
  cv::imshow("window_name", img_show);

  //    std::cout << good_matches.size() << std::endl;
  //  }
  exit(0);
}

void VisualLidarFusionFrame::updateGoodIndexes(Indexes &indexes) {
  good_indexes_ = indexes;
};

ImagePointsd VisualLidarFusionFrame::getGoodPoints() const {
  ImagePointsd good_points;

  for (const auto &good_index : good_indexes_) {
    good_points.push_back(key_points_[good_index].pt);
  }

  return good_points;
}
KeyPoints VisualLidarFusionFrame::getGoodKeyPoints() const {
  KeyPoints key_points;
  for (const auto &good_index : good_indexes_) {
    key_points.push_back(key_points_[good_index]);
  }

  return key_points;
}

cv::Mat VisualLidarFusionFrame::getGoodDescriptors() const {
  cv::Mat descriptors;
  for (const auto &good_index : good_indexes_) {
    descriptors.push_back(descriptors_.row(good_index));
  }

  return descriptors;
}

double VisualLidarFusionFrame::evaluateOverlappingWithFrame(
    const IFrame::Ptr &frame, const Transform3D &transform_from_this_to_frame) {
}

}  // namespace vlo
