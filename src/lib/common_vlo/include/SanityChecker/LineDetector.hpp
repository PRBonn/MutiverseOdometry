#pragma once
#include "common_vlo.hpp"
#include "geometry/geometry.hpp"
#include "IFrame.hpp"
#include <eigen3/Eigen/Core>
#include <opencv2/imgproc.hpp>
/**
 * @brief Namespace for the Visual Lidar Odometry Library
 */
namespace vlo {
/**
 * @brief The LineDetector class declares a class for line detection (currently
 * only from LidarScans) [todo: create an abstract interface]
 */
class LineDetector {
 public:
  LineDetector();
  /**
   * @brief Find lines and returns them somehow [todo]
   * @param frame where the lines should be located
   */
  void findLines(const IFrame::Ptr &frame);
  /**
   * @brief Since currently it only detects from the lidar scans then we have a
   * member variable to represent the line as a point cloud
   */
  PointCloudPtr lines_;
  /**
   * @brief camera_lidar_transform should be removed from here
   */
  Transform3D camera_lidar_transform_ = Transform3D::Identity();

  std::vector<cv::Point2d> findTrapeoisodalConvexHull(const cv::Mat & image, const std::vector<cv::KeyPoint> &keypoints);
};
}  // namespace vlo
