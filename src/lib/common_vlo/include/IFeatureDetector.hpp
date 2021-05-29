#pragma once
#include "common_vlo.hpp"

/**
 * @brief Namespace for the Visual Lidar Odometry Library
 */
namespace vlo {
/**
 * @brief  The IFeatureDetector class declares an interface for any type of
 * FeatureDetector [currently only for images]
 */
class IFeatureDetector {
  // virtual ~IFeatureDetector(){}
 public:
  /**
   * @brief An abstract interface for keypoints and descriptors calculating for
   * the image [todo: shouldn't be abstract just specialized for vision purposes]
   * @param current image
   * @param [output] returns calculated keypoints by this variable
   * @param [output] returns calculated descriptor by this variable
   */
  virtual void calculateKeyPointsAndDescriptors(const cv::Mat& image,
                                                KeyPoints& key_points,
                                                cv::Mat& descriptors) = 0;
  /**
   * @brief An abstract interface for keypoints and descriptors calculating for
   * the image [todo: shouldn't be abstract just specialized for vision purposes]
   * @param current image
   * @param [output] returns calculated keypoints by this variable
   * @param [output] returns calculated descriptor by this variable
   */

  virtual void calculateGridKeyPointsAndDescriptors(
      const cv::Mat& image, int grid_size,
      int max_number_key_points_per_cell, KeyPoints &keypoints, cv::Mat &descriptors) = 0;
};

}  // namespace vlo
