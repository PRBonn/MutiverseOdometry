#pragma once
#include <array>
#include <vector>

#include <opencv2/opencv.hpp>
#include <visual_odometry/VisualFrame.hpp>

#include "IFeatureDetector.hpp"
#include "IFrame.hpp"
#include "common_vlo.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/xfeatures2d.hpp"
/**
 * @brief Namespace for the Visual Lidar Odometry Library
 */
namespace vlo {
using KeyPoints = std::vector<cv::KeyPoint>;
/**
 * @brief Class FeatureDetector to recognize features from image [todo: should
 * be renamed]
 */
class FeatureDetector : public IFeatureDetector {
 public:
  /**
   * @brief Constructor.
   * @param Feature2D from opencv
   */
  FeatureDetector(cv::Ptr<cv::Feature2D>);
  /**
   * @brief calculate keypoints
   * @param current frame with image
   */
  void calculateKeyPoints(IFrame::Ptr& frame);
  /**
   * @brief calculate keypoints
   * @param current image
   * @param [output] calculated key points
   */
  void calculateKeyPoints(const cv::Mat& image, KeyPoints& key_points);
  /**
   * @brief calculate descriptor for keypoints [todo: add assert if there is no
   * keypoints]
   * @param current frame with image
   */
  void calculateDescriptors(IFrame::Ptr& frame);
  /**
   * @brief calculate keypoints
   * @param current image
   * @param  key points
   * @param [output] calculated descriptor
   */
  void calculateDescriptors(const cv::Mat& image, KeyPoints& key_points,
                            cv::Mat& descriptors);
  /**
   * @brief calculate keypoints and descriptors
   * @param current frame with image
   */
  void calculateKeyPointsAndDescriptors(IFrame::Ptr& frame);
  /**
   * @brief calculate keypoints and descriptors
   * @param current image
   * @param [output] key points
   * @param [output] calculated descriptor
   */
  void calculateKeyPointsAndDescriptors(const cv::Mat& image,
                                        KeyPoints& key_points,
                                        cv::Mat& descriptors) override;
  /**
   * @brief calculate keypoints and descriptors with uniform sampling
   * @param current frame
   * @param grid size
   * @param max number of keypoints in the cell
   */
  void calculateGridKeyPointsAndDescriptors(const cv::Mat& image, int grid_size,
                                            int max_number_key_points_per_cell,
                                            KeyPoints &keypoints,
                                            cv::Mat &descriptors);

 private:
  /**
   * @brief Pointer to the descriptor from opencv class.
   */
  cv::Ptr<cv::Feature2D> descriptor_;
};
}  // namespace vlo
