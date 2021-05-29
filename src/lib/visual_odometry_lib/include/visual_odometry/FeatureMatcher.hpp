#pragma once
#include <vector>

#include <opencv2/opencv.hpp>

#include "common_vlo.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "utils/conversions.hpp"
#include "visual_odometry/Map.hpp"
#include "visual_odometry/VisualFrame.hpp"
using namespace cv;
/**
 * @brief Namespace for the Visual Lidar Odometry Library
 */
namespace vlo {
/**
 * @brief Class FeatureMatcher to match features from image to image [todo:
 * should be more abstract and renamed].
 */
class FeatureMatcher {
 public:
  /**
   * @brief Constructor.
   * @param DescriptorMatcher from opencv.
   */
  FeatureMatcher(Ptr<DescriptorMatcher>);
  /**
   * @brief Finds matches between images.
   * @param image 1 descriptors.
   * @param image 2 descriptors.
   * @param [output] Matches indexes.
   */
  void match(const Mat &, const Mat &, std::vector<DMatch> &);
  /**
   * @brief Finds matches between images.
   * @param frame 1 descriptors.
   * @param frame 2 descriptors.
   * @param [output] Matches indexes.
   */
  void match(const VisualFrame::Ptr &, const VisualFrame::Ptr &,
             std::vector<DMatch> &);
  /**
   * @brief Finds matches between image and map (in feature-descriptor space).
   * @param image 1 descriptors.
   * @param map descriptors.
   * @param [output] Matches indexes.
   */
  void matchToMap(const cv::Mat &current_frame_descriptors,
                  const cv::Mat &map_points_descriptors,
                  std::vector<DMatch> &good_matches,
                  std::vector<int> &potential_new_points_indexes);
  /**
   * @brief Finds matches between image and map (in feature-descriptor space).
   * @param Frame 1.
   * @param Map descriptors.
   * @param [output] Matches indexes.
   * @param [output] Vector of indexes points that haven't found any association.
   */
  void matchToMap(const VisualFrame::Ptr &current_frame,
                  const Map &map_points_descriptors,
                  std::vector<DMatch> &good_matches,
                  std::vector<int> &potential_new_points);
  /**
   * @brief Finds matches between map and image (in 2D space).
   * @param Map
   * @param Current frame.
   * @param [output] Matches indexes.
   */
  void matchToImage(const Map::Ptr &map, const VisualFrame::Ptr &current_frame,
                    std::vector<DMatch> &good_matches);

 private:
  /**
   * @brief Pointer to the matcher from opencv class.
   */
  Ptr<DescriptorMatcher> matcher_;
};
}  // namespace vlo
