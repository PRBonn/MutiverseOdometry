#pragma once

#include "common_vlo.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <visual_odometry/FeatureDetector.hpp>

#include "IFrame.hpp"

#include "geometry/Camera.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "visual_odometry/MapPoint.hpp"

/**
 * @brief Namespace for the Visual Lidar Odometry Library
 */
namespace vlo {

class FeatureDetector;
/**
 * @brief The VisualFrame class declares a class for keeping camera-type data
 * and process in odometry pipeline.
 */
class VisualFrame : public IFrame {
 public:
  /**
   * @brief Pointer for an interface.
   */
  using Ptr = std::shared_ptr<VisualFrame>;
  /**
   * @brief Constructor.
   * @param Image
   * @param Camera sensor
   * @param Feature Detector
   */
  VisualFrame(const cv::Mat &image, const Camera::Ptr &camera,
              const double &time_stamp = -1);
  /**
   * @brief Desctuor.
   */
  ~VisualFrame() = default;
  /**
   * @brief Interface for adding points to the main map
   * @param map point
   */
  void addMapPoint(const MapPoint::Ptr &map_point);
  /**
   * @brief todo: remove
   */
  cv::Mat getGoodDescriptorFromIndex(int index);
  /**
   * @brief todo: remove
   */
  cv::KeyPoint getGoodKeyFromIndex(int index);
  /**
   * @brief Update good indexes that were taken under
   * consideration for finding transformation between two nodes.
   * @param Indexes that are valid.
   */
  void updateGoodIndexes(Indexes &indexes);
  /**
   * @brief Print good indexes
   */
  void printIndexes() const;
  /**
   * @brief Get keypoints but represented as points2d
   */
  ImagePointsd getPoints() const;
  /**
   * @brief Get Good descriptors
   */
  cv::Mat getGoodDescriptors() const;
  /**
   * @brief Return good keypoints
   * @return Return good keypoints based on good indexes that were updated,
   * intitially (without any updates) it returns just all key points.
   */
  KeyPoints getGoodKeyPoints() const;
  /**
   * @brief Return good points
   * @return Return good keypoints that are transformed to cv::Points2D based on
   * good indexes that were updated, intitially (without any updates) it returns
   * just all key points.
   */
  ImagePointsd getGoodPoints() const;
  /**
   * @return Returns 3x4 projection matrix in eigen format
   */
  Mat34 getProjectionMatrix_eigen() const;
  /**
   * @return Returns 3x4 projection matrix in opencv format
   */
  cv::Mat getProjectionMatrix() const;
  /**
   * @return Factor index for frame enumeration
   */
  static int factory_id_;
  /**
   * @return Member variable to keep triangulated points 3D.
   */
  std::set<MapPoint::Ptr> points3d_;

  double evaluateOverlappingWithFrame(const IFrame::Ptr & frame, const Transform3D & transform_from_this_to_frame) override;

};

};  // namespace vlo
