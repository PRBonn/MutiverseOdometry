#pragma once
#include <eigen3/Eigen/Core>

#include <IVisualOdometer.hpp>
#include <opencv2/core/eigen.hpp>

#include "common_vlo.hpp"
#include "utils/conversions.hpp"
#include "visual_odometry/FeatureDetector.hpp"
#include "visual_odometry/FeatureMatcher.hpp"
#include "visual_odometry/Map.hpp"
#include "visual_odometry/MapPoint.hpp"
#include "visual_odometry/VisualFrame.hpp"
/**
 * @brief Namespace for the Visual Lidar Odometry Library
 */
namespace vlo {
/**
 * @brief The VO_PnP class declares the Visual Odometry based on PNP method,
 * with building a local map.
 */
class VO_PnP : public IVisualOdometer {
 public:
  /**
   * @brief Pointer for an interface.
   */
  using Ptr = std::shared_ptr<VO_PnP>;
  /**
   * @brief Constructor
   * @param Feature detector.
   * @param Feature matcher.
   * @param Initial pose.
   */
  VO_PnP(const cv::Ptr<Feature2D> &, const cv::Ptr<DescriptorMatcher> &,
         const Transform3D &);
  /**
   * @brief A member function to override. Runs the pipeline of lidar
   * odometry.
   * @param visual_frame Frame/node with acquired point cloud data
   */
  void processFrame(VisualFrame::Ptr frame) override;
  /**
   * @brief update the reference frame and push frame to history [todo: should
   * be moved to Iodometer]
   */
   double calculateCrossCheckingError(const IFrame::Ptr & current_frame, const IFrame::Ptr & reference_frame) override;
  void addKeyFrame(VisualFrame::Ptr);
  /**
   * @brief function that takes externally scale e.g. from the ground truth
   * @param Vector of translation.
   */
  void calculateScaleFromGroundTruth(Vec3);
  /**
   * @brief Triangulation, saves in the current frame
   * @param Points from current frame
   * @param Points from reference frame
   */
  void doTriangulation(ImagePointsd, ImagePointsd);
  /**
   * @brief Finds map points in the current view, based on geometry
   * @param current frame
   * @param map
   */
  void findMapPointsInCameraViewBasedOnGeometry(const VisualFrame::Ptr &frame,
                                                Map &map_points_in_view);
  /**
   * @brief Calculate matches between the current frame and reference frame
   * @param current frame
   * @param reference frame
   */
  void findCurrentImageToReferenceImageMatches();
  /**
   * @brief Constant velocity model.
   * @param points 3d
   * @param image points
   * @param mask
   */
  void pnp_run(const Points3Dd &points_3D, const ImagePointsd &points_image,
               cv::Mat &mask);
  /**
   * @brief Constant velocity model.
   */
  void constantVelocityModel();
  /**
   * @brief Save valuable map points
   * @param points 3D
   * @param corresponding points 2D
   */
  void pushCurrentTriangulatedPointsToMap(
      const Points3Dd &triangulated_points,
      const ImagePointsd &good_points_current_scene);
  /**
   * @brief Calculate transform based on 5point algorithm
   * @param current frame
   * @param reference frame
   */
  void imageToimageTransform5point(
      const ImagePointsd &good_points_current_scene,
      const ImagePointsd &good_points_previous_scene,
      Points3Dd &triangulated_points_out);
  /**
   * @brief Get Pointer to the map.
   */
  boost::circular_buffer<MapPoint::Ptr> getMap();
  /**
   * @brief Save valuable map points
   * @param points 3D
   * @param corresponding points 2D
   */
  void saveValuableMapPoints(const Points3Dd &good_triangulated_points,
                             const ImagePointsd &good_image_points);
  /**
   * @brief ground_truth_relative_transformation_
   */
  Transform3D ground_truth_relative_transformation_ = Transform3D::Identity();

 private:
  /**
   * @brief Feature detector that Visual Odometry uses [remove]
   */
  FeatureDetector feature_detector_;
  /**
   * @brief Feature matcher that Visual Odometry uses.
   */
  FeatureMatcher feature_matcher_;
  /**
   * @brief Map pointer.
   */
  Map::Ptr map_;
  /**
   * @brief State of the visual odometer.
   */
  VO_STATE vo_state_ = VO_STATE::STARTING;
  /**
   * @brief Scale
   */
  double scale_ = 1;
  /**
   * @brief todo: remove
   */
  const int k_buff_size_ = 20;
};

#endif  // VISUALODOMETER_HPP
