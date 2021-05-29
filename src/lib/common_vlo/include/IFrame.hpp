#pragma once
#include "IFeatureDetector.hpp"
#include "ISensor.hpp"
#include "common_vlo.hpp"
/**
 * @brief Namespace for the Visual Lidar Odometry Library
 */
namespace vlo {
/**
 * @brief The IFrame class declares an interface for any type of Frame.
 * Basically a frame is a kind of node that is composed of sensors readings
 * (lidar data, visual data), also node keeps track of properties of given node
 * such as pose, velocity, acceleration.
 */
class IFrame {
 public:
  /**
   * @brief Pointer for an interface.
   */
  using Ptr = std::shared_ptr<IFrame>;
  /**
   * @brief Virtual desctuctor.
   */
virtual ~IFrame(){};
  /**
   * @brief Update good indexes that were taken under consideration for finding transformation between two nodes
   * @param Indexes that are valid
   */
  virtual void updateGoodIndexes(Indexes &indexes) = 0;
  /**
   * @brief Return good points
   * @return Return good keypoints that are transformed to cv::Points2D based on good indexes that were updated, intitially (without any updates) it returns just all key points.
   */
  virtual ImagePointsd getGoodPoints() const = 0;
  /**
   * @brief Return good keypoints
   * @return Return good keypoints based on good indexes that were updated, intitially (without any updates) it returns just all key points.
   */
  virtual KeyPoints getGoodKeyPoints() const = 0;
  /**
   * @brief Return good descriptors
   * @return Return good descriptors based on good indexes that were updated, intitially (without any updates) it returns just all descriptors.
   */
  virtual cv::Mat getGoodDescriptors() const = 0;
  /**
   * @brief Time stamp when data was acquired.
   */

  virtual double evaluateOverlappingWithFrame(const IFrame::Ptr & frame, const Transform3D & transform_from_this_to_frame) = 0;
  double time_stamp_;
  /**
   * @brief Id of dataframe acquired from given type of sensor. Sometime it may
   * happen that order of time stamps may be different from id if it's
   * considered multi-camera-lidar system.
   */
  unsigned long id_;
  /**
   * @brief Point cloud associated with the given node, it may be either from
   * triangulation or from lidar scans.
   */
  PointCloud point_cloud_;
  /**
   * @brief Point cloud associated with the given node, it may be either from
   * triangulation or from lidar scans.
   */
  PointCloud point_cloud_colored_;

  bool colored = false;

  /**
   * @brief Image associated with the given node.
   */
  cv::Mat image_;
  /**
   * @brief Pose of the node (translation + rotation).
   */
  Transform3D pose_ = Transform3D::Identity();
  /**
   * @brief Speed (magnitude of velocity).
   */
  double speed = 0.0;
  /**
   * @brief Acceleration (magnitude).
   */
  double acceleration = 0.0;
  /**
   * @brief Descriptor for the node data points (either for image, for point
   * cloud, or combined descriptor).
   */
  cv::Mat descriptors_;
  /**
   * @brief Points 2D where features are located on the image plane.
   */
  KeyPoints key_points_;
  /**
   * @brief Since point_cloud_, descriptors_, key_points_ track all features
   * that were detected, sometimes (i.e. when calculating frame to frame), we
   * want only to keep track of the good feature points that are associated with
   * the finding transformation between two nodes
   */
  Indexes good_indexes_;
  /**
   * @brief Sensor type associated with the given node (either camera or lidar)
   */
  ISensor::Ptr sensor_;
};

}  // namespace vlo
