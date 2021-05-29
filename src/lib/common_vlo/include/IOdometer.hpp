#pragma once
#include "IFrame.hpp"
#include "SanityChecker/SanityChecker.hpp"
#include "common_vlo.hpp"
/**
 * @brief Namespace for the Visual Lidar Odometry Library
 */
namespace vlo {

/**
 * @brief The IOdometer class declares an interface for any type of Odometery.
 */
class IOdometer {
 public:
  /**
   * @brief Pointer for an interface.
   */
  using Ptr = std::shared_ptr<IOdometer>;
  virtual ~IOdometer(){};
  /**
   * @brief An abstract member function to override. Computes the transformation
   * between two frames of readings (that are used as nodes)
   * @param current_frame Pointer to the frame with data currently acquired.
   * @param reference_frame Pointer to the frame with acquired in the past time
   * stamp
   * @return It returnes the relative transformation from reference_frame to the
   * current_frame and additionally it informs whether the transformation is
   * reliable (in terms of inner mechanism of failure detection of the
   * respective odometry.
   */
  virtual std::tuple<Transform3D, bool> findFrameToFrameTransformBetween(
      IFrame::Ptr& current_frame, IFrame::Ptr& reference_frame) = 0;
  /**
   * @brief An abstract member function to override. Computes the transformation
   * between two frames of readings (that are used as nodes)
   * @param current_frame Pointer to the frame with data currently acquired.
   * @param map map of point cloud to refer to
   * @return It returnes the relative transformation from reference_frame to the
   * current_frame and additionally it informs whether the transformation is
   * reliable (in terms of inner mechanism of failure detection of the
   * respective odometry.
   */
  virtual std::tuple<Transform3D, bool> findFrameToFrameTransformBetween(
      IFrame::Ptr& current_frame, PointCloud& map) = 0;

  /**
   * @brief It returns the last estimated pose.
   * @return It returnes the last estimated pose.
   */
  Transform3D getLastPose() const;
  /**
   * @brief Member variable to keep track of trajectory [todo: should be
   * exchanged with estimation of list of nodes]
   */
  Trajectory trajectory_;
  /**
   * @brief Sanity checker
   */
  SanityChecker sanity_checker_;
  /**
   * @brief Method name
   */
  std::string method_name = "IOdometer";
  /**
   * @brief  [todo: should be moved to Iodometer]
   */
  bool debug_ = true;
  /**
   * @brief  [todo: should be moved to Iodometer]
   */
  bool verbose_ = true;

  Eigen::Matrix4d last_transform_ = Eigen::Matrix4d::Identity();

  virtual double calculateCrossCheckingError(
      const IFrame::Ptr& current_frame, const IFrame::Ptr& reference_frame) = 0;
};

}  // namespace vlo
