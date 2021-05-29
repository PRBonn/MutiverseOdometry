#pragma once

#include <iostream>

#include "IFrame.hpp"
#include "IOdometer.hpp"
#include "LidarOdometerFrameToFrame.hpp"
#include "VO_FrameToFrame_5point.hpp"
#include "common_vlo.hpp"
/**
 * @brief Namespace for the Visual Lidar Odometry Library
 */
namespace vlo {
using Trajectory = std::vector<Transform3D>;
/**
 * @brief The VLO6dof class implements the method visual odometry guessing for
 * lidar odometry.
 */
class VLO6dof : public IOdometer {
 public:
  /**
   * @brief Pointer for an interface.
   */
  using Ptr = std::shared_ptr<VLO6dof>;
  /**
   * @brief Constructor.
   * @param Visual Odometer class
   * @param Lidar Odometer class.
   * @param camera lidar transform (to do : remove)
   * @param initial pose
   */
  VLO6dof(const VO_FrameToFrame_5point::Ptr &visual_odometer,
          const LidarOdometerFrameToFrame::Ptr &lidar_odometer,
          const Transform3D &camera_lidar_transform,
          const Transform3D &initial_pose);
  /**
   * @brief Runs the pipeline of visual lidar odometry [todo move to iodometry].
   * @param lidar_frame Frame/node with acquired point cloud data
   */
  void addVisualAndLidarFrames(IFrame::Ptr &visual_lidar_frame);
   double calculateCrossCheckingError(const IFrame::Ptr & current_frame, const IFrame::Ptr & reference_frame) override;
  /**
   * @brief It returns the last transformation from node frame to node frame
   * [todo: should be moved to Iodometer]
   * @return It returns the last transformation from node frame to node frame
   */
  Transform3D getFrameToFrameTransformation() const;
  /**
   * @brief  todo: remove
   */
  void calculateScaleFromGroundTruthVLO(const Vec3 &d_ground_truth);
  /**
   * @brief A overriden member function. It computes the transformation between
   * two frames of readings (that are used as nodes)
   * @param current_frame Pointer to the frame with data currently acquired.
   * @param reference_frame Pointer to the frame with acquired in the past time
   * stamp
   * @return It returnes the relative transformation from reference_frame to the
   * current_frame and additionally it informs whether the transformation is
   * reliable (in terms of inner mechanism of failure detection of the
   * respective odometry.
   */
  std::tuple<Transform3D, bool> findFrameToFrameTransformBetween(
      IFrame::Ptr &current_frame, IFrame::Ptr &reference_frame) override;
  std::tuple<Transform3D, bool> findFrameToFrameTransformBetween(
      IFrame::Ptr &current_frame, PointCloud &map) override
  {};
  /**
   * @brief update the reference frame and push frame to history [todo: should
   * be moved to Iodometer]
   */
  void addKeyFrame(const IFrame::Ptr &frame);
  /**
   * @brief It push the current transform to the history [todo: should be moved
   * to Iodometer]
   */
  void pushFrameToHistory(const IFrame::Ptr &camera_frame);

 private:
  /**
   * @brief Visual Odometer class
   */
  VO_FrameToFrame_5point::Ptr visual_odometer_;
  /**
   * @brief Lidar Odometer class.
   */
  LidarOdometerFrameToFrame::Ptr lidar_odometer_;
  /**
   * @brief  state of the odometry [todo: should be moved to Iodometer]
   */
  VLO_STATE state_ = VLO_STATE::INIT;
  /**
   * @brief  todo: remove
   */
  Transform3D camera_lidar_transform_ = Transform3D::Identity();
  /**
   * @brief scale for direction vector.
   */
  Vec3 scale_xyz_ = Vec3(1.0, 1.0, 1.0);
  /**
   * @brief scale for direction vector.
   */
  double scale_ = 1.0;
  /**
   * @brief [todo: remove]
   */
  double scale_gt_ = 1.0;
  /**
   * @brief history of poses [todo: should be moved to Iodometer]
   */
  std::vector<IFrame::Ptr> trajectory_nodes_;
  /**
   * @brief  current lidar frame with data [todo: should be moved to Iodometer]
   */
  IFrame::Ptr current_frame_;
  /**
   * @brief  previous lidar frame with data [todo: should be moved to Iodometer]
   */
  IFrame::Ptr reference_frame_;
};

};  // namespace vlo
