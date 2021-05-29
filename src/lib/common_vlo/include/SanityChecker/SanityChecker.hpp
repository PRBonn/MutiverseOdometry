#pragma once
#include "IFrame.hpp"
#include "common_vlo.hpp"
/**
 * @brief Namespace for the Visual Lidar Odometry Library
 */
namespace vlo {
/**
 * @brief The SanityChecker class declares a class for checking whether the
 * transformation that were estimated from the IOdometer classes (and their
 * derivatives) are valid
 */
class SanityChecker {
 public:
  /**
   * @brief Pointer for an interface.
   */
  using Ptr = std::shared_ptr<SanityChecker>;
  SanityChecker();
  /**
   * @brief It checks consistency of the transformation between current frame
   * (node) and trajectories of previous node and says whether the pose that
   * were estimated is possible or not
   * @param current frame/node
   * @param trajectory of previous frames/nodes
   * @return returns whether we should trust the estimated transformation
   */
  bool trustResult(const IFrame::Ptr& current_frame,
                   std::vector<IFrame::Ptr>& nodes_trajectory) const;
  /**
   * @brief It checks consistency of the transformation between current frame
   * (node) and trajectories of previous node and says whether the pose that
   * were estimated is possible or not
   * @param current frame/node
   * @param trajectory of previous frames/nodes
   * @return returns whether we should trust the estimated transformation
   */
  bool dynamicsChecker(const IFrame::Ptr& current_frame,
                       std::vector<IFrame::Ptr>& nodes_trajectory) const;
  std::tuple<double, double, bool, vlo::Vec3> dynamicsChecker(
      const Transform3D& proposal_transform, const IFrame::Ptr& reference_frame,
      const IFrame::Ptr& current_frame);
  /**
   * @brief It checks consistency of the transformation between current frame
   * (node) and the reference frame trajecto
   * @param current frame/node
   * @param trajectory of previous frames/nodes
   * @return returns whether we should trust the estimated transformation
   */
  bool check(const IFrame::Ptr& current_frame,
             const IFrame::Ptr& reference_frame) const;

  bool checkAckermann(Transform3D& frame_to_frame_transform, double dt) const;
  std::tuple<double, bool> checkAckermann(const Transform3D &frame_to_frame_transform,
                 const IFrame::Ptr &current_frame,
                 const IFrame::Ptr &reference_frame);
  bool checkAckermann(const IFrame::Ptr& current_frame,
                      const IFrame::Ptr& reference_frame) const;

  void activateDynamicsChecker() { check_dynamics_ = true; }
  void activateAckermannChecker() { check_ackermann_ = true; }
  /**
   * @brief Max magnitude acceleration for the given odometry checking
   */
  double max_acceleration_ = 20.0;
  /**
   * @brief Max magnitude velocity for the given odometry checking
   */
  double max_velocity_ = 0.0;
  /**
   * @brief Max x displacement for the given odometry checking
   */
  double max_x_displacement_ = 0.0;
  /**
   * @brief Max y displacement for the given odometry checking
   */
  double max_y_displacement_ = 0.0;
  bool check_ackermann_ = false;
  bool check_dynamics_ = false;
  bool activated_ = false;

  double previous_velocity_ = 0.0;
  Vec3 previous_velocity_vector_ = Vec3(0,0,0);
};
}  // namespace vlo
