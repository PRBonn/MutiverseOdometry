#pragma once
#include <eigen3/Eigen/Core>
#include <fstream>
#include <iostream>
#include <memory>

#include "IFrame.hpp"
#include "IOdometer.hpp"
#include "common_vlo.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/xfeatures2d.hpp"
/**
 * @brief Namespace for the Visual Lidar Odometry Library
 */
namespace vlo {
/**
 * @brief The IVisualOdometer class declares an interface for any type of Visual
 * Odometery.
 */
class IVisualOdometer : public IOdometer {
 public:
  /**
   * @brief Pointer for an interface.
   */
  using Ptr = std::shared_ptr<IVisualOdometer>;
  /**
   * @brief Constructor.
   * @param Initial pose.
   */
  IVisualOdometer(const Transform3D& initial_pose);
  /**
   * @brief Destructor.
   */
  ~IVisualOdometer();
  /**
   * @brief An abstract member function to override. Runs the pipeline of visual
   * odometry.
   * @param visual Frame/node with acquired point cloud data
   */
  virtual void processFrame(IFrame::Ptr& camera_frame) = 0;
  /**
   * @brief It returns the last estimated pose. [todo: remove]
   * @return It returnes the last estimated pose.
   */
  Transform3D getLastPose() const;
  /**
   * @brief It returns the last transformation from node frame to node frame
   * [todo: should be moved to Iodometer]
   * @return It returns the last transformation from node frame to node frame
   */
  Transform3D getFrameToFrameTransformation() const;
  /**
   * @brief remove
   */
  std::vector<IFrame::Ptr> getFramesHistory() const;
  /**
   * @brief It push the current transform to the history [todo: should be moved
   * to Iodometer]
   */
  void pushFrameToHistory(IFrame::Ptr& camera_frame);

  virtual void calculateScale(const Vec3 &);
  /**
   * @brief  [todo: should be moved to Iodometer]
   */
  bool debug_ = true;
  /**
   * @brief  [todo: should be moved to Iodometer]
   */
  bool verbose_ = true;

  // protected:
 public:
  /**
   * @brief  current lidar frame with data [todo: should be moved to Iodometer]
   */
  IFrame::Ptr current_frame_ = nullptr;
  /**
   * @brief  previous lidar frame with data [todo: should be moved to Iodometer]
   */
  IFrame::Ptr reference_frame_ = nullptr;
  /**
   * @brief  state of the odometry [todo: should be moved to Iodometer]
   */
  VLO_STATE vo_state_ = VLO_STATE::INIT;
  /**
   * @brief history of poses [todo: should be moved to Iodometer]
   */
  std::vector<IFrame::Ptr> history_frames_;
  /**
   * @brief Initial pose
   */
  Transform3D initial_pose_ = Transform3D::Identity();
  /**
   * @brief Scale
   */
  double scale_ = 1.0;


};

};  // namespace vlo
