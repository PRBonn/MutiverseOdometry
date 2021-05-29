#pragma once

#include <fstream>
#include <iostream>
#include <memory>

#include "IFrame.hpp"
#include "IOdometer.hpp"
#include "common_vlo.hpp"
/**
 * @brief Namespace for the Visual Lidar Odometry Library
 */
namespace vlo {
/**
 * @brief The ILidarOdometer class declares an interface for any type of Lidar
 * Odometery.
 */
class ILidarOdometer : public IOdometer {
 public:
  /**
   * @brief Pointer for an interface.
   */
  using Ptr = std::shared_ptr<ILidarOdometer>;
  ILidarOdometer();
  ~ILidarOdometer();
  /**
   * @brief An abstract member function to override. Runs the pipeline of lidar
   * odometry.
   * @param lidar_frame Frame/node with acquired point cloud data
   */
  virtual void processFrame(IFrame::Ptr lidar_frame) = 0;
  virtual void hintForICP(const Transform3D &);
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
   * @brief It push the current transform to the history [todo: should be moved
   * to Iodometer]
   */
  void pushFrameToHistory(IFrame::Ptr lidar_frame);
  /**
   * @brief  [todo: should be moved to Iodometer]
   */
  bool debug_ = false;
  /**
   * @brief  [todo: should be moved to Iodometer]
   */
  bool verbose_ = false;

   void takeLocalMap(PointCloud &);

 public:
  /**
   * @brief  current lidar frame with data [todo: should be moved to Iodometer]
   */
  IFrame::Ptr current_lidar_frame_ = nullptr;
  /**
   * @brief  previous lidar frame with data [todo: should be moved to Iodometer]
   */
  IFrame::Ptr reference_lidar_frame_ = nullptr;
  /**
   * @brief  state of the odometry [todo: should be moved to Iodometer]
   */
  VLO_STATE lo_state_ = VLO_STATE::INIT;
  /**
   * @brief history of poses [todo: should be moved to Iodometer]
   */
  std::vector<IFrame::Ptr> history_frames_;
};

};  // namespace vlo
