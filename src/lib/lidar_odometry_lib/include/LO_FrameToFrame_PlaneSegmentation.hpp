#pragma once

#include "ILidarOdometer.hpp"
#include "iostream"
#include <memory>
#include "IFrame.hpp"
/**
 * @brief Namespace for the Visual Lidar Odometry Library
 */
namespace vlo
{
class LO_FrameToFrame_PlaneSegmentation : public ILidarOdometer
{
public:
  LO_FrameToFrame_PlaneSegmentation(const Transform3D &initial_pose);
  void processFrame(IFrame::Ptr lidar_frame) override;
  void scanToScanTransformICP();

private:
  Transform3D initial_pose_;

};
};
