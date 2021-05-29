#pragma once


#include "ILidarOdometer.hpp"
#include "iostream"
#include <memory>
/**
 * @brief Namespace for the Visual Lidar Odometry Library
 */
namespace vlo
{
class LO_FrameToFrame_ImageBasedPlaneSegmentation : public ILidarOdometer
{
public:
  LO_FrameToFrame_ImageBasedPlaneSegmentation();
  LO_FrameToFrame_ImageBasedPlaneSegmentation(Transform3D initial_pose);
  void processFrame(LidarFrame::Ptr lidar_frame) override;
  void scanToScanTransformICP();
  void setupICPConfigFile(const std::string configFile);
private:
  Transform3D initial_pose_;

};
};
