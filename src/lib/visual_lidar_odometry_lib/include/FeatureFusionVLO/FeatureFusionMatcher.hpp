#pragma once
#include <vector>

#include "IFrame.hpp"
#include "common_vlo.hpp"
#include "lidar_odometry/LidarFrame.hpp"
#include "visual_odometry/VisualFrame.hpp"
namespace vlo {
struct DescriptorWithPoint{
    cv::Mat descriptor;
    Vec3 point3d;
    int idx;
};
class FeatureFusionMatcher {
 public:
  FeatureFusionMatcher();
  void match(IFrame::Ptr &current_frame, IFrame::Ptr &reference_frame, Transform3D & camera_lidar_transform,
             std::vector<cv::DMatch> &matches);
  std::vector<DescriptorWithPoint>  matchWithinTheFrame( IFrame::Ptr & frame, const Transform3D &camera_lidar_transform) ;
};

}  // namespace vlo
