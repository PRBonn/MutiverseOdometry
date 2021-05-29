#pragma once
#include "common_vlo.hpp"
/**
 * @brief Namespace for the Visual Lidar Odometry Library
 */
namespace vlo {
/**
 * @brief  The ISensor class declares an interface for any type of Sensor (camera, lidar).
 */
class ISensor {
 public:
  /**
   * @brief Pointer for an interface.
   */
  using Ptr = std::shared_ptr<ISensor>;
  virtual ~ISensor(){};
  /**
   * @brief Returns sensor matrix that is associated with it (for camera is a camera matrix)
   */
  virtual cv::Mat getSensorMatrix() const = 0;
};

}  // namespace vlo
