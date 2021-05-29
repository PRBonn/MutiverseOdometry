#pragma once
#include <opencv2/core/core.hpp>

#include "common_vlo.hpp"
/**
 * @brief Namespace for the Visual Lidar Odometry Library
 */
namespace vlo {
/**
 * @brief The MapPoint class stores information about map point in the map. It
 * has positions, frames from where were visible, features, etc.
 */
class MapPoint {
 public:
  /**
   * @brief Pointer for an interface.
   */
  using Ptr = std::shared_ptr<MapPoint>;
  /**
   * @brief Constructor.
   * @param Position 3D.
   * @param Descriptor of the feature (either 3D or 2D).
   * @param Direction vector from the view point.
   * @param Color.
   */
  MapPoint(const Vec3 &pos, const cv::Mat &descriptor, const Vec3 &norm,
           const cv::Vec3b &color);
  /**
   * @brief set position of the point.
   * @param Position 3D.
   */
  inline void setPosition(const Vec3 &pos);
  /**
   * @return Get 3D position of the point map.
   */
  Vec3 getPosition();
  /**
   * @brief Get color RGB [0-255].
   * @return RGB [0-255]
   */
  cv::Vec3b getColor();
  /**
   * @brief Returns ID
   * @return ID.
   */
  inline int getID() { return id_; }
  /**
   * @brief Map point id.
   */
  unsigned int id_;
  /**
   * @brief Creator of ids of points.
   */
  static int factory_id_;
  /**
   * @brief Position of the map point.
   */
  Vec3 position_;
  /**
   * @brief Viewing direction of the map point.
   */
  Vec3 viewing_direction_;
  /**
   * @brief Color of point cloud.
   */
  cv::Vec3b color_;
  /**
   * @brief Number of times the point was visible.
   */
  uint visible_times_ = 0;
  /**
   * @brief Descriptor of the map point.
   */
  cv::Mat descriptor_;
};
}  // namespace vlo
