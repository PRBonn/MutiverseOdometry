#pragma once
#include <boost/circular_buffer.hpp>

#include "common_vlo.hpp"
#include "utils/conversions.hpp"
#include "visual_odometry/MapPoint.hpp"
#include "visual_odometry/VisualFrame.hpp"
/**
 * @brief Namespace for the Visual Lidar Odometry Library
 */
namespace vlo {
/**
 * @brief Class Map to keep local map [for the time being]
 */
class Map {
 public:
  /**
   * @brief Pointer for an interface.
   */
  using Ptr = std::shared_ptr<Map>;
  /**
   * @brief todo: remove
   */
  std::unordered_map<int, VisualFrame::Ptr> keyframes_;
  /**
   * @brief Constructor.
   */
  Map();
  /**
   * @brief Circular buffor to keep map points and their features.
   */
  boost::circular_buffer<MapPoint::Ptr> map_points_;
  /**
   * @brief TODO: Remove.
   */
  void insertKeyFrame(VisualFrame::Ptr frame);
  /**
   * @brief Add map point to the map.
   * @param Point to add
   */
  void addMapPoint(const MapPoint::Ptr &point);
  /**
   * @brief Get Pointer to the map.
   */
  boost::circular_buffer<MapPoint::Ptr> getMap();
  /**
   * @brief Clean map.
   */
  void eraseMap();
  /**
   * @brief Get descriptor of each of the map point.
   * @return matrix of descriptors.
   */
  cv::Mat getDescriptors() const;
  /**
   * @return Numbers of points in the map.
   */
  int size() const;
  /**
   * @brief Operator for easy map point access.
   */
  MapPoint::Ptr operator[](int index);
  /**
   * @brief Returns points 3d that are visible from current view point.
   * @param current frame
   * @param [output] points 3d
   */
  void getPointsExpressedInGivenFrame(const VisualFrame::Ptr &frame,
                                      Points3Dd& points3d);
};
}  // namespace vlo
