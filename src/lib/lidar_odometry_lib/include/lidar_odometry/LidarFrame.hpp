#pragma once
#include <open3d/Open3D.h>

#include "IFrame.hpp"
#include "common_vlo.hpp"
/**
 * @brief Namespace for the Visual Lidar Odometry Library
 */
namespace vlo {
/**
 * @brief The LidarFrame class declares a class for keeping lidar-type data and
 * process in odometry pipeline.
 */
class LidarFrame : public IFrame {
 public:
  /**
   * @brief Pointer for an interface.
   */
  using Ptr = std::shared_ptr<LidarFrame>;
  /**
   * @brief Constructor.
   * @param point_cloud that is acquired from the lidar.
   */
  LidarFrame(PointCloud &point_cloud,
             Transform3D transform_to_reference_link = Transform3D::Identity());
  ~LidarFrame();
  /**
   * @brief static variable to numerate order of nodes.
   */
  static int factory_id_;
  /**
   * @brief [not implemented yet] Update good indexes that were taken under
   * consideration for finding transformation between two nodes.
   * @param Indexes that are valid.
   */
  void updateGoodIndexes(Indexes &indexes) override {
    std::cout << "not implemented yet";
  };
  /**
   * @brief [not implemented yet] Return good points
   * @return Return good keypoints that are transformed to cv::Points2D based on
   * good indexes that were updated, intitially (without any updates) it returns
   * just all key points.
   */
  ImagePointsd getGoodPoints() const override {
    std::cout << "not implemented yet";
  };
  /**
   * @brief [not implemented yet] Return good keypoints
   * @return Return good keypoints based on good indexes that were updated,
   * intitially (without any updates) it returns just all key points.
   */
  KeyPoints getGoodKeyPoints() const override {
    std::cout << "not implemented yet";
  };
  cv::Mat getGoodDescriptors() const override {
    std::cout << "not implemented yet";
  };
  double evaluateOverlappingWithFrame(
      const IFrame::Ptr &frame,
      const Transform3D &transform_from_this_to_frame) override;


};

}  // namespace vlo
