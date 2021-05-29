#pragma once
#include "IFrame.hpp"
#include "lidar_odometry/LidarFrame.hpp"
#include "visual_odometry/VisualFrame.hpp"
/**
 * @brief Namespace for the Visual Lidar Odometry Library
 */
namespace vlo {
/**
 * @brief The VisualLidarFrame class declares a class for keeping
 * lidar-visual-type data and process in odometry pipeline.
 */
class VisualLidarFrame : public IFrame {
 public:
  /**
   * @brief Pointer for an interface.
   */
  using Ptr = std::shared_ptr<VisualLidarFrame>;
  /**
   * @brief Constructor
   * @param visual_frame visual frame with image from camera
   * @param lidar_frame lidar frame with point cloud from lidar
   * @param camera_lidar_transform transformation between camera and lidar
   */
  VisualLidarFrame(
      const VisualFrame::Ptr &visual_frame, const LidarFrame::Ptr &lidar_frame,
      const Transform3D &camera_lidar_transform = Transform3D::Identity(), bool color = false);

  /**
   * @brief [not implemented yet] Update good indexes that were taken under
   * consideration for finding transformation between two nodes
   * @param Indexes that are valid
   */
  void updateGoodIndexes(Indexes &indexes) override;
  /**
   * @brief [not implemented yet] Return good points
   * @return Return good keypoints that are transformed to cv::Points2D based on
   * good indexes that were updated, intitially (without any updates) it returns
   * just all key points.
   */
  ImagePointsd getGoodPoints() const override;
  /**
   * @brief [not implemented yet] Return good keypoints
   * @return Return good keypoints based on good indexes that were updated,
   * intitially (without any updates) it returns just all key points.
   */
  KeyPoints getGoodKeyPoints() const override;
  /**
   * @brief [not implemented yet] Return good descriptors
   * @return Return good descriptors based on good indexes that were updated,
   * intitially (without any updates) it returns just all descriptors.
   */
  cv::Mat getGoodDescriptors() const override;
  /**
   * @brief Transformation (translation + rotation) between camera and lidar
   * frame.
   */
  Transform3D camera_lidar_transform_;

  double evaluateOverlappingWithFrame(
      const IFrame::Ptr &frame,
      const Transform3D &transform_from_this_to_frame) override;

 private:
  /**
   * @brief member variable to keep visual frame
   */
  VisualFrame::Ptr visual_frame_;
  /**
   * @brief member variable to keep lidar frame
   */
  LidarFrame::Ptr lidar_frame_;
};

}  // namespace vlo
