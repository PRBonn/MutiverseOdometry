#pragma once

#include <open3d/pipelines/registration/Registration.h>

#include <fstream>
#include <iostream>

#include "ILidarOdometer.hpp"
#include "common_vlo.hpp"
#include "utils_vlo.hpp"
#include "lidar_odometry/LidarFrame.hpp"
#include "lidar_odometry/point_cloud_analysis.hpp"
/**
 * @brief Namespace for the Visual Lidar Odometry Library
 */
namespace vlo {

using CellErrorScale = std::pair<double, double>;
using GridErrorScale = std::vector<CellErrorScale>;
/**
 * @brief The LidarOdometer1DOF that does icp only in 1 dimension where the
 * translation vector points.
 */
class LidarOdometer1DOF {
 public:
  /**
   * @brief Pointer for an interface.
   */
  using Ptr = std::shared_ptr<LidarOdometer1DOF>;
  /**
   * @brief Constructor
   */
  LidarOdometer1DOF();
  /**
   * @brief Function that setups direction of the guess for icp.
   * @param direction vector [todo: check whether the rotation matrix is
   * identity]
   */
  void directionHintICP(const vlo::Transform3D &t_direction);
  /**
   * @brief member functions that calculate scale of the direction vector (for
   * grid search).
   * @param current node/frame
   * @param previous node/frame
   * @return returns scale in translation
   */
  double getScale(const IFrame::Ptr &current_frame,
                  const IFrame::Ptr &reference_frame);
  /**
   * @brief member functions that does 1d ICP.
   * @param current point cloud
   * @param previous point cloud
   * @param vector direction
   * @param scale guess
   * @param max distance to consider in kdtree
   * @param max number of iteration for icp finding
   * @return returns scale in translation
   */
  double do_icp_1d(const PointCloud &current_point_cloud,
                   const PointCloud &previous_point_cloud,
                   const Vec3 &t_direction, double scale,
                   std::vector<double> max_distances, int maxiterations);

  /**
   * @brief member functions that does 1d gree search.
   * @param current point cloud
   * @param previous point cloud
   * @param vector direction
   * @param scale minimum
   * @param scale maximum
   * @param guess for the scale
   * @param number of grids
   * @return returns scale in translation
   */
  double searchScale(const PointCloud &current_point_cloud,
                     const PointCloud &previous_point_cloud,
                     const Vec3 &t_direction, double s_min, double s_max,
                     double s_0, int ngrid);
  /**
   * @brief Function to evaluate data association between two point clouds.
   * @param source point cloud
   * @param target point cloud
   * @param kdtree that were created from target point cloud (it is useful when
   * there is multiple iteration for the same point cloud)
   * @param max correspondence distance
   * @param scale for which to evaluate
   * @param direction translation
   * @param max correspondence distance
   * @return registration result (rmse and correspondence)
   */
  open3d::pipelines::registration::RegistrationResult evaluateScale(
      const PointCloud &current_point_cloud,
      const PointCloud &previous_point_cloud,
      const open3d::geometry::KDTreeFlann &target_kdtree, double scale,
      Vec3 d_translation,
      double max_correspondence_distance =
          std::numeric_limits<double>::infinity());
  /**
   * @brief Function to evaluate data association between two point clouds.
   * @param source point cloud
   * @param target point cloud
   * @param kdtree that were created from target point cloud (it is useful when
   * there is multiple iteration for the same point cloud)
   * @param output point cloud that has been translated
   * @param scale for which to evaluate
   * @param direction translation
   * @param max correspondence distance
   * @return registration result (rmse and correspondence)
   */
  open3d::pipelines::registration::RegistrationResult evaluateScale(
      const PointCloud &current_point_cloud,
      const PointCloud &previous_point_cloud,
      const open3d::geometry::KDTreeFlann &target_kdtree,
      PointCloud &output_point_cloud, double scale, Vec3 d_translation,
      double max_correspondence_distance =
          std::numeric_limits<double>::infinity());
  /**
   * @brief  Run in debug mode.
   */
  bool debug_ = false;

 private:
  /**
   * @brief scale for direction vector.
   */
  double scale_ = 1.0;
  /**
   * @brief direction vector.
   */
  vlo::Transform3D t_direction_icp_ = vlo::Transform3D::Identity();
  /**
   * @brief  initial pose [todo: should be moved to Iodometer].
   */
  vlo::Transform3D initial_pose_ = vlo::Transform3D::Identity();
  /**
   * @brief  remove.
   */
  vlo::Transform3D camera_lidar_transform_ = Transform3D::Identity();
  /**
   * @brief  Should output information about state.
   */
  bool verbose_ = false;
};
};  // namespace vlo
