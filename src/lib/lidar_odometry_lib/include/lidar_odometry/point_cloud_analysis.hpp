#pragma once

#include <open3d/geometry/KDTreeFlann.h>
#include <open3d/geometry/PointCloud.h>
#include <open3d/pipelines/registration/Registration.h>

#include <cstdlib>

#include "common_vlo.hpp"
/**
 * @brief Namespace for the Visual Lidar Odometry Library
 */
namespace vlo {
/**
 * @brief Function to evaluate data association between two point clouds
 * @param source point cloud
 * @param target point cloud
 * @param kdtree that were created from target point cloud (it is useful when
 * there is multiple iteration for the same point cloud)
 * @param max correspondence distance
 * @param initial guess [todo: name should be change to transformation]
 */
open3d::pipelines::registration::RegistrationResult
getRegistrationResultAndCorrespondences(
    const open3d::geometry::PointCloud &source,
    const open3d::geometry::PointCloud &target,
    const open3d::geometry::KDTreeFlann &target_kdtree,
    double max_correspondence_distance, const Eigen::Matrix4d &transformation);

}  // namespace vlo
