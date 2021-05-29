#include "lidar_odometry/point_cloud_analysis.hpp"
#include <open3d/pipelines/registration/Registration.h>
namespace vlo {



open3d::pipelines::registration::RegistrationResult
getRegistrationResultAndCorrespondences(
    const open3d::geometry::PointCloud &source,
       const open3d::geometry::PointCloud &target,
    const open3d::geometry::KDTreeFlann &target_kdtree,
    double max_correspondence_distance, const Eigen::Matrix4d &transformation) {
  open3d::pipelines::registration::RegistrationResult result(transformation);
  if (max_correspondence_distance <= 0.0) {
    return result;
  }

  double error2 = 0.0;

#ifdef _OPENMP
#pragma omp parallel
  {
#endif
    double error2_private = 0.0;
    open3d::pipelines::registration::CorrespondenceSet correspondence_set_private;
#ifdef _OPENMP
#pragma omp for nowait
#endif
    for (int i = 0; i < (int)source.points_.size(); i++) {
      std::vector<int> indices(1);
      std::vector<double> dists(1);
      const auto &point = source.points_[i];
      if (target_kdtree.SearchHybrid(point, max_correspondence_distance, 1,
                                     indices, dists) > 0) {
        error2_private += dists[0];
        correspondence_set_private.push_back(Eigen::Vector2i(i, indices[0]));
      }
    }
#ifdef _OPENMP
#pragma omp critical
#endif
    {
      for (int i = 0; i < (int)correspondence_set_private.size(); i++) {
        result.correspondence_set_.push_back(correspondence_set_private[i]);
      }
      error2 += error2_private;
    }
#ifdef _OPENMP
  }
#endif

  if (result.correspondence_set_.empty()) {
    result.fitness_ = 0.0;
    result.inlier_rmse_ = 0.0;
  } else {
    size_t corres_number = result.correspondence_set_.size();
    result.fitness_ = (double)corres_number / (double)source.points_.size();
    result.inlier_rmse_ = std::sqrt(error2 / (double)corres_number);
  }
  return result;
}



}  // namespace vlo
