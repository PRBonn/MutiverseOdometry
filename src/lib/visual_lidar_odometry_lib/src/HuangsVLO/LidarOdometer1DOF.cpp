#include "HuangsVLO/LidarOdometer1DOF.hpp"

namespace vlo {

LidarOdometer1DOF::LidarOdometer1DOF(){};

double LidarOdometer1DOF::getScale(const IFrame::Ptr &current_frame,
                                   const IFrame::Ptr &reference_frame) {
  PointCloudPtr rotated_point_cloud =
      std::make_shared<PointCloud>(current_frame->point_cloud_);
  rotated_point_cloud->Rotate(t_direction_icp_.rotation(),
                              Vec3::Zero());
  auto t1 = std::chrono::steady_clock::now();
  auto scale_grid =
      searchScale(*rotated_point_cloud, reference_frame->point_cloud_,
                  t_direction_icp_.translation(), 0.0, 6.0, 3.0,
                  5);  // TODO PARAMETRIZE
  auto t2 = std::chrono::steady_clock::now();
  auto period = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1);
  if (verbose_) {
    std::cout << "Scale search time: " << period.count() << std::endl;
  }
  std::vector<double> max_distances;
  max_distances.push_back(std::numeric_limits<double>::infinity());
//  max_distances.push_back(1.0);
//  max_distances.push_back(0.5);
  t1 = std::chrono::steady_clock::now();
  auto scale = do_icp_1d(*rotated_point_cloud, reference_frame->point_cloud_,
                         t_direction_icp_.translation(), scale_grid,
                         max_distances, 5);
  t2 = std::chrono::steady_clock::now();
  period = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1);
  if (verbose_) {
    std::cout << "1d icp time: " << period.count() << std::endl;
  }
  scale_ = scale;
  if (verbose_) {
    std::cout << "Scale from grid search: " << scale_grid
              << ", scale after icp " << scale_ << std::endl;
  }

  return scale_;
}

void LidarOdometer1DOF::directionHintICP(const vlo::Transform3D &t_direction) {
  t_direction_icp_ = t_direction;
}

double LidarOdometer1DOF::do_icp_1d(const PointCloud &rotated_point_cloud,
                                    const PointCloud &previous_point_cloud,
                                    const Vec3 &d_translation, double scale,
                                    std::vector<double> max_distances,
                                    int max_iterations = 5) {
  open3d::geometry::KDTreeFlann kdtree;
  kdtree.SetGeometry(previous_point_cloud);

  for (auto &max_distance : max_distances) {
    auto last_error_distance = std::numeric_limits<double>::infinity();
    double current_error_distance = 0.0;

    for (auto iteration = 0; iteration < max_iterations; iteration++) {
      PointCloud output_translated_point_cloud;
      //      auto t1 = std::chrono::steady_clock::now();
      auto result = evaluateScale(rotated_point_cloud, previous_point_cloud,
                                  kdtree, output_translated_point_cloud, scale,
                                  d_translation, max_distance);
      //      auto t2 = std::chrono::steady_clock::now();
      //      auto period =
      //          std::chrono::duration_cast<std::chrono::milliseconds>(t2 -
      //          t1);
      //      std::cout << "evaluate Scale finding in 1dicp : " <<
      //      period.count()
      //                << std::endl;

      double x = 0, y = 0, z = 0;

#pragma omp parallel for default(shared) reduction(+ : x, y, z)
      for (int i = 0; i < result.correspondence_set_.size(); i++) {
        Vec3 diffVector =
            output_translated_point_cloud
                .points_[result.correspondence_set_[i][0]] -
            previous_point_cloud.points_[result.correspondence_set_[i][1]];
        x += diffVector[0];
        y += diffVector[1];
        z += diffVector[2];
      }

      Vec3 vector_diffVector_sum = Vec3(x, y, z);
      scale_ = scale;
      Vec3 average_vector_diff =
          vector_diffVector_sum /
          static_cast<double>(result.correspondence_set_.size());
      double refined_scale = average_vector_diff.transpose().dot(d_translation);
      scale -= refined_scale;
      current_error_distance = result.inlier_rmse_;
      if (std::abs(last_error_distance - current_error_distance) < 0.0001) {
        break;
      }
      last_error_distance = current_error_distance;
    }
  }
  return scale;
}



double LidarOdometer1DOF::searchScale(const PointCloud &rotated_point_cloud,
                                      const PointCloud &previous_point_cloud,
                                      const Vec3 &t_direction, double s_min,
                                      double s_max, double s_0, int ngrid = 5) {
  open3d::geometry::KDTreeFlann kdtree;
  kdtree.SetGeometry(previous_point_cloud);

  double scale_to_return = 1.0;
  auto error_s_min_eval = evaluateScale(
      rotated_point_cloud, previous_point_cloud, kdtree, /* *s_min_init,*/
      s_min, t_direction);
  auto error_s_max_eval = evaluateScale(
      rotated_point_cloud, previous_point_cloud, kdtree, /* *s_max_init,*/
      s_max, t_direction);
  GridErrorScale grid_error_scale(ngrid);
  CellErrorScale cell_error_s_max{error_s_max_eval.inlier_rmse_, s_max};
  CellErrorScale cell_error_s_min{error_s_min_eval.inlier_rmse_, s_min};
  grid_error_scale.front() = cell_error_s_min;
  grid_error_scale.back() = cell_error_s_max;
  for (auto iteration = 0; iteration < 10; iteration++) {
    // std::cout << " Iteration " << iteration << std::endl;
    auto hypothesis_s = Eigen::ArrayXd::LinSpaced(ngrid, s_min, s_max);
    auto t1 = std::chrono::steady_clock::now();
    //#pragma omp parallel for shared(hypothesis_s, rotated_point_cloud,
    // previous_point_cloud,kdtree, t_direction,grid_error_scale)
    for (auto i_grid = 1; i_grid < hypothesis_s.size() - 1; i_grid++) {
      //  std::cout << " i grid: " << i_grid << std::endl;
      auto t1 = std::chrono::steady_clock::now();
      auto evaluation_result =
          evaluateScale(rotated_point_cloud, previous_point_cloud, kdtree,
                        hypothesis_s[i_grid], t_direction);
      CellErrorScale CellErrorScale{evaluation_result.inlier_rmse_,
                                    hypothesis_s[i_grid]};
      grid_error_scale[i_grid] = CellErrorScale;
      auto t2 = std::chrono::steady_clock::now();
      auto period2 =
          std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1);
      //  std::cout << "evaluate takes; " << period2.count() << std::endl;
    }
    auto t2 = std::chrono::steady_clock::now();
    auto period2 =
        std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1);
    std::sort(grid_error_scale.begin(), grid_error_scale.end(),
              vlo::sortbyfirst);
    auto e1 = grid_error_scale[0].first;
    auto e2 = grid_error_scale[1].first;
    scale_to_return = grid_error_scale[0].second;
    if (std::abs(e1 - e2) < 0.0001) {
      break;
    }
    if (grid_error_scale[0].second < grid_error_scale[1].second) {
      grid_error_scale.back() = grid_error_scale[1];
      s_min = grid_error_scale[0].second;
      s_max = grid_error_scale[1].second;
    } else {
      grid_error_scale.back() = grid_error_scale[0];
      grid_error_scale.front() = grid_error_scale[1];
      s_min = grid_error_scale.front().second;
      s_max = grid_error_scale.back().second;
    }
  }
  return scale_to_return;
}

// open3d::registration::RegistrationResult LidarOdometer1DOF::evaluateScale(
//    const PointCloud &current_point_cloud,
//    const PointCloud &previous_point_cloud, double scale, Vec3 d_translation,
//    double max_correspondence_distance) {
//  auto real_translation_hypothesis = d_translation * scale;
//  PointCloudPtr current = std::make_shared<PointCloud>(current_point_cloud);
//  Transform3D transform3D = Transform3D::Identity();
//  transform3D.translation() = real_translation_hypothesis;
//  auto results = open3d::registration::EvaluateRegistration(
//      *current, previous_point_cloud, max_correspondence_distance,
//      transform3D.matrix());
//  return results;
//}

// open3d::registration::RegistrationResult LidarOdometer1DOF::evaluateScale(
//    const PointCloud &current_point_cloud,
//    const PointCloud &previous_point_cloud, PointCloud &output_point_cloud,
//    double scale, Vec3 d_translation, double max_correspondence_distance) {
//  auto real_translation_hypothesis = d_translation * scale;
//  output_point_cloud = current_point_cloud;
//  output_point_cloud.Translate(real_translation_hypothesis);
//  auto results = open3d::registration::EvaluateRegistration(
//      output_point_cloud, previous_point_cloud, max_correspondence_distance,
//      Eigen::Matrix4d::Identity());
//  return results;
//}

open3d::pipelines::registration::RegistrationResult
LidarOdometer1DOF::evaluateScale(
    const PointCloud &current_point_cloud,
    const PointCloud &previous_point_cloud,
    const open3d::geometry::KDTreeFlann &target_kdtree, double scale,
    Vec3 d_translation, double max_correspondence_distance) {
  auto real_translation_hypothesis = d_translation * scale;
  PointCloudPtr current = std::make_shared<PointCloud>(current_point_cloud);
  Transform3D transform3D = Transform3D::Identity();
  transform3D.translation() = real_translation_hypothesis;
  current->Translate(transform3D.translation());
  auto results = getRegistrationResultAndCorrespondences(
      *current, previous_point_cloud, target_kdtree,
      max_correspondence_distance, transform3D.matrix());
  return results;
}

open3d::pipelines::registration::RegistrationResult
LidarOdometer1DOF::evaluateScale(
    const PointCloud &current_point_cloud,
    const PointCloud &previous_point_cloud,
    const open3d::geometry::KDTreeFlann &target_kdtree,
    PointCloud &output_point_cloud, double scale, Vec3 d_translation,
    double max_correspondence_distance) {
  auto real_translation_hypothesis = d_translation * scale;
  output_point_cloud = current_point_cloud;
  output_point_cloud.Translate(real_translation_hypothesis);
  auto results = getRegistrationResultAndCorrespondences(
      output_point_cloud, previous_point_cloud, target_kdtree,
      max_correspondence_distance, Eigen::Matrix4d::Identity());
  return results;
}

}  // namespace vlo
