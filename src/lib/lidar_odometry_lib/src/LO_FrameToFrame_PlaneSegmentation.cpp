#include "LO_FrameToFrame_PlaneSegmentation.hpp"

namespace vlo {
LO_FrameToFrame_PlaneSegmentation::LO_FrameToFrame_PlaneSegmentation(
    const Transform3D &initial_pose)
    : initial_pose_(initial_pose){};

void LO_FrameToFrame_PlaneSegmentation::processFrame(IFrame::Ptr lidar_frame) {
  current_lidar_frame_ = lidar_frame;
  current_lidar_frame_->point_cloud_.EstimateNormals(
      open3d::geometry::KDTreeSearchParamHybrid(0.1, 30));
  if (lo_state_ == LO_STATE::INIT) {
    current_lidar_frame_->pose_ = initial_pose_;
    pushFrameToHistory(current_lidar_frame_);
    lo_state_ = LO_STATE::RUNNING;
  } else if (lo_state_ == LO_STATE::RUNNING) {
    scanToScanTransformICP();
    pushFrameToHistory(current_lidar_frame_);
  }
}

void LO_FrameToFrame_PlaneSegmentation::
    scanToScanTransformICP()  // TODO: you can call this function more
                              // reasonable
{
  auto [current_points, current_indexes] =
      current_lidar_frame_->point_cloud_.SegmentPlane(0.2);
  auto [reference_points, reference_indexes] =
      reference_lidar_frame_->point_cloud_.SegmentPlane(0.2);

  auto current_pcd_without_plane =
      current_lidar_frame_->point_cloud_.SelectByIndex(current_indexes, true);
  auto reference_pcd_without_plane =
      reference_lidar_frame_->point_cloud_.SelectByIndex(reference_indexes,
                                                         true);

  //////////////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////////////////

  bool remove_nan = true;
  bool remove_infinite = true;
  size_t nb_neighbors = 20;
  double std_ratio = 2.0;
  size_t nb_points = 16;
  double search_radius = 0.05;
  double voxel_size = 0.05;
  double distance_threshold = voxel_size * 0.4;

  auto current_pcd_plane =
      current_lidar_frame_->point_cloud_.SelectByIndex(current_indexes, false);
  auto reference_pcd_plane = reference_lidar_frame_->point_cloud_.SelectByIndex(
      reference_indexes, false);
  auto current_pcd_plane_down = current_pcd_plane->VoxelDownSample(voxel_size);
  current_pcd_plane_down->EstimateNormals(
      open3d::geometry::KDTreeSearchParamHybrid(0.1, 30));
  current_pcd_plane_down->RemoveNonFinitePoints(remove_nan, remove_infinite);
  current_pcd_plane_down->RemoveRadiusOutliers(nb_points, search_radius);
  current_pcd_plane_down->RemoveStatisticalOutliers(nb_neighbors, std_ratio);
  auto current_pcd_plane_down_fpfh = open3d::registration::ComputeFPFHFeature(
      *current_pcd_plane_down,
      open3d::geometry::KDTreeSearchParamHybrid(0.25, 100));

  auto reference_pcd_plane_down =
      reference_pcd_plane->VoxelDownSample(voxel_size);
  reference_pcd_plane_down->EstimateNormals(
      open3d::geometry::KDTreeSearchParamHybrid(0.1, 30));
  reference_pcd_plane_down->RemoveNonFinitePoints(remove_nan, remove_infinite);
  reference_pcd_plane_down->RemoveRadiusOutliers(nb_points, search_radius);
  reference_pcd_plane_down->RemoveStatisticalOutliers(nb_neighbors, std_ratio);

  auto reference_pcd_plane_down_fpfh = open3d::registration::ComputeFPFHFeature(
      *reference_pcd_plane_down,
      open3d::geometry::KDTreeSearchParamHybrid(0.25, 100));

  /////////////////////////////////
  // std::shared_ptr<open3d::geometry::PointCloud> pointcloud_ptr_try0(new
  // open3d::geometry::PointCloud); open3d::geometry::PointCloud pointcloud_print
  // = current_lidar_frame_->point_cloud_; *pointcloud_ptr_try0 =
  //pointcloud_print;
  // open3d::visualization::DrawGeometries({pointcloud_ptr_try0},"current");
  // open3d::visualization::DrawGeometries({current_pcd_plane},"plane");
  // open3d::visualization::DrawGeometries({current_pcd_without_plane},"rest");
  /////////////////////////////////////////////////////////////////////////////////////
  auto current_pcd_without_plane_down =
      current_pcd_without_plane->VoxelDownSample(voxel_size);
  current_pcd_without_plane_down->EstimateNormals(
      open3d::geometry::KDTreeSearchParamHybrid(0.1, 30));
  current_pcd_without_plane_down->RemoveNonFinitePoints(remove_nan,
                                                        remove_infinite);
  current_pcd_without_plane_down->RemoveRadiusOutliers(nb_points,
                                                       search_radius);
  current_pcd_without_plane_down->RemoveStatisticalOutliers(nb_neighbors,
                                                            std_ratio);

  auto current_pcd_without_plane_down_fpfh =
      open3d::registration::ComputeFPFHFeature(
          *current_pcd_without_plane_down,
          open3d::geometry::KDTreeSearchParamHybrid(0.4, 100));

  auto reference_pcd_without_plane_down =
      reference_pcd_without_plane->VoxelDownSample(voxel_size);
  reference_pcd_without_plane_down->EstimateNormals(
      open3d::geometry::KDTreeSearchParamHybrid(0.1, 30));
  reference_pcd_without_plane_down->RemoveNonFinitePoints(remove_nan,
                                                          remove_infinite);
  reference_pcd_without_plane_down->RemoveRadiusOutliers(nb_points,
                                                         search_radius);
  reference_pcd_without_plane_down->RemoveStatisticalOutliers(nb_neighbors,
                                                              std_ratio);

  auto reference_pcd_without_plane_down_fpfh =
      open3d::registration::ComputeFPFHFeature(
          *reference_pcd_without_plane_down,
          open3d::geometry::KDTreeSearchParamHybrid(0.4, 100));
  ///////////////////////////////////////////////////////////////////////////////////
  /* double voxel_size = 0.05;

  auto reference_pcd_plane = reference_lidar_frame_->point_cloud_;
  std::shared_ptr<open3d::geometry::PointCloud> pointcloud_ptr_try0(new
  open3d::geometry::PointCloud); *pointcloud_ptr_try0 = reference_pcd_plane;
  auto reference_pcd_plane_down =
  pointcloud_ptr_try0->VoxelDownSample(voxel_size);
  reference_pcd_plane_down->EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(0.1,
  30)); auto reference_pcd_plane_down_fpfh =
  open3d::registration::ComputeFPFHFeature(*reference_pcd_plane_down,
  open3d::geometry::KDTreeSearchParamHybrid(0.25, 100));

  auto current_pcd_plane = current_lidar_frame_->point_cloud_;
  std::shared_ptr<open3d::geometry::PointCloud> pointcloud_ptr_try1(new
  open3d::geometry::PointCloud); *pointcloud_ptr_try1 = current_pcd_plane; auto
  current_pcd_plane_down = pointcloud_ptr_try1->VoxelDownSample(voxel_size);
  current_pcd_plane_down->EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(0.1,
  30)); auto current_pcd_plane_down_fpfh =
  open3d::registration::ComputeFPFHFeature(*current_pcd_plane_down,
  open3d::geometry::KDTreeSearchParamHybrid(0.25, 100));
*/
  ///////////////////////////////////////////////////////////////////////////////

  std::vector<
      std::reference_wrapper<const open3d::registration::CorrespondenceChecker>>
      correspondence_checker;
  auto correspondence_checker_edge_length =
      open3d::registration::CorrespondenceCheckerBasedOnEdgeLength(0.9);
  auto correspondence_checker_distance =
      open3d::registration::CorrespondenceCheckerBasedOnDistance(0.075);
  auto correspondence_checker_normal =
      open3d::registration::CorrespondenceCheckerBasedOnNormal(0.52359878);

  correspondence_checker.push_back(correspondence_checker_edge_length);
  correspondence_checker.push_back(correspondence_checker_distance);
  // correspondence_checker.push_back(correspondence_checker_normal);
  auto result_ransac =
      open3d::registration::RegistrationRANSACBasedOnFeatureMatching(
          *current_pcd_without_plane_down, *reference_pcd_without_plane_down,
          *current_pcd_without_plane_down_fpfh,
          *reference_pcd_without_plane_down_fpfh, 0.075,
          open3d::registration::TransformationEstimationPointToPoint(false), 3,
          correspondence_checker,
          open3d::registration::RANSACConvergenceCriteria(4000000, 1000));

  auto result_icp = open3d::registration::RegistrationICP(
      *current_pcd_without_plane_down, *reference_pcd_without_plane_down, 0.3,
      result_ransac.transformation_,
      open3d::registration::TransformationEstimationPointToPlane());

  /* std::shared_ptr<open3d::geometry::PointCloud> source_transformed_ptr(
      new open3d::geometry::PointCloud);
  std::shared_ptr<open3d::geometry::PointCloud> target_ptr(new
  open3d::geometry::PointCloud); *source_transformed_ptr =
  *current_pcd_plane_down; *target_ptr = *reference_pcd_plane_down; */
  // source_transformed_ptr->Transform(result_icp.transformation_);
  // open3d::visualization::DrawGeometries({source_transformed_ptr,
  // target_ptr},"Registration result");

  // open3d::visualization::DrawGeometries({current_pcd_plane_down,
  // reference_pcd_plane_down},"view temp");

  current_lidar_frame_->pose_.matrix() =
      reference_lidar_frame_->pose_.matrix() * result_icp.transformation_;
  //////////////////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////////////////

  /*         auto result = open3d::registration::RegistrationICP(
              *current_pcd_without_plane, *reference_pcd_without_plane, 0.3,
              Eigen::Matrix4d::Identity(),
              open3d::registration::TransformationEstimationPointToPlane());
          std::cout << "result: " << result.transformation_ << std::endl;

          current_lidar_frame_->pose_.matrix() =
              reference_lidar_frame_->pose_.matrix() *
              result.transformation_; */
}

};  // namespace vlo
