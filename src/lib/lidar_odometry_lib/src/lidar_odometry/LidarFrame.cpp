#include "lidar_odometry/LidarFrame.hpp"

namespace vlo {

void cropCloudWithBox(PointCloud &point_cloud, double z_level) {
  auto bbox = open3d::geometry::AxisAlignedBoundingBox(Vec3(-40, -30, -z_level),
                                                       Vec3(40, 30, z_level));

  point_cloud = *point_cloud.Crop(bbox);
}

void removeRoofPoints(PointCloud &point_cloud, double min_depth) {
  auto rth = min_depth / 2.0;
  auto zth = 1.0;
  auto bbox = open3d::geometry::AxisAlignedBoundingBox(Vec3(-rth, -rth, -zth),
                                                       Vec3(+rth, +rth, +zth));
  auto ind = bbox.GetPointIndicesWithinBoundingBox(point_cloud.points_);
  auto far_points = point_cloud.SelectByIndex(ind, true);
  auto near_points = point_cloud.SelectByIndex(ind);
  point_cloud = *far_points;
}

void approximateNormals(PointCloud &point_cloud) {
  point_cloud.normals_.resize(point_cloud.points_.size());
  for (size_t i = 0; i < point_cloud.points_.size(); i++) {
    auto normal = point_cloud.points_[i] / point_cloud.points_[i].norm();
    point_cloud.normals_[i] = normal;
  }
}

void removeStatisticalOutliers(PointCloud &point_cloud, uint nb_neighbors,
                               double std_ratio) {
  auto [inlier_cloud, ind] =
      point_cloud.RemoveStatisticalOutliers(nb_neighbors, std_ratio);
  auto statistical_outliers = point_cloud.SelectByIndex(ind, true);
  point_cloud = *inlier_cloud;
}

void display_inlier_outlier(PointCloud &inlier_cloud,
                            PointCloud &outlier_cloud) {
  inlier_cloud.PaintUniformColor(Vec3(0.8, 0.8, 0.8));
  outlier_cloud.PaintUniformColor(Vec3(1, 0, 0));

  open3d::visualization::DrawGeometries(
      {std::make_shared<PointCloud>(outlier_cloud),
       std::make_shared<PointCloud>(inlier_cloud)},
      "Name", 1600, 900);
}

void filter_far_points(PointCloud &point_cloud) {
  PointCloud out_cloud;
  for (size_t i = 0; i < point_cloud.points_.size(); i++) {
    auto norm = point_cloud.points_[i].norm();
    if (norm > 0.0 and norm < 50.0) {
      out_cloud.points_.push_back(point_cloud.points_[i]);
    }
  }
  point_cloud = out_cloud;
  point_cloud.EstimateNormals(
      open3d::geometry::KDTreeSearchParamHybrid(0.1, 30));
}
void preprocessPointCloud(PointCloud &point_cloud, uint nb_neighbors = 10,
                          double std_ratio = 1.0, double z_level = 3.0,
                          double min_depth = 5.5, bool compute_normals = true) {
  //  PointCloud point_cloud_prev = point_cloud;
  filter_far_points(point_cloud);
//  display_inlier_outlier(point_cloud, point_cloud_prev);

  //  point_cloud_prev = point_cloud;
  cropCloudWithBox(point_cloud, z_level);
 //  display_inlier_outlier(point_cloud, point_cloud_prev);

    //point_cloud_prev = point_cloud;
  removeRoofPoints(point_cloud, min_depth);
      //display_inlier_outlier(point_cloud, point_cloud_prev);

    //  point_cloud_prev = point_cloud;
  removeStatisticalOutliers(point_cloud, nb_neighbors, std_ratio);
  //display_inlier_outlier(point_cloud, point_cloud_prev);
  approximateNormals(point_cloud);
  point_cloud.EstimateNormals();

}

int LidarFrame::factory_id_ = 0;

LidarFrame::LidarFrame(PointCloud &point_cloud,
                       Transform3D transform_to_reference_link) {
  id_ = factory_id_++;


//preprocessPointCloud(point_cloud);
  std::cout << "After processing. " << point_cloud.points_.size() << std::endl;
 //point_cloud_ = point_cloud;
  std::cout << "Approximate normals. " << std::endl;
  approximateNormals(point_cloud);
  std::cout << "Estimate normals. " << std::endl;
 point_cloud.EstimateNormals();
 std::cout << "Transform Point Cloud to camera reference frame." << std::endl;
  point_cloud_ = point_cloud.Transform(transform_to_reference_link.matrix());

  point_cloud_ = *point_cloud_.VoxelDownSample(0.25);
}
LidarFrame::~LidarFrame() {}

double LidarFrame::evaluateOverlappingWithFrame(
    const IFrame::Ptr &frame, const Transform3D &transform_from_this_to_frame) {

}

}  // namespace vlo
