#include "GicpLO/GicpLO.hpp"

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/ndt.h>
#include <ros/ros.h>

#include <iostream>
//#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

#include <GicpLO/fast_gicp/gicp/fast_gicp.hpp>

#include "open3d_conversions/open3d_conversions.h"
#include <pcl/filters/approximate_voxel_grid.h>

namespace vlo {
GicpLO::GicpLO(const vlo::Transform3D &initial_pose,
               const std::string &config_file)
    : initial_pose_(initial_pose){
          //  std::ifstream ifs(config_file.c_str());
          //  icp.loadFromYaml(ifs);
      };

void GicpLO::processFrame(IFrame::Ptr lidar_frame) {
  current_lidar_frame_ = lidar_frame;

  if (lo_state_ == VLO_STATE::INIT) {
    current_lidar_frame_->pose_ = initial_pose_;
    pushFrameToHistory(current_lidar_frame_);
    lo_state_ = VLO_STATE::RUNNING;
  } else if (lo_state_ == VLO_STATE::RUNNING) {
    auto [frame_to_frame_transform, trust_result] =
        findFrameToFrameTransformBetween(current_lidar_frame_,
                                         reference_lidar_frame_);
    current_lidar_frame_->pose_.matrix() =
        reference_lidar_frame_->pose_.matrix() *
        frame_to_frame_transform.matrix();
    scale_ = frame_to_frame_transform.translation();

    pushFrameToHistory(current_lidar_frame_);
  }
}

pcl::PointCloud<pcl::PointXYZ>::Ptr align(
    pcl::Registration<pcl::PointXYZ, pcl::PointXYZ>::Ptr registration,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &target_cloud,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &source_cloud,
    const Mat44f &last_transform = Mat44f::Identity()) {
  registration->setInputTarget(target_cloud);
  registration->setInputSource(source_cloud);
  pcl::PointCloud<pcl::PointXYZ>::Ptr aligned(
      new pcl::PointCloud<pcl::PointXYZ>());

  auto t1 = ros::WallTime::now();
  registration->align(*aligned, last_transform);
  auto t2 = ros::WallTime::now();
  std::cout << "single : " << (t2 - t1).toSec() * 1000 << "[msec]" << std::endl;

  //  for (int i = 0; i < 10; i++) {
  //    registration->align(*aligned);
  //  }
  auto t3 = ros::WallTime::now();
  std::cout << "10times: " << (t3 - t2).toSec() * 1000 << "[msec]" << std::endl;
  std::cout << "fitness: " << registration->getFitnessScore() << std::endl
            << std::endl;

  return aligned;
}

std::tuple<Transform3D, bool> GicpLO::findFrameToFrameTransformBetween(
    IFrame::Ptr &current_frame, IFrame::Ptr &reference_frame) {
  std::cout << "GICP looks for the transformation between "
               "current_frame and reference_frame"
            << std::endl;
  std::cout << "GICP: Last transform: " << std::endl;
  std::cout << last_transform_.matrix() << std::endl;

  sensor_msgs::PointCloud2 current_pc_temp;
  sensor_msgs::PointCloud2 reference_pc_temp;

  open3d_conversions::open3dToRos(current_frame->point_cloud_, current_pc_temp,
                                  "world");
  open3d_conversions::open3dToRos(reference_frame->point_cloud_,
                                  reference_pc_temp, "world");

  pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr reference_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::moveFromROSMsg<pcl::PointXYZ>(current_pc_temp, *current_cloud);
  pcl::moveFromROSMsg<pcl::PointXYZ>(reference_pc_temp, *reference_cloud);




//  double downsample_resolution = 0.25;
//  pcl::ApproximateVoxelGrid<pcl::PointXYZ> voxelgrid;
//  voxelgrid.setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);





  fast_gicp::FastGICP<pcl::PointXYZ, pcl::PointXYZ> gicp;
  gicp.setMaxCorrespondenceDistance(1.0);


//  voxelgrid.setInputCloud(reference_cloud);

//  voxelgrid.filter(*reference_cloud);

//  voxelgrid.setInputCloud(current_cloud);

//  voxelgrid.filter(*current_cloud);


  gicp.setInputTarget(reference_cloud);
  gicp.setInputSource(current_cloud);
  auto aligned = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  gicp.align(*aligned, last_transform_.matrix().cast<float>());
  gicp.swapSourceAndTarget();
  std::cout << gicp.getFinalTransformation().cast<double>().matrix()
            << std::endl;

  Transform3D final_transform = Transform3D::Identity();
  final_transform.matrix() =
      gicp.getFinalTransformation().matrix().cast<double>();

  last_transform_ = final_transform.matrix().cast<double>();

  return {final_transform, true};
}

std::tuple<Transform3D, bool> GicpLO::findFrameToFrameTransformBetween(
    IFrame::Ptr &current_frame, PointCloud &map) {
  std::cout << "Point2Plane ICP looks for the transformation between "
               "current_frame and map"
            << std::endl;
  std::cout << "Point2Plane ICP: Last transform: " << std::endl;

  return {Transform3D::Identity(), true};
}

double GicpLO::calculateCrossCheckingError(const IFrame::Ptr &current_frame,
                                           const IFrame::Ptr &reference_frame) {
  // TODO:implement
  return 5.0;
}

void GicpLO::hintForICP(const Transform3D &guess_for_icp) {
  guess_for_icp_ = guess_for_icp;
}

OdometerStruct getGicpLOAndInitialize(Transform3D reference_pose,
                                      std::string config_file, bool debug,
                                      bool verbose) {
  std::shared_ptr<vlo::GicpLO> point2plane_lo =
      std::make_shared<vlo::GicpLO>(reference_pose, config_file);
  point2plane_lo->verbose_ = debug;
  point2plane_lo->debug_ = verbose;

  return vlo::OdometerStruct{point2plane_lo, Transform3D::Identity()};
}

};  // namespace vlo
